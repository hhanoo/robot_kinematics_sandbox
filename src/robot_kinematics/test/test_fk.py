"""
FK Unit Tests
=============
Two independent ground truths:
1. Closed-form zero-pose position/orientation derived by hand from the
   standard DH convention (not via the code under test).
2. Generic URDF chain composition from the xacro-expanded robot model
   (the exact model RViz displays), compared over random joint angles.
"""

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

from robot_kinematics.dh import UR10E_DH, dh_transform
from robot_kinematics.fk import fk, fk_frames

XACRO_PATH = (
    Path(__file__).resolve().parents[2]
    / "robot_description"
    / "urdf"
    / "ur10e.urdf.xacro"
)

# UR10e standard DH constants (same values as the xacro properties)
D1, A2, A3 = 0.1807, -0.6127, -0.57155
D4, D5, D6 = 0.17415, 0.11985, 0.11655


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


class TestZeroPose:
    def test_zero_pose_position(self):
        # Hand-derived closed form at q=0: x = a2+a3, y = -(d4+d6), z = d1-d5
        T = fk(np.zeros(6))
        expected = np.array([A2 + A3, -(D4 + D6), D1 - D5])
        np.testing.assert_allclose(T[:3, 3], expected, atol=1e-9)

    def test_zero_pose_orientation(self):
        # Hand-derived at q=0: R = Rx(90deg)
        T = fk(np.zeros(6))
        np.testing.assert_allclose(T[:3, :3], rot_x(math.pi / 2), atol=1e-9)


class TestFrames:
    def test_fk_frames_shape_and_consistency(self):
        q = np.array([0.3, -0.7, 1.1, -0.4, 0.9, -1.2])
        frames = fk_frames(q)
        assert frames.shape == (7, 4, 4)
        np.testing.assert_allclose(frames[0], np.eye(4), atol=1e-12)
        np.testing.assert_allclose(frames[-1], fk(q), atol=1e-12)

    def test_rotation_matrices_are_orthonormal(self):
        rng = np.random.default_rng(0)
        for _ in range(10):
            q = rng.uniform(-np.pi, np.pi, 6)
            for T in fk_frames(q):
                R = T[:3, :3]
                np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-9)
                assert math.isclose(np.linalg.det(R), 1.0, abs_tol=1e-9)


class TestDHTransform:
    def test_pure_d_translation(self):
        T = dh_transform(0.0, 1.5, 0.0, 0.0)
        np.testing.assert_allclose(T[:3, 3], [0, 0, 1.5], atol=1e-12)
        np.testing.assert_allclose(T[:3, :3], np.eye(3), atol=1e-12)

    def test_pure_a_translation(self):
        T = dh_transform(0.0, 0.0, 2.0, 0.0)
        np.testing.assert_allclose(T[:3, 3], [2, 0, 0], atol=1e-12)

    def test_theta_rotates_about_z(self):
        T = dh_transform(math.pi / 2, 0.0, 1.0, 0.0)
        # x-offset a is applied after the z-rotation → lands on +y
        np.testing.assert_allclose(T[:3, 3], [0, 1, 0], atol=1e-12)


# ---------------------------------------------------------------------------
# Ground truth from the xacro-expanded URDF (requires the xacro python module
# from ros-humble-xacro; run inside the project container).
# ---------------------------------------------------------------------------


def _urdf_root():
    xacro = pytest.importorskip("xacro")
    doc = xacro.process_file(str(XACRO_PATH))
    return ET.fromstring(doc.toxml())


def _rpy_matrix(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


def _axis_angle_matrix(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    C = 1 - c
    return np.array(
        [
            [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
            [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
            [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
        ]
    )


def _urdf_chain_fk(root, q):
    """Compose base_link→tool0 by walking joint parents back from tool0."""
    joints_by_child = {}
    for j in root.iter("joint"):
        child = j.find("child").attrib["link"]
        joints_by_child[child] = j

    chain = []
    link = "tool0"
    while link != "base_link":
        j = joints_by_child[link]
        chain.append(j)
        link = j.find("parent").attrib["link"]
    chain.reverse()

    q_by_joint = {f"link_{i+1}_joint": q[i] for i in range(6)}

    T = np.eye(4)
    for j in chain:
        origin = j.find("origin")
        xyz = [float(v) for v in origin.attrib.get("xyz", "0 0 0").split()]
        rpy = [float(v) for v in origin.attrib.get("rpy", "0 0 0").split()]
        step = np.eye(4)
        step[:3, :3] = _rpy_matrix(*rpy)
        step[:3, 3] = xyz
        T = T @ step
        if j.attrib["type"] == "revolute":
            axis = [float(v) for v in j.find("axis").attrib["xyz"].split()]
            rot = np.eye(4)
            rot[:3, :3] = _axis_angle_matrix(axis, q_by_joint[j.attrib["name"]])
            T = T @ rot
    return T


class TestAgainstURDF:
    def test_fk_matches_urdf_chain_random_q(self):
        root = _urdf_root()
        rng = np.random.default_rng(42)
        for _ in range(100):
            q = rng.uniform(-2 * np.pi, 2 * np.pi, 6)
            T_dh = fk(q)
            T_urdf = _urdf_chain_fk(root, q)
            np.testing.assert_allclose(T_dh[:3, 3], T_urdf[:3, 3], atol=1e-6)
            np.testing.assert_allclose(T_dh[:3, :3], T_urdf[:3, :3], atol=1e-6)
