"""
Cartesian Trajectory Unit Tests
===============================
Pose path geometry (straight line, slerp, circular arc) and
Cartesian-to-joint conversion via seeded IK.
"""

import math

import numpy as np

from robot_kinematics.fk import fk
from robot_kinematics.ik import rotation_vector
from robot_trajectory.cartesian_traj import (
    cartesian_to_joint,
    circle_pose_path,
    linear_pose_path,
    slerp,
)

Q_A = np.array([0.3, -1.4, 1.6, -1.8, -1.5, 0.2])
Q_B = np.array([0.5, -1.2, 1.3, -1.6, -1.4, 0.4])


def _rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


class TestSlerp:
    def test_endpoints(self):
        R0, R1 = np.eye(3), _rot_z(math.pi / 2)
        np.testing.assert_allclose(slerp(R0, R1, 0.0), R0, atol=1e-12)
        np.testing.assert_allclose(slerp(R0, R1, 1.0), R1, atol=1e-9)

    def test_halfway_is_half_angle(self):
        R0, R1 = np.eye(3), _rot_z(math.pi / 2)
        np.testing.assert_allclose(slerp(R0, R1, 0.5), _rot_z(math.pi / 4), atol=1e-9)

    def test_result_is_rotation_matrix(self):
        R0, R1 = _rot_z(0.3), _rot_z(2.1)
        for s in np.linspace(0, 1, 7):
            R = slerp(R0, R1, s)
            np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-9)
            assert math.isclose(np.linalg.det(R), 1.0, abs_tol=1e-9)


class TestLinearPath:
    def test_endpoints_and_collinearity(self):
        T0, T1 = fk(Q_A), fk(Q_B)
        poses = linear_pose_path(T0, T1, 11)
        assert len(poses) == 11
        np.testing.assert_allclose(poses[0], T0, atol=1e-12)
        np.testing.assert_allclose(poses[-1], T1, atol=1e-9)
        # Intermediate positions must lie on the straight segment
        p0, p1 = T0[:3, 3], T1[:3, 3]
        for k, T in enumerate(poses):
            expected = p0 + (p1 - p0) * k / 10.0
            np.testing.assert_allclose(T[:3, 3], expected, atol=1e-9)


class TestCirclePath:
    def test_points_stay_on_circle(self):
        T0 = fk(Q_A)
        center = T0[:3, 3] + np.array([0.1, 0.0, 0.0])
        axis = np.array([0.0, 0.0, 1.0])
        poses = circle_pose_path(T0, center, axis, 2 * math.pi, 36)
        radius = np.linalg.norm(T0[:3, 3] - center)
        for T in poses:
            r = np.linalg.norm(T[:3, 3] - center)
            assert math.isclose(r, radius, abs_tol=1e-9)

    def test_full_turn_returns_to_start(self):
        T0 = fk(Q_A)
        center = T0[:3, 3] + np.array([0.1, 0.0, 0.0])
        axis = np.array([0.0, 0.0, 1.0])
        poses = circle_pose_path(T0, center, axis, 2 * math.pi, 36)
        np.testing.assert_allclose(poses[-1][:3, 3], T0[:3, 3], atol=1e-9)

    def test_orientation_is_constant(self):
        T0 = fk(Q_A)
        center = T0[:3, 3] + np.array([0.0, 0.1, 0.0])
        poses = circle_pose_path(T0, center, np.array([0, 0, 1.0]), math.pi, 18)
        for T in poses:
            np.testing.assert_allclose(T[:3, :3], T0[:3, :3], atol=1e-12)


class TestCartesianToJoint:
    def test_tracks_linear_path(self):
        T0, T1 = fk(Q_A), fk(Q_B)
        poses = linear_pose_path(T0, T1, 21)
        result = cartesian_to_joint(poses, Q_A)
        assert result.success
        assert result.failed_index == -1
        # Every waypoint must be reached within IK tolerance
        for T_goal, q in zip(poses, result.q):
            T = fk(q)
            assert np.linalg.norm(T[:3, 3] - T_goal[:3, 3]) < 1e-3
            assert np.linalg.norm(rotation_vector(T_goal[:3, :3] @ T[:3, :3].T)) < 1e-3

    def test_joint_continuity(self):
        # Seeding each waypoint with the previous solution must avoid jumps
        T0, T1 = fk(Q_A), fk(Q_B)
        poses = linear_pose_path(T0, T1, 41)
        result = cartesian_to_joint(poses, Q_A)
        assert result.success
        assert np.max(np.abs(np.diff(result.q, axis=0))) < 0.2

    def test_unreachable_waypoint_reports_failure(self):
        T0 = fk(Q_A)
        T_far = T0.copy()
        T_far[:3, 3] = [3.0, 0.0, 0.5]
        poses = linear_pose_path(T0, T_far, 10)
        result = cartesian_to_joint(poses, Q_A)
        assert result.success is False
        assert 0 <= result.failed_index < 10
        assert np.all(np.isfinite(result.q))
