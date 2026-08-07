"""
DLS IK Unit Tests
=================
FK -> IK roundtrip over random reachable targets, graceful failure on
unreachable targets, and stability at/near the wrist singularity.
"""

import math

import numpy as np
import pytest

from robot_kinematics.fk import fk
from robot_kinematics.ik import IKResult, rotation_vector, solve_ik

POS_TOL = 1e-3  # 1 mm
ROT_TOL = math.radians(0.1)  # 0.1 deg


def _pose_errors(target, q):
    T = fk(q)
    pos_err = np.linalg.norm(target[:3, 3] - T[:3, 3])
    rot_err = np.linalg.norm(rotation_vector(target[:3, :3] @ T[:3, :3].T))
    return pos_err, rot_err


class TestRotationVector:
    def test_identity_is_zero(self):
        np.testing.assert_allclose(rotation_vector(np.eye(3)), np.zeros(3), atol=1e-12)

    def test_small_rotation_about_z(self):
        a = 1e-4
        c, s = math.cos(a), math.sin(a)
        R = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        np.testing.assert_allclose(rotation_vector(R), [0, 0, a], atol=1e-10)

    def test_half_pi_rotation_about_x(self):
        a = math.pi / 2
        c, s = math.cos(a), math.sin(a)
        R = np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])
        np.testing.assert_allclose(rotation_vector(R), [a, 0, 0], atol=1e-10)


class TestIKRoundtrip:
    def test_converges_from_perturbed_seed(self):
        # Tracking scenario: seed near the solution (previous waypoint)
        rng = np.random.default_rng(11)
        for _ in range(30):
            q_true = rng.uniform(-np.pi, np.pi, 6)
            target = fk(q_true)
            q_seed = q_true + rng.uniform(-0.3, 0.3, 6)
            result = solve_ik(target, q_seed)
            assert result.success
            pos_err, rot_err = _pose_errors(target, result.q)
            assert pos_err < POS_TOL
            assert rot_err < ROT_TOL

    def test_converges_from_home_seed(self):
        # Cold start from a fixed home posture to a few reachable targets
        home = np.array(
            [0.0, -math.pi / 2, math.pi / 2, -math.pi / 2, -math.pi / 2, 0.0]
        )
        rng = np.random.default_rng(23)
        n_success = 0
        for _ in range(10):
            q_true = home + rng.uniform(-0.8, 0.8, 6)
            target = fk(q_true)
            result = solve_ik(target, home)
            if result.success:
                pos_err, rot_err = _pose_errors(target, result.q)
                assert pos_err < POS_TOL
                assert rot_err < ROT_TOL
                n_success += 1
        # DLS is a local method; from a cold seed most but not necessarily
        # all targets must converge
        assert n_success >= 8

    def test_result_reports_true_errors(self):
        q_true = np.array([0.4, -1.1, 1.3, -1.6, -1.2, 0.7])
        target = fk(q_true)
        result = solve_ik(target, q_true + 0.1)
        pos_err, rot_err = _pose_errors(target, result.q)
        assert math.isclose(result.pos_error, pos_err, rel_tol=1e-6, abs_tol=1e-12)
        assert math.isclose(result.rot_error, rot_err, rel_tol=1e-6, abs_tol=1e-12)


class TestIKFailure:
    def test_unreachable_target_fails_gracefully(self):
        # UR10e reach is ~1.3 m; 3 m away is impossible
        target = np.eye(4)
        target[:3, 3] = [3.0, 0.0, 0.5]
        result = solve_ik(target, np.zeros(6))
        assert result.success is False
        assert np.all(np.isfinite(result.q))
        assert np.isfinite(result.pos_error)
        assert result.iterations > 0

    def test_starts_at_singularity_without_nan(self):
        # q = 0 is a wrist singularity; DLS must not blow up there
        q_true = np.array([0.2, -0.5, 0.6, -0.3, 0.4, 0.1])
        target = fk(q_true)
        result = solve_ik(target, np.zeros(6))
        assert np.all(np.isfinite(result.q))
        if result.success:
            pos_err, rot_err = _pose_errors(target, result.q)
            assert pos_err < POS_TOL
            assert rot_err < ROT_TOL
