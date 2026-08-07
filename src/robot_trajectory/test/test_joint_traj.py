"""
Joint Trajectory Unit Tests
===========================
Boundary conditions (rest-to-rest), velocity / acceleration limit
compliance, and consistency between q, qd and qdd samples.
"""

import numpy as np

from robot_trajectory.joint_traj import min_duration, quintic_joint_trajectory

Q0 = np.array([0.0, -1.57, 1.57, -1.57, -1.57, 0.0])
QF = np.array([0.5, -1.0, 1.0, -2.0, -1.2, 0.8])


class TestBoundaryConditions:
    def test_starts_and_ends_at_given_configurations(self):
        traj = quintic_joint_trajectory(Q0, QF)
        np.testing.assert_allclose(traj.q[0], Q0, atol=1e-12)
        np.testing.assert_allclose(traj.q[-1], QF, atol=1e-12)

    def test_rest_to_rest(self):
        traj = quintic_joint_trajectory(Q0, QF)
        np.testing.assert_allclose(traj.qd[0], np.zeros(6), atol=1e-9)
        np.testing.assert_allclose(traj.qd[-1], np.zeros(6), atol=1e-9)
        np.testing.assert_allclose(traj.qdd[0], np.zeros(6), atol=1e-9)
        np.testing.assert_allclose(traj.qdd[-1], np.zeros(6), atol=1e-9)

    def test_time_stamps(self):
        traj = quintic_joint_trajectory(Q0, QF, dt=0.02)
        assert traj.t[0] == 0.0
        dts = np.diff(traj.t)
        np.testing.assert_allclose(dts, dts[0], atol=1e-12)
        assert traj.t[-1] >= min_duration(Q0, QF, 2.0, 4.0) - 1e-9

    def test_zero_displacement(self):
        traj = quintic_joint_trajectory(Q0, Q0)
        np.testing.assert_allclose(traj.q, np.tile(Q0, (len(traj.t), 1)), atol=1e-12)


class TestLimits:
    def test_velocity_limit(self):
        v_max, a_max = 1.5, 3.0
        traj = quintic_joint_trajectory(Q0, QF, v_max=v_max, a_max=a_max)
        assert np.max(np.abs(traj.qd)) <= v_max * (1.0 + 1e-6)

    def test_acceleration_limit(self):
        v_max, a_max = 1.5, 3.0
        traj = quintic_joint_trajectory(Q0, QF, v_max=v_max, a_max=a_max)
        assert np.max(np.abs(traj.qdd)) <= a_max * (1.0 + 1e-6)

    def test_explicit_duration_overrides_limits(self):
        traj = quintic_joint_trajectory(Q0, QF, duration=3.0, dt=0.02)
        np.testing.assert_allclose(traj.t[-1], 3.0, atol=1e-9)


class TestConsistency:
    def test_velocity_matches_position_derivative(self):
        traj = quintic_joint_trajectory(Q0, QF, dt=0.001)
        qd_num = np.gradient(traj.q, traj.t, axis=0)
        np.testing.assert_allclose(traj.qd[5:-5], qd_num[5:-5], atol=1e-3)

    def test_acceleration_matches_velocity_derivative(self):
        traj = quintic_joint_trajectory(Q0, QF, dt=0.001)
        qdd_num = np.gradient(traj.qd, traj.t, axis=0)
        np.testing.assert_allclose(traj.qdd[5:-5], qdd_num[5:-5], atol=1e-2)

    def test_position_stays_within_bounds(self):
        # The 10-15-6 quintic profile is monotonic: no overshoot
        traj = quintic_joint_trajectory(Q0, QF)
        lo, hi = np.minimum(Q0, QF), np.maximum(Q0, QF)
        assert np.all(traj.q >= lo - 1e-9)
        assert np.all(traj.q <= hi + 1e-9)
