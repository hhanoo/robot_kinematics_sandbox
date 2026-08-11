"""
Jog Step Unit Tests
===================
One differential-IK step must move tool0 along the commanded twist
(verified with FK), stay finite at singularities, and respect the clamp.
"""

import numpy as np

from robot_kinematics.fk import fk
from robot_kinematics.jog import jog_step

Q0 = np.array([0.3, -1.4, 1.6, -1.8, -1.5, 0.2])
DT = 0.02


class TestJogStep:
    def test_zero_twist_keeps_q(self):
        q1 = jog_step(Q0, np.zeros(6), DT)
        np.testing.assert_allclose(q1, Q0, atol=1e-12)

    def test_linear_axes_move_tool_in_commanded_direction(self):
        for axis in range(3):
            twist = np.zeros(6)
            twist[axis] = 0.1
            q1 = jog_step(Q0, twist, DT)
            dp = fk(q1)[:3, 3] - fk(Q0)[:3, 3]
            # Dominant motion along the commanded axis, roughly v*dt
            assert dp[axis] > 0.5 * 0.1 * DT
            assert dp[axis] == max(abs(dp[0]), abs(dp[1]), abs(dp[2]))

    def test_finite_at_singularity(self):
        twist = np.array([0.1, 0.0, 0.0, 0.0, 0.0, 0.0])
        q1 = jog_step(np.zeros(6), twist, DT)
        assert np.all(np.isfinite(q1))

    def test_step_clamp(self):
        # Huge twist must be limited by max_dq
        twist = np.array([100.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        q1 = jog_step(Q0, twist, DT, max_dq=0.05)
        assert np.linalg.norm(q1 - Q0) <= 0.05 + 1e-12
