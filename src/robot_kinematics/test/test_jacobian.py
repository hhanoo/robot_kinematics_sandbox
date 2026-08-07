"""
Jacobian Unit Tests
===================
Ground truth: central finite differences of FK. For each joint j,
column j of the Jacobian must match (d pose / d q_j).
"""

import numpy as np

from robot_kinematics.fk import fk
from robot_kinematics.jacobian import jacobian

EPS = 1e-6


def _numeric_jacobian(q):
    """Central-difference Jacobian: position rows from p(q), angular rows
    from the rotation vector of R(q+dq) R(q-dq)^T."""
    J = np.zeros((6, len(q)))
    for j in range(len(q)):
        qp, qm = q.copy(), q.copy()
        qp[j] += EPS
        qm[j] -= EPS
        Tp, Tm = fk(qp), fk(qm)
        J[:3, j] = (Tp[:3, 3] - Tm[:3, 3]) / (2 * EPS)
        R_err = Tp[:3, :3] @ Tm[:3, :3].T
        # small-angle rotation vector from skew-symmetric part
        w = (
            np.array(
                [
                    R_err[2, 1] - R_err[1, 2],
                    R_err[0, 2] - R_err[2, 0],
                    R_err[1, 0] - R_err[0, 1],
                ]
            )
            / 2.0
        )
        J[3:, j] = w / (2 * EPS)
    return J


class TestJacobian:
    def test_shape(self):
        assert jacobian(np.zeros(6)).shape == (6, 6)

    def test_matches_finite_differences_random_q(self):
        rng = np.random.default_rng(7)
        for _ in range(20):
            q = rng.uniform(-np.pi, np.pi, 6)
            J = jacobian(q)
            J_num = _numeric_jacobian(q)
            np.testing.assert_allclose(J, J_num, atol=1e-5)

    def test_singular_at_zero_pose(self):
        # q5 = 0 aligns joint axes 4 and 6 (wrist singularity), so the
        # Jacobian must lose rank at the zero pose
        J = jacobian(np.zeros(6))
        assert np.linalg.matrix_rank(J, tol=1e-9) < 6
