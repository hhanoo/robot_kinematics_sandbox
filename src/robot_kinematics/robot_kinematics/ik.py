"""
Damped Least Squares IK
=======================
Iterative differential IK using the DLS update (same formulation as the
Isaac Lab DifferentialIKController with ik_method="dls"):

    dq = J^T (J J^T + lambda^2 I)^-1 e

where e is the 6D pose error (position + rotation vector, base frame).
"""

import math
from dataclasses import dataclass

import numpy as np

from robot_kinematics.fk import fk
from robot_kinematics.jacobian import jacobian


# =========================================================
# Result container
# =========================================================
@dataclass
class IKResult:
    success: bool
    q: np.ndarray
    pos_error: float
    rot_error: float
    iterations: int


# =========================================================
# SO(3) helper
# =========================================================
def rotation_vector(R):
    """
    Rotation vector (axis * angle) of a rotation matrix

    Args:
        R (np.ndarray): 3x3 rotation matrix (SO(3))

    Returns:
        np.ndarray: rotation vector t * a (3,), t in [0, pi]
    """
    # Inverse of Rodrigues: R = I + sin(t)K + (1-cos(t))K^2 (K = skew of axis a).
    # R encodes the answer in three places:
    #   antisymmetric part (R - R^T)/2 = sin(t)K  -> axis (normal case)
    #   trace tr(R) = 1 + 2cos(t)                 -> angle size
    #   symmetric part (R + I)/2 = aa^T           -> axis (t ~ pi fallback)

    # 1. Read w = sin(t) * a from the antisymmetric part
    w = 0.5 * np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    s = np.linalg.norm(w)  # sin(t), >= 0 since t in [0, pi]

    # 2. Read cos(t) from the trace (clip guards float rounding)
    c = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)

    # 3. atan2(sin, cos) is stable over the whole range (unlike arccos alone)
    angle = math.atan2(s, c)

    if s < 1e-10:
        if c > 0.0:
            # t ~ 0: axis is undefined but w = sin(t)*a ~ t*a is the answer
            return w
        # t ~ pi: recover axis from (R + I)/2 = aa^T (largest-diagonal column)
        A = (R + np.eye(3)) / 2.0
        i = int(np.argmax(np.diag(A)))
        axis = A[:, i] / math.sqrt(A[i, i])
        axis = axis / np.linalg.norm(axis)
        return angle * axis

    # 4. General case: axis = w / sin(t)
    return angle * w / s


# =========================================================
# DLS solver
# =========================================================
def _pose_error(target, T):
    """6D error twist: [dp; rotation_vector(R_t R^T)]."""
    e = np.empty(6)
    e[:3] = target[:3, 3] - T[:3, 3]
    e[3:] = rotation_vector(target[:3, :3] @ T[:3, :3].T)
    return e


def solve_ik(
    target,
    q0,
    dh=None,
    damping=0.05,
    max_iters=200,
    tol_pos=1e-5,
    tol_rot=1e-5,
    max_step=0.5,
):
    """
    Solve IK for a 4x4 target pose starting from seed q0

    Never raises on non-convergence: check IKResult.success.

    Args:
        target (np.ndarray): Goal pose, 4x4 homogeneous transform (base frame)
        q0 (array-like): Seed joint angles [rad]; DLS is a local method, so it
                         converges to the solution branch nearest to this seed
        dh (np.ndarray): DH table (a, d, alpha) per joint; None = UR10E_DH
        damping (float): DLS damping lambda; larger = stable near
            singularities but slower, smaller = faster but can overshoot
        max_iters (int): Iteration limit before giving up
        tol_pos (float): Position convergence tolerance [m]
        tol_rot (float): Orientation convergence tolerance [rad]
        max_step (float): Per-iteration joint-step norm limit [rad], keeps
                          the Jacobian linearization valid when the target is far

    Returns:
        IKResult: (success, q, pos_error, rot_error, iterations)
    """
    q = np.asarray(q0, dtype=float).copy()
    lam2 = damping * damping

    for it in range(max_iters):
        # 1. Current 6D error to target (position + rotation vector)
        e = _pose_error(target, fk(q, dh))
        pos_err = float(np.linalg.norm(e[:3]))
        rot_err = float(np.linalg.norm(e[3:]))

        # 2. Converged? Done
        if pos_err < tol_pos and rot_err < tol_rot:
            return IKResult(True, q, pos_err, rot_err, it)

        # 3. DLS step (lambda^2 keeps it bounded at singularities)
        J = jacobian(q, dh)
        dq = J.T @ np.linalg.solve(J @ J.T + lam2 * np.eye(6), e)

        # 4. Clamp step size (Jacobian is only a local approximation)
        step = np.linalg.norm(dq)
        if step > max_step:
            dq *= max_step / step

        # 5. Apply and iterate
        q += dq

    # 6. Out of iterations: report the final state as-is
    e = _pose_error(target, fk(q, dh))
    pos_err = float(np.linalg.norm(e[:3]))
    rot_err = float(np.linalg.norm(e[3:]))
    success = pos_err < tol_pos and rot_err < tol_rot
    return IKResult(success, q, pos_err, rot_err, max_iters)
