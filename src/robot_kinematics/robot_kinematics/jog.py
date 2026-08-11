"""
Cartesian Jog
=============
One differential-IK step: integrate a base-frame twist for dt seconds
using the same DLS form as the iterative IK solver.

    dq = J^T (J J^T + lambda^2 I)^-1 (twist * dt)
"""

import numpy as np

from robot_kinematics.jacobian import jacobian


# =========================================================
# Jog step
# =========================================================
def jog_step(q, twist, dt, dh=None, damping=0.05, max_dq=0.1):
    """
    One jog integration step toward a commanded twist

    Args:
        q (array-like): Current joint angles [rad]
        twist (array-like): Base-frame twist [vx, vy, vz, wx, wy, wz]
        dt (float): Integration period [s]
        dh (np.ndarray): DH table; None = UR10E_DH
        damping (float): DLS damping lambda
        max_dq (float): Joint-step norm limit [rad]

    Returns:
        np.ndarray: New joint angles (input q is not modified)
    """
    q = np.asarray(q, dtype=float)
    e = np.asarray(twist, dtype=float) * dt

    # 1. DLS step (bounded at singularities by lambda^2)
    J = jacobian(q, dh)
    dq = J.T @ np.linalg.solve(J @ J.T + damping * damping * np.eye(6), e)

    # 2. Clamp the step norm (keeps the linearization valid)
    step = np.linalg.norm(dq)
    if step > max_dq:
        dq *= max_dq / step
    return q + dq
