"""
Geometric Jacobian
==================
Geometric Jacobian (6xN) for the revolute serial chain read from the URDF.

Rows 0-2: linear velocity, rows 3-5: angular velocity (base frame).
Column i: [z_i x (p_e - p_i); z_i], where z_i is the world axis of joint i,
p_i is a point on that axis and p_e is the tip position.
"""

import numpy as np

from robot_kinematics.fk import fk_joints


# =========================================================
# Geometric Jacobian
# =========================================================
def jacobian(q, chain=None):
    """Geometric Jacobian (6xN) at joint configuration q, in the base frame."""
    frames, axes, origins = fk_joints(q, chain)
    p_e = frames[-1][:3, 3]
    n = len(axes)
    J = np.zeros((6, n))
    for i in range(n):
        J[:3, i] = np.cross(axes[i], p_e - origins[i])
        J[3:, i] = axes[i]
    return J
