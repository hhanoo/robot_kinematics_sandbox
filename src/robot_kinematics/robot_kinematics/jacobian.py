"""
Geometric Jacobian
==================
Geometric Jacobian (6xN) for a revolute serial chain defined by standard DH.

Rows 0-2: linear velocity, rows 3-5: angular velocity (base frame).
Column i: [z_i x (p_e - p_i); z_i], where z_i / p_i are the rotation axis
and origin of joint i+1 (frame i), and p_e is the end-effector position.
"""

import numpy as np

from robot_kinematics.fk import fk_frames


# =========================================================
# Geometric Jacobian
# =========================================================
def jacobian(q, dh=None):
    """Geometric Jacobian (6xN) at joint configuration q, in the base frame."""
    frames = fk_frames(q, dh)
    p_e = frames[-1][:3, 3]
    n = len(frames) - 1
    J = np.zeros((6, n))
    for i in range(n):
        z_i = frames[i][:3, 2]
        p_i = frames[i][:3, 3]
        J[:3, i] = np.cross(z_i, p_e - p_i)
        J[3:, i] = z_i
    return J
