"""
Forward Kinematics
==================
Forward kinematics from standard DH parameters.

Features:
- fk_frames: cumulative frames along the chain (for Jacobian / visualization)
- fk: base -> end-effector homogeneous transform
"""

import numpy as np

from robot_kinematics.dh import UR10E_DH, dh_transform


# =========================================================
# Forward kinematics
# =========================================================
def fk_frames(q, dh=None):
    """Cumulative frames along the chain.

    Returns an (n+1, 4, 4) array: base frame (identity) followed by the
    frame after each joint. The last entry is the end-effector (tool0).
    """
    if dh is None:
        dh = UR10E_DH
    q = np.asarray(q, dtype=float)
    frames = np.empty((len(q) + 1, 4, 4))
    frames[0] = np.eye(4)
    for i, (theta, (a, d, alpha)) in enumerate(zip(q, dh)):
        frames[i + 1] = frames[i] @ dh_transform(theta, d, a, alpha)
    return frames


def fk(q, dh=None):
    """Base -> end-effector homogeneous transform (4x4)."""
    return fk_frames(q, dh)[-1]
