"""
Forward Kinematics
==================
Forward kinematics for a serial chain read from the robot URDF.

Features:
- fk_joints: link frames plus each joint's world axis and origin
- fk_frames: link frames along the chain (for Jacobian / collision)
- fk: base -> tip homogeneous transform
"""

import numpy as np

from robot_kinematics.chain import axis_rotation, load_default


# =========================================================
# Forward kinematics
# =========================================================
def fk_joints(q, chain=None):
    """Walk the chain once, returning everything the callers need.

    Args:
        q (array-like): Joint angles [rad], one per movable joint
        chain (Chain): Kinematic chain; None = the bundled robot

    Returns:
        tuple: (frames, axes, origins) where frames is (n+1, 4, 4) link
        frames starting at the base and ending at the tip, axes is (n, 3)
        joint axes in the base frame, and origins is (n, 3) points on those
        axes in the base frame.
    """
    if chain is None:
        chain = load_default()
    q = np.asarray(q, dtype=float)
    if len(q) != len(chain):
        raise ValueError(f"expected {len(chain)} joint angles, got {len(q)}")

    n = len(chain)
    frames = np.empty((n + 1, 4, 4))
    axes = np.empty((n, 3))
    origins = np.empty((n, 3))

    T = np.eye(4)
    frames[0] = T
    for i, seg in enumerate(chain.segments):
        # 1. Move to the joint frame, where the axis is defined
        T_joint = T @ seg.pre
        axes[i] = T_joint[:3, :3] @ seg.axis
        origins[i] = T_joint[:3, 3]

        # 2. Rotate, then step to the child link frame
        rot = np.eye(4)
        rot[:3, :3] = axis_rotation(seg.axis, q[i])
        T = T_joint @ rot @ seg.post
        frames[i + 1] = T

    return frames, axes, origins


def fk_frames(q, chain=None):
    """Link frames along the chain, (n+1, 4, 4) from base to tip."""
    return fk_joints(q, chain)[0]


def fk(q, chain=None):
    """Base -> tip homogeneous transform."""
    return fk_frames(q, chain)[-1]
