"""
Cartesian Trajectory
====================
Straight-line and circular-arc pose paths, converted to joint paths by
solving IK per waypoint with the previous solution as seed.
"""

import math
from dataclasses import dataclass

import numpy as np

from robot_kinematics.ik import rotation_vector, solve_ik


# =========================================================
# Result container
# =========================================================
@dataclass
class CartesianJointPath:
    success: bool
    q: np.ndarray  # (N, dof) joint path (valid rows up to failure)
    failed_index: int  # -1 if fully solved


# =========================================================
# SO(3) helpers
# =========================================================
def _exp_so3(w):
    """Rodrigues formula: rotation matrix of rotation vector w."""
    angle = np.linalg.norm(w)
    K = np.array([[0.0, -w[2], w[1]], [w[2], 0.0, -w[0]], [-w[1], w[0], 0.0]])
    if angle < 1e-12:
        return np.eye(3) + K
    K /= angle
    return np.eye(3) + math.sin(angle) * K + (1.0 - math.cos(angle)) * (K @ K)


def slerp(R0, R1, s):
    """
    Interpolate between two rotation matrices along the geodesic

    Args:
        R0 (np.ndarray): Start rotation (3x3)
        R1 (np.ndarray): End rotation (3x3)
        s (float): Interpolation parameter in [0, 1]

    Returns:
        np.ndarray: Interpolated rotation (3x3)
    """
    # Relative rotation as a vector, scaled and re-applied
    w = rotation_vector(R0.T @ R1)
    return R0 @ _exp_so3(s * w)


# =========================================================
# Pose paths
# =========================================================
def linear_pose_path(T0, T1, n):
    """
    Straight-line pose path (position lerp + orientation slerp)

    Args:
        T0 (np.ndarray): Start pose (4x4)
        T1 (np.ndarray): End pose (4x4)
        n (int): Number of waypoints including both endpoints

    Returns:
        list[np.ndarray]: n poses (4x4)
    """
    p0, p1 = T0[:3, 3], T1[:3, 3]
    poses = []
    for k in range(n):
        s = k / (n - 1)
        T = np.eye(4)
        T[:3, :3] = slerp(T0[:3, :3], T1[:3, :3], s)
        T[:3, 3] = (1.0 - s) * p0 + s * p1
        poses.append(T)
    return poses


def circle_pose_path(T0, center, axis, angle, n):
    """
    Circular-arc position path with constant orientation

    Args:
        T0 (np.ndarray): Start pose (4x4); its position defines the radius
        center (array-like): Circle center [m] (base frame)
        axis (array-like): Rotation axis (unit not required)
        angle (float): Total sweep angle [rad]
        n (int): Number of waypoints including both endpoints

    Returns:
        list[np.ndarray]: n poses (4x4)
    """
    center = np.asarray(center, dtype=float)
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    r0 = T0[:3, 3] - center

    poses = []
    for k in range(n):
        theta = angle * k / (n - 1)
        T = np.eye(4)
        T[:3, :3] = T0[:3, :3]
        T[:3, 3] = center + _exp_so3(theta * axis) @ r0
        poses.append(T)
    return poses


# =========================================================
# Cartesian -> joint conversion
# =========================================================
def cartesian_to_joint(poses, q_seed, **ik_kwargs):
    """
    Solve IK along a pose path, seeding each waypoint with the previous
    solution to keep the joint path continuous

    Args:
        poses (list[np.ndarray]): Waypoint poses (4x4)
        q_seed (array-like): Seed joint angles for the first waypoint [rad]
        **ik_kwargs: Extra keyword arguments forwarded to solve_ik

    Returns:
        CartesianJointPath: (success, q, failed_index)
    """
    q = np.asarray(q_seed, dtype=float).copy()
    out = np.zeros((len(poses), len(q)))

    for i, T in enumerate(poses):
        result = solve_ik(T, q, **ik_kwargs)
        out[i] = result.q
        if not result.success:
            return CartesianJointPath(False, out, i)
        q = result.q
    return CartesianJointPath(True, out, -1)
