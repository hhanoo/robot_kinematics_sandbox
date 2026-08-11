"""
Pose Conversions
================
geometry_msgs Pose <-> 4x4 homogeneous matrix, via unit quaternions.
Pure math helpers (quat_to_matrix / matrix_to_quat) are ROS-independent.
"""

import math

import numpy as np
from geometry_msgs.msg import Pose


# =========================================================
# Quaternion <-> rotation matrix
# =========================================================
def quat_to_matrix(x, y, z, w):
    """
    Rotation matrix of a quaternion (normalized internally)

    Args:
        x, y, z, w (float): Quaternion components

    Returns:
        np.ndarray: 3x3 rotation matrix
    """
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    # fmt: off
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
            [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
        ]
    )
    # fmt: on


def matrix_to_quat(R):
    """
    Quaternion (x, y, z, w) of a rotation matrix

    Args:
        R (np.ndarray): 3x3 rotation matrix

    Returns:
        tuple: (x, y, z, w)
    """
    # Shepperd's method: pick the largest diagonal combination for stability
    t = np.trace(R)
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        return (
            (R[2, 1] - R[1, 2]) / s,
            (R[0, 2] - R[2, 0]) / s,
            (R[1, 0] - R[0, 1]) / s,
            0.25 * s,
        )
    i = int(np.argmax(np.diag(R)))
    if i == 0:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        return (
            0.25 * s,
            (R[0, 1] + R[1, 0]) / s,
            (R[0, 2] + R[2, 0]) / s,
            (R[2, 1] - R[1, 2]) / s,
        )
    if i == 1:
        s = math.sqrt(1.0 - R[0, 0] + R[1, 1] - R[2, 2]) * 2.0
        return (
            (R[0, 1] + R[1, 0]) / s,
            0.25 * s,
            (R[1, 2] + R[2, 1]) / s,
            (R[0, 2] - R[2, 0]) / s,
        )
    s = math.sqrt(1.0 - R[0, 0] - R[1, 1] + R[2, 2]) * 2.0
    return (
        (R[0, 2] + R[2, 0]) / s,
        (R[1, 2] + R[2, 1]) / s,
        0.25 * s,
        (R[1, 0] - R[0, 1]) / s,
    )


# =========================================================
# Pose msg <-> homogeneous matrix
# =========================================================
def pose_to_matrix(pose):
    """geometry_msgs/Pose -> 4x4 homogeneous transform."""
    T = np.eye(4)
    o = pose.orientation
    T[:3, :3] = quat_to_matrix(o.x, o.y, o.z, o.w)
    T[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return T


def matrix_to_pose(T):
    """4x4 homogeneous transform -> geometry_msgs/Pose."""
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = (float(v) for v in T[:3, 3])
    x, y, z, w = matrix_to_quat(T[:3, :3])
    pose.orientation.x = float(x)
    pose.orientation.y = float(y)
    pose.orientation.z = float(z)
    pose.orientation.w = float(w)
    return pose
