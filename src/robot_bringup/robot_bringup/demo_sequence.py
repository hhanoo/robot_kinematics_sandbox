"""
Demo Sequence Builder
=====================
Builds the full demo joint sequence (pure numpy, ROS-independent):

    zero -> home (joint quintic)
    home -> IK target pose (DLS IK + joint quintic)
    straight Cartesian line
    full circle (constant orientation)
    back to home (joint quintic)
"""

import math
from dataclasses import dataclass

import numpy as np

from robot_kinematics.fk import fk
from robot_kinematics.ik import solve_ik
from robot_trajectory.cartesian_traj import (
    cartesian_to_joint,
    circle_pose_path,
    linear_pose_path,
)
from robot_trajectory.joint_traj import quintic_joint_trajectory

# fmt: off
HOME = np.array([0.0, -math.pi / 2, math.pi / 2, -math.pi / 2, -math.pi / 2, 0.0])
# fmt: on

# Joint-space motion limits (smooth demo pace)
V_MAX = 1.2  # [rad/s]
A_MAX = 2.5  # [rad/s^2]

# Cartesian segment parameters
LIN_SPEED = 0.1  # [m/s]
LINE_OFFSET = np.array([0.10, 0.10, -0.10])  # [m] straight-line displacement
CIRCLE_RADIUS = 0.08  # [m]
CIRCLE_AXIS = np.array([0.0, 0.0, 1.0])

# IK target: pose of this reference posture, solved from the HOME seed
Q_TARGET_REF = HOME + np.array([0.4, 0.3, -0.4, 0.2, 0.3, 0.5])


# =========================================================
# Result container
# =========================================================
@dataclass
class Segment:
    name: str
    start: int  # first row index (inclusive)
    end: int  # last row index (exclusive)


@dataclass
class DemoSequence:
    dt: float
    q: np.ndarray  # (N, 6) joint rows sampled every dt
    segments: list  # list[Segment]


# =========================================================
# Helpers
# =========================================================
def _cartesian_rows(poses, q_seed, name):
    """Solve IK along poses; raise with segment context on failure."""
    result = cartesian_to_joint(poses, q_seed)
    if not result.success:
        raise RuntimeError(
            f"IK failed in segment '{name}' at waypoint {result.failed_index}"
        )
    return result.q


def _n_waypoints(path_length, dt):
    """Waypoint count so consecutive samples move LIN_SPEED * dt apart."""
    return max(2, int(round(path_length / (LIN_SPEED * dt))) + 1)


# =========================================================
# Sequence builder
# =========================================================
def build_demo_sequence(dt=0.02):
    """
    Build the demo joint sequence sampled every dt

    Args:
        dt (float): Sample period [s]

    Returns:
        DemoSequence: (dt, q, segments)

    Raises:
        RuntimeError: If IK fails while building a Cartesian segment
    """
    parts = []  # list of (name, rows)

    # 1. zero -> home
    parts.append(
        ("home", quintic_joint_trajectory(np.zeros(6), HOME, V_MAX, A_MAX, dt).q)
    )

    # 2. home -> IK target pose
    target = fk(Q_TARGET_REF)
    ik = solve_ik(target, HOME)
    if not ik.success:
        raise RuntimeError("IK failed for the demo target pose")
    parts.append(
        ("ik_target", quintic_joint_trajectory(HOME, ik.q, V_MAX, A_MAX, dt).q)
    )

    # 3. straight Cartesian line
    q_now = parts[-1][1][-1]
    T_now = fk(q_now)
    T_line = T_now.copy()
    T_line[:3, 3] = T_now[:3, 3] + LINE_OFFSET
    n = _n_waypoints(float(np.linalg.norm(LINE_OFFSET)), dt)
    parts.append(
        ("line", _cartesian_rows(linear_pose_path(T_now, T_line, n), q_now, "line"))
    )

    # 4. full circle around a center beside the current position
    q_now = parts[-1][1][-1]
    T_now = fk(q_now)
    center = T_now[:3, 3] + np.array([0.0, CIRCLE_RADIUS, 0.0])
    n = _n_waypoints(2.0 * math.pi * CIRCLE_RADIUS, dt)
    poses = circle_pose_path(T_now, center, CIRCLE_AXIS, 2.0 * math.pi, n)
    parts.append(("circle", _cartesian_rows(poses, q_now, "circle")))

    # 5. back to home
    q_now = parts[-1][1][-1]
    parts.append(("return", quintic_joint_trajectory(q_now, HOME, V_MAX, A_MAX, dt).q))

    # Concatenate, dropping the duplicated junction row of each new segment
    rows = [parts[0][1]]
    segments = [Segment(parts[0][0], 0, len(parts[0][1]))]
    for name, q in parts[1:]:
        start = segments[-1].end
        rows.append(q[1:])
        segments.append(Segment(name, start, start + len(q) - 1))
    return DemoSequence(dt, np.vstack(rows), segments)
