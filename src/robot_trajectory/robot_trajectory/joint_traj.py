"""
Joint-Space Trajectory
======================
Quintic (5th-order) polynomial interpolation between two joint
configurations, time-parameterized by velocity / acceleration limits.

The rest-to-rest quintic profile s(tau) = 10 tau^3 - 15 tau^4 + 6 tau^5
has peak velocity 15/8 * dq / T and peak acceleration 10/sqrt(3) * dq / T^2,
which gives the minimum duration meeting given limits.
"""

import math
from dataclasses import dataclass

import numpy as np


# =========================================================
# Result container
# =========================================================
@dataclass
class JointTrajectory:
    t: np.ndarray  # (N,) time stamps [s]
    q: np.ndarray  # (N, dof) positions [rad]
    qd: np.ndarray  # (N, dof) velocities [rad/s]
    qdd: np.ndarray  # (N, dof) accelerations [rad/s^2]


# =========================================================
# Quintic interpolation
# =========================================================
def min_duration(q0, qf, v_max, a_max):
    """
    Minimum rest-to-rest quintic duration meeting velocity/acceleration limits

    Args:
        q0 (array-like): Start joint angles [rad]
        qf (array-like): End joint angles [rad]
        v_max (float or array-like): Per-joint velocity limit [rad/s]
        a_max (float or array-like): Per-joint acceleration limit [rad/s^2]

    Returns:
        float: Minimum duration [s] (0.0 for zero displacement)
    """
    dq = np.abs(np.asarray(qf, dtype=float) - np.asarray(q0, dtype=float))
    v_max = np.broadcast_to(np.asarray(v_max, dtype=float), dq.shape)
    a_max = np.broadcast_to(np.asarray(a_max, dtype=float), dq.shape)

    # Slowest joint dictates the common duration
    t_vel = 15.0 * dq / (8.0 * v_max)
    t_acc = np.sqrt(10.0 * dq / (math.sqrt(3.0) * a_max))
    return float(max(t_vel.max(), t_acc.max()))


def quintic_joint_trajectory(q0, qf, v_max=2.0, a_max=4.0, dt=0.02, duration=None):
    """
    Rest-to-rest quintic trajectory between two joint configurations

    Args:
        q0 (array-like): Start joint angles [rad]
        qf (array-like): End joint angles [rad]
        v_max (float or array-like): Per-joint velocity limit [rad/s]
        a_max (float or array-like): Per-joint acceleration limit [rad/s^2]
        dt (float): Sample period [s]
        duration (float): Explicit duration [s]; None = shortest within limits

    Returns:
        JointTrajectory: (t, q, qd, qdd) uniformly sampled
    """
    q0 = np.asarray(q0, dtype=float)
    qf = np.asarray(qf, dtype=float)
    dq = qf - q0

    # 1. Duration from limits unless given explicitly (>= dt for sampling)
    T = duration if duration is not None else min_duration(q0, qf, v_max, a_max)
    T = max(T, dt)

    # 2. Uniform time grid including both endpoints
    n = int(math.ceil(T / dt)) + 1
    t = np.linspace(0.0, T, n)
    tau = t / T

    # 3. Quintic profile and its time derivatives
    s = 10.0 * tau**3 - 15.0 * tau**4 + 6.0 * tau**5
    sd = (30.0 * tau**2 - 60.0 * tau**3 + 30.0 * tau**4) / T
    sdd = (60.0 * tau - 180.0 * tau**2 + 120.0 * tau**3) / T**2

    # 4. Scale the scalar profile onto every joint
    q = q0[None, :] + np.outer(s, dq)
    qd = np.outer(sd, dq)
    qdd = np.outer(sdd, dq)
    return JointTrajectory(t, q, qd, qdd)
