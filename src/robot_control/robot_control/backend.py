"""
Motion Backend
==============
Owns "where q comes from and where commands go". Both backends expose the
same two members, so motion_server never learns which one it drives.

- SimBackend: integrates q internally and publishes /joint_states, so
  robot_state_publisher + RViz can follow without a physics engine.
- GazeboBackend: reads q from the joint_state_broadcaster and writes to a
  position controller, leaving the joint state to the simulator.
- MujocoBackend: steps MuJoCo in process, so it also publishes the joint
  state nobody else would, and can open MuJoCo's own viewer.
"""

import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from robot_kinematics.chain import XACRO_PATH


# =========================================================
# Simulation backend (driver stand-in)
# =========================================================
class SimBackend:
    """Holds the joint vector and publishes it as /joint_states."""

    def __init__(self, node, joint_names, q0):
        self._names = list(joint_names)
        self._q = np.asarray(q0, dtype=float).copy()
        self._pub = node.create_publisher(JointState, "joint_states", 10)
        self._clock = node.get_clock()

    @property
    def q(self):
        """Current joint vector (copy)."""
        return self._q.copy()

    def write(self, q):
        """Store the new joint vector and publish it."""
        self._q = np.asarray(q, dtype=float).copy()
        msg = JointState()
        msg.header.stamp = self._clock.now().to_msg()
        msg.name = self._names
        msg.position = [float(v) for v in self._q]
        self._pub.publish(msg)


# =========================================================
# Gazebo backend (ros2_control position commands)
# =========================================================
class GazeboBackend:
    """Reads q from /joint_states and commands a position controller."""

    def __init__(self, node, joint_names, q0):
        self._names = list(joint_names)
        self._q = np.asarray(q0, dtype=float).copy()
        self._pub = node.create_publisher(
            Float64MultiArray, "joint_position_controller/commands", 10
        )
        node.create_subscription(JointState, "joint_states", self._on_state, 10)

    def _on_state(self, msg):
        # The broadcaster may reorder or omit joints
        index = {name: i for i, name in enumerate(msg.name)}
        if any(name not in index for name in self._names):
            return
        self._q = np.array([msg.position[index[n]] for n in self._names], dtype=float)

    @property
    def q(self):
        """Latest joint vector reported by the simulator (copy)."""
        return self._q.copy()

    def write(self, q):
        """Send the joint vector as a position command.

        Nothing is stored: the simulator owns the state, and reading back
        the command instead of the measurement would hide tracking error.
        """
        msg = Float64MultiArray()
        msg.data = [float(v) for v in q]
        self._pub.publish(msg)


# =========================================================
# MuJoCo backend (in-process physics)
# =========================================================
class MujocoBackend:
    """Owns a MuJoCo model, steps it every tick, and publishes what it reads.

    Unlike Gazebo there is no separate simulator process, so this backend
    advances the physics itself and takes over publishing /joint_states.
    """

    def __init__(self, node, joint_names, q0, viewer=False, config=None):
        import mujoco

        from robot_control.mujoco_model import build_model

        self._mj = mujoco
        self._names = list(joint_names)
        share = XACRO_PATH.parents[1]
        self._model = build_model(XACRO_PATH, share, self._names, config)
        self._data = mujoco.MjData(self._model)
        self._data.qpos[:] = np.asarray(q0, dtype=float)
        self._data.ctrl[:] = self._data.qpos
        mujoco.mj_forward(self._model, self._data)

        self._pub = node.create_publisher(JointState, "joint_states", 10)
        self._clock = node.get_clock()
        self._dt = node.dt
        self._viewer = self._open_viewer(node) if viewer else None

    def _open_viewer(self, node):
        """MuJoCo's own window, which needs a display and so may not open."""
        from mujoco import viewer

        try:
            handle = viewer.launch_passive(
                self._model, self._data, show_left_ui=False, show_right_ui=False
            )
        except Exception as exc:  # noqa: BLE001 - optional, never fatal
            node.get_logger().warn(f"MuJoCo viewer unavailable: {exc}")
            return None
        node.get_logger().info("MuJoCo viewer open")
        return handle

    @property
    def q(self):
        """Joint vector measured in the simulator (copy)."""
        return self._data.qpos.copy()

    def write(self, q):
        """Set the servo targets, advance one control period, and publish."""
        self._data.ctrl[:] = np.asarray(q, dtype=float)
        for _ in range(max(1, round(self._dt / self._model.opt.timestep))):
            self._mj.mj_step(self._model, self._data)

        msg = JointState()
        msg.header.stamp = self._clock.now().to_msg()
        msg.name = self._names
        msg.position = [float(v) for v in self._data.qpos]
        msg.velocity = [float(v) for v in self._data.qvel]
        self._pub.publish(msg)

        if self._viewer is not None and self._viewer.is_running():
            self._viewer.sync()
