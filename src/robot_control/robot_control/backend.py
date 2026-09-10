"""
Motion Backend
==============
Owns "where q comes from and where commands go". Both backends expose the
same two members, so motion_server never learns which one it drives.

- SimBackend: integrates q internally and publishes /joint_states, so
  robot_state_publisher + RViz can follow without a physics engine.
- GazeboBackend: reads q from the joint_state_broadcaster and writes to a
  position controller, leaving the joint state to the simulator.
"""

import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


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
