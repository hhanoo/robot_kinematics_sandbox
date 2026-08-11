"""
Motion Backend
==============
Owns "where q comes from and where commands go". SimBackend is the
stage-2 stand-in for a robot driver: it integrates q internally and
publishes /joint_states so robot_state_publisher + RViz can follow.

Swap points for later stages (same interface):
- Gazebo / real robot: read q from the driver, write to a controller.
"""

import numpy as np
from sensor_msgs.msg import JointState


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
