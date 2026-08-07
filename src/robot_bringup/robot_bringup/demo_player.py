#!/usr/bin/env python3
"""
Demo Player Node
================
Streams the demo joint sequence (FK/IK/trajectory showcase) to
/joint_states so robot_state_publisher + RViz can visualize it.

Parameters:
- rate (double): Publish rate [Hz], also the sequence sample rate
- loop (bool): Restart the sequence when it ends
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from robot_bringup.demo_sequence import build_demo_sequence

JOINT_NAMES = [
    "link_1_joint",
    "link_2_joint",
    "link_3_joint",
    "link_4_joint",
    "link_5_joint",
    "link_6_joint",
]


class DemoPlayer(Node):
    """Publishes the pre-built demo sequence as JointState at a fixed rate."""

    def __init__(self):
        super().__init__("demo_player")

        # Parameters
        self.declare_parameter("rate", 50.0)
        self.declare_parameter("loop", True)
        rate = self.get_parameter("rate").value
        self.loop = self.get_parameter("loop").value

        # Build the whole sequence up front (fail fast on IK errors)
        try:
            self.seq = build_demo_sequence(dt=1.0 / rate)
        except RuntimeError as e:
            self.get_logger().error(f"Demo sequence build failed: {e}")
            raise SystemExit(1)
        self.get_logger().info(
            f"Demo sequence ready: {len(self.seq.q)} samples, "
            f"{len(self.seq.q) / rate:.1f} s, segments: "
            + ", ".join(s.name for s in self.seq.segments)
        )

        # Publisher and playback timer
        self.pub = self.create_publisher(JointState, "joint_states", 10)
        self.index = 0
        self.timer = self.create_timer(1.0 / rate, self.on_timer)

    def on_timer(self):
        # Announce the segment when entering it
        for seg in self.seq.segments:
            if seg.start == self.index:
                self.get_logger().info(f"Segment: {seg.name}")

        # Publish the current row
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = [float(v) for v in self.seq.q[self.index]]
        self.pub.publish(msg)

        # Advance; hold the last sample or loop
        if self.index < len(self.seq.q) - 1:
            self.index += 1
        elif self.loop:
            self.index = 0


def main(args=None):
    rclpy.init(args=args)
    node = DemoPlayer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
