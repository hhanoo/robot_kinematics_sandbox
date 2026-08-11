#!/usr/bin/env python3
"""
Keyboard Teleop Node
====================
Reads single keys from a raw terminal and publishes base-frame
TwistStamped jog commands. Stopping is delegated to the motion server's
deadman timeout (key release cannot be detected in a terminal; OS key
repeat keeps commands flowing while a key is held).

Keys: w/s +-x  a/d +-y  r/f +-z  u/o +-rx  i/k +-ry  j/l +-rz
      +/- speed scale   q quit
"""

import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node

BASE_LIN = 0.1  # [m/s]
BASE_ANG = 0.5  # [rad/s]
# fmt: off
KEYMAP = {
    "w": (0, +1), "s": (0, -1),   # x
    "a": (1, +1), "d": (1, -1),   # y
    "r": (2, +1), "f": (2, -1),   # z
    "u": (3, +1), "o": (3, -1),   # rx
    "i": (4, +1), "k": (4, -1),   # ry
    "j": (5, +1), "l": (5, -1),   # rz
}
# fmt: on
HELP = (
    "\n[teleop_keyboard]\n"
    "  w/s: +-x   a/d: +-y   r/f: +-z\n"
    "  u/o: +-rx  i/k: +-ry  j/l: +-rz\n"
    "  +/-: speed scale   q: quit\n"
)


def main(args=None):
    rclpy.init(args=args)
    node = Node("teleop_keyboard")
    pub = node.create_publisher(TwistStamped, "jog_twist", 10)
    scale = 1.0
    print(HELP)
    print(f"scale: {scale:.2f}")

    settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            # Wait up to 0.1 s for a key; no key = publish nothing (deadman)
            ready, _, _ = select.select([sys.stdin], [], [], 0.1)
            if not ready:
                continue
            key = sys.stdin.read(1)
            if key == "q":
                break
            if key in ("+", "="):
                scale *= 1.25
                print(f"scale: {scale:.2f}")
                continue
            if key == "-":
                scale /= 1.25
                print(f"scale: {scale:.2f}")
                continue
            if key not in KEYMAP:
                continue
            axis, sign = KEYMAP[key]
            msg = TwistStamped()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            value = sign * scale * (BASE_LIN if axis < 3 else BASE_ANG)
            if axis == 0:
                msg.twist.linear.x = value
            elif axis == 1:
                msg.twist.linear.y = value
            elif axis == 2:
                msg.twist.linear.z = value
            elif axis == 3:
                msg.twist.angular.x = value
            elif axis == 4:
                msg.twist.angular.y = value
            else:
                msg.twist.angular.z = value
            pub.publish(msg)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
