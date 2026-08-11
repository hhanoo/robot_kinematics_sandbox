#!/usr/bin/env python3
"""
Goal Marker Node
================
RViz 6-DOF interactive marker for setting motion goals. Dragging only
moves the marker; execution happens from the right-click menu (MoveJ /
MoveL / Reset to tool), so accidental drags never move the robot.
"""

import rclpy
from geometry_msgs.msg import PoseStamped
from interactive_markers import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from rclpy.node import Node
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    Marker,
)

from robot_interfaces.srv import MoveJ, MoveL


class MarkerServer(Node):
    """Publishes the goal marker and forwards menu actions to services."""

    def __init__(self):
        super().__init__("marker_server")

        # Service clients
        self.cli_j = self.create_client(MoveJ, "/motion_server/move_j")
        self.cli_l = self.create_client(MoveL, "/motion_server/move_l")

        # Marker server + menu
        self.server = InteractiveMarkerServer(self, "goal_marker")
        self.menu = MenuHandler()
        self.menu.insert("MoveJ here", callback=self.on_move_j)
        self.menu.insert("MoveL here", callback=self.on_move_l)
        self.menu.insert("Reset to tool", callback=self.on_reset)

        # Marker follows the first /tool_pose, then stays user-controlled
        self._tool_pose = None
        self._initialized = False
        self.create_subscription(PoseStamped, "tool_pose", self.on_tool_pose, 10)

    # =========================================================
    # Marker construction
    # =========================================================
    def _make_marker(self, pose):
        marker = InteractiveMarker()
        marker.header.frame_id = "base_link"
        marker.name = "goal"
        marker.description = "motion goal (right-click)"
        marker.scale = 0.25
        marker.pose = pose

        # Center sphere (also the menu click target)
        sphere = Marker()
        sphere.type = Marker.SPHERE
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.06
        sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = (
            0.2, 0.6, 1.0, 0.8,
        )
        center = InteractiveMarkerControl()
        center.interaction_mode = InteractiveMarkerControl.MENU
        center.always_visible = True
        center.markers.append(sphere)
        marker.controls.append(center)

        # 3 move axes + 3 rotate rings
        for axis, (x, y, z, w) in {
            "x": (1.0, 0.0, 0.0, 1.0),
            "y": (0.0, 1.0, 0.0, 1.0),
            "z": (0.0, 0.0, 1.0, 1.0),
        }.items():
            for mode, prefix in (
                (InteractiveMarkerControl.MOVE_AXIS, "move"),
                (InteractiveMarkerControl.ROTATE_AXIS, "rotate"),
            ):
                ctrl = InteractiveMarkerControl()
                ctrl.name = f"{prefix}_{axis}"
                ctrl.interaction_mode = mode
                ctrl.orientation.x = x
                ctrl.orientation.y = y
                ctrl.orientation.z = z
                ctrl.orientation.w = w
                marker.controls.append(ctrl)
        return marker

    # =========================================================
    # Callbacks
    # =========================================================
    def on_tool_pose(self, msg):
        self._tool_pose = msg.pose
        if not self._initialized:
            self._initialized = True
            self.server.insert(self._make_marker(msg.pose))
            self.menu.apply(self.server, "goal")
            self.server.applyChanges()
            self.get_logger().info("goal marker ready")

    def _call(self, client, feedback, label):
        if not client.service_is_ready():
            self.get_logger().warn(f"{label}: service not available")
            return
        req = client.srv_type.Request()
        req.target = feedback.pose
        req.duration = 0.0
        future = client.call_async(req)
        future.add_done_callback(lambda f: self._on_response(label, f))

    def _on_response(self, label, future):
        try:
            res = future.result()
        except Exception as e:  # noqa: BLE001 — log-and-continue boundary
            self.get_logger().warn(f"{label}: call failed - {e}")
            return
        self._report(label, res)

    def _report(self, label, res):
        if res.success:
            self.get_logger().info(f"{label}: accepted")
        else:
            self.get_logger().warn(f"{label}: rejected - {res.message}")

    def on_move_j(self, feedback):
        self._call(self.cli_j, feedback, "MoveJ")

    def on_move_l(self, feedback):
        self._call(self.cli_l, feedback, "MoveL")

    def on_reset(self, feedback):
        if self._tool_pose is not None:
            self.server.setPose("goal", self._tool_pose)
            self.server.applyChanges()


def main(args=None):
    rclpy.init(args=args)
    node = MarkerServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
