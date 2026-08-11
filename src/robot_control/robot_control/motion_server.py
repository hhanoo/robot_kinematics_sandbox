#!/usr/bin/env python3
"""
Motion Server Node
==================
Runtime motion control: accepts tool0 pose goals over services and plays
the planned joint trajectory through a backend (SimBackend publishes
/joint_states). Goals are accepted only when idle; rejection is returned
immediately in the service response.

Services:
- ~/move_j (robot_interfaces/MoveJ): one IK solve + joint quintic
- ~/move_l (robot_interfaces/MoveL): linear pose path + seeded IK
- ~/stop (std_srvs/Trigger): hold the current position immediately

Topics:
- /motion_state (std_msgs/String): idle / moving / jog
- /tool_pose (geometry_msgs/PoseStamped): current FK result
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from robot_control.backend import SimBackend
from robot_control.conversions import matrix_to_pose, pose_to_matrix
from robot_control.state_machine import MotionStateMachine
from robot_interfaces.srv import MoveJ, MoveL
from robot_kinematics.fk import fk
from robot_kinematics.ik import solve_ik
from robot_kinematics.jog import jog_step
from robot_trajectory.cartesian_traj import cartesian_to_joint, linear_pose_path
from robot_trajectory.joint_traj import quintic_joint_trajectory

JOINT_NAMES = [
    "link_1_joint",
    "link_2_joint",
    "link_3_joint",
    "link_4_joint",
    "link_5_joint",
    "link_6_joint",
]
# fmt: off
HOME = [0.0, -math.pi / 2, math.pi / 2, -math.pi / 2, -math.pi / 2, 0.0]
# fmt: on


class MotionServer(Node):
    """Owns the joint state; executes pose goals as joint trajectories."""

    def __init__(self):
        super().__init__("motion_server")

        # Parameters
        self.declare_parameter("rate", 50.0)
        self.declare_parameter("home", HOME)
        self.declare_parameter("v_max", 2.0)
        self.declare_parameter("a_max", 4.0)
        self.declare_parameter("linear_speed", 0.1)
        self.declare_parameter("jog_max_linear", 0.25)
        self.declare_parameter("jog_max_angular", 1.0)
        self.declare_parameter("jog_deadman_timeout", 0.3)
        rate = self.get_parameter("rate").value
        home = self.get_parameter("home").value
        self.v_max = self.get_parameter("v_max").value
        self.a_max = self.get_parameter("a_max").value
        self.linear_speed = self.get_parameter("linear_speed").value
        self.jog_max_lin = self.get_parameter("jog_max_linear").value
        self.jog_max_ang = self.get_parameter("jog_max_angular").value
        self.dt = 1.0 / rate

        # State machine, backend, trajectory buffer
        self.sm = MotionStateMachine(self.get_parameter("jog_deadman_timeout").value)
        self.backend = SimBackend(self, JOINT_NAMES, home)
        self._traj = None
        self._traj_i = 0
        self._jog_twist = np.zeros(6)

        # Publishers
        self.pub_state = self.create_publisher(String, "motion_state", 10)
        self.pub_tool = self.create_publisher(PoseStamped, "tool_pose", 10)
        self._last_state = ""
        self._tick_count = 0

        # Services
        self.create_service(MoveJ, "~/move_j", self.on_move_j)
        self.create_service(MoveL, "~/move_l", self.on_move_l)
        self.create_service(Trigger, "~/stop", self.on_stop)
        self.create_subscription(TwistStamped, "jog_twist", self.on_jog_twist, 10)

        # Playback timer
        self.create_timer(self.dt, self.on_timer)
        self.get_logger().info("motion_server ready (idle at home)")

    # =========================================================
    # Service handlers (accept/reject immediately, never block)
    # =========================================================
    def on_move_j(self, req, res):
        ok, why = self.sm.can_accept_move()
        if not ok:
            res.success, res.message = False, why
            return res

        # 1. One IK solve from the current posture
        q_now = self.backend.q
        target = pose_to_matrix(req.target)
        ik = solve_ik(target, q_now)
        if not ik.success:
            res.success = False
            res.message = (
                f"IK failed: pos_err={ik.pos_error:.4f}, rot_err={ik.rot_error:.4f}"
            )
            return res

        # 2. Joint quintic (duration 0 = shortest within limits)
        duration = req.duration if req.duration > 0.0 else None
        traj = quintic_joint_trajectory(
            q_now, ik.q, self.v_max, self.a_max, self.dt, duration
        )
        self._start(traj.q)
        res.success, res.message = True, ""
        return res

    def on_move_l(self, req, res):
        ok, why = self.sm.can_accept_move()
        if not ok:
            res.success, res.message = False, why
            return res

        # 1. Waypoints spaced linear_speed * dt apart (constant tool speed)
        q_now = self.backend.q
        T_now = fk(q_now)
        target = pose_to_matrix(req.target)
        dist = float(np.linalg.norm(target[:3, 3] - T_now[:3, 3]))
        if req.duration > 0.0:
            n = max(2, int(round(req.duration / self.dt)) + 1)
        else:
            n = max(2, int(round(dist / (self.linear_speed * self.dt))) + 1)

        # 2. Full path IK before starting (fail fast, reject on any waypoint)
        path = cartesian_to_joint(linear_pose_path(T_now, target, n), q_now)
        if not path.success:
            res.success = False
            res.message = f"IK failed at waypoint {path.failed_index}/{n}"
            return res

        self._start(path.q)
        res.success, res.message = True, ""
        return res

    def on_stop(self, req, res):
        self._traj = None
        self.sm.stop()
        res.success, res.message = True, "stopped"
        return res

    def _start(self, rows):
        self._traj = rows
        self._traj_i = 0
        self.sm.enter_moving()

    def on_jog_twist(self, msg):
        # Clamp per-axis groups, then hand to the state machine
        t = msg.twist
        lin = np.array([t.linear.x, t.linear.y, t.linear.z])
        ang = np.array([t.angular.x, t.angular.y, t.angular.z])
        n_lin, n_ang = np.linalg.norm(lin), np.linalg.norm(ang)
        if n_lin > self.jog_max_lin:
            lin *= self.jog_max_lin / n_lin
        if n_ang > self.jog_max_ang:
            ang *= self.jog_max_ang / n_ang
        if self.sm.on_jog(self._now()):
            self._jog_twist = np.concatenate([lin, ang])

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # =========================================================
    # Playback timer
    # =========================================================
    def on_timer(self):
        # 1. Advance the trajectory or hold position
        self.sm.tick(self._now())
        if self.sm.state == MotionStateMachine.MOVING and self._traj is not None:
            q = self._traj[self._traj_i]
            self._traj_i += 1
            if self._traj_i >= len(self._traj):
                self._traj = None
                self.sm.finish_move()
        elif self.sm.state == MotionStateMachine.JOG:
            q = jog_step(self.backend.q, self._jog_twist, self.dt)
        else:
            q = self.backend.q
        self.backend.write(q)

        # 2. Publish tool pose from FK
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.pose = matrix_to_pose(fk(q))
        self.pub_tool.publish(msg)

        # 3. Publish state on change + 1 Hz heartbeat
        self._tick_count += 1
        if (
            self.sm.state != self._last_state
            or self._tick_count % int(1.0 / self.dt) == 0
        ):
            self._last_state = self.sm.state
            self.pub_state.publish(String(data=self.sm.state))


def main(args=None):
    rclpy.init(args=args)
    node = MotionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
