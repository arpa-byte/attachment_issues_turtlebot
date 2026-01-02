#!/usr/bin/env python3
"""
attachment_executor/behavior_executor.py
Low-level behavior executor for the fuzzy attachment model
DOES NOT modify CI.py — only listens and acts.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from std_msgs.msg import String
import numpy as np
import time
import math

class BehaviorExecutor(Node):
    def __init__(self):
        super().__init__('behavior_executor')

        # --- State ---
        self.is_holding_toy = False
        self.current_action = None
        self.last_action_time = time.time()
        self.action_start_time = time.time()

        # --- Pose storage ---
        self.robot_pose = None
        self.owner_pose = None
        self.stranger_pose = None
        self.toy_pose = None

        # --- Publishers ---
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.feedback_pub = self.create_publisher(String, '/action_feedback', 10)

        # --- Subscribers ---
        self.create_subscription(String, '/current_action', self.action_callback, 10)
        self.create_subscription(Pose2D, '/robot_position', self.robot_cb, 10)
        self.create_subscription(Pose2D, '/owner_position', self.owner_cb, 10)
        self.create_subscription(Pose2D, '/stranger_position', self.stranger_cb, 10)
        self.create_subscription(Pose2D, '/toy_pose', self.toy_cb, 10)

        # --- Timer: 20 Hz control loop ---
        self.create_timer(0.05, self.control_loop)  # 20 Hz

        self.get_logger().info("Behavior Executor initialized and waiting for actions...")

    # --- Callbacks ---
    def robot_cb(self, msg): self.robot_pose = msg
    def owner_cb(self, msg): self.owner_pose = msg
    def stranger_cb(self, msg): self.stranger_pose = msg
    def toy_cb(self, msg): self.toy_pose = msg

    def action_callback(self, msg):
        new_action = msg.data.strip()
        if new_action != self.current_action:
            self.get_logger().info(f"New action received: {new_action}")
            self.current_action = new_action
            self.action_start_time = time.time()
            # Reset timeout on new action
            if new_action in ["PICK_TARGET", "DELIVER_TO_OWNER"]:
                self.get_logger().info("Critical action started — monitoring for success...")

    # --- Feedback Helper ---
    def send_feedback(self, result: str):
        msg = String()
        msg.data = result
        self.feedback_pub.publish(msg)
        self.get_logger().info(f"Feedback sent: {result}")

    # --- Distance & Angle Helpers ---
    def dist_to(self, pose):
        if self.robot_pose is None or pose is None:
            return float('inf')
        dx = pose.x - self.robot_pose.x
        dy = pose.y - self.robot_pose.y
        return math.hypot(dx, dy)

    def angle_to(self, pose):
        if self.robot_pose is None or pose is None:
            return 0.0
        dx = pose.x - self.robot_pose.x
        dy = pose.y - self.robot_pose.y
        target_yaw = math.atan2(dy, dx)
        angle_error = target_yaw - self.robot_pose.theta
        # Normalize to [-pi, pi]
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi
        return angle_error

    # --- Simple Go-To Behavior ---
    def go_to(self, target_pose, max_linear=0.22, max_angular=1.0, stop_dist=0.3):
        twist = Twist()
        dist = self.dist_to(target_pose)
        angle_err = self.angle_to(target_pose)

        if dist < stop_dist:
            return Twist()  # stopped

        linear_vel = min(max_linear, dist * 0.8)
        angular_vel = max(min(max_angular, angle_err * 2.0), -max_angular)

        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        return twist

    # --- Main Control Loop ---
    def control_loop(self):
        if self.robot_pose is None:
            return

        if self.current_action is None:
            # No action yet — stay still
            self.cmd_vel_pub.publish(Twist())
            return

        twist = Twist()
        now = time.time()
        elapsed = now - self.action_start_time

        action = self.current_action

        # ========================================
        # ACTION IMPLEMENTATIONS
        # ========================================

        if action == "FOLLOW_OWNER_HIGH_VEL":
            twist = self.go_to(self.owner_pose, max_linear=0.22, stop_dist=0.2)

        elif action == "FOLLOW_OWNER_MID_VEL":
            twist = self.go_to(self.owner_pose, max_linear=0.15, stop_dist=0.2)

        elif action == "FOLLOW_STRANGER_LOW_VEL":
            twist = self.go_to(self.stranger_pose, max_linear=0.10, stop_dist=0.2)

        elif action == "PICK_TARGET":
            if self.is_holding_toy:
                self.send_feedback("SUCCESS")
                self.get_logger().warn("Already holding toy — skipping pick")
            else:
                twist = self.go_to(self.toy_pose, max_linear=0.18, stop_dist=0.2)
                if self.dist_to(self.toy_pose) < 0.30:
                    self.is_holding_toy = True
                    self.send_feedback("SUCCESS")
                    self.get_logger().info("Toy PICKED UP!")

        elif action == "DELIVER_TO_OWNER":
            if not self.is_holding_toy:
                self.send_feedback("FAIL")
                self.get_logger().warn("Tried to deliver but not holding toy!")
            else:
                twist = self.go_to(self.owner_pose, max_linear=0.18, stop_dist=0.2)
                if self.dist_to(self.owner_pose) < 0.20:
                    self.is_holding_toy = False
                    self.send_feedback("SUCCESS")
                    self.get_logger().info("Toy DELIVERED to owner!")

        elif action == "PLACE_TARGET":
            if self.is_holding_toy:
                self.is_holding_toy = False
                self.send_feedback("SUCCESS")
                self.get_logger().info("Toy PLACED on ground (no owner nearby)")
            twist = Twist()  # stop

        elif action in ["WAIT_AT_DOOR", "IDLE"]:
            twist = Twist()

        else:
            self.get_logger().warn(f"Unknown action: {action}")
            twist = Twist()

        # Safety timeout: if stuck for >10s on navigation action, send FAIL
        if action in ["FOLLOW_OWNER_HIGH_VEL", "FOLLOW_OWNER_MID_VEL", "FOLLOW_STRANGER_LOW_VEL",
                      "PICK_TARGET", "DELIVER_TO_OWNER"] and elapsed > 12.0:
            if not self.is_holding_toy and "DELIVER" not in action:
                self.send_feedback("FAIL")
                self.get_logger().warn("Action timeout — sending FAIL")

        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down behavior executor...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()