#!/usr/bin/env python3
"""
Publish static poses for owner, stranger, and toy
Matches your static_room.world exactly
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
import time


class StaticPosePublisher(Node):
    def __init__(self):
        super().__init__('static_pose_publisher')

        # Publishers
        self.owner_pub = self.create_publisher(Pose2D, '/owner_position', 10)
        self.stranger_pub = self.create_publisher(Pose2D, '/stranger_position', 10)
        self.toy_pub = self.create_publisher(Pose2D, '/toy_pose', 10)

        # Static poses from your world file
        self.owner_pose = Pose2D(x=-1.0, y=-1.0, theta=0.0)
        self.stranger_pose = Pose2D(x=1.0, y=-1.0, theta=0.0)
        self.toy_pose = Pose2D(x=1.0, y=1.0, theta=0.0)

        # 10 Hz publishing
        self.timer = self.create_timer(0.1, self.publish_poses)

        self.get_logger().info("Static pose publisher running:")
        self.get_logger().info("  Owner   → (-1.0, -1.0)")
        self.get_logger().info("  Stranger→ ( 1.0, -1.0)")
        self.get_logger().info("  Toy     → ( 1.0,  1.0)")

    def publish_poses(self):
        now = self.get_clock().now().to_msg()
        self.owner_pub.publish(self.owner_pose)
        self.stranger_pub.publish(self.stranger_pose)
        self.toy_pub.publish(self.toy_pose)


def main(args=None):
    rclpy.init(args=args)
    node = StaticPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()