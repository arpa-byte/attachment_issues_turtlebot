#!/usr/bin/env python3
"""
Publish /robot_position as geometry_msgs/Pose2D from /odom
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
import math  



class OdomToPose2D(Node):
    def __init__(self):
        super().__init__('odom_to_pose2d')
        self.pub = self.create_publisher(Pose2D, '/robot_position', 10)
        self.sub = self.create_subscription(
            Odometry, '/odom', self.callback, 10
        )
        self.get_logger().info("odom_to_pose2d → /robot_position started")

    def callback(self, msg):
        p = Pose2D()
        p.x = msg.pose.pose.position.x
        p.y = msg.pose.pose.position.y

        # Extract yaw from quaternion – correct field names
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        p.theta = math.atan2(siny_cosp, cosy_cosp)

        self.pub.publish(p)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToPose2D()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()