#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
from tf_transformations import euler_from_quaternion


class PoseRelay(Node):
    def __init__(self, odom_topic, pose_topic):
        super().__init__('pose_relay_' + pose_topic.replace('/', ''))

        self.pose_pub = self.create_publisher(Pose2D, pose_topic, 10)
        self.create_subscription(Odometry, odom_topic, self.cb, 10)

    def cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        (_, _, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])

        pose = Pose2D()
        pose.x = p.x
        pose.y = p.y
        pose.theta = yaw

        self.pose_pub.publish(pose)

class StaticToyPose(Node):
    def __init__(self):
        super().__init__('toy_pose_publisher')

        self.pub = self.create_publisher(Pose2D, '/toy_pose', 10)

        self.pose = Pose2D()
        self.pose.x = 0.8     # MUST match your world file
        self.pose.y = 0.6
        self.pose.theta = 0.0

        self.timer = self.create_timer(1.0, self.publish_pose)

    def publish_pose(self):
        self.pub.publish(self.pose)



def main():
    rclpy.init()

    nodes = [
        PoseRelay('/odom', '/robot_pose'),
        PoseRelay('/human_owner/odom', '/owner_pose'),
        PoseRelay('/human_stranger/odom', '/stranger_pose'),
        StaticToyPose(),  # ✅ ADD THIS
    ]

    executor = rclpy.executors.MultiExecutor()
    for n in nodes:
        executor.add_node(n)

    executor.spin()


if __name__ == '__main__':
    main()
