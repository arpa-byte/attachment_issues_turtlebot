#!/usr/bin/env python3
"""
REAL pose relay from Gazebo model states
Works with any placement of human1 (owner), human2 (stranger), toy
No hard-coded positions — reads actual world state
"""
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose2D
import math


class GazeboModelPoseRelay(Node):
    def __init__(self):
        super().__init__('gazebo_model_pose_relay')

        self.owner_pub = self.create_publisher(Pose2D, '/owner_position', 10)
        self.stranger_pub = self.create_publisher(Pose2D, '/stranger_position', 10)
        self.toy_pub = self.create_publisher(Pose2D, '/toy_pose', 10)

        self.sub = self.create_subscription(
            ModelStates,
            '/model_states',  # This topic exists in Gazebo classic
            self.callback,
            10
        )

        self.get_logger().info("Real Gazebo pose relay ACTIVE — reading actual model positions")


    def callback(self, msg):
        owner_pose = None
        stranger_pose = None
        toy_pose = None

        for i, name in enumerate(msg.name):
            if name == 'human1':  # RED = owner
                p = msg.pose[i].position
                owner_pose = Pose2D(x=p.x, y=p.y, theta=0.0)
            elif name == 'human2':  # BLUE = stranger
                p = msg.pose[i].position
                stranger_pose = Pose2D(x=p.x, y=p.y, theta=0.0)
            elif name == 'toy':
                p = msg.pose[i].position
                toy_pose = Pose2D(x=p.x, y=p.y, theta=0.0)

        if owner_pose:
            self.owner_pub.publish(owner_pose)
        if stranger_pose:
            self.stranger_pub.publish(stranger_pose)
        if toy_pose:
            self.toy_pub.publish(toy_pose)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboModelPoseRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()