#!/usr/bin/env python3
"""
full_sim.launch.py
Launches:
  1. Gazebo simulation (static_room.world + TurtleBot3)
  2. static_poses (owner, stranger, toy)
  3. odom_to_pose2d
  4. behaviour_executor
→ Leaves the brain (CI.py) to be launched manually
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Packages
    human_detection_sim_pkg = FindPackageShare('human_detection_sim')
    pose_bridge_pkg = FindPackageShare('pose_bridge')
    attachment_executor_pkg = FindPackageShare('attachment_executor')

    # Argument: world file
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([
            human_detection_sim_pkg,
            'worlds',
            'static_room.world'
        ]),
        description='Full path to Gazebo world file'
    )

    return LaunchDescription([

        world_arg,

        # 1. Gazebo server + client
        ExecuteProcess(
            cmd=['gzserver', '--verbose', LaunchConfiguration('world')],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['gzclient'],
            output='screen'
        ),

        # 2. Static pose publisher (owner, stranger, toy)
        Node(
            package='pose_bridge',
            executable='static_poses',
            name='static_pose_publisher',
            output='screen'
        ),

        # 3. odom → /robot_position (Pose2D)
        Node(
            package='pose_bridge',
            executable='odom_to_pose2d',
            name='odom_to_pose2d',
            output='screen'
        ),

        # 4. Behavior executor (the body)
        Node(
            package='attachment_executor',
            executable='behaviour_executor',
            name='behavior_executor',
            output='screen'
        ),
    ])