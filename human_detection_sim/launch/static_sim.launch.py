import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # Declare world argument
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([
            FindPackageShare('human_detection_sim'),
            'worlds',
            'static_room.world'
        ]),
        description='Full path to Gazebo world file'
    )

    world_path = LaunchConfiguration('world')

    # Launch gzserver (physics + world + robot from SDF)
    gzserver = ExecuteProcess(
        cmd=['gzserver', '--verbose', world_path],
        output='screen'
    )

    # Launch gzclient (GUI only)
    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    return LaunchDescription([
        world_arg,
        gzserver,
        gzclient,   # only in simulation.launch.py
        Node(
            package='human_mover',
            executable='mover_node',
            name='human_mover',
            output='screen'
        )
        #Node(
        #    package='human_detection_sim',
        #    executable='pose_publisher',
        #    name='pose_relay',
        #    output='screen'
        #),
    ])