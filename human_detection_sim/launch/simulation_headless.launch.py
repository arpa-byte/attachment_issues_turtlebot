import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([
            FindPackageShare('human_detection_sim'),
            'worlds',
            'new_simple_room.world'
        ]),
        description='Full path to Gazebo world file'
    )

    world_path = LaunchConfiguration('world')

    # Launch ONLY gzserver with your world (robot included in SDF)
    gzserver = ExecuteProcess(
        cmd=['gzserver', '--verbose', world_path],
        output='screen'
    )

    return LaunchDescription([
        world_arg,
        gzserver
    ])