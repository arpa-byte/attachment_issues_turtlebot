import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    package_dir = get_package_share_directory('human_detection_sim')
    world_path = os.path.join(package_dir, 'worlds', 'simple_room.world')

    # Launch gzserver directly (headless mode)
    gazebo_launch = ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so', world_path],
        output='screen'
    )

    turtlebot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('turtlebot3_gazebo'),
                'launch',
                'robot_state_publisher.launch.py'
            ])
        ])
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'turtlebot3_burger', '-x', '0', '-y', '0', '-z', '0'],
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        turtlebot_spawn,
        spawn_entity
    ])