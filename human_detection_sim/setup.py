# ~/comp_attachment/src/human_detection_sim/setup.py

from setuptools import setup
import os
from glob import glob

package_name = 'human_detection_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # Standard ROS2 package index
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Install launch files
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),

        # Install world files (including .xacro)
        ('share/' + package_name + '/worlds', [
            'worlds/static_room.world.xacro',
            'worlds/static_room_two.world',
            'worlds/simple_room.world',
            'worlds/new_simple_room.world',
            # add any others if you have them
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='Human detection simulation nodes with configurable scenarios',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_publisher = human_detection_sim.pose_publisher:main',
        ],
    },
)