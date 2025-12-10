from setuptools import find_packages, setup

package_name = 'pose_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubmac',
    maintainer_email='arpanekka.a1.73@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'odom_to_pose2d = pose_bridge.odom_to_pose2d:main',
            'static_poses = pose_bridge.static_pose_publisher:main',
        ],
    },
)
