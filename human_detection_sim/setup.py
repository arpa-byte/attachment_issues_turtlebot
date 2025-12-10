from setuptools import setup

package_name = 'human_detection_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='Human detection simulation nodes',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'pose_publisher = human_detection_sim.pose_publisher:main',
        ],
    },
)
