from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'attachment'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include model files - FIXED PATH
        (os.path.join('share', package_name, 'models'), 
         glob('models/*.pth') + glob('models/*.pkl')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='Social attachment behavior manager for robots',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [

            'CI = attachment.CI:main',
        ],
    },
)
# from setuptools import find_packages, setup
# import os
# from glob import glob

# package_name = 'attachment'

# setup(
#     name=package_name,
#     version='0.0.0',
#     packages=find_packages(exclude=['test']),
#     data_files=[
#         ('share/ament_index/resource_index/packages',
#             ['resource/' + package_name]),
#         ('share/' + package_name, ['package.xml']),
#         # Include launch files if you have any
#         (os.path.join('share', package_name, 'launch'), 
#          glob('launch/*.launch.py') if os.path.exists('launch') else []),
#         # Include model files
#         (os.path.join('share', package_name, 'models'),
#          glob(os.path.join('models', '*'))),
#         # (os.path.join('share', package_name, 'models'), 
#         #  glob('models/*.pth') + glob('models/*.pkl') if os.path.exists('models') else []),
#         # Include config files if you have any
#         (os.path.join('share', package_name, 'config'), 
#          glob('config/*.yaml') if os.path.exists('config') else []),
#     ],
#     install_requires=[
#         'setuptools',
#         'numpy',
#         'scipy',
#         'scikit-learn',
#         'matplotlib',
#         'torch',
#         'pandas',
#         'joblib',
#     ],
#     zip_safe=True,
#     maintainer='ubuntu',
#     maintainer_email='ubuntu@todo.todo',
#     description='Social attachment behavior manager for robots',
#     license='Apache-2.0',
#     tests_require=['pytest'],
#     entry_points={
#         'console_scripts': [
#             'behavior_manager_ai = attachment.behavior_manager_ai:main',
#             'behavior_manager_rb = attachment.behavior_manager_rb:main',
#             'test_attachment_node = attachment.test_attachment_node:main', 
#         ],
#     },
# )
