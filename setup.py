from setuptools import setup
from glob import glob
import os

package_name = 'ur5e_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='UR5e navigation using ROS 2 Iron and MoveIt',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lifecycle_node = ur5e_navigation.lifecycle_node:main',
            'navigation_service = ur5e_navigation.navigation_service:main',
            'reach_pose_action_server = ur5e_navigation.reach_pose_action:main',
        ],
    },
)
