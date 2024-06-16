# ur5e_navigation/launch/launch_lifecycle_navigation.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ur5e_navigation',
            executable='lifecycle_navigation_node',
            name='ur5e_lifecycle_node',
            output='screen'
        )
    ])
