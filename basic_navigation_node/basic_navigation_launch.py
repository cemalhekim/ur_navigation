from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ur5e_navigation',
            executable='basic_navigation_node',
            name='basic_navigation_node',
            output='screen'
        )
    ])
