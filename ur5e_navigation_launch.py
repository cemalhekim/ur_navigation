# ur5e_navigation/launch/ur5e_navigation_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ur5e_navigation',
            executable='lifecycle_node',
            name='ur5e_lifecycle_node',
            output='screen'
        ),
        Node(
            package='ur5e_navigation',
            executable='navigation_service',
            name='navigation_service',
            output='screen'
        ),
        Node(
            package='ur5e_navigation',
            executable='reach_pose_action_server',
            name='reach_pose_action_server',
            output='screen'
        ),
    ])
