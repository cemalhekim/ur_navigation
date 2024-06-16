# ur5e_navigation/launch/launch_reach_pose_action.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ur5e_navigation',
            executable='reach_pose_action_server',
            name='reach_pose_action_server',
            output='screen'
        )
    ])
