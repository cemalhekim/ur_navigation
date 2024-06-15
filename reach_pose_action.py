# ur5e_navigation/ur5e_navigation/reach_pose_action.py

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from moveit_msgs.action import MoveGroup

class ReachPoseActionServer(Node):
    def __init__(self):
        super().__init__('reach_pose_action_server')
        self._action_server = ActionServer(
            self,
            MoveGroup,
            'reach_pose',
            self.execute_callback
        )
        self.get_logger().info('Action server initialized')

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        # Implement pose reaching logic here
        goal_handle.succeed()

        result = MoveGroup.Result()
        result.error_code.val = MoveGroup.Result.SUCCESS
        return result

def main(args=None):
    rclpy.init(args=args)
    node = ReachPoseActionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
