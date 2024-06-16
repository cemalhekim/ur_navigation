# ur5e_navigation/ur5e_navigation/reach_pose_action.py

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from moveit_msgs.action import MoveGroup
from geometry_msgs.msg import PoseStamped

class ReachPoseActionServer(Node):
    def __init__(self):
        super().__init__('reach_pose_action_server')
        self._action_server = ActionServer(
            self,
            MoveGroup,
            'reach_pose',
            self.execute_callback
        )
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.group_name = "manipulator"
        self.move_group = MoveGroupCommander(self.group_name)
        self.get_logger().info('MoveIt! commander initialized')
        self.get_logger().info('Action server initialized')

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        target_pose = goal_handle.request.target_pose

        self.move_group.set_pose_target(target_pose.pose)
        plan = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        if plan:
            goal_handle.succeed()
            result = MoveGroup.Result()
            result.error_code.val = MoveGroup.Result.SUCCESS
        else:
            goal_handle.abort()
            result = MoveGroup.Result()
            result.error_code.val = MoveGroup.Result.FAILURE

        return result

def main(args=None):
    rclpy.init(args=args)
    node = ReachPoseActionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
