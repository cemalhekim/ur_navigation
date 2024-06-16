import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander, PlanningSceneInterface, RobotCommander

class BasicNavigationNode(Node):
    def __init__(self):
        super().__init__('basic_navigation_node')
        self.srv = self.create_service(Trigger, 'navigate_to_pose', self.navigate_to_pose_callback)
        self.get_logger().info('Basic navigation node initialized')

        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.group_name = "manipulator"
        self.move_group = MoveGroupCommander(self.group_name)
        self.get_logger().info('MoveIt! commander initialized')

    def navigate_to_pose_callback(self, request, response):
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_link"
        target_pose.pose.position.x = 0.4  # Example target coordinates
        target_pose.pose.position.y = 0.1
        target_pose.pose.position.z = 0.4
        target_pose.pose.orientation.w = 1.0

        self.move_group.set_pose_target(target_pose.pose)

        plan = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        if plan:
            response.success = True
            response.message = "Navigation succeeded"
        else:
            response.success = False
            response.message = "Navigation failed"
        
        return response

def main(args=None):
    rclpy.init(args=args)
    node = BasicNavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
