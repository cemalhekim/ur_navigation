import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.node import Node
from std_srvs.srv import Trigger
from moveit_commander import MoveGroupCommander, PlanningSceneInterface, RobotCommander
from geometry_msgs.msg import PoseStamped

class UR5eLifecycleNode(LifecycleNode):
    def __init__(self):
        super().__init__('ur5e_lifecycle_node')
        self.get_logger().info('Lifecycle node initialized')

        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.group_name = "manipulator"
        self.move_group = MoveGroupCommander(self.group_name)
        self.get_logger().info('MoveIt! commander initialized')

        self.srv = self.create_service(Trigger, 'navigate_to_pose', self.navigate_to_pose_callback)

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring...')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Activating...')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Deactivating...')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Cleaning up...')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Shutting down...')
        return TransitionCallbackReturn.SUCCESS

# Implement a Servcie

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
    lifecycle_node = UR5eLifecycleNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(lifecycle_node)

    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        lifecycle_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
