# ur5e_navigation/ur5e_navigation/lifecycle_node.py

import rclpy
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn

class UR5eLifecycleNode(LifecycleNode):
    def __init__(self):
        super().__init__('ur5e_lifecycle_node')
        self.get_logger().info('Lifecycle node initialized')

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring...')
        # Add configuration code here
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Activating...')
        # Add activation code here
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Deactivating...')
        # Add deactivation code here
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Cleaning up...')
        # Add cleanup code here
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Shutting down...')
        # Add shutdown code here
        return TransitionCallbackReturn.SUCCESS

def main(args=None):
    rclpy.init(args=args)
    lifecycle_node = UR5eLifecycleNode()
    executor = rclpy.executors.SingleThreadedExecutor()
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
