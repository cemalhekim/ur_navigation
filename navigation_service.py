# ur5e_navigation/ur5e_navigation/navigation_service.py

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class NavigationService(Node):
    def __init__(self):
        super().__init__('navigation_service')
        self.srv = self.create_service(Trigger, 'navigate_to_pose', self.navigate_to_pose_callback)
        self.get_logger().info('Service node initialized')

    def navigate_to_pose_callback(self, request, response):
        self.get_logger().info('Navigating to pose...')
        # Implement navigation logic here
        response.success = True
        response.message = "Navigation completed"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = NavigationService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
