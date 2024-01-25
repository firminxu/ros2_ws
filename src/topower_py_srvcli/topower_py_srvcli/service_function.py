from tutorial_interfaces.srv import ToPowerTwoInts

 
import rclpy
from rclpy.node import Node

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(ToPowerTwoInts, 'to_power_two_ints', self.srv_callback)
 
    def srv_callback(self, request, response):
        response.result = request.x **request.y
        self.get_logger().info('Incoming request\nx: %d y: %d' % (request.x, request.y))
        return response
    
def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    minimal_service.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()