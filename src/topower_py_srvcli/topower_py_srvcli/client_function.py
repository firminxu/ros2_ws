import sys
from tutorial_interfaces.srv import ToPowerTwoInts
import rclpy
from rclpy.node import Node

class MinimalClientAysnc(Node):
    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(ToPowerTwoInts, 'to_power_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = ToPowerTwoInts.Request()
    def send_request(self):
        self.req.x = int(sys.argv[1])
        self.req.y = int(sys.argv[2])
        self.future = self.cli.call_async(self.req)
        # rclpy.spin_until_future_complete(self, self.future)
        # return self.future.result()


def main(args=None):
    rclpy.init(args=args)
    minimal_client=MinimalClientAysnc()
    minimal_client.send_request()
    while rclpy.ok():
        rclpy.spin_once(minimal_client)
        if minimal_client.future.done():
            try:
                response=minimal_client.future.result()
            
            except Exception as e:
                minimal_client.get_logger().info('Service call failed %r' % (e,))
                break
            
            else:
                minimal_client.get_logger().info(
                    'Result of to_power_two_ints: for %d ** %d= %d' %
                    (minimal_client.req.x, minimal_client.req.y, response.result))
    minimal_client.destroy_node()
    rclpy.shutdown()