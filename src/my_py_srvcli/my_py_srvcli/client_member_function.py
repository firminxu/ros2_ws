from tutorial_interfaces.srv import MultipleTwoInts
import sys
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        # 创建一个可以调用“multiple_two_ints”服务的客户端
        self.cli = self.create_client(MultipleTwoInts, 'multiple_two_ints')       # CHANGE
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = MultipleTwoInts.Request()                                   # CHANGE

    def send_request(self):
        self.req.a = int(sys.argv[1])
        self.req.b = int(sys.argv[2])
        # 调用self.cli服务客户端对象
        # self.req 包含请求数据
        # self.call_async() 对服务进行异步调用，传递请求
        # 返回的 future 对象跟踪待处理的响应
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    # 调用minimal_client 对象上的send_request() 方法。
    minimal_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(minimal_client)
        if minimal_client.future.done():
            try:
                response = minimal_client.future.result()
            except Exception as e:
                minimal_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                minimal_client.get_logger().info(
                    'Result of multiple_two_ints: for %d * %d = %d' %                                # CHANGE
                    (minimal_client.req.a, minimal_client.req.b, response.product))  # CHANGE
            break

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()