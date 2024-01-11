# create a node to print "hello world" with rclpy
import rclpy
from rclpy.node import Node

class HelloWorldNode(Node):
    def __init__(self):
        super().__init__("hello_world_node")
        self.get_logger().info("Hello World!")
        
        
def main(args=None):
    rclpy.init(args=args)
    node = HelloWorldNode()
    rclpy.spin(node)
    rclpy.shutdown()