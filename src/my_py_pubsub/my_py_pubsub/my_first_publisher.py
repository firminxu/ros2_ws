import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class FirstPubliser(Node):
    def __init__(self):
        super().__init__("first_publisher")
        self.publisher_ = self.create_publisher(String, "topic", 10)
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = "Practice is a goood way to learn: %d" % self.i
        self.publisher_.publish(msg)
        self.get_logger().info("Publishing: '%s'" % msg.data)
        self.i += 1
        
def main(args=None):
    rclpy.init(args=args)
    node = FirstPubliser()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()