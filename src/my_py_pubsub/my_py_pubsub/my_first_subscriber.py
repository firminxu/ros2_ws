import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class FirstSubscriber(Node):
    def __init__(self):
        super().__init__("first_subscriber")
        # 创建一个订阅者，该订阅者将从“topic”主题接收 String (std_msgs/String) 类型的消息。
        # 当收到消息时，会调用 self.listener_callback 方法并传递消息。 这允许节点处理来自该主题的任何发布者的传入数据。
        self.subscription = self.create_subscription(String, "topic", self.listener_callback, 10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info("I knew: %s" % msg.data)
        
def main(args=None):
    rclpy.init(args=args)
    node = FirstSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()