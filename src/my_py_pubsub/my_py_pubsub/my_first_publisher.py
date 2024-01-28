import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class FirstPubliser(Node):
    def __init__(self):
        super().__init__("first_publisher")
        # self.create_publisher 创建 Publisher 实例
        # 它传递了消息类型（字符串）、话题名称（“topic”）和队列大小（10）
        # 这允许节点稍后向该话题发布字符串消息
        self.publisher_ = self.create_publisher(String, "topic", 10)
        timer_period = 1.0
        #使用 self.create_timer() 创建计时器
        # 计时器将每隔timer_period秒调用self.timer_callback方法
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = "Practice is a goood way to learn: %d" % self.i
        # 调用publisher_.publish()
        # 传递消息（msg）进行发布
        # 这会将消息广播给“topic”的所有订阅者
        # 通过发布消息，节点现在可以与订阅该主题的其他节点通信数据。 这允许不同的节点在ROS中异步交换信息。
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