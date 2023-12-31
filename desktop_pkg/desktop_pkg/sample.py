import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Test(Node):

    def __init__(self):
        super().__init__('test')
        self.publisher_ = self.create_publisher(String, 'pub_topic', 10)
        self.create_subscription(
            String,
            'sub_topic',
            self.listener_callback,
            10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = Test()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()