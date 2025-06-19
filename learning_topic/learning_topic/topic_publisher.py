import rclpy
from rclpy.node import Node
from std_msgs.msg import String 

class Publisher(Node):
    def __init__(self, name):
        super().__init__(name)
        self.publisher = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
    
    def timer_callback(self):
        msg = String()
        msg.data = 'Hello, world!'
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = Publisher('topic_publisher')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
