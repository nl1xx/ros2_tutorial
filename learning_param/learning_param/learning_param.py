import rclpy
from rclpy.node import Node
import rclpy.parameter

class ParamLearningNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.timer = self.create_timer(1.0, self.timer_callback) 
        self.declare_parameter('name', 'yb')

    def timer_callback(self):
        name = self.get_parameter('name').get_parameter_value().string_value
        self.get_logger().info(f'{name}!')
        edit = rclpy.parameter.Parameter('name', rclpy.Parameter.Type.STRING, 'pro')
        self.set_parameters([edit])
        self.get_logger().info(f'Changed name to {self.get_parameter("name").get_parameter_value().string_value}!')

def main(args=None):
    rclpy.init(args=args)
    node = ParamLearningNode('learning_param_node')  
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
