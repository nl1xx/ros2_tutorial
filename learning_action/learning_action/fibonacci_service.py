import rclpy
from rclpy.node import Node
from learning_interfaces.action import Fibonacci
from rclpy.action import ActionServer
import time


class FibonacciActionServer(Node):
    def __init__(self, name):
        super().__init__(name)
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback
        )
    
    def execute_callback(self, goal_handle):
        n = goal_handle.request.order
        feedback_msg = Fibonacci.Feedback()
        for i in range(n + 1):
            feedback_msg.partial_sequence.append(self.fibonacci(i))
            self.get_logger().info(f'Feedback: {feedback_msg.partial_sequence}')
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.5)
        
        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence
        return result

    def fibonacci(self, n):
        if n == 1 or n == 0:
            return n
        else:
            return self.fibonacci(n-1) + self.fibonacci(n-2)

def main(args=None):
    rclpy.init(args=args)
    action_server = FibonacciActionServer('fibonacci_server')
    rclpy.spin(action_server)
    action_server.destroy_node()
    rclpy.shutdown()