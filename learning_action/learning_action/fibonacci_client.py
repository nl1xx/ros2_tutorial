import rclpy
from rclpy.node import Node
from learning_interfaces.action import Fibonacci
from rclpy.action import ActionClient

class FibonacciClient(Node):
    def __init__(self, name):
        super().__init__(name)
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')
        self._action_client.wait_for_server()
        self.send_goal(5)  # 直接调用 send_goal 方法

    def send_goal(self, n):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = n
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Final Fibonacci sequence: {result.sequence}')
        rclpy.shutdown()  # 关闭节点

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: {feedback.partial_sequence}')

def main(args=None):
    rclpy.init(args=args)
    node = FibonacciClient('fibonacci_client')
    rclpy.spin(node)
    node.destroy_node()
