#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTolerance
import time

class ArmMover(Node):
    def __init__(self):
        super().__init__('arm_mover')
        # 使用正确的控制器名称
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )
        self._goal_handle = None
        self._send_goal_future = None
        self._get_result_future = None

    def send_goal(self, joint_positions, duration_sec, goal_name):
        # 等待Action Server启动
        if not self._action_client.wait_for_server():
            self.get_logger().error("Action Server not available after 10 seconds!")
            return

        # 创建Goal消息
        goal_msg = FollowJointTrajectory.Goal()
        
        # 设置关节名称（必须与URDF和控制器配置匹配）
        goal_msg.trajectory.joint_names = [
            'j1', 'j2', 'j3',
            'j4', 'j5', 'j6'
        ]
        
        # 创建轨迹点
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start.sec = int(duration_sec)
        point.time_from_start.nanosec = int((duration_sec - int(duration_sec)) * 1e9)
        
        # 添加轨迹点到目标
        goal_msg.trajectory.points = [point]
        
        # 添加容差以避免奇异点报错 - 修复部分
        goal_msg.goal_tolerance = []  # 先清空或初始化
        for joint_name in goal_msg.trajectory.joint_names:
            tolerance = JointTolerance()  # 使用正确导入的 JointTolerance
            tolerance.name = joint_name
            tolerance.position = 0.1  # 10度容差
            goal_msg.goal_tolerance.append(tolerance)
        
        # 设置路径容差
        goal_msg.path_tolerance = []
        goal_msg.goal_time_tolerance.sec = 10
        goal_msg.goal_time_tolerance.nanosec = 0
        
        # 发送目标
        self.get_logger().info(f"Sending {goal_name} goal...")
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._goal_handle = goal_handle
        
        # 请求结果（异步）
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: error_code={result.error_code}, error_string={result.error_string}')
        
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # 简化反馈输出
        self.get_logger().info(f'Feedback: positions={[round(p, 3) for p in feedback.actual.positions]}')

def main(args=None):
    rclpy.init(args=args)
    arm_mover = ArmMover()
    
    # 示例1：移动到肘部奇异位形 (q3=0)
    elbow_singularity = [0.207, 0.794, 0.0, 0.190, 0.121, 0.466]  # 关节角度(弧度)
    arm_mover.send_goal(elbow_singularity, 8.0, "Elbow Singularity")
    
    # 等待第一个目标完成
    rclpy.spin(arm_mover)
    
    # # 示例2：移动到腕部奇异位形 (q5=0)
    # time.sleep(1)  # 短暂暂停
    # wrist_singularity = [0.0, -1.57, 1.57, 0.0, 0.0, 0.0]
    # arm_mover.send_goal(wrist_singularity, 8.0, "Wrist Singularity")
    
    # rclpy.spin(arm_mover)
    rclpy.shutdown()

if __name__ == '__main__':
    main()