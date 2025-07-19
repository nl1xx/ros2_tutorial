#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from enum import Enum
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class TaskState(Enum):
    IDLE = 0
    MOVING_TO_OBJECT = 1
    GRASPING = 2
    MOVING_TO_DESTINATION = 3
    PLACING = 4
    DONE = 5

class PickPlaceFSM(Node):
    def __init__(self):
        super().__init__('pick_place_fsm')
        
        # 初始状态
        self.current_state = TaskState.IDLE
        self.target_object = None
        self.destination = None
        
        # 发布器和订阅器
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.gripper_pub = self.create_publisher(String, '/gripper_command', 10)
        
        # 定时器 - 每100ms更新一次状态机
        self.timer = self.create_timer(0.1, self.update_fsm)
        
        # 模拟传感器数据
        self.at_object = False
        self.object_grasped = False
        self.at_destination = False
        
        self.get_logger().info('FSM节点已启动')
    
    def update_fsm(self):
        """FSM主循环"""
        if self.current_state == TaskState.IDLE:
            # 等待任务
            if self.target_object is not None:
                self.transition_to(TaskState.MOVING_TO_OBJECT)
                
        elif self.current_state == TaskState.MOVING_TO_OBJECT:
            self.move_to_object()
            if self.at_object:
                self.transition_to(TaskState.GRASPING)
                
        elif self.current_state == TaskState.GRASPING:
            self.grasp_object()
            if self.object_grasped:
                self.transition_to(TaskState.MOVING_TO_DESTINATION)
                
        elif self.current_state == TaskState.MOVING_TO_DESTINATION:
            self.move_to_destination()
            if self.at_destination:
                self.transition_to(TaskState.PLACING)
                
        elif self.current_state == TaskState.PLACING:
            self.place_object()
            self.transition_to(TaskState.DONE)
            
        elif self.current_state == TaskState.DONE:
            self.get_logger().info('任务完成！')
            self.current_state = TaskState.IDLE
            self.target_object = None
    
    def transition_to(self, new_state):
        """状态转移"""
        self.get_logger().info(f'状态转移: {self.current_state.name} -> {new_state.name}')
        self.current_state = new_state
    
    def move_to_object(self):
        """移动到物体"""
        cmd = Twist()
        cmd.linear.x = 0.2  # 前进
        self.cmd_vel_pub.publish(cmd)
        # 实际应用中这里应该调用导航或MoveIt
        
    def grasp_object(self):
        """抓取物体"""
        msg = String()
        msg.data = "close"
        self.gripper_pub.publish(msg)
        self.get_logger().info('执行抓取')
        # 模拟抓取成功
        self.object_grasped = True
        
    def move_to_destination(self):
        """移动到目标位置"""
        cmd = Twist()
        cmd.linear.x = 0.2
        self.cmd_vel_pub.publish(cmd)
        
    def place_object(self):
        """放置物体"""
        msg = String()
        msg.data = "open"
        self.gripper_pub.publish(msg)
        self.get_logger().info('执行放置')
    
    def start_task(self, object_name, destination):
        """启动新任务"""
        self.target_object = object_name
        self.destination = destination
        self.get_logger().info(f'开始任务: 抓取{object_name}并放到{destination}')

def main(args=None):
    rclpy.init(args=args)
    fsm = PickPlaceFSM()
    
    # 模拟启动一个任务
    fsm.start_task("red_cube", "box_a")
    
    rclpy.spin(fsm)
    fsm.destroy_node()
    rclpy.shutdown()
