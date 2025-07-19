import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from std_msgs.msg import String 
from geometry_msgs.msg import Pose
import py_trees

class MoveItActionNode(py_trees.behaviour.Behaviour):
    """调用MoveIt的行为树节点"""
    def __init__(self, node, target_pose, name="MoveIt Action"):
        super().__init__(name)
        self.node = node
        self.target_pose = target_pose
        self.action_client = ActionClient(
            node, 
            MoveGroup, 
            '/move_group'
        )
        self.goal_handle = None
        
    def initialise(self):
        # 创建MoveIt目标
        goal = MoveGroup.Goal()
        goal.request.group_name = "arm"
        goal.request.num_planning_attempts = 10
        
        # 设置目标位姿
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "base_link"
        pose_goal.pose = self.target_pose
        
        # 发送目标
        self.goal_handle = self.action_client.send_goal_async(goal)
        
    def update(self):
        if self.goal_handle is None:
            return py_trees.common.Status.RUNNING
            
        if self.goal_handle.done():
            result = self.goal_handle.result()
            if result.error_code == 1:  # SUCCESS
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.FAILURE
        else:
            return py_trees.common.Status.RUNNING

class GripperControlNode(py_trees.behaviour.Behaviour):
    """控制抓手的行为树节点"""
    def __init__(self, node, command, name="Gripper Control"):
        super().__init__(name)
        self.node = node
        self.command = command  # "open" or "close"
        self.publisher = node.create_publisher(
            String, 
            '/gripper_command', 
            10
        )
        
    def update(self):
        msg = String()
        msg.data = self.command
        self.publisher.publish(msg)
        self.node.get_logger().info(f'Gripper command: {self.command}')
        # 假设抓手动作立即完成
        return py_trees.common.Status.SUCCESS
