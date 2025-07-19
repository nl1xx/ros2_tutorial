#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import py_trees
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import String, Bool
from sensor_msgs.msg import PointCloud2
import numpy as np

# 补全缺失的行为树节点
class MoveToSafePosition(py_trees.behaviour.Behaviour):
    """移动到安全位置"""
    def __init__(self, motion_planner):
        super().__init__("Move to Safe Position")
        self.motion_planner = motion_planner
        self.goal_handle = None
        
    def initialise(self):
        # 安全位置通常是已知的固定位置，例如机械臂的home位置
        safe_pose = Pose()
        safe_pose.position.x = 0.0
        safe_pose.position.y = 0.0
        safe_pose.position.z = 0.2  # 20cm高度
        safe_pose.orientation.w = 1.0
        
        self.goal_handle = self.motion_planner.move_to_pose(safe_pose)
        
    def update(self):
        if self.motion_planner.is_motion_complete(self.goal_handle):
            return py_trees.common.Status.SUCCESS
        elif self.motion_planner.is_motion_failed(self.goal_handle):
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.RUNNING

class DetectPlaceLocation(py_trees.behaviour.Behaviour):
    """检测放置位置"""
    def __init__(self, perception, place_location_name):
        super().__init__(f"Detect {place_location_name}")
        self.perception = perception
        self.place_location_name = place_location_name
        
    def update(self):
        place_pose = self.perception.get_object_pose(self.place_location_name)
        if place_pose is not None:
            self.blackboard.set("place_location_pose", place_pose)
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

class MoveToPrePlace(py_trees.behaviour.Behaviour):
    """移动到放置预位置"""
    def __init__(self, motion_planner):
        super().__init__("Move to Pre-place")
        self.motion_planner = motion_planner
        self.goal_handle = None
        
    def initialise(self):
        place_pose = self.blackboard.get("place_location_pose")
        pre_place_pose = self.compute_pre_place(place_pose)
        self.goal_handle = self.motion_planner.move_to_pose(pre_place_pose)
        
    def update(self):
        if self.motion_planner.is_motion_complete(self.goal_handle):
            return py_trees.common.Status.SUCCESS
        elif self.motion_planner.is_motion_failed(self.goal_handle):
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.RUNNING
    
    def compute_pre_place(self, place_pose):
        """计算放置预位姿（在放置位置上方）"""
        pre_place_pose = Pose()
        pre_place_pose.position.x = place_pose.position.x
        pre_place_pose.position.y = place_pose.position.y
        pre_place_pose.position.z = place_pose.position.z + 0.1  # 上方10cm
        pre_place_pose.orientation = place_pose.orientation
        return pre_place_pose

class MoveToPlace(py_trees.behaviour.Behaviour):
    """移动到放置位置"""
    def __init__(self, motion_planner):
        super().__init__("Move to Place")
        self.motion_planner = motion_planner
        self.goal_handle = None
        
    def initialise(self):
        place_pose = self.blackboard.get("place_location_pose")
        self.goal_handle = self.motion_planner.move_to_pose(place_pose)
        
    def update(self):
        if self.motion_planner.is_motion_complete(self.goal_handle):
            return py_trees.common.Status.SUCCESS
        elif self.motion_planner.is_motion_failed(self.goal_handle):
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.RUNNING

class CheckPlaceSuccess(py_trees.behaviour.Behaviour):
    """检查放置是否成功"""
    def __init__(self, perception):
        super().__init__("Check Place Success")
        self.perception = perception
        
    def update(self):
        # 检查是否在放置位置检测到物体
        place_pose = self.blackboard.get("place_location_pose")
        # 假设放置成功需要检查物体是否在放置位置
        # 这里简化逻辑，实际应根据感知数据判断
        if place_pose is not None:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

class PickPlaceSystem(Node):
    """完整的抓取-放置系统"""
    
    def __init__(self):
        super().__init__('pick_place_system')
        
        # 初始化各个子系统
        self.perception_system = PerceptionInterface(self)
        self.grasp_planner = GraspPlannerInterface(self)
        self.motion_planner = MotionPlannerInterface(self)
        self.gripper_controller = GripperInterface(self)
        
        # 任务参数
        self.target_object = "red_cube"
        self.place_location = "blue_box"
        
        # 创建行为树
        self.behavior_tree = self.create_behavior_tree()
        
        # 启动行为树
        self.timer = self.create_timer(0.1, self.tick_behavior_tree)
        
    def create_behavior_tree(self):
        """构建完整的抓取-放置行为树"""
        
        # 根节点
        root = py_trees.composites.Sequence("Complete Pick and Place", memory=True)
        
        # 1. 抓取序列
        pick_fallback = py_trees.composites.Selector("Pick Fallback", memory=False)
        
        # 1.1 主抓取序列
        main_pick_sequence = py_trees.composites.Sequence("Main Pick", memory=True)
        main_pick_sequence.add_children([
            DetectObject(self.perception_system, self.target_object),
            GenerateGrasps(self.grasp_planner),
            SelectBestGrasp(self.grasp_planner),
            MoveToPreGrasp(self.motion_planner),
            OpenGripper(self.gripper_controller),
            MoveToGrasp(self.motion_planner),
            CloseGripper(self.gripper_controller),
            CheckGraspSuccess(self.gripper_controller),
            LiftObject(self.motion_planner)
        ])
        
        # 1.2 抓取失败恢复
        pick_recovery = py_trees.composites.Sequence("Pick Recovery", memory=True)
        pick_recovery.add_children([
            LogMessage("抓取失败，尝试恢复"),
            OpenGripper(self.gripper_controller),
            MoveToSafePosition(self.motion_planner),
        ])
        
        # 使用 Retry 装饰器包装主抓取序列
        retry_pick = py_trees.decorators.Retry(
            name="Retry Pick",
            child=main_pick_sequence,
            num_failures=2
        )
        
        pick_fallback.add_children([retry_pick, pick_recovery])
        
        # 2. 放置序列
        place_sequence = py_trees.composites.Sequence("Place Sequence", memory=True)
        place_sequence.add_children([
            DetectPlaceLocation(self.perception_system, self.place_location),
            MoveToPrePlace(self.motion_planner),
            MoveToPlace(self.motion_planner),
            OpenGripper(self.gripper_controller),
            CheckPlaceSuccess(self.perception_system),
            MoveToSafePosition(self.motion_planner)
        ])
        
        # 3. 组装完整树
        root.add_children([pick_fallback, place_sequence])
        
        return py_trees.trees.BehaviourTree(root)
    
    def tick_behavior_tree(self):
        """定期更新行为树"""
        self.behavior_tree.tick()
        
        # 打印当前状态（调试用）
        if hasattr(self, '_last_status') and self._last_status != self.behavior_tree.root.status:
            self.get_logger().info(f'行为树状态: {self.behavior_tree.root.status}')
            self._last_status = self.behavior_tree.root.status

# 行为树节点实现
class DetectObject(py_trees.behaviour.Behaviour):
    """检测目标物体"""
    def __init__(self, perception, object_name):
        super().__init__(f"Detect {object_name}")
        self.perception = perception
        self.object_name = object_name
        
    def update(self):
        object_pose = self.perception.get_object_pose(self.object_name)
        if object_pose is not None:
            self.blackboard.set("target_object_pose", object_pose)
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

class GenerateGrasps(py_trees.behaviour.Behaviour):
    """生成抓取位姿"""
    def __init__(self, grasp_planner):
        super().__init__("Generate Grasps")
        self.grasp_planner = grasp_planner
        
    def update(self):
        object_pose = self.blackboard.get("target_object_pose")
        grasps = self.grasp_planner.generate_grasps(object_pose)
        if len(grasps) > 0:
            self.blackboard.set("grasp_candidates", grasps)
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

class SelectBestGrasp(py_trees.behaviour.Behaviour):
    """选择最佳抓取"""
    def __init__(self, grasp_planner):
        super().__init__("Select Best Grasp")
        self.grasp_planner = grasp_planner
        
    def update(self):
        grasps = self.blackboard.get("grasp_candidates")
        best_grasp = self.grasp_planner.select_best_grasp(grasps)
        if best_grasp is not None:
            self.blackboard.set("selected_grasp", best_grasp)
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

class MoveToPreGrasp(py_trees.behaviour.Behaviour):
    """移动到预抓取位姿"""
    def __init__(self, motion_planner):
        super().__init__("Move to Pre-grasp")
        self.motion_planner = motion_planner
        self.goal_handle = None
        
    def initialise(self):
        grasp = self.blackboard.get("selected_grasp")
        pre_grasp = self.compute_pre_grasp(grasp)
        self.goal_handle = self.motion_planner.move_to_pose(pre_grasp)
        
    def update(self):
        if self.motion_planner.is_motion_complete(self.goal_handle):
            return py_trees.common.Status.SUCCESS
        elif self.motion_planner.is_motion_failed(self.goal_handle):
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.RUNNING
    
    def compute_pre_grasp(self, grasp):
        """计算预抓取位姿（在抓取位姿上方）"""
        pre_grasp = Pose()
        pre_grasp.position.x = grasp.position.x
        pre_grasp.position.y = grasp.position.y
        pre_grasp.position.z = grasp.position.z + 0.1  # 上方10cm
        pre_grasp.orientation = grasp.orientation
        return pre_grasp

class OpenGripper(py_trees.behaviour.Behaviour):
    """打开抓手"""
    def __init__(self, gripper):
        super().__init__("Open Gripper")
        self.gripper = gripper
        
    def update(self):
        if self.gripper.open():
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

class CloseGripper(py_trees.behaviour.Behaviour):
    """关闭抓手"""
    def __init__(self, gripper):
        super().__init__("Close Gripper")
        self.gripper = gripper
        
    def update(self):
        if self.gripper.close():
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

class CheckGraspSuccess(py_trees.behaviour.Behaviour):
    """检查抓取是否成功"""
    def __init__(self, gripper):
        super().__init__("Check Grasp Success")
        self.gripper = gripper
        
    def update(self):
        if self.gripper.is_object_grasped():
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

# 接口类实现
class PerceptionInterface:
    """感知系统接口"""
    def __init__(self, node):
        self.node = node
        self.objects = {}  # 存储检测到的物体
        
        # 订阅点云
        self.pointcloud_sub = node.create_subscription(
            PointCloud2,
            '/camera/depth/points',
            self.pointcloud_callback,
            10
        )
        
    def pointcloud_callback(self, msg):
        # 实际应该进行物体检测和位姿估计
        # 这里使用模拟数据
        self.objects["red_cube"] = self.create_mock_pose(0.5, 0.0, 0.1)
        self.objects["blue_box"] = self.create_mock_pose(0.3, 0.3, 0.0)
        
    def get_object_pose(self, object_name):
        return self.objects.get(object_name, None)
    
    def create_mock_pose(self, x, y, z):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0
        return pose

class GraspPlannerInterface:
    """抓取规划接口"""
    def __init__(self, node):
        self.node = node
        
    def generate_grasps(self, object_pose):
        """生成抓取候选"""
        grasps = []
        
        # 生成多个抓取位姿（简化版）
        for i in range(5):
            grasp = Pose()
            grasp.position.x = object_pose.position.x
            grasp.position.y = object_pose.position.y
            grasp.position.z = object_pose.position.z + 0.05
            
            # 不同的抓取角度
            angle = i * np.pi / 5
            grasp.orientation.z = np.sin(angle / 2)
            grasp.orientation.w = np.cos(angle / 2)
            
            grasps.append(grasp)
            
        return grasps
    
    def select_best_grasp(self, grasps):
        """选择最佳抓取（这里简单返回第一个）"""
        if len(grasps) > 0:
            return grasps[0]
        return None

class MotionPlannerInterface:
    """运动规划接口"""
    def __init__(self, node):
        self.node = node
        self.current_goals = {}
        self.goal_counter = 0
        
    def move_to_pose(self, target_pose):
        """规划并执行到目标位姿的运动"""
        # 实际应该调用MoveIt
        # 这里返回一个模拟的goal handle
        self.goal_counter += 1
        goal_id = self.goal_counter
        self.current_goals[goal_id] = {
            'target': target_pose,
            'status': 'executing',
            'start_time': self.node.get_clock().now()
        }
        return goal_id
    
    def is_motion_complete(self, goal_handle):
        """检查运动是否完成"""
        if goal_handle in self.current_goals:
            # 模拟2秒完成运动
            elapsed = (self.node.get_clock().now() - 
                      self.current_goals[goal_handle]['start_time']).nanoseconds / 1e9
            if elapsed > 2.0:
                self.current_goals[goal_handle]['status'] = 'completed'
                return True
        return False
    
    def is_motion_failed(self, goal_handle):
        """检查运动是否失败"""
        return False  # 简化版总是成功

class GripperInterface:
    """抓手控制接口"""
    def __init__(self, node):
        self.node = node
        self.is_closed = False
        self.has_object = False
        
        self.gripper_pub = node.create_publisher(
            String, 
            '/gripper_command', 
            10
        )
        
    def open(self):
        """打开抓手"""
        msg = String()
        msg.data = "open"
        self.gripper_pub.publish(msg)
        self.is_closed = False
        self.has_object = False
        return True
    
    def close(self):
        """关闭抓手"""
        msg = String()
        msg.data = "close"
        self.gripper_pub.publish(msg)
        self.is_closed = True
        # 模拟抓取成功
        self.has_object = True
        return True
    
    def is_object_grasped(self):
        """检查是否抓住物体"""
        return self.has_object

# 工具函数
class LogMessage(py_trees.behaviour.Behaviour):
    """日志消息节点"""
    def __init__(self, message):
        super().__init__("Log")
        self.message = message
        
    def update(self):
        print(f"[BT Log] {self.message}")
        return py_trees.common.Status.SUCCESS

class MoveToGrasp(py_trees.behaviour.Behaviour):
    """移动到抓取位姿"""
    def __init__(self, motion_planner):
        super().__init__("Move to Grasp")
        self.motion_planner = motion_planner
        self.goal_handle = None
        
    def initialise(self):
        grasp = self.blackboard.get("selected_grasp")
        self.goal_handle = self.motion_planner.move_to_pose(grasp)
        
    def update(self):
        if self.motion_planner.is_motion_complete(self.goal_handle):
            return py_trees.common.Status.SUCCESS
        elif self.motion_planner.is_motion_failed(self.goal_handle):
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.RUNNING

class LiftObject(py_trees.behaviour.Behaviour):
    """提升物体"""
    def __init__(self, motion_planner):
        super().__init__("Lift Object")
        self.motion_planner = motion_planner
        self.goal_handle = None
        
    def initialise(self):
        current_pose = self.blackboard.get("selected_grasp")
        lift_pose = Pose()
        lift_pose.position.x = current_pose.position.x
        lift_pose.position.y = current_pose.position.y
        lift_pose.position.z = current_pose.position.z + 0.2  # 提升20cm
        lift_pose.orientation = current_pose.orientation
        self.goal_handle = self.motion_planner.move_to_pose(lift_pose)
        
    def update(self):
        if self.motion_planner.is_motion_complete(self.goal_handle):
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING

def main(args=None):
    rclpy.init(args=args)
    system = PickPlaceSystem()
    
    try:
        rclpy.spin(system)
    except KeyboardInterrupt:
        pass
    finally:
        system.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()