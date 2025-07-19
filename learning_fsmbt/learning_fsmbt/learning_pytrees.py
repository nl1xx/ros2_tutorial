#!/usr/bin/env python3
import py_trees
import rclpy
from rclpy.node import Node
import time

class CheckObjectDetected(py_trees.behaviour.Behaviour):
    """条件节点：检查是否检测到物体"""
    def __init__(self, name="Check Object"):
        super().__init__(name)
        
    def update(self):
        # 模拟检测结果
        detected = True  # 实际应该从感知系统获取
        if detected:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

class MoveToObject(py_trees.behaviour.Behaviour):
    """动作节点：移动到物体"""
    def __init__(self, name="Move to Object"):
        super().__init__(name)
        self.start_time = None
        
    def initialise(self):
        self.start_time = time.time()
        print(f"开始移动到物体...")
        
    def update(self):
        # 模拟移动过程（实际应调用导航系统）
        elapsed = time.time() - self.start_time
        if elapsed < 2.0:  # 假设需要2秒
            return py_trees.common.Status.RUNNING
        else:
            print("到达物体位置")
            return py_trees.common.Status.SUCCESS

class GraspObject(py_trees.behaviour.Behaviour):
    """动作节点：抓取物体"""
    def __init__(self, name="Grasp Object"):
        super().__init__(name)
        
    def update(self):
        print("执行抓取...")
        # 实际应该发送抓手控制命令
        time.sleep(1)
        print("抓取成功")
        return py_trees.common.Status.SUCCESS

class MoveToDestination(py_trees.behaviour.Behaviour):
    """动作节点：移动到目标位置"""
    def __init__(self, name="Move to Destination"):
        super().__init__(name)
        self.start_time = None
        
    def initialise(self):
        self.start_time = time.time()
        print("开始移动到目标位置...")
        
    def update(self):
        elapsed = time.time() - self.start_time
        if elapsed < 2.0:
            return py_trees.common.Status.RUNNING
        else:
            print("到达目标位置")
            return py_trees.common.Status.SUCCESS

class PlaceObject(py_trees.behaviour.Behaviour):
    """动作节点：放置物体"""
    def __init__(self, name="Place Object"):
        super().__init__(name)
        
    def update(self):
        print("执行放置...")
        time.sleep(1)
        print("放置成功")
        return py_trees.common.Status.SUCCESS

def create_pick_place_tree():
    """创建抓取-放置行为树"""
    
    # 创建根节点（序列）
    root = py_trees.composites.Sequence("Pick and Place", memory=True)
    
    # 创建抓取子树
    grasp_sequence = py_trees.composites.Sequence("Grasp Sequence", memory=True)
    grasp_sequence.add_children([
        CheckObjectDetected(),
        MoveToObject(),
        GraspObject()
    ])
    
    # 创建放置子树
    place_sequence = py_trees.composites.Sequence("Place Sequence", memory=True)
    place_sequence.add_children([
        MoveToDestination(),
        PlaceObject()
    ])
    
    # 组装完整的树
    root.add_children([grasp_sequence, place_sequence])
    
    return root

def print_tree(root, indent="", is_last=False):
    """递归打印行为树的 ASCII 表示"""
    prefix = indent + ("└─ " if is_last else "├─ ")
    print(f"{prefix}{root.name} [{type(root).__name__}]")
    
    # 如果是复合节点，递归打印子节点
    if isinstance(root, py_trees.composites.Composite):
        for i, child in enumerate(root.children):
            is_last_child = i == len(root.children) - 1
            new_indent = indent + ("    " if is_last else "│   ")
            print_tree(child, new_indent, is_last_child)

def main():
    # 创建行为树
    tree = create_pick_place_tree()
    
    # 可视化树结构
    print("\n行为树结构:")
    print_tree(tree)
    
    # 创建行为树执行器
    behaviour_tree = py_trees.trees.BehaviourTree(tree)
    
    # 执行行为树
    print("\n开始执行任务...")
    behaviour_tree.setup(timeout=15)
    
    # 持续tick直到完成
    while True:
        behaviour_tree.tick()
        if tree.status == py_trees.common.Status.SUCCESS:
            print("\n任务完成!")
            break
        elif tree.status == py_trees.common.Status.FAILURE:
            print("\n任务失败!")
            break
        time.sleep(0.5)

if __name__ == '__main__':
    main()