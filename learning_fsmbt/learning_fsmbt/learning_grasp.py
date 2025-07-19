import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Quaternion
import tf2_ros
import numpy as np
from scipy.spatial.transform import Rotation

class SimpleGraspGenerator:
    """简单的抓取位姿生成器"""
    
    def __init__(self, object_dimensions):
        """
        Args:
            object_dimensions: [length, width, height]
        """
        self.dimensions = object_dimensions
    
    def generate_top_grasps(self, n_grasps=10):
        """生成从上方抓取的位姿"""
        grasps = []
        
        # 在物体顶部均匀采样抓取点
        for i in range(n_grasps):
            # 随机选择抓取位置
            x = np.random.uniform(-self.dimensions[0]/2, self.dimensions[0]/2)
            y = np.random.uniform(-self.dimensions[1]/2, self.dimensions[1]/2)
            z = self.dimensions[2]/2 + 0.1  # 物体顶部上方10cm
            
            # 抓取姿态（从上往下）
            grasp_pose = {
                'position': [x, y, z],
                'orientation': [0, np.pi, 0],  # 欧拉角
                'width': min(self.dimensions[0], self.dimensions[1]) * 0.8
            }
            
            grasps.append(grasp_pose)
        
        return grasps
    
    def generate_side_grasps(self, n_grasps=10):
        """生成从侧面抓取的位姿"""
        grasps = []
        
        for i in range(n_grasps):
            # 选择抓取的边
            angle = i * 2 * np.pi / n_grasps
            
            # 计算抓取位置
            x = (self.dimensions[0]/2 + 0.1) * np.cos(angle)
            y = (self.dimensions[1]/2 + 0.1) * np.sin(angle)
            z = self.dimensions[2]/2
            
            # 抓取姿态（指向物体中心）
            grasp_pose = {
                'position': [x, y, z],
                'orientation': [0, np.pi/2, angle],
                'width': self.dimensions[2] * 0.8
            }
            
            grasps.append(grasp_pose)
        
        return grasps

class GraspVisualizer(Node):
    def __init__(self):
        super().__init__('grasp_visualizer')
        
        self.marker_pub = self.create_publisher(
            MarkerArray, 
            '/grasp_markers', 
            10
        )
        
        # 创建示例物体和抓取位姿
        self.object_dims = [0.1, 0.08, 0.15]  # 10x8x15cm的盒子
        self.grasp_generator = SimpleGraspGenerator(self.object_dims)
        
        # 定时发布可视化
        self.timer = self.create_timer(1.0, self.publish_visualization)
        
    def publish_visualization(self):
        marker_array = MarkerArray()
        
        # 1. 可视化物体（立方体）
        object_marker = Marker()
        object_marker.header.frame_id = "base_link"
        object_marker.header.stamp = self.get_clock().now().to_msg()
        object_marker.ns = "object"
        object_marker.id = 0
        object_marker.type = Marker.CUBE
        object_marker.action = Marker.ADD
        
        object_marker.pose.position.x = 0.5  # 物体在base_link前方0.5m
        object_marker.pose.position.y = 0.0
        object_marker.pose.position.z = self.object_dims[2]/2
        object_marker.pose.orientation.w = 1.0
        
        object_marker.scale.x = self.object_dims[0]
        object_marker.scale.y = self.object_dims[1]
        object_marker.scale.z = self.object_dims[2]
        
        object_marker.color.r = 0.5
        object_marker.color.g = 0.5
        object_marker.color.b = 0.5
        object_marker.color.a = 0.8
        
        marker_array.markers.append(object_marker)
        
        # 2. 可视化抓取位姿
        grasps = self.grasp_generator.generate_top_grasps(5)
        grasps += self.grasp_generator.generate_side_grasps(8)
        
        for i, grasp in enumerate(grasps):
            # 创建坐标系标记
            frame_marker = Marker()
            frame_marker.header.frame_id = "base_link"
            frame_marker.header.stamp = self.get_clock().now().to_msg()
            frame_marker.ns = "grasp_frames"
            frame_marker.id = i + 1
            frame_marker.type = Marker.ARROW
            frame_marker.action = Marker.ADD
            
            # 设置位置（相对于物体中心）
            frame_marker.pose.position.x = 0.5 + grasp['position'][0]
            frame_marker.pose.position.y = grasp['position'][1]
            frame_marker.pose.position.z = grasp['position'][2]
            
            # 设置姿态
            r = Rotation.from_euler('xyz', grasp['orientation'])
            q = r.as_quat()
            frame_marker.pose.orientation.x = q[0]
            frame_marker.pose.orientation.y = q[1]
            frame_marker.pose.orientation.z = q[2]
            frame_marker.pose.orientation.w = q[3]
            
            # 箭头表示抓取方向
            frame_marker.scale.x = 0.1  # 长度
            frame_marker.scale.y = 0.01  # 宽度
            frame_marker.scale.z = 0.01  # 高度
            
            # 根据质量评分设置颜色（这里用随机值模拟）
            quality = np.random.rand()
            frame_marker.color.r = 1.0 - quality
            frame_marker.color.g = quality
            frame_marker.color.b = 0.0
            frame_marker.color.a = 0.8
            
            marker_array.markers.append(frame_marker)
            
            # 添加抓手宽度可视化（两个小立方体表示手指）
            for j, side in enumerate([-1, 1]):
                finger_marker = Marker()
                finger_marker.header = frame_marker.header
                finger_marker.ns = "grasp_fingers"
                finger_marker.id = i * 10 + j
                finger_marker.type = Marker.CUBE
                finger_marker.action = Marker.ADD
                
                # 手指位置
                finger_marker.pose = frame_marker.pose
                finger_offset = side * grasp['width'] / 2
                # 根据抓取方向计算手指偏移
                # 这里简化处理，实际应该根据抓取姿态计算
                finger_marker.pose.position.y += finger_offset * np.cos(grasp['orientation'][2])
                finger_marker.pose.position.x += finger_offset * np.sin(grasp['orientation'][2])
                
                finger_marker.scale.x = 0.02
                finger_marker.scale.y = 0.02
                finger_marker.scale.z = 0.05
                
                finger_marker.color = frame_marker.color
                
                marker_array.markers.append(finger_marker)
        
        self.marker_pub.publish(marker_array)
        self.get_logger().info(f'Published {len(grasps)} grasp poses')

def main(args=None):
    rclpy.init(args=args)
    visualizer = GraspVisualizer()
    rclpy.spin(visualizer)
    rclpy.shutdown()

if __name__ == '__main__':
    main()