import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# 抓取质量度量工具类
class GraspQualityMetrics:
    """抓取质量度量工具类"""
    
    @staticmethod
    def force_closure_check(contact_points, contact_normals, friction_coeff=0.5):
        """
        检查力闭合条件
        
        Args:
            contact_points: 接触点位置 (n, 3)
            contact_normals: 接触点法向量 (n, 3)
            friction_coeff: 摩擦系数
            
        Returns:
            bool: 是否满足力闭合
        """
        # 构建抓取矩阵
        n_contacts = len(contact_points)
        G = np.zeros((6, 3 * n_contacts))  # 6DOF, 每个接触点3个力分量
        
        for i in range(n_contacts):
            # 法向力贡献
            G[0:3, 3*i] = contact_normals[i]
            # 力矩贡献
            G[3:6, 3*i] = np.cross(contact_points[i], contact_normals[i])
            
        # 检查矩阵秩
        rank = np.linalg.matrix_rank(G)
        return rank == 6  # 满秩表示力闭合
    
    @staticmethod
    def epsilon_metric(grasp_matrix, friction_cone):
        """
        计算epsilon质量度量
        
        epsilon表示能抵抗的最小扰动力
        """
        # 这里是简化实现
        # 实际需要求解优化问题
        return 0.1  # 示例返回值

# 简单的抓取位姿生成器
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

# 可视化抓取质量度量工具类
class GraspVisualizer:
    """抓取可视化工具类"""
    
    @staticmethod
    def visualize_grasp(contact_points, contact_normals, object_dims, is_closed):
        """
        可视化抓取
        
        Args:
            contact_points: 接触点位置 (n, 3)
            contact_normals: 接触点法向量 (n, 3)
            object_dims: 物体尺寸 [length, width, height]
            is_closed: 是否满足力闭合
        """
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        # 绘制物体（简化为立方体）
        l, w, h = object_dims
        GraspVisualizer._plot_cube(ax, l, w, h)
        
        # 绘制接触点
        for i, (point, normal) in enumerate(zip(contact_points, contact_normals)):
            ax.scatter(point[0], point[1], point[2], color='r', s=100)
            ax.quiver(point[0], point[1], point[2], 
                     normal[0], normal[1], normal[2], 
                     color='b', length=0.05, arrow_length_ratio=0.3)
        
        # 设置显示效果
        ax.set_xlim(-l/2 - 0.1, l/2 + 0.1)
        ax.set_ylim(-w/2 - 0.1, w/2 + 0.1)
        ax.set_zlim(0, h + 0.1)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title(f'Grasp Visualization (Force Closure: {is_closed})')
        
        plt.show()
    
    @staticmethod
    def _plot_cube(ax, l, w, h):
        """绘制立方体"""
        # 定义立方体顶点
        vertices = np.array([
            [-l/2, -w/2, 0],  # 左下前
            [l/2, -w/2, 0],   # 右下前
            [l/2, w/2, 0],    # 右下后
            [-l/2, w/2, 0],   # 左下后
            [-l/2, -w/2, h],  # 左上前
            [l/2, -w/2, h],   # 右上前
            [l/2, w/2, h],    # 右上后
            [-l/2, w/2, h]    # 左上后
        ])
        
        # 定义立方体面
        faces = [
            [0, 1, 2, 3],  # 底面
            [4, 5, 6, 7],  # 顶面
            [0, 1, 5, 4],  # 前面
            [1, 2, 6, 5],  # 右面
            [2, 3, 7, 6],  # 后面
            [3, 0, 4, 7]   # 左面
        ]
        
        # 绘制立方体
        for face in faces:
            ax.add_collection3d(Poly3DCollection([vertices[face]], 
                                               facecolors='cyan', 
                                               linewidths=1, 
                                               edgecolors='r', 
                                               alpha=.25))
    
    @staticmethod
    def visualize_grasp_poses(grasp_poses, object_dims):
        """
        可视化多个抓取位姿
        
        Args:
            grasp_poses: 抓取位姿列表
            object_dims: 物体尺寸 [length, width, height]
        """
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        # 绘制物体
        l, w, h = object_dims
        GraspVisualizer._plot_cube(ax, l, w, h)
        
        # 绘制抓取位姿
        for i, grasp in enumerate(grasp_poses):
            pos = grasp['position']
            ax.scatter(pos[0], pos[1], pos[2], color='g', s=100)
            ax.text(pos[0], pos[1], pos[2], f'G{i}', color='g')
        
        # 设置显示效果
        ax.set_xlim(-l/2 - 0.1, l/2 + 0.1)
        ax.set_ylim(-w/2 - 0.1, w/2 + 0.1)
        ax.set_zlim(0, h + 0.1)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('Grasp Poses Visualization')
        
        plt.show()

if __name__ == "__main__":
    # 示例物体尺寸（长、宽、高）
    object_dims = [0.2, 0.2, 0.1]  # 例如一个扁平的盒子
    
    # 创建抓取生成器
    generator = SimpleGraspGenerator(object_dims)
    
    # 生成顶部抓取
    top_grasps = generator.generate_top_grasps(n_grasps=5)
    print("Top grasps:")
    for grasp in top_grasps:
        print(f"Position: {grasp['position']}, Orientation: {grasp['orientation']}, Width: {grasp['width']}")
    
    # 生成侧面抓取
    side_grasps = generator.generate_side_grasps(n_grasps=5)
    print("\nSide grasps:")
    for grasp in side_grasps:
        print(f"Position: {grasp['position']}, Orientation: {grasp['orientation']}, Width: {grasp['width']}")
    
    # 模拟接触点和法向量（例如4个接触点）
    contact_points = np.array([
        [-0.05, -0.05, 0.05],  # 左前
        [0.05, -0.05, 0.05],   # 右前
        [-0.05, 0.05, 0.05],   # 左后
        [0.05, 0.05, 0.05]     # 右后
    ])
    
    contact_normals = np.array([
        [1, 0, 0],  # 指向右
        [-1, 0, 0], # 指向左
        [0, 1, 0],  # 指向上
        [0, -1, 0]  # 指向下
    ])
    
    # 检查力闭合
    is_closed = GraspQualityMetrics.force_closure_check(contact_points, contact_normals)
    print("\nForce closure check result:", is_closed)
    
    # 计算epsilon质量度量
    # 这里需要一个完整的抓取矩阵和摩擦锥模型
    # 示例中简化处理
    print("\nEpsilon quality metric:", GraspQualityMetrics.epsilon_metric(None, None))
    
    # 可视化抓取
    GraspVisualizer.visualize_grasp(contact_points, contact_normals, object_dims, is_closed)
    
    # 可视化抓取位姿
    GraspVisualizer.visualize_grasp_poses(top_grasps + side_grasps, object_dims)