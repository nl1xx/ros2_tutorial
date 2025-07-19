import numpy as np

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
            
            # 添加摩擦锥约束（简化版）
            # 实际应该使用完整的摩擦锥模型
        
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
