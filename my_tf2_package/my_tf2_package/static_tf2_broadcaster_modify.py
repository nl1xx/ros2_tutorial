# Simplified Pseudocode Structure
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
# import tf_transformations # Or: from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Rotation as R
import numpy as np

class StaticFramePublisher(Node):
    def __init__(self):
        super().__init__('static_tf2_broadcaster')
        # 初始化静态变换发布器
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        # 调用函数发布变换
        self.make_transforms()

    def make_transforms(self):
        # 创建 TransformStamped 消息
        t = TransformStamped()

        # 填充消息头
        t.header.stamp = self.get_clock().now().to_msg() # 获取当前时间
        t.header.frame_id = 'base_link'  # 父坐标系
        t.child_frame_id = 'lidar_link' # 子坐标系

        # 设置平移 (例如，激光雷达在基座前方0.2m，上方0.2m)
        t.transform.translation.x = 0.2
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.2

        # 设置旋转 (例如， 绕z轴选择2弧度)
        # 使用 scipy.spatial.transform 从欧拉角创建四元数
        rotation = R.from_euler('xyz', [0, 0, 2], degrees=False)
        q = rotation.as_quat() # 返回 [x, y, z, w] 顺序
        t.transform.rotation.x = float(q[0])
        t.transform.rotation.y = float(q[1])
        t.transform.rotation.z = float(q[2])
        t.transform.rotation.w = float(q[3]) # 注意 w 是最后一个

        # 发送静态变换
        self.tf_static_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = StaticFramePublisher()
    try:
        rclpy.spin(node) # 静态变换发布一次即可，但保持节点运行以供查询
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

# (C++ 版本类似，使用 tf2_ros::StaticTransformBroadcaster 类)