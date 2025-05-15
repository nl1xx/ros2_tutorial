import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
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
        # 创建一个列表来存储所有静态变换
        transforms = []

        # 发布第一个静态变换：从 base_link 到 lidar_link
        transforms.append(self.create_transform('base_link', 'lidar_link', [0.1, 0.0, 0.2], [0, 0, 0]))

        # 发布第二个静态变换：从 base_link 到 camera_link
        # 假设相机安装在 base_link 的前方 0.3m，上方 0.1m，且绕 x 轴旋转 45 度
        transforms.append(self.create_transform('base_link', 'camera_link', [0.3, 0.0, 0.1], [np.pi/4, 0, 0]))

        # 发送所有静态变换
        self.tf_static_broadcaster.sendTransform(transforms)

    def create_transform(self, parent_frame, child_frame, translation, rotation):
        # 创建 TransformStamped 消息
        t = TransformStamped()

        # 填充消息头
        t.header.stamp = self.get_clock().now().to_msg()  # 获取当前时间
        t.header.frame_id = parent_frame  # 父坐标系
        t.child_frame_id = child_frame  # 子坐标系

        # 设置平移
        t.transform.translation.x = translation[0]
        t.transform.translation.y = translation[1]
        t.transform.translation.z = translation[2]

        # 设置旋转
        # 使用 scipy.spatial.transform 从欧拉角创建四元数
        rotation = R.from_euler('xyz', rotation, degrees=False)
        q = rotation.as_quat()  # 返回 [x, y, z, w] 顺序
        t.transform.rotation.x = float(q[0])
        t.transform.rotation.y = float(q[1])
        t.transform.rotation.z = float(q[2])
        t.transform.rotation.w = float(q[3])  # 注意 w 是最后一个

        return t

def main(args=None):
    rclpy.init(args=args)
    node = StaticFramePublisher()
    try:
        rclpy.spin(node)  # 静态变换发布一次即可，但保持节点运行以供查询
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
