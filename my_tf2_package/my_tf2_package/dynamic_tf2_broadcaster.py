# Simplified Pseudocode Structure
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
# import tf_transformations
from scipy.spatial.transform import Rotation as R
import numpy as np
import math

class DynamicFramePublisher(Node):
    def __init__(self):
        super().__init__('dynamic_tf2_broadcaster')
        # 初始化动态变换发布器
        self.tf_broadcaster = TransformBroadcaster(self)
        # 创建一个定时器，以指定频率调用回调函数
        self.timer = self.create_timer(0.1, self.broadcast_timer_callback) # 10 Hz
        self.x_pos = 0.0 # 用于模拟移动的状态变量

    def broadcast_timer_callback(self):
        # 创建 TransformStamped 消息
        t = TransformStamped()

        # 填充消息头
        t.header.stamp = self.get_clock().now().to_msg() # 每次发布使用当前时间
        t.header.frame_id = 'odom'      # 父坐标系 (里程计)
        t.child_frame_id = 'base_link' # 子坐标系 (机器人基座)

        # 模拟运动 (例如，沿 X 轴缓慢前进)
        self.x_pos += 0.01 # 更新位置
        t.transform.translation.x = self.x_pos
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        # 模拟微小旋转 (可选, e.g., 绕 Z 轴轻微摆动)
        current_time_sec = self.get_clock().now().nanoseconds / 1e9
        angle_z = 0.1 * math.sin(current_time_sec)
        rotation = R.from_euler('z', angle_z, degrees=False)
        q = rotation.as_quat()
        t.transform.rotation.x = float(q[0])
        t.transform.rotation.y = float(q[1])
        t.transform.rotation.z = float(q[2])
        t.transform.rotation.w = float(q[3])

        # 发送动态变换
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = DynamicFramePublisher()
    try:
        rclpy.spin(node) # 持续运行以周期性发布变换
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

# (C++ 版本类似，使用 tf2_ros::TransformBroadcaster 类)
