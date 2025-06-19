import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
from scipy.spatial.transform import Rotation as R

# 假设 calculate_fk 函数已在别处定义或导入
def calculate_fk(joint_positions):
    # 提取关节角度
    theta1 = joint_positions[0]  # wmm_joint_1的角度
    theta2 = joint_positions[1]  # wmm_joint_2的角度
    
    # 计算各变换矩阵
    # 从base_link到wmm_joint_1的平移
    T1 = np.eye(4)
    T1[2, 3] = 0.1  # z方向平移0.1m
    
    # wmm_joint_1绕z轴的旋转
    Rz = R.from_euler('z', theta1).as_matrix()
    
    # 从link_1到wmm_joint_2的平移
    T2 = np.eye(4)
    T2[2, 3] = 0.5  # z方向平移0.5m
    
    # wmm_joint_2绕y轴的旋转
    Ry = R.from_euler('y', theta2).as_matrix()
    
    # 末端执行器固定平移
    T3 = np.eye(4)
    T3[2, 3] = 0.5  # z方向平移0.5m
    
    # 组合变换矩阵
    transform = T1 @ Rz @ T2 @ Ry @ T3
    
    return transform

# 辅助函数：将 4x4 矩阵转换为 TransformStamped 消息
def matrix_to_transform_stamped(matrix, stamp, frame_id, child_frame_id):
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = frame_id
    t.child_frame_id = child_frame_id

    t.transform.translation.x = float(matrix[0, 3])
    t.transform.translation.y = float(matrix[1, 3])
    t.transform.translation.z = float(matrix[2, 3])

    rotation_matrix = matrix[0:3, 0:3]
    quat = R.from_matrix(rotation_matrix).as_quat()
    t.transform.rotation.x = float(quat[0])
    t.transform.rotation.y = float(quat[1])
    t.transform.rotation.z = float(quat[2])
    t.transform.rotation.w = float(quat[3])

    return t

class FKCalculatorNode(Node):
    def __init__(self):
        super().__init__('fk_calculator_node')
        self.set_parameters([{'use_sim_time': True}])  # 使用仿真时间

        self.joint_names = ['wmm_joint_1', 'wmm_joint_2']  # 替换为你的关节名称
        self.joint_positions = [0.0] * len(self.joint_names)
        self.last_joint_state_time = None

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10  # QoS profile depth
        )

        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.02, self.publish_fk_tf)  # 定时发布 TF (例如 50Hz)

        self.get_logger().info('FK Calculator Node Started')

    def joint_state_callback(self, msg):
        try:
            current_positions = []
            name_to_pos = dict(zip(msg.name, msg.position))
            for name in self.joint_names:
                if name in name_to_pos:
                    current_positions.append(name_to_pos[name])
                else:
                    self.get_logger().warn(f"Joint '{name}' not found in /joint_states message.")
                    return

            if len(current_positions) == len(self.joint_names):
                self.joint_positions = current_positions
                self.last_joint_state_time = msg.header.stamp
            else:
                self.get_logger().error("Could not extract all required joint positions from /joint_states.")

        except Exception as e:
            self.get_logger().error(f"Error processing joint_state message: {e}")

    def publish_fk_tf(self):
        if self.last_joint_state_time is None:
            return

        try:
            pose_matrix = calculate_fk(self.joint_positions)

            stamp = self.get_clock().now().to_msg()  # 使用当前仿真时间

            transform_stamped = matrix_to_transform_stamped(
                pose_matrix,
                stamp,
                'base_link',  # 替换为你的基座坐标系名称
                'fk_calculated_end_effector'  # 自定义的目标 TF 帧名称
            )

            self.tf_broadcaster.sendTransform(transform_stamped)

        except Exception as e:
            self.get_logger().error(f'Error calculating/publishing FK TF: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = FKCalculatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()