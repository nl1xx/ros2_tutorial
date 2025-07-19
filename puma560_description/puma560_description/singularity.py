#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import roboticstoolbox as rtb # 假设使用 roboticstoolbox (Assume using roboticstoolbox)
import os # 用于检查文件路径 (For checking file paths)

# --- 配置区域 (Configuration Area) ---
WMM_URDF_PATH = '/home/promise/ros2_ws/src/puma560_description/urdf/puma560_robot.urdf' 
# 关节名称和顺序 (Joint names and order - 必须与 URDF 和 /joint_states 中的顺序一致)
WMM_JOINT_NAMES = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6'] 
# ------------------------------------

class SingularityMonitor(Node):
    def __init__(self):
        super().__init__('singularity_monitor')

        # 加载机器人模型 (Load robot model)
        self.robot = self.load_robot_model()
        if self.robot is None:
            self.get_logger().error("Failed to load robot model. Shutting down.")
            rclpy.shutdown() # 无法加载模型则退出 (Exit if model cannot be loaded)
            return

        self.joint_names = WMM_JOINT_NAMES
        self.last_q = None # 用于避免重复计算 (To avoid redundant calculations)

        # 订阅关节状态话题 (Subscribe to joint state topic)
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states', # 确认 WMM 的关节状态话题名 (Confirm WMM's joint state topic name)
            self.joint_state_callback,
            10) # QoS profile depth
        self.get_logger().info('Singularity Monitor Node Started and Subscribed to /joint_states')

    def load_robot_model(self):
        """从 URDF 加载机器人模型"""
        """Load robot model from URDF"""
        if not os.path.exists(WMM_URDF_PATH):
            self.get_logger().error(f"URDF file not found at: {WMM_URDF_PATH}")
            return None
        try:
            # 使用 roboticstoolbox 加载 URDF
            # Use roboticstoolbox to load URDF
            # 注意：rtb.models.URDF.WMM() 是一个示例调用，实际加载方式可能不同
            # Note: rtb.models.URDF.WMM() is an example call, actual loading might differ
            # 更通用的加载方式可能是:
            # A more general way to load might be:
            robot_model = rtb.ERobot.URDF(WMM_URDF_PATH) # ERobot can load URDF
            # 或者如果 WMM 是一个预定义的模型类:
            # Or if WMM is a predefined model class:
            # robot_model = rtb.models.URDF.WMM() 

            # 验证关节数量是否匹配 (Verify if the number of joints matches)
            if robot_model.n!= len(WMM_JOINT_NAMES):
                 self.get_logger().error(f"Mismatch in joint count: Model has {robot_model.n}, expected {len(WMM_JOINT_NAMES)}")
                 return None
            self.get_logger().info(f"Robot model '{robot_model.name}' loaded successfully with {robot_model.n} joints.")
            return robot_model
        except Exception as e:
            self.get_logger().error(f"Failed to load robot model from URDF: {e}")
            return None

    def joint_state_callback(self, msg: JointState):
        """处理接收到的关节状态消息"""
        """Process received joint state messages"""
        if self.robot is None:
            return # 如果模型未加载，则不处理 (Do not process if model is not loaded)

        # 检查消息是否包含所有需要的关节名 (Check if the message contains all required joint names)
        if not all(name in msg.name for name in self.joint_names):
            # self.get_logger().warn("JointState message does not contain all required joint names yet.", throttle_duration_sec=5)
            return

        # 按预定顺序提取关节角度 (Extract joint angles in the predefined order)
        try:
            q = np.array([msg.position[msg.name.index(j_name)] for j_name in self.joint_names])
        except ValueError as e:
            self.get_logger().error(f"Error extracting joint positions: {e}")
            return

        # 检查 q 是否与上次相同，避免不必要的计算 (Check if q is the same as last time to avoid unnecessary computation)
        if self.last_q is not None and np.allclose(q, self.last_q, atol=1e-4):
            return
        self.last_q = q

        # 计算雅可比矩阵 (Calculate Jacobian matrix)
        try:
            # 使用 roboticstoolbox 计算空间雅可比 J0
            # Calculate spatial Jacobian J0 using roboticstoolbox
            J = self.robot.jacob0(q) 
        except Exception as e:
            self.get_logger().warn(f"Could not calculate Jacobian for q={q}: {e}", throttle_duration_sec=5)
            return

        if J is not None and J.shape == (6, self.robot.n):
            # 计算奇异性指标 (Calculate singularity metrics)

            # 1. 条件数 (Condition Number - 适用于所有矩阵)
            #    (Condition Number - applicable to all matrices)
            cond_J = np.linalg.cond(J)
            self.get_logger().info(f'Condition Number(J): {cond_J:.4e}')

            # 2. 行列式 (Determinant - 仅适用于方阵)
            #    (Determinant - only applicable to square matrices)
            if J.shape == J.shape[1]:
                det_J = np.linalg.det(J)
                # 使用科学计数法或指定小数位数以处理非常小的值
                # Use scientific notation or specify decimal places for very small values
                self.get_logger().info(f'Determinant(J): {det_J:.4e}') 

            # 3. (可选) 最小奇异值 (Optional: Minimum Singular Value)
            #    (Useful as it approaches zero near singularity)
            try:
                U, s, Vh = np.linalg.svd(J)
                min_sv = np.min(s) if len(s) > 0 else 0.0
                self.get_logger().info(f'Min Singular Value: {min_sv:.4e}')
            except np.linalg.LinAlgError:
                 self.get_logger().warn("SVD computation failed.", throttle_duration_sec=5)

def main(args=None):
    rclpy.init(args=args)
    node = SingularityMonitor()
    # 只有在模型成功加载后才 spin (Only spin if the model was loaded successfully)
    if node.robot is not None:
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        except Exception as e:
            node.get_logger().fatal(f"Unhandled exception in spin: {e}")
        finally:
            # Cleanup
            node.destroy_node()
            rclpy.shutdown()
    else:
         node.get_logger().info("Node shutting down due to model loading failure.")


if __name__ == '__main__':
    main()