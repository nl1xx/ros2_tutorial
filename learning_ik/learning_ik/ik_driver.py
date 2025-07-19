import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import JointState
import numpy as np
from scipy.spatial.transform import Rotation

# 导入 Action 和相关消息
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance  # 修正：独立导入JointTolerance
from trajectory_msgs.msg import JointTrajectoryPoint
from rclpy.duration import Duration
import time

from roboticstoolbox import DHRobot, RevoluteDH
import spatialmath as sm

class Puma560IkDriver(Node):
    def __init__(self):
        super().__init__('puma560_ik_driver')
        self.get_logger().info('PUMA560 IK Driver Node started.')

        # PUMA560 关节名称
        self.joint_names = [
            'j1', 
            'j2', 
            'j3',
            'j4', 
            'j5', 
            'j6'
        ]
        self.num_joints = len(self.joint_names)
        
        # 创建 PUMA560 机器人模型
        self.robot = self.create_puma560_model()

        # 订阅目标位姿
        self.target_pose_sub = self.create_subscription(
            PoseStamped,
            '/target_pose',
            self.target_pose_callback,
            10)

        # 订阅当前关节状态
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
        self.current_joint_state = None
        self.joint_map = {}

        # FollowJointTrajectory Action Client
        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory')

        # IK 参数
        self.ik_max_iterations = 200
        self.ik_tolerance = 1e-4
        self.ik_gain = 0.2

        self.get_logger().info(f'Controlling PUMA560 joints: {self.joint_names}')
        self.get_logger().info('Waiting for Action Server...')
        if not self.trajectory_client.wait_for_server(timeout_sec=5.0):
             self.get_logger().error('Action Server not available!')
             rclpy.shutdown()
             return
        self.get_logger().info('Action Server found.')

    def create_puma560_model(self):
        """创建 PUMA560 机器人模型"""
        # PUMA560 DH 参数 (单位: 米)
        d = [0, 0.2435, -0.0934, 0, 0, 0]
        a = [0, 0.4318, -0.0203, 0, 0, 0]
        alpha = [np.pi/2, 0, -np.pi/2, np.pi/2, -np.pi/2, 0]
        
        links = []
        for j in range(self.num_joints):
            links.append(RevoluteDH(d=d[j], a=a[j], alpha=alpha[j]))
        
        robot = DHRobot(links, name='PUMA560')
        
        # 设置关节限位
        qlim_lower = [-2.7925, -3.927, -0.7854, -3.1416, -1.9199, -3.1416]
        qlim_upper = [2.7925, 0.7854, 3.927, 3.1416, 1.9199, 3.1416]
        robot.qlim = np.array([qlim_lower, qlim_upper])
        
        return robot

    def calculate_fk(self, q):
        """计算正向运动学"""
        T = self.robot.fkine(q)
        return T.A

    def calculate_jacobian(self, q):
        """计算雅可比矩阵"""
        J = self.robot.jacob0(q)
        return J

    def calculate_pose_error(self, T_desired, T_current):
        """计算位姿误差"""
        # 位置误差
        pos_error = T_desired[:3, 3] - T_current[:3, 3]
        
        # 方向误差
        R_desired = T_desired[:3, :3]
        R_current = T_current[:3, :3]
        R_error = R_desired @ R_current.T
        r = Rotation.from_matrix(R_error)
        rot_vec = r.as_rotvec()
        
        return np.concatenate((pos_error, rot_vec))

    def joint_state_callback(self, msg: JointState):
        # 更新当前关节状态
        if self.current_joint_state is None:
            self.current_joint_state = np.zeros(self.num_joints)
            self.joint_map = {name: i for i, name in enumerate(msg.name)}
            
            for name in self.joint_names:
                if name not in self.joint_map:
                    self.get_logger().error(f"Joint '{name}' not found in /joint_states message!")
                    return

        try:
            for i, name in enumerate(self.joint_names):
                idx_in_msg = self.joint_map[name]
                self.current_joint_state[i] = msg.position[idx_in_msg]
        except (IndexError, KeyError):
            self.get_logger().warn("Error reading joint states.")

    def pose_stamped_to_matrix(self, pose: Pose):
        """将Pose转换为4x4矩阵"""
        position = np.array([pose.position.x, pose.position.y, pose.position.z])
        orientation_q = np.array([pose.orientation.x, pose.orientation.y, 
                                 pose.orientation.z, pose.orientation.w])
        rotation_matrix = Rotation.from_quat(orientation_q).as_matrix()

        T = np.identity(4)
        T[:3, :3] = rotation_matrix
        T[:3, 3] = position
        return T

    def target_pose_callback(self, msg: PoseStamped):
        self.get_logger().info('Received target pose request.')

        if self.current_joint_state is None:
            self.get_logger().warn('Current joint state not yet received. Waiting...')
            timeout = 5.0
            start_time = time.time()
            while self.current_joint_state is None and (time.time() - start_time) < timeout:
                rclpy.spin_once(self, timeout_sec=0.1)
            if self.current_joint_state is None:
                self.get_logger().error("Failed to get initial joint states. Aborting IK.")
                return

        T_desired = self.pose_stamped_to_matrix(msg.pose)
        q_initial = self.current_joint_state.copy()

        # 调用 IK 求解器
        q_solution, success = self.solve_ik_pseudoinverse(
            T_desired, q_initial,
            self.ik_max_iterations, self.ik_tolerance, self.ik_gain
        )

        if success:
            self.get_logger().info(f'IK solution found: {q_solution}')
            self.send_trajectory_goal(q_solution)
        else:
            self.get_logger().warn('IK failed to find a solution for the target pose.')

    def solve_ik_pseudoinverse(self, T_desired, q_initial, max_iterations, tolerance, gain):
        """使用伪逆法求解逆运动学"""
        q = q_initial.copy().astype(float)
        
        for i in range(max_iterations):
            T_current = self.calculate_fk(q)
            error = self.calculate_pose_error(T_desired, T_current)
            error_norm = np.linalg.norm(error)
            
            if error_norm < tolerance:
                self.get_logger().info(f'IK converged after {i} iterations with error: {error_norm}')
                return q, True

            J = self.calculate_jacobian(q)
            
            if J.shape[1] != self.num_joints:
                self.get_logger().error(f"Jacobian shape mismatch: {J.shape} vs {self.num_joints}")
                return q, False

            try:
                # 使用阻尼最小二乘法 (DLS)
                lambda_val = 0.01
                J_pinv = np.linalg.pinv(J.T @ J + lambda_val**2 * np.eye(J.shape[1])) @ J.T
                dq = J_pinv @ error * gain
                q = q + dq
                
                # 应用关节限位
                for j in range(self.num_joints):
                    q[j] = np.clip(q[j], self.robot.qlim[0][j], self.robot.qlim[1][j])
                
                if i % 20 == 0:
                    self.get_logger().debug(f'IK iter {i}: error={error_norm:.6f}, q={q}')
            except np.linalg.LinAlgError:
                self.get_logger().warn('Singularity encountered in pseudo-inverse calculation')
                return q, False
        
        self.get_logger().warn(f'IK failed to converge after {max_iterations} iterations')
        return q, False

    def send_trajectory_goal(self, q_target, time_to_reach_sec=5.0):
        """发送单点 FollowJointTrajectory Goal"""
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = q_target.tolist()
        point.velocities = [0.0] * self.num_joints
        point.accelerations = [0.0] * self.num_joints
        duration = Duration(seconds=float(time_to_reach_sec))
        point.time_from_start = duration.to_msg()

        goal_msg.trajectory.points = [point]

        # 设置路径容差 - 使用正确的JointTolerance消息
        goal_msg.path_tolerance = []
        for name in self.joint_names:
            tolerance = JointTolerance()  # 修正：使用独立导入的JointTolerance
            tolerance.name = name
            tolerance.position = 0.1  # 弧度
            goal_msg.path_tolerance.append(tolerance)

        self.get_logger().info('Sending trajectory goal to controller...')
        self._send_goal_future = self.trajectory_client.send_goal_async(
            goal_msg,
            feedback_callback=self.trajectory_feedback_callback)

        self._send_goal_future.add_done_callback(self.trajectory_goal_response_callback)

    def trajectory_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Trajectory goal REJECTED by action server')
            return
        self.get_logger().info('Trajectory goal ACCEPTED by action server')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.trajectory_get_result_callback)

    def trajectory_get_result_callback(self, future):
        result = future.result().result
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info('Trajectory execution SUCCESSFUL')
        else:
            self.get_logger().error(f'Trajectory execution FAILED: {result.error_string} (Code: {result.error_code})')

    def trajectory_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().debug(f'Trajectory progress: {feedback.actual.time_from_start.sec} seconds')

def main(args=None):
    rclpy.init(args=args)
    node = Puma560IkDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()