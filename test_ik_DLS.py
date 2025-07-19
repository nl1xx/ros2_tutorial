import numpy as np
from scipy.spatial.transform import Rotation
import roboticstoolbox as rtb
from spatialmath import SE3
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from roboticstoolbox.backends.PyPlot import PyPlot

# 创建机器人模型 (UR5)
robot = rtb.models.UR5()


# --- 正向运动学和雅可比矩阵计算 ---
def calculate_fk(q):
    T = robot.fkine(q)
    return T.A


def calculate_jacobian(q):
    return robot.jacobe(q)


# --- 更稳定的姿态误差计算 ---
def calculate_pose_error(T_desired, T_current):
    """计算位姿误差向量 (6D)"""
    # 位置误差 (3D)
    pos_error = T_desired[:3, 3] - T_current[:3, 3]

    # 姿态误差 - 使用四元数表示避免奇异性
    R_desired = T_desired[:3, :3]
    R_current = T_current[:3, :3]

    # 使用更稳定的方法计算旋转差
    rot_desired = Rotation.from_matrix(R_desired)
    rot_current = Rotation.from_matrix(R_current)

    # 直接计算相对旋转
    rot_error = rot_desired * rot_current.inv()

    # 将旋转误差转换为角轴表示
    orient_error = rot_error.as_rotvec()

    # 合并为 6D 误差向量
    return np.concatenate((pos_error, orient_error))


# --- 基于任务优先级和自适应阻尼的 IK 求解器 ---
def solve_ik_adaptive(T_desired, q_initial, max_iterations=100, tolerance=1e-4):
    """使用自适应阻尼和任务优先级策略求解数值 IK"""
    q = q_initial.copy().astype(float)
    num_joints = len(q_initial)
    error_history = []

    # UR5关节角度限制
    q_min = np.array([-np.pi, -np.pi, -np.pi, -np.pi, -np.pi, -np.pi])
    q_max = np.array([np.pi, np.pi, np.pi, np.pi, np.pi, np.pi])

    # 初始阻尼系数
    lambda_val = 0.01
    min_lambda = 0.001
    max_lambda = 1.0

    # 位置和姿态的权重因子
    w_position = 1.0
    w_orientation = 0.5  # 姿态权重较低，因为姿态误差通常更大

    for i in range(max_iterations):
        T_current = calculate_fk(q)
        error_full = calculate_pose_error(T_desired, T_current)

        # 分离位置和姿态误差
        pos_error = error_full[:3]
        orient_error = error_full[3:]

        pos_error_norm = np.linalg.norm(pos_error)
        orient_error_norm = np.linalg.norm(orient_error)
        total_error = np.linalg.norm(error_full)
        error_history.append(total_error)

        print(f"迭代 {i:3d}: 位置误差 = {pos_error_norm:.6f}, "
              f"姿态误差 = {orient_error_norm:.6f}, "
              f"总误差 = {total_error:.6f}, λ = {lambda_val:.4f}")

        if total_error < tolerance:
            print(f"成功收敛于 {i} 次迭代。")
            return q, True, error_history

        J = calculate_jacobian(q)

        # 分离位置和姿态的雅可比
        J_pos = J[:3, :]
        J_orient = J[3:, :]

        # 计算位置增量
        try:
            # 位置增量计算 (使用阻尼最小二乘法)
            J_pos_T = J_pos.T
            A_pos = J_pos @ J_pos_T + lambda_val * np.eye(3)
            dq_pos = J_pos_T @ np.linalg.solve(A_pos, w_position * pos_error)
        except np.linalg.LinAlgError:
            dq_pos = np.zeros(num_joints)

        # 计算姿态增量
        try:
            # 姿态增量计算 (使用阻尼最小二乘法)
            J_orient_T = J_orient.T
            A_orient = J_orient @ J_orient_T + lambda_val * np.eye(3)
            dq_orient = J_orient_T @ np.linalg.solve(A_orient, w_orientation * orient_error)
        except np.linalg.LinAlgError:
            dq_orient = np.zeros(num_joints)

        # 合并增量
        dq = dq_pos + dq_orient

        # 应用关节限位
        q = np.clip(q + dq, q_min, q_max)

        # 自适应调整阻尼系数
        if i > 0 and error_history[-1] > error_history[-2]:
            # 误差增加，增大阻尼
            lambda_val = min(lambda_val * 1.5, max_lambda)
        else:
            # 误差减少，减小阻尼
            lambda_val = max(lambda_val * 0.9, min_lambda)

    print(f"达到最大迭代次数 {max_iterations} 未收敛。")
    return q, False, error_history


# --- 验证目标位姿可达性 ---
def is_pose_reachable(T_target, q_init, tolerance=0.1):
    """检查目标位姿是否可达"""
    # 使用机器人内置IK方法作为参考
    try:
        sol = robot.ik_LM(T_target, q0=q_init, tol=tolerance)
        return sol.success
    except:
        return False


# --- 主程序 ---
if __name__ == "__main__":
    # 1. 选择一个更容易达到的目标位姿
    # 使用机器人的中性位置作为参考
    q_neutral = np.array([0, -np.pi / 2, np.pi / 2, -np.pi / 2, -np.pi / 2, 0])
    T_neutral = calculate_fk(q_neutral)

    # 在可达范围内创建目标位姿
    T_target = SE3(T_neutral)
    # 稍微调整位置
    T_target.t[0] += 0.1  # x方向移动10cm
    T_target.t[2] += 0.1  # z方向移动10cm

    print("目标位姿:\n", T_target.A)

    # 2. 验证目标位姿是否可达
    if not is_pose_reachable(SE3(T_target.A), q_neutral):
        print("警告：目标位姿可能不可达！使用替代目标。")
        # 使用中性位姿作为替代
        T_target = SE3(T_neutral)

    # 3. 设置初始关节角度
    q_init = q_neutral.copy()

    print("初始关节角度 (度):", np.degrees(q_init))

    # 4. 可视化初始配置
    backend = PyPlot()
    backend.launch(name="UR5 IK 求解")
    backend.add(robot)
    robot.q = q_init
    backend.step()

    # 5. 求解逆运动学
    q_solution, success, error_history = solve_ik_adaptive(
        T_target.A,
        q_init,
        max_iterations=100000,
        tolerance=1e-4
    )

    # 6. 结果分析
    if success:
        print("\nIK 求解成功!")
        print("关节角度解 (弧度):", q_solution)
        print("关节角度解 (度数):", np.degrees(q_solution))

        # 验证解的正确性
        T_achieved = calculate_fk(q_solution)
        print("\n实际达到的位姿:\n", T_achieved)
        print("目标位姿:\n", T_target.A)

        # 计算位置误差
        pos_error = np.linalg.norm(T_target.t - T_achieved[:3, 3])
        print(f"位置误差: {pos_error * 1000:.4f} mm")

        # 计算姿态误差
        R_target = T_target.R
        R_achieved = T_achieved[:3, :3]
        orient_error = Rotation.from_matrix(R_target) * Rotation.from_matrix(R_achieved).inv()
        orient_error_deg = np.degrees(np.linalg.norm(orient_error.as_rotvec()))
        print(f"姿态误差: {orient_error_deg:.4f} 度")

        # 可视化最终配置
        robot.q = q_solution
        backend.step()

        # 绘制误差收敛曲线
        plt.figure()
        plt.plot(error_history, 'b-o', linewidth=2)
        plt.title('Inverse kinematic convergence curve')
        plt.xlabel('The number of iterations')
        plt.ylabel('Error norms')
        plt.grid(True)
        plt.yscale('log')
        plt.show(block=True)  # 阻塞模式
    else:
        print("\nIK 求解失败。尝试不同的初始值或调整目标位姿。")

        # 绘制误差曲线以分析失败原因
        plt.figure()
        plt.plot(error_history, 'b-o', linewidth=2)
        plt.title('Inverse kinematic convergence curve')
        plt.xlabel('The number of iterations')
        plt.ylabel('Error norms')
        plt.grid(True)
        plt.yscale('log')
        plt.show(block=True)  # 阻塞模式

        # 显示最后达到的位姿
        T_final = calculate_fk(q_solution)
        print("最后达到的位姿:\n", T_final)
        print("目标位姿:\n", T_target.A)

        # 分析位置差异
        pos_diff = T_target.t - T_final[:3, 3]
        print("位置差异 (mm):", pos_diff * 1000)

        # 分析姿态差异
        R_diff = Rotation.from_matrix(T_target.R) * Rotation.from_matrix(T_final[:3, :3]).inv()
        print("姿态差异 (度):", np.degrees(R_diff.as_rotvec()))
