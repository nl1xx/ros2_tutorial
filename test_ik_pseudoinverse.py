<<<<<<< HEAD
import numpy as np
from scipy.spatial.transform import Rotation
from roboticstoolbox import DHRobot, RevoluteDH
import spatialmath as sm
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

# 创建6轴机器人模型 (PUMA560 示例)
def create_robot_model():
    # DH参数: [a, alpha, d, theta]
    return DHRobot([
        RevoluteDH(a=0, alpha=np.pi / 2, d=0),
        RevoluteDH(a=0.4318, alpha=0, d=0),
        RevoluteDH(a=0.0203, alpha=-np.pi / 2, d=0.15005),
        RevoluteDH(a=0, alpha=np.pi / 2, d=0.4318),
        RevoluteDH(a=0, alpha=-np.pi / 2, d=0),
        RevoluteDH(a=0, alpha=0, d=0),
    ], name="PUMA560")


# 初始化机器人模型（全局变量）
robot = create_robot_model()


# --- 使用 roboticstoolbox 实现的函数 ---
def calculate_fk(q):
    """使用 roboticstoolbox 计算正向运动学"""
    T = robot.fkine(q)
    return T.A  # 返回4x4齐次变换矩阵


def calculate_jacobian(q):
    """使用 roboticstoolbox 计算几何雅可比矩阵"""
    return robot.jacob0(q)  # 返回6xN的雅可比矩阵


# --- 核心 IK 求解器 (添加误差记录功能) ---
def calculate_pose_error(T_desired, T_current):
    """计算位姿误差向量 (6D)"""
    # 位置误差 (3D)
    pos_error = T_desired[:3, 3] - T_current[:3, 3]

    # 姿态误差 (3D - 角轴表示)
    R_desired = T_desired[:3, :3]
    R_current = T_current[:3, :3]
    try:
        error_rot_mat = R_desired @ R_current.T
        rotation = Rotation.from_matrix(error_rot_mat)
        orient_error = rotation.as_rotvec()
    except ValueError as e:
        print(f"Error calculating orientation error: {e}. Using zero vector.")
        orient_error = np.zeros(3)

    # 合并为6D误差向量
    error_vec = np.concatenate((pos_error, orient_error))
    return error_vec


def solve_ik_pseudoinverse(T_desired, q_initial, max_iterations=100, tolerance=1e-4, gain=0.1):
    """使用雅可比伪逆法求解数值IK"""
    q = q_initial.copy().astype(float)
    num_joints = len(q_initial)
    error_history = []  # 存储每次迭代的误差值

    for i in range(max_iterations):
        T_current = calculate_fk(q)
        error = calculate_pose_error(T_desired, T_current)
        error_magnitude = np.linalg.norm(error)
        error_history.append(error_magnitude)  # 记录当前误差

        print(f"Iteration {i}: Error = {error_magnitude:.6f}")

        if error_magnitude < tolerance:
            print(f"成功收敛于 {i} 次迭代。")
            return q, True, error_history

        J = calculate_jacobian(q)
        if J.shape[1] != num_joints:
            print(f"雅可比矩阵维度不匹配: 期望 {num_joints} 列, 实际 {J.shape[1]}")
            return q, False, error_history

        try:
            J_pinv = np.linalg.pinv(J, rcond=1e-4)
        except np.linalg.LinAlgError:
            print("奇异点警告：雅可比矩阵伪逆计算失败。")
            return q, False, error_history

        dq = J_pinv @ error * gain
        q = q + dq

    print(f"达到最大迭代次数 {max_iterations} 未收敛。")
    return q, False, error_history


# --- 绘制误差收敛曲线 ---
def plot_error_convergence(error_history):
    plt.figure(figsize=(10, 6))
    plt.plot(error_history, 'b-o', linewidth=2, markersize=6)
    plt.title('Inverse kinematic convergence curve', fontsize=14)
    plt.xlabel('The number of iterations', fontsize=12)
    plt.ylabel('Error norms (m/rad)', fontsize=12)
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.yscale('log')
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    T_target = sm.SE3(0.5, 0.2, 0.3) * sm.SE3.RPY([0, np.pi / 2, 0])
    T_desired = T_target.A

    # 初始关节角猜测
    q_init = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])

    print("目标位姿:\n", np.round(T_desired, 3))
    print("初始关节角:", np.round(q_init, 3))

    # 求解IK并获取误差历史
    q_solution, success, error_history = solve_ik_pseudoinverse(
        T_desired,
        q_init,
        max_iterations=50,
        tolerance=1e-5,
        gain=0.5
    )

    # 绘制误差收敛曲线
    plot_error_convergence(error_history)

    if success:
        print("\nIK 求解成功!")
        print("关节角解:", np.round(q_solution, 4))

        # 验证解的正确性
        T_achieved = calculate_fk(q_solution)
        print("\n达到的位姿:\n", np.round(T_achieved, 4))
        print("位置误差:", np.linalg.norm(T_desired[:3, 3] - T_achieved[:3, 3]))

        # 可视化机器人位姿
        print("\n可视化机器人位姿...")
        plt.figure()
        robot.plot(q_solution, block=False, eeframe=True, shadow=True)
        plt.title('IK求解结果可视化', fontsize=14)
        plt.show(block=True)
    else:
        print("\nIK 求解失败!")
=======
import numpy as np
from scipy.spatial.transform import Rotation
from roboticstoolbox import DHRobot, RevoluteDH
import spatialmath as sm
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

# 创建6轴机器人模型 (PUMA560 示例)
def create_robot_model():
    # DH参数: [a, alpha, d, theta]
    return DHRobot([
        RevoluteDH(a=0, alpha=np.pi / 2, d=0),
        RevoluteDH(a=0.4318, alpha=0, d=0),
        RevoluteDH(a=0.0203, alpha=-np.pi / 2, d=0.15005),
        RevoluteDH(a=0, alpha=np.pi / 2, d=0.4318),
        RevoluteDH(a=0, alpha=-np.pi / 2, d=0),
        RevoluteDH(a=0, alpha=0, d=0),
    ], name="PUMA560")


# 初始化机器人模型（全局变量）
robot = create_robot_model()


# --- 使用 roboticstoolbox 实现的函数 ---
def calculate_fk(q):
    """使用 roboticstoolbox 计算正向运动学"""
    T = robot.fkine(q)
    return T.A  # 返回4x4齐次变换矩阵


def calculate_jacobian(q):
    """使用 roboticstoolbox 计算几何雅可比矩阵"""
    return robot.jacob0(q)  # 返回6xN的雅可比矩阵


# --- 核心 IK 求解器 (添加误差记录功能) ---
def calculate_pose_error(T_desired, T_current):
    """计算位姿误差向量 (6D)"""
    # 位置误差 (3D)
    pos_error = T_desired[:3, 3] - T_current[:3, 3]

    # 姿态误差 (3D - 角轴表示)
    R_desired = T_desired[:3, :3]
    R_current = T_current[:3, :3]
    try:
        error_rot_mat = R_desired @ R_current.T
        rotation = Rotation.from_matrix(error_rot_mat)
        orient_error = rotation.as_rotvec()
    except ValueError as e:
        print(f"Error calculating orientation error: {e}. Using zero vector.")
        orient_error = np.zeros(3)

    # 合并为6D误差向量
    error_vec = np.concatenate((pos_error, orient_error))
    return error_vec


def solve_ik_pseudoinverse(T_desired, q_initial, max_iterations=100, tolerance=1e-4, gain=0.1):
    """使用雅可比伪逆法求解数值IK"""
    q = q_initial.copy().astype(float)
    num_joints = len(q_initial)
    error_history = []  # 存储每次迭代的误差值

    for i in range(max_iterations):
        T_current = calculate_fk(q)
        error = calculate_pose_error(T_desired, T_current)
        error_magnitude = np.linalg.norm(error)
        error_history.append(error_magnitude)  # 记录当前误差

        print(f"Iteration {i}: Error = {error_magnitude:.6f}")

        if error_magnitude < tolerance:
            print(f"成功收敛于 {i} 次迭代。")
            return q, True, error_history

        J = calculate_jacobian(q)
        if J.shape[1] != num_joints:
            print(f"雅可比矩阵维度不匹配: 期望 {num_joints} 列, 实际 {J.shape[1]}")
            return q, False, error_history

        try:
            J_pinv = np.linalg.pinv(J, rcond=1e-4)
        except np.linalg.LinAlgError:
            print("奇异点警告：雅可比矩阵伪逆计算失败。")
            return q, False, error_history

        dq = J_pinv @ error * gain
        q = q + dq

    print(f"达到最大迭代次数 {max_iterations} 未收敛。")
    return q, False, error_history


# --- 绘制误差收敛曲线 ---
def plot_error_convergence(error_history):
    plt.figure(figsize=(10, 6))
    plt.plot(error_history, 'b-o', linewidth=2, markersize=6)
    plt.title('Inverse kinematic convergence curve', fontsize=14)
    plt.xlabel('The number of iterations', fontsize=12)
    plt.ylabel('Error norms (m/rad)', fontsize=12)
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.yscale('log')
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    T_target = sm.SE3(0.5, 0.2, 0.3) * sm.SE3.RPY([0, np.pi / 2, 0])
    T_desired = T_target.A

    # 初始关节角猜测
    q_init = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])

    print("目标位姿:\n", np.round(T_desired, 3))
    print("初始关节角:", np.round(q_init, 3))

    # 求解IK并获取误差历史
    q_solution, success, error_history = solve_ik_pseudoinverse(
        T_desired,
        q_init,
        max_iterations=50,
        tolerance=1e-5,
        gain=0.5
    )

    # 绘制误差收敛曲线
    plot_error_convergence(error_history)

    if success:
        print("\nIK 求解成功!")
        print("关节角解:", np.round(q_solution, 4))

        # 验证解的正确性
        T_achieved = calculate_fk(q_solution)
        print("\n达到的位姿:\n", np.round(T_achieved, 4))
        print("位置误差:", np.linalg.norm(T_desired[:3, 3] - T_achieved[:3, 3]))

        # 可视化机器人位姿
        print("\n可视化机器人位姿...")
        plt.figure()
        robot.plot(q_solution, block=False, eeframe=True, shadow=True)
        plt.title('IK求解结果可视化', fontsize=14)
        plt.show(block=True)
    else:
        print("\nIK 求解失败!")
>>>>>>> 20f8e51d558ac903605a46ec3db2554ae42e5131
