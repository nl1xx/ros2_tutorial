import roboticstoolbox as rtb
import numpy as np


robot = rtb.models.DH.Panda()
print(robot)


def calculate_jacobian_toolbox(robot, q):
    """使用 roboticstoolbox 计算雅可比"""
    if robot is None:
        print("Robot model not loaded.")
        return None
    try:
        # 计算空间雅可比 (Calculate spatial Jacobian J0)
        J = robot.jacob0(q)
        # 或者计算物体雅可比 (Or calculate body Jacobian Je)
        # J = robot.jacobe(q)
        return J
    except ValueError as e:
        # 捕捉可能的维度不匹配或无效 q 值错误
        print(f"Error calculating Jacobian (likely invalid q): {e}")
        return None
    except Exception as e:
        print(f"Unexpected error calculating Jacobian: {e}")
        return None


if robot is not None:
    if robot.qlim.ndim != 2 or robot.qlim.shape[0] != robot.n or robot.qlim.shape[1] != 2:
        print(f"Warning: qlim shape is unexpected: {robot.qlim.shape}")
        joint_range_min = -np.pi
        joint_range_max = np.pi
        q_sample = np.random.uniform(joint_range_min, joint_range_max, robot.n)
    else:
        # 使用模型的关节数生成随机 q
        q_sample = np.random.uniform(robot.qlim[:, 0], robot.qlim[:, 1])

    print(f"Calculating Jacobian for q: {q_sample}")
    J_matrix = calculate_jacobian_toolbox(robot, q_sample)
    if J_matrix is not None:
        print(f"Jacobian (shape {J_matrix.shape}):\n", J_matrix)
