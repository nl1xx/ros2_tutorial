import numpy as np
from scipy.spatial.transform import Rotation as R


def create_transformation_matrix(rot_axis, rot_angle, trans):
    """
    创建 4x4 齐次变换矩阵。

    Args:
        rot_axis (numpy.ndarray): 旋转轴向量
        rot_angle (float): 旋转角度（弧度）
        trans (numpy.ndarray): 平移向量

    Returns:
        numpy.ndarray: 4x4 变换矩阵
    """
    T = np.eye(4)

    # 计算旋转矩阵
    rotation = R.from_rotvec(rot_angle * rot_axis)
    T[0:3, 0:3] = rotation.as_matrix()

    # 设置平移
    T[0:3, 3] = trans

    return T


def calculate_fk(joint_angles):
    """
    计算指定连杆的正向运动学（以 WMM 机械臂为例）。

    Args:
        joint_angles (list or np.array): 关节角度列表 [q1, q2, q3]

    Returns:
        np.ndarray: 表示位姿的 4x4 齐次变换矩阵
    """
    # 定义机械臂的几何参数
    L1 = 0.5  # 连杆1的长度
    L2 = 0.4  # 连杆2的长度
    offset_z1 = 0.1  # 连杆1相对基座的Z轴偏移

    # 获取关节角度
    q1 = joint_angles[0]
    q2 = joint_angles[1]
    q3 = joint_angles[2]

    # 创建变换矩阵
    # 关节1: 绕Z轴旋转，平移Z方向
    T_1_0 = create_transformation_matrix(
        rot_axis=np.array([0, 0, 1]),
        rot_angle=q1,
        trans=np.array([0.0, 0.0, offset_z1])
    )

    # 关节2: 绕Y轴旋转，平移X方向
    T_2_1 = create_transformation_matrix(
        rot_axis=np.array([0, 1, 0]),
        rot_angle=q2,
        trans=np.array([L1, 0.0, 0.0])
    )

    # 关节3: 绕Y轴旋转，平移X方向
    T_3_2 = create_transformation_matrix(
        rot_axis=np.array([0, 1, 0]),
        rot_angle=q3,
        trans=np.array([L2, 0.0, 0.0])
    )

    # 计算最终变换矩阵
    T_target_0 = T_1_0 @ T_2_1 @ T_3_2

    return T_target_0


if __name__ == "__main__":
    # 关节角度
    angles = [np.pi / 4, np.pi / 6, np.pi / 8]

    # 计算正向运动学
    pose_matrix = calculate_fk(angles)

    print("Calculated Pose Matrix:")
    print(pose_matrix)
