<<<<<<< HEAD
import numpy as np
import roboticstoolbox as rtb


robot = rtb.models.DH.Panda()


def calculate_jacobian_toolbox(robot, q):
    return robot.jacob0(q)


# 1. 设定测试构型和关节速度
q_test = np.array([0.1, np.pi / 4, np.pi / 4, 0.2, np.pi / 6, 0.3, 0.0])  # 非奇异构型
dq_test = np.array([0.5, 0, 0, 0, 0, 0, 0])  # 仅第一个关节运动


# 2. 计算雅可比矩阵
J_test = calculate_jacobian_toolbox(robot, q_test)

if J_test is not None:
    print(f"雅可比矩阵 J(q) (尺寸 {J_test.shape}): {np.round(J_test, 3)}")

    # 3. 计算末端执行器速度旋量
    V_test = J_test @ dq_test
    print(f"末端速度旋量 V = J @ dq: {np.round(V_test, 3)}")

    # 4. 分解并解释结果
    omega_s = V_test[:3]  # 角速度
    v_s = V_test[3:]  # 线速度

    print(f"角速度 omega_s: {np.round(omega_s, 3)} rad/s")
    print(f"线速度 v_s:    {np.round(v_s, 3)} m/s")
else:
    print("错误：雅可比矩阵计算失败！")
=======
import numpy as np
import roboticstoolbox as rtb


robot = rtb.models.DH.Panda()


def calculate_jacobian_toolbox(robot, q):
    return robot.jacob0(q)


# 1. 设定测试构型和关节速度
q_test = np.array([0.1, np.pi / 4, np.pi / 4, 0.2, np.pi / 6, 0.3, 0.0])  # 非奇异构型
dq_test = np.array([0.5, 0, 0, 0, 0, 0, 0])  # 仅第一个关节运动


# 2. 计算雅可比矩阵
J_test = calculate_jacobian_toolbox(robot, q_test)

if J_test is not None:
    print(f"雅可比矩阵 J(q) (尺寸 {J_test.shape}): {np.round(J_test, 3)}")

    # 3. 计算末端执行器速度旋量
    V_test = J_test @ dq_test
    print(f"末端速度旋量 V = J @ dq: {np.round(V_test, 3)}")

    # 4. 分解并解释结果
    omega_s = V_test[:3]  # 角速度
    v_s = V_test[3:]  # 线速度

    print(f"角速度 omega_s: {np.round(omega_s, 3)} rad/s")
    print(f"线速度 v_s:    {np.round(v_s, 3)} m/s")
else:
    print("错误：雅可比矩阵计算失败！")
>>>>>>> 20f8e51d558ac903605a46ec3db2554ae42e5131
