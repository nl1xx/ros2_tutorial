import kinpy as kp
import numpy as np


urdf_path = './simple_arm.urdf'
# 末端连杆名称
end_effector_link_name = "tool0"


# 构建运动链
with open(urdf_path, 'r', encoding='utf-8') as f:
    chain = kp.build_chain_from_urdf(f.read())
print(f"Robot chain loaded successfully from {urdf_path}")
print(f"Joint names: {chain.get_joint_parameter_names()}")

# 构建SerialChain用于雅可比计算
serial_chain = kp.build_serial_chain_from_urdf(open(urdf_path, 'r', encoding='utf-8').read(), end_effector_link_name)

# 生成与关节数匹配的随机 q 值
num_joints = len(chain.get_joint_parameter_names())
# 使用较小范围的随机值
q_values = np.random.rand(num_joints) * np.pi / 2

# 计算雅可比矩阵
try:
    J = serial_chain.jacobian(q_values)
    print(f"Jacobian Matrix (shape {J.shape}):\n", J)
except Exception as e:
    print(f"Error calculating Jacobian: {e}")

# 计算正向运动学
try:
    fk = serial_chain.forward_kinematics(q_values, end_only=True)
    print(f"Forward Kinematics Result:\n{fk}")
except Exception as e:
    print(f"Error calculating Forward Kinematics: {e}")
