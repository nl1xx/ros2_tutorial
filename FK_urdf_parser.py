import numpy as np
from scipy.spatial.transform import Rotation as Rot
from urdf_parser_py.urdf import URDF


def parse_urdf(file_path):
    try:
        # 尝试直接加载
        robot = URDF.from_xml_file(file_path)
    except UnicodeDecodeError:
        # 如果失败，显式指定编码为UTF-8
        with open(file_path, 'r', encoding='utf-8') as f:
            xml_string = f.read()
        robot = URDF.from_xml_string(xml_string)

    kinematic_params = {
        'joint_info': [],
        'link_lengths': {}
    }

    for joint in robot.joints:
        joint_data = {
            'name': joint.name,
            'type': joint.type,
            'parent': joint.parent,
            'child': joint.child,
            'axis': joint.axis,
            'origin': {
                'xyz': joint.origin.xyz if joint.origin else [0, 0, 0],
                'rpy': joint.origin.rpy if joint.origin else [0, 0, 0]
            }
        }
        kinematic_params['joint_info'].append(joint_data)

    for link in robot.links:
        link_length = 0.0

        if link.visual:
            # 检查link.visual是列表还是单独的对象
            if isinstance(link.visual, list) and len(link.visual) > 0:
                visual = link.visual[0]
            else:
                visual = link.visual

            if visual.geometry:
                if hasattr(visual.geometry, 'box') and visual.geometry.box:
                    link_length = visual.geometry.box.size[0]
                elif hasattr(visual.geometry, 'cylinder') and visual.geometry.cylinder:
                    link_length = visual.geometry.cylinder.length
                elif hasattr(visual.geometry, 'sphere') and visual.geometry.sphere:
                    link_length = visual.geometry.sphere.radius * 2

                kinematic_params['link_lengths'][link.name] = link_length

    return kinematic_params


def homogeneous_transform(xyz, rpy):
    x, y, z = xyz
    roll, pitch, yaw = rpy

    rotation = Rot.from_euler('xyz', [roll, pitch, yaw]).as_matrix()

    T = np.eye(4)
    T[:3, :3] = rotation
    T[:3, 3] = [x, y, z]

    return T


def forward_kinematics(params, joint_angles):
    T = np.eye(4)

    for i, joint in enumerate(params['joint_info']):
        link_length = params['link_lengths'].get(joint['child'], 0.0)
        angle = joint_angles.get(joint['name'], 0.0)

        axis = joint['axis'] if joint['axis'] else [1, 0, 0]
        origin_xyz = joint['origin']['xyz']
        origin_rpy = joint['origin']['rpy']

        joint_T = homogeneous_transform(origin_xyz, origin_rpy)

        if axis == [1, 0, 0]:
            rot = Rot.from_euler('x', angle).as_matrix()
        elif axis == [0, 1, 0]:
            rot = Rot.from_euler('y', angle).as_matrix()
        else:
            rot = Rot.from_euler('z', angle).as_matrix()

        rotation_T = np.eye(4)
        rotation_T[:3, :3] = rot
        rotation_T[:3, 3] = [0, 0, link_length / 2]  # 连杆长度的一半

        T = T @ joint_T @ rotation_T

    position = T[:3, 3]
    orientation = T[:3, :3]

    return position, orientation


def main():
    urdf_file = 'robot.urdf'

    params = parse_urdf(urdf_file)

    joint_angles = {
        'joint1': np.pi / 4,
        'joint2': np.pi / 6,
        'joint3': np.pi / 8
    }

    position, orientation = forward_kinematics(params, joint_angles)

    print("末端执行器位置:", position)
    print("末端执行器姿态:")
    print(orientation)


if __name__ == "__main__":
    main()
