import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
# import xacro # 如果使用 xacro 文件则需要导入

def generate_launch_description():
    # 获取 URDF 文件路径
    urdf_file_name = 'simple_robot.urdf' # URDF 文件名
    package_name = 'learning_urdf' # 替换为你的包名
    urdf_path = os.path.join(
        get_package_share_directory(package_name),
        'urdf', # 假设 URDF 在 'urdf' 子目录下
        urdf_file_name)

    # 读取 URDF 文件内容
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()
    robot_description = {'robot_description': robot_desc} # 参数字典

    # robot_state_publisher 节点: 发布 TF 变换
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description] # 加载 URDF
    )

    # joint_state_publisher_gui 节点: 提供 GUI 来控制关节状态
    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
    )

    # RViz2 节点: 可视化工具
    # 可选: 指定 RViz 配置文件路径
    rviz_config_file = os.path.join(get_package_share_directory(package_name), 'rviz', 'urdf.rviz')
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file] # 使用指定的 RViz 配置文件启动
    )

    return LaunchDescription([
        node_robot_state_publisher,
        node_joint_state_publisher_gui,
        node_rviz
    ])
# (注意: 你需要创建一个简单的 RViz 配置文件 urdf_config.rviz 并保存在包的 rviz 目录下，或者在 RViz 启动后手动配置)