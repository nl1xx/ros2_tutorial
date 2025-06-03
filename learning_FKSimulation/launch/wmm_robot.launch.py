import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_my_fk_pkg = get_package_share_directory('learning_FKSimulation')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # URDF文件路径
    urdf_file_path = os.path.join(pkg_my_fk_pkg, 'urdf', 'wmm.urdf')
    if not os.path.exists(urdf_file_path):
        raise FileNotFoundError(f"URDF file not found at {urdf_file_path}")
    
    # RViz文件路径
    rviz_config_file = os.path.join(pkg_my_fk_pkg, 'rviz', 'fk_display.rviz')

    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # 修改为WMM机械臂实际的关节名称列表，顺序要与FK计算节点期望的一致
    wmm_joint_names = ['wmm_joint_1', 'wmm_joint_2']
    # 修改为WMM机械臂的基座连杆名称
    wmm_base_link = 'base_link'


    # 1. Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        # launch_arguments={'world': 'path/to/your/world_file.world'}.items(), # 可选：指定世界文件
    )

    # 2. Robot State Publisher
    # Reads URDF, subscribes to /joint_states, publishes /tf and /robot_description
    with open(urdf_file_path, 'r') as f:
        robot_desc = f.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': robot_desc}],
    )

    # 3. Spawn WMM Robot in Gazebo
    # The URDF should have the <plugin name="gazebo_ros_joint_state_publisher">
    # so Gazebo will publish /joint_states
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', # Use the robot_description published by robot_state_publisher
                   '-entity', 'wmm_robot',      # Name of the robot in Gazebo
                   '-x', '0.0', '-y', '0.0', '-z', '0.1', # Position to spawn at
                  ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 4. Joint State Publisher GUI (可选, 用于手动控制, 如果Gazebo插件已配置，则此项可注释掉)
    # joint_state_publisher_gui_node = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     name='joint_state_publisher_gui',
    #     parameters=[{'use_sim_time': use_sim_time}],
    #     remappings=[('/joint_states', '/gui_joint_states')] # 如果你想同时运行gazebo插件和GUI，需要重映射
    # )

    # 5. FK Calculator Node
    fk_calculator_node = Node(
        package='learning_FKSimulation',
        executable='FK', 
        name='fk_calculator_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'joint_names': wmm_joint_names},
            {'base_link_frame': wmm_base_link},
            {'fk_ee_frame': 'fk_calculated_end_effector'} # 与Python节点中child_frame_id一致
        ]
    )

    # 6. RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file], # 可选: 加载预定义的RViz配置
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock'),
        # set_gazebo_model_path, # 如果你需要设置模型路径
        gazebo,
        robot_state_publisher_node,
        spawn_entity_node,
        # joint_state_publisher_gui_node, # 如果需要，取消注释
        fk_calculator_node,
        rviz_node
    ])