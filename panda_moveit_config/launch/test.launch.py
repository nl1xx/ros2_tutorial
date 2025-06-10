from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess
from launch_ros.descriptions import ParameterValue
from launch.substitutions import Command

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def generate_launch_description():
    # --- 声明 use_sim_time 参数 ---
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # --- 加载机器人描述文件 ---
    # 获取包路径
    pkg_wmm_description = get_package_share_directory('moveit_resources_panda_moveit_config')
    pkg_wmm_moveit_config = get_package_share_directory('moveit_resources_panda_moveit_config')
    
    # 处理URDF文件
    urdf_file = os.path.join(pkg_wmm_description, 'config', 'panda.urdf.xacro')
    robot_description_content = Command(['xacro ', urdf_file])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}
    
    # 处理SRDF文件
    srdf_file = os.path.join(pkg_wmm_moveit_config, 'config', 'panda_arm.srdf.xacro')
    robot_description_semantic_content = Command(['xacro ', srdf_file])
    robot_description_semantic = {
        'robot_description_semantic': ParameterValue(robot_description_semantic_content, value_type=str)
    }

    # --- 加载运动学配置 ---
    kinematics_yaml = load_yaml('moveit_resources_panda_moveit_config', 'config/kinematics.yaml')
    robot_description_kinematics = {'robot_description_kinematics': kinematics_yaml}

    # --- 加载 OMPL 规划配置 ---
    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_planning_yaml = load_yaml('moveit_resources_panda_moveit_config', 'config/ompl_planning.yaml')
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)

    # --- 启动 robot_state_publisher ---
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    # Static TF 
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "panda_link0"],
    )

    # --- 启动 Gazebo ---
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={'world': 'empty.sdf'}.items()
    )
    
    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
        '-entity', 'panda',
        '-topic', 'robot_description',
        '-x', '0.0',     
        '-y', '0.0',
        '-z', '0.5'
        ],
        output='screen'
    )

    # --- 加载控制器配置 ---
    controllers_yaml = load_yaml('moveit_resources_panda_moveit_config', 'config/ros2_controllers.yaml')
    ros2_control_params = {'robot_description': robot_description_content, **controllers_yaml}

    # --- 启动 ros2_control 控制器管理器 ---
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_control_params, {'use_sim_time': use_sim_time}],
        output="screen",
    )

    # --- 加载并启动控制器 ---
    spawn_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )
    
    spawn_jtc = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["panda_arm_controller"],
        output="screen",
    )

    spawn_hand = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["panda_hand_controller"],
        output="screen",
    )

    # --- 启动 move_group 节点 ---
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            ompl_planning_pipeline_config,
            {'use_sim_time': use_sim_time},
            # 加载关节限制配置
            load_yaml('moveit_resources_panda_moveit_config', 'config/joint_limits.yaml')
        ]
    )

    # --- 启动 RViz ---
    rviz_config = PathJoinSubstitution([
        FindPackageShare('moveit_resources_panda_moveit_config'),
        'launch',
        'moveit.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            robot_description_kinematics,
            {'use_sim_time': use_sim_time}
        ]
    )

    return LaunchDescription([
        declare_use_sim_time_arg,
        gazebo,
        rsp_node,
        static_tf_node,
        spawn_entity_node,
        ros2_control_node,
        spawn_jsb,
        spawn_jtc,
        spawn_hand,
        move_group_node,
        rviz_node
    ])
