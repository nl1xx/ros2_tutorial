import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    package_name='learning_ros_control'
    urdf_file_path = os.path.join(get_package_share_directory(package_name), 'urdf', 'wmm.xacro')
    robot_description = Command(['xacro ', urdf_file_path])
    
    # 配置robot_state_publisher节点
    params = {'robot_description': robot_description}
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
                )]),
            )

    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'wmm'],
        output='screen'
    )

    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state']
    )

    diff_drive_spawn = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive']
    )

    arm_controller_spawn = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller']
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        joint_broad_spawner,
        diff_drive_spawn,
        arm_controller_spawn
    ])