# wmm.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    robot_desc = ParameterValue(
        Command([
            'xacro ',
            PathJoinSubstitution([
                get_package_share_directory('custom_end_effector'),
                'urdf',
                'end_effector.xacro'
            ])
        ]),
        value_type=str
    )
    robot_description = {'robot_description': robot_desc}
    
    # robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description] 
    )
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
                )]),
            )
    
    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
    )

    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'end_effector'],
        output='screen'
    )

    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state']
    )

    left_controller_spawn = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['left_controller']
    )

    right_controller_spawn = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['right_controller']
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        node_joint_state_publisher_gui,
        spawn_entity,
        joint_broad_spawner,
        left_controller_spawn,
        right_controller_spawn
    ])
