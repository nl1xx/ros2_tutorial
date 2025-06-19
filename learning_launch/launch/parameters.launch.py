from launch import LaunchDescription                  
from launch.actions import DeclareLaunchArgument       
from launch.substitutions import LaunchConfiguration, TextSubstitution

from launch_ros.actions import Node                   


def generate_launch_description():                     
   background_r_launch_arg = DeclareLaunchArgument(
      'background_r', default_value=TextSubstitution(text='0')     # 创建一个Launch文件内参数（arg）background_r
   )
   background_g_launch_arg = DeclareLaunchArgument(
      'background_g', default_value=TextSubstitution(text='84')    # 创建一个Launch文件内参数（arg）background_g
   )
   background_b_launch_arg = DeclareLaunchArgument(
      'background_b', default_value=TextSubstitution(text='122')   # 创建一个Launch文件内参数（arg）background_b
   )

   return LaunchDescription([                                      
      background_r_launch_arg,                                     # 调用以上创建的参数（arg）
      background_g_launch_arg,
      background_b_launch_arg,
      Node(                                                        
         package='turtlesim',
         executable='turtlesim_node',                              
         name='sim',                                               
         parameters=[{                                             # ROS参数列表
            'background_r': LaunchConfiguration('background_r'),   # 创建参数background_r
            'background_g': LaunchConfiguration('background_g'),   # 创建参数background_g
            'background_b': LaunchConfiguration('background_b'),   # 创建参数background_b
         }]
      ),
   ])
