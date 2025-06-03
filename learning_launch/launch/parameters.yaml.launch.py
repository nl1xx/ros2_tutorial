import os
from ament_index_python.packages import get_package_share_directory  # 查询功能包路径的方法
from launch import LaunchDescription 
from launch_ros.actions import Node 


def generate_launch_description():     
   config = os.path.join( 
      get_package_share_directory('learning_launch'),
      'config',
      'turtlesim.yaml'
      )

   return LaunchDescription([         
      Node(                           
         package='turtlesim',          
         executable='turtlesim_node', 
         namespace='turtlesim2',       
         name='sim',                  
         parameters=[config]    
      )
   ])
