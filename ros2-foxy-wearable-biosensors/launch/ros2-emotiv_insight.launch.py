import os
from ament_index_python.packages import get_package_share_directory , get_search_paths
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch import LaunchDescription, Action
import launch


def generate_launch_description():  
    # Subject Information
    Subject_Number = "P1"

    
    ###### Physiological Sensor
    ros2_emotiv_insight_node = Node(
            package='ros2-foxy-wearable-biosensors', 
            #namespace='Subject_Number',
            executable='ROS2_Emotiv_Insight_Node',
            name='emotiv_insight_node',
            #output='screen',
            parameters=[{'Sensor_Enable': True, 
            'Chunk_Enable': True,
            'Chunk_Length': 128,            
            }] 
        )
  
    return LaunchDescription([
        ros2_emotiv_insight_node,        
    ])
