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
    ros2_foxy_polar_h10_node = Node(
            package='polar_h10', 
            #namespace='Subject_Number',
            executable='polar_h10_node',
            name='polar_h10_node',
            #output='screen',
            parameters=[{'Sensor_Enable': True, 
            'Chunk_Enable': True,
            'Chunk_Length': 10,
            ### For sensor devices
            'Device_Mac_Address': 'C9:61:FF:AC:8E:23',
            }] 
        )
  
    return LaunchDescription([
        ros2_foxy_polar_h10_node,        
    ])
