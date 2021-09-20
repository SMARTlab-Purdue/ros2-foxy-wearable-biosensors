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
    ros2_foxy_empatica_e4_node = Node(
            package='empatica_e4', 
            #namespace='Subject_Number',
            executable='empatica_e4_node',
            name='empatica_e4_node',
            #output='screen',
            parameters=[{'Sensor_Enable': True, 
            'Chunk_Enable': True,
            'Chunk_Length': 128,
            ### For sensor devices
            'DeviceID': 'CC34CD',
            'E4_Host_IP': '192.168.50.225',
            'E4_Host_Port': 28000
            }] 
        )
  
    return LaunchDescription([
        ros2_foxy_empatica_e4_node,        
    ])
