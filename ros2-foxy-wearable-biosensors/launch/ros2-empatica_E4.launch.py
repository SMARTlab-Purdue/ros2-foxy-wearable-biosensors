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

    ###### Run ros2bag
    data_recording = ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-a', '-o', Subject_Number],
            #output='screen'
        )

    
    ###### Physiological Sensor
    ros2_foxy_empatica_e4_node = Node(
            package='ros2-foxy-wearable-biosensors', 
            #namespace='Subject_Number',
            executable='ROS2_Empatica_E4_Node',
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
