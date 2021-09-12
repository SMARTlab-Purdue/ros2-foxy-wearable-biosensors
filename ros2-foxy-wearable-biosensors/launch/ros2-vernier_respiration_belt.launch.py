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
    ros2_foxy_vernier_respiration_belt_node = Node(
            package='ros2-foxy-wearable-biosensors', 
            #namespace='Subject_Number',
            executable='ROS2_Vernier_Respiration_Belt_Node',
            name='vernier_respiration_belt_node',
            #output='screen',
            parameters=[{'Sensor_Enable': True, 
            'Chunk_Enable': True,
            'Chunk_Length': 20,
            ### For sensor devices
            'Device_Name': 'GDX-RB 0K2002Z5',
            'Device_Sampling_Rate': 10 #unit = milliseconds
            }] 
        )
  
    return LaunchDescription([
        ros2_foxy_vernier_respiration_belt_node,        
    ])
