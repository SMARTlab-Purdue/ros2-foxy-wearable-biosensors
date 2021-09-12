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
    ros2_foxy_shimmer3_gsr_node = Node(
            package='ros2-foxy-wearable-biosensors', 
            #namespace='Subject_Number',
            executable='ROS2_Shimmer3_GSR_Node',
            name='shimmer3_gsr_node',
            #output='screen',
            parameters=[{'Sensor_Enable': True, 
            'Chunk_Enable': True,
            'Chunk_Length': 128,
            ### For sensor devices
            # Ensure the Bluetooth radio is available by running the '''$ hciconfig''' command.
            # Scan for the Shimmer by running the '''$ hcitool scan''' command.
            'Device_Name': '00:06:66:F2:AF:E9',
            }] 
        )
  
    return LaunchDescription([
        ros2_foxy_shimmer3_gsr_node,        
    ])
