#!/usr/bin/env python3
#[SMART LAB] ROS template by Wonse Jo
import sys
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray, Bool

# For Godirect libs.
from gdx import gdx
# define information of Veriner sensors 



class ros2_vernier_respiration_belt(Node):
    #############################################################
    # Main Loop
    #############################################################
    def __init__(self):
        super().__init__('vernier_respiration_belt_node')
        
        # For the generlized structure of ROS2 Node
        self.declare_parameter('Sensor_Enable', True) # Enable to publish sensor data (true)
        self.Parm_Sensor_Enable = self.get_parameter('Sensor_Enable').value 
        self.declare_parameter('Chunk_Enable', True) # Enable to publish chunk data (true)
        self.Parm_Chunk_Enable = self.get_parameter('Chunk_Enable').value 
        self.declare_parameter('Chunk_Length', 128) # Define the length of the chunk data
        self.Parm_Chunk_Length = self.get_parameter('Chunk_Length').value 

        
        # For the Veriner Respriation Belt H10 device.
        self.declare_parameter('Device_Name', "GDX-RB 0K2002Z5") # Should be changed inro your device name
        self.Parm_Device_Mac_Address = self.get_parameter('Device_Mac_Address').value 



        self.sensor_sampling_rate = 10 #unit = milliseconds 

        self.gdx = gdx.gdx()  
        self.gdx.open_ble(sensor_name)
        # 1: Force (N), 2: Respiration Rate (bpm), 4: Ste# 1: Force (N), 2: Respiration Rateps (steps), 5: Step Rate (spm)
        self.gdx.select_sensors([1,2,4,5]) 


        #############################################################
        # Global Variables
        #############################################################
        #example: self.dx_1_read_position_data = 0
        self.sensor_status = 0
        #############################################################
        # Publisher Parts
        #############################################################
        self.pub_respiration_belt_data = self.create_publisher(Float32MultiArray, "biosensors/veriner_respiration_belt/data", 10)
        
	    #############################################################
        # ROS main loop
        #############################################################
        while rclpy.ok():
            if self.sensor_status == True:
                self.gdx.start(self.sensor_sampling_rate)                 
                arr_resp_belt_data = gdx.read()
                self.pub_respiration_belt_data.publish(Float32MultiArray(data=arr_resp_belt_data))
            else:
                self.gdx.stop()
                pass  

            rclpy.spin(self)

    #############################################################
    # Subscriber Functions
    #############################################################
    def sub_respiration_belt_state_callback(self,msg): #For a callback function
        self.sensor_status = msg.data
        #print(self.sensor_status)
        



def main(args=None):
    rclpy.init(args=args)

    vernier_resp_belt_node = ros2_vernier_respiration_belt()

    try:
        

    except KeyboardInterrupt:
        gdx.close() 
        print('repeater stopped cleanly')

if __name__ == '__main__':
    main()