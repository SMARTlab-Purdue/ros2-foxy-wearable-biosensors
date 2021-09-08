## This ROS2 node for reading a vernier respiration belt. This node is made based on this website; https://github.com/VernierST/godirect-examples/tree/main/python

import sys
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, Float32MultiArray, Bool

# For Godirect libs.
from gdx import gdx   # should install godirect; $ pip3 install godirect
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
        self.Parm_Device_Name = self.get_parameter('Device_Name').value 
        self.declare_parameter('Device_Sampling_Rate', 10) #unit = milliseconds 
        self.Parm_Device_Sampling_Rate = self.get_parameter('Device_Sampling_Rate').value 


        self.gdx = gdx.gdx()  
        self.gdx.open_ble(self.Parm_Device_Name)
        # 1: Force (N), 2: Respiration Rate (bpm), 4: Step, 5: Step Rate (spm)
        # Step = number of steps that are detected by the sensor
        # Step Rate = The Step Rate channel steps per minute (SPM). The sample window for the calculation is 10 seconds.
        self.gdx.select_sensors([1,2]) 
        
        
        self.data_index = 0
        self.bpm_data_index = 0
        self.bpm_chunk_data = []


        
        #############################################################
        # Publisher Parts
        #############################################################
        self.pub_respiration_belt_bpm_data = self.create_publisher(Float32, "biosensors/veriner_respiration_belt/bpm", 10) #raw Respiration Rate (bpm)
        self.pub_respiration_belt_force_data = self.create_publisher(Float32, "biosensors/veriner_respiration_belt/force", 10) #raw Force (N)
        
        if self.Parm_Chunk_Enable:
            self.pub_respiration_belt_bpm_chunk_data = self.create_publisher(Float32MultiArray, "biosensors/veriner_respiration_belt/bpm_chunk", 10) #chunk Respiration Rate (bpm)
            self.pub_respiration_belt_force_chunk_data = self.create_publisher(Float32MultiArray, "biosensors/veriner_respiration_belt/force_chunk", 10) #chunk Force (N)

	    #############################################################
        # ROS main loop
        #############################################################
        while rclpy.ok():
            if self.Parm_Sensor_Enable:
                self.gdx.start(self.Parm_Device_Sampling_Rate)                 
                arr_resp_belt_data = self.gdx.read()
                self.pub_respiration_belt_data.publish(Float32MultiArray(data=arr_resp_belt_data))

                #Force is a default channel that is active when the sensor is connected. The force channel measures respiration effort. This is the force exerted by the chest during respiration. Inhalation will be observed as an increase in force. Exhalation will be observed as a decrease in force.
                #Respiration Rate is the other default channel that is active when the sensor is connected. This channel detects inhalations and calculates the number of breaths per minute (BPM). The sample window for the calculation is 30 seconds. The advance interval is 10 seconds. The value will update every 10 seconds.


                self.pub_respiration_belt_force_data.publish(Float32(data=arr_resp_belt_data[0]))
                self.pub_respiration_belt_bpm_data.publish(Float32(data=arr_resp_belt_data[1]))

                if self.Parm_Chunk_Enable:
                    self.force_chunk_data.append(arr_resp_belt_data[0])
                    self.bpm_chunk_data.append(arr_resp_belt_data[1])

                    #print(self.bvp_data_index, stream_id.name, timestamp, sample) #For test
                    if self.data_index  == self.Parm_Chunk_Length - 1:
                        # publish chunk
                        self.pub_respiration_belt_force_chunk_data.publish(Float32MultiArray(data = self.force_chunk_data))
                        self.pub_respiration_belt_bpm_chunk_data.publish(Float32MultiArray(data = self.bpm_chunk_data))
                        
                        #Reset index and ref_array
                        self.force_chunk_data = []
                        self.bpm_chunk_data = []
                        self.data_index = 0
                    else:
                        self.data_index += 1
            else:
                self.gdx.stop()
                pass  

            rclpy.spin(self)

def main(args=None):
    rclpy.init(args=args)
    vernier_resp_belt_node = ros2_vernier_respiration_belt()
    try:
        while rclpy.ok():
            rclpy.spin(vernier_resp_belt_node)
    except KeyboardInterrupt:
        gdx.close() 
        print('repeater stopped cleanly')
    except BaseException:
        gdx.close() 
        print('exception in repeater:', file=sys.stderr)
        raise
    finally:        
        gdx.close() 
        vernier_resp_belt_node.destroy_node()
        rclpy.shutdown() 

if __name__ == '__main__':
    main()


    

