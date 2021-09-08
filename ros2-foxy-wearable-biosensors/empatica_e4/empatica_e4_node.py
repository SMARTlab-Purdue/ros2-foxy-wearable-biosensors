## This ROS2 node for reading Empatica E4 is developed based on the Manuel Olguín Muñoz's open-e4-client  (github: molguin92). If you want to see more detail without ROS2, please visit his/her github; https://github.com/molguin92/open-e4-client

#ROS2 Libs.
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from geometry_msgs.msg import Twist, Quaternion, Vector3
from std_msgs.msg import Empty, Float32, Float32MultiArray
from sensor_msgs.msg import Imu

#Emotiv Libs.
## Need to add a window machine for running LSL and connecting with Empatica
import time
from e4client import * # you need to install open-e4-client: $ pip install open-e4-client

#Python Libs.
import sys
import numpy as np


class ros2_empatica_e4(Node):
    def __init__(self):
        super().__init__('empatica_e4_node')

        self.bvp_data_index = 0
        self.bvp_chunk_data = []

        self.gsr_data_index = 0
        self.gsr_chunk_data = []

        self.temp_data_index = 0
        self.temp_chunk_data = []

        # For the generlized structure of ROS2 Node
        self.declare_parameter('Sensor_Enable', True) # Enable to publish sensor data (true)
        self.Parm_Sensor_Enable = self.get_parameter('Sensor_Enable').value 
        self.declare_parameter('Chunk_Enable', True) # Enable to publish chunk data (true)
        self.Parm_Chunk_Enable = self.get_parameter('Chunk_Enable').value 
        self.declare_parameter('Chunk_Length', 128) # Define the length of the chunk data
        self.Parm_Chunk_Length = self.get_parameter('Chunk_Length').value 
        
        # For the E4 streaming server.
        self.declare_parameter('DeviceID', 'CC34CD') # Should be replaced into your device ID.
        self.Parm_DeviceID = self.get_parameter('DeviceID').value 
        self.declare_parameter('E4_Host_IP', '192.168.50.225') # Should be replaced into your Windown machine installed E4 streaming server.
        self.Parm_E4_Host_IP = self.get_parameter('E4_Host_IP').value 
        self.declare_parameter('E4_Host_Port', 28000) # # Should be replaced into your Windown machine installed E4 streaming server.
        self.Parm_E4_Host_Port = self.get_parameter('E4_Host_Port').value 


        #====================================================#
        ####  Publisher Section                           ####
        #====================================================#
        if self.Parm_Sensor_Enable:
            # BVP data 
            self.pub_empatic_e4_bvp = self.create_publisher(Float32, 'biosensors/empatica_e4/bvp', 10) 
            # GSR data
            self.pub_empatic_e4_gsr = self.create_publisher(Float32, 'biosensors/empatica_e4/gsr', 10)         
            # Temperature data
            self.pub_empatic_e4_temp = self.create_publisher(Float32, 'biosensors/empatica_e4/st', 10)         
            # HR data
            self.pub_empatic_e4_hr = self.create_publisher(Float32, 'biosensors/empatica_e4/hr', 10)        
            # IBI data
            self.pub_empatic_e4_ibi = self.create_publisher(Float32, 'biosensors/empatica_e4/ibi', 10)        
            # ACC data
            self.pub_empatic_e4_acc = self.create_publisher(Float32MultiArray, 'biosensors/empatica_e4/acc', 10)        
            # BAT data
            self.pub_empatic_e4_bat = self.create_publisher(Float32, 'biosensors/empatica_e4/bat', 10)      
            # Tag data (button on the E4)
            self.pub_empatic_e4_tag = self.create_publisher(Empty, 'biosensors/empatica_e4/tag', 10) 

        if self.Parm_Sensor_Enable and self.Parm_Chunk_Enable:
            self.pub_empatic_e4_bvp_chunck = self.create_publisher(Float32MultiArray, 'biosensors/empatica_e4/bvp_chunk', 10) 
            self.pub_empatic_e4_gsr_chunck = self.create_publisher(Float32MultiArray, 'biosensors/empatica_e4/gsr_chunk', 10) 
            self.pub_empatic_e4_temp_chunk = self.create_publisher(Float32MultiArray, 'biosensors/empatica_e4/st_chunk', 10)         


        with E4StreamingClient(self.Parm_E4_Host_IP, self.Parm_E4_Host_Port) as client:
            devs = client.list_connected_devices()
            #print(devs[0], type(devs[0]))
            #connecting_devices= deviceID # Was devs[0] \
            try:
                with client.connect_to_device(self.Parm_DeviceID) as conn:
                    if self.Parm_Sensor_Enable:
                        conn.subscribe_to_stream(E4DataStreamID.BVP, self.extract_data)
                        conn.subscribe_to_stream(E4DataStreamID.GSR, self.extract_data)
                        conn.subscribe_to_stream(E4DataStreamID.TEMP, self.extract_data)
                        conn.subscribe_to_stream(E4DataStreamID.HR, self.extract_data)
                        conn.subscribe_to_stream(E4DataStreamID.IBI, self.extract_data)
                        conn.subscribe_to_stream(E4DataStreamID.TAG, self.extract_data)
                        conn.subscribe_to_stream(E4DataStreamID.ACC, self.extract_data)
                        conn.subscribe_to_stream(E4DataStreamID.BAT, self.extract_data)

                    while rclpy.ok():   
                        pass

            except:
                print("Please check or turn on the E4 Streaming server on other machines (recommend using Window).")
                exit()
                
    def extract_data(self, stream_id, timestamp, *sample) -> None:
        if stream_id.name == "BVP":
            # need to make it chuck data            
            self.pub_empatic_e4_bvp.publish(Float32(data = sample[0]))
            if self.Parm_Chunk_Enable:
                self.bvp_chunk_data.append(sample[0])
                if self.bvp_data_index  == self.Parm_Chunk_Length - 1:
                    # publish chunk
                    self.pub_empatic_e4_bvp_chunck.publish(Float32MultiArray(data = self.bvp_chunk_data))
                    #print(self.bvp_chunk_data) #For test
                    
                    #Reset index and ref_array
                    self.bvp_chunk_data = []
                    self.bvp_data_index = 0
                else:
                    self.bvp_data_index += 1

                
        elif stream_id.name == "GSR":
            self.pub_empatic_e4_gsr.publish(Float32(data = sample[0]))
            #print(self.gsr_data_index, stream_id.name, timestamp, sample) #For test
            if self.Parm_Chunk_Enable:
                self.gsr_chunk_data.append(sample[0])
                if self.gsr_data_index == self.Parm_Chunk_Length - 1:
                    self.pub_empatic_e4_gsr_chunck.publish(Float32MultiArray(data = self.gsr_chunk_data))
                    
                    #Reset index and ref_array
                    self.gsr_chunk_data = []
                    self.gsr_data_index = 0
                else:
                    self.gsr_data_index += 1

        elif stream_id.name == "TEMP": # Sampling rate: 
            self.temp_chunk_data.append(sample[0])
            #print(self.temp_data_index, stream_id.name, timestamp, sample) #For test
            if self.temp_data_index  == 7:
                temp_chunk_avg = np.mean(self.temp_chunk_data, dtype=np.float64)
                self.pub_empatic_e4_temp.publish(Float32(data =temp_chunk_avg))
                if self.Parm_Chunk_Enable:                # publish chunk                
                    self.pub_empatic_e4_temp_chunk.publish(Float32MultiArray(data = self.temp_chunk_data))
                    
                #Reset index and ref_array
                self.temp_chunk_data = []
                self.temp_data_index = 0
            else:
                self.temp_data_index += 1
            
        elif stream_id.name == "HR":
            #print(sample)
            self.pub_empatic_e4_hr.publish(Float32(data =sample[0]))

        elif stream_id.name == "IBI":
            #print(sample)
            self.pub_empatic_e4_ibi.publish(Float32(data =sample[0]))
                
        elif stream_id.name == "ACC":
            acc_data = [sample[0], sample[1], sample[2]]
            self.pub_empatic_e4_acc.publish(Float32MultiArray(data = acc_data))
            
            # Need to find more details regarding the accelerometer, then we will update it on next v0.0.2
            #self.acc_imu_msg.header.frame_id = "empatica_e4"
            #self.acc_imu_msg.angular_velocity.x=sample[0]
            #self.acc_imu_msg.angular_velocity.y=sample[1]
            #self.acc_imu_msg.angular_velocity.z=sample[2]           
            #self.pub_empatic_e4_acc_imu.publish(self.acc_imu_msg)

        elif stream_id.name == "TAG":
            self.pub_empatic_e4_tag.publish(Empty())

        elif stream_id.name == "BAT":
            self.pub_empatic_e4_bat.publish(Float32(data = sample[0]))
            #print(sample[0])
        else:
            print("Unknown Errors")

    
def main(args=None):
    rclpy.init(args=args)
    empatica_E4_node = ros2_empatica_e4()
    try:
        while rclpy.ok():
            rclpy.spin(empatica_E4_node)
    except KeyboardInterrupt:
        print('repeater stopped cleanly')
    except BaseException:
        print('exception in repeater:', file=sys.stderr)
        raise
    finally:        
        empatica_E4_node.destroy_node()
        rclpy.shutdown() 

if __name__ == '__main__':
    main()


    

