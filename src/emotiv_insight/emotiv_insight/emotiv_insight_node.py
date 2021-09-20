## This ROS2 node for reading Emotiv Insight is developed based on an example of cortex-v2. If you want to see more detail without ROS2, please visit the emotive github; https://github.com/Emotiv/cortex-v2-example
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter


from geometry_msgs.msg import Twist, Quaternion, Vector3
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout
from sensor_msgs.msg import Imu, MagneticField


#Emotiv Lib
# Need to install emotiva App from emotiv
import websocket #'pip install websocket-client' for install
from datetime import datetime
import json
import ssl
import time
import sys
import numpy as np


# define request id
QUERY_HEADSET_ID                    =   1
CONNECT_HEADSET_ID                  =   2
REQUEST_ACCESS_ID                   =   3
AUTHORIZE_ID                        =   4
CREATE_SESSION_ID                   =   5
SUB_REQUEST_ID                      =   6
SETUP_PROFILE_ID                    =   7
QUERY_PROFILE_ID                    =   8
TRAINING_ID                         =   9
DISCONNECT_HEADSET_ID               =   10
CREATE_RECORD_REQUEST_ID            =   11
STOP_RECORD_REQUEST_ID              =   12
EXPORT_RECORD_ID                    =   13
INJECT_MARKER_REQUEST_ID            =   14
SENSITIVITY_REQUEST_ID              =   15
MENTAL_COMMAND_ACTIVE_ACTION_ID     =   16
MENTAL_COMMAND_BRAIN_MAP_ID         =   17
MENTAL_COMMAND_TRAINING_THRESHOLD   =   18


# Emotiv License Information (you should have the emotiv license from emotiv website)
user_details = {
	"license" : "your emotivpro license, which could use for third party app",
	"client_id" : "your client id",
	"client_secret" : "your client secret",
	"debit" : 100
}

user = user_details

class ros2_emotiv_insight(Node):
    def __init__(self, user, debug_mode=False):
        super().__init__('emotiv_insight_node')
        
        # For the generlized structure of ROS2 Node
        self.declare_parameter('Sensor_Enable', True) # Enable to publish sensor data (true)
        self.Parm_Sensor_Enable = self.get_parameter('Sensor_Enable').value 
        self.declare_parameter('Chunk_Enable', True) # Enable to publish chunk data (true)
        self.Parm_Chunk_Enable = self.get_parameter('Chunk_Enable').value 
        self.declare_parameter('Chunk_Length', 128) # Define the length of the chunk data
        self.Parm_Chunk_Length = self.get_parameter('Chunk_Length').value


        # Emotiv Setting
        url = "wss://localhost:6868"
        self.ws = websocket.create_connection(url,
                                            sslopt={"cert_reqs": ssl.CERT_NONE})
        self.user = user
        self.debug = debug_mode

        self.data_size_eeg_chunk = self.Parm_Chunk_Length - 1 # Sample rate: 1 Hz
        self.data_size_pow_chunk = int((self.Parm_Chunk_Length - 1)/16) # Sample rate: 1 Hz

        self.bool_act_chunk_eeg = False
        self.bool_act_chunk_pow = False

        self.do_prepare_steps()     

        self.imu_msg = Imu()
        self.mag_msg = MagneticField()

        self.eeg_dimension_layout= MultiArrayLayout()
        self.eeg_dimension_layout.dim.append(MultiArrayDimension(label = "[COUNTER,INTERPOLATED,AF3,T7,Pz,T8,AF4,RAW_CQ]", size = 8))

        self.pow_dimension_layout= MultiArrayLayout()
        self.pow_dimension_layout.dim.append(MultiArrayDimension(label = "[AF3/theta, AF3/alpha, AF3/betaL, AF3/betaH, AF3/gamma, T7/theta, T7/alpha, T7/betaL, T7/betaH, T7/gamma, Pz/theta, Pz/alpha, Pz/betaL, Pz/betaH, Pz/gamma,T8/theta, T8/alpha, T8/betaL, T8/betaH, T8/gamma, AF4/theta, AF4/alpha, AF4/betaL, AF4/betaH, AF4/gamma]", size = 25))


        # let's build a matrix for EEG Chunk:
        self.eeg_chunk_msg = Float32MultiArray()
        self.eeg_chunk_msg.layout.dim.append(MultiArrayDimension())
        self.eeg_chunk_msg.layout.dim.append(MultiArrayDimension())
        self.eeg_chunk_msg.layout.dim[0].label = "['COUNTER','INTERPOLATED','AF3','T7','Pz','T8','AF4','RAW_CQ']"
        self.eeg_chunk_msg.layout.dim[1].label = "data length" #data size 128 (default)

        #"COUNTER","INTERPOLATED","AF3","T7","Pz","T8","AF4","RAW_CQ"
        self.eeg_chunk_msg.layout.dim[0].size = 8
        self.eeg_chunk_msg.layout.dim[1].size = self.data_size_eeg_chunk+1
        self.eeg_chunk_msg.layout.dim[0].stride = 8*(self.data_size_eeg_chunk+1)
        self.eeg_chunk_msg.layout.dim[1].stride = 8
        self.eeg_chunk_msg.layout.data_offset = 0

        self.eeg_chunk_msg_data = np.zeros(8*(self.data_size_eeg_chunk + 1), dtype=np.float32)


        # let's build a matrix for pow Chunk:
        self.pow_chunk_msg = Float32MultiArray()
        self.pow_chunk_msg.layout.dim.append(MultiArrayDimension())
        self.pow_chunk_msg.layout.dim.append(MultiArrayDimension())
        self.pow_chunk_msg.layout.dim[0].label = "[AF3/theta, AF3/alpha, AF3/betaL, AF3/betaH, AF3/gamma, T7/theta, T7/alpha, T7/betaL, T7/betaH, T7/gamma, Pz/theta, Pz/alpha, Pz/betaL, Pz/betaH, Pz/gamma,T8/theta, T8/alpha, T8/betaL, T8/betaH, T8/gamma, AF4/theta, AF4/alpha, AF4/betaL, AF4/betaH, AF4/gamma]"
        self.pow_chunk_msg.layout.dim[1].label = "data length" #data size 128 (default)

        #"COUNTER","INTERPOLATED","AF3","T7","Pz","T8","AF4","RAW_CQ"
        self.pow_chunk_msg.layout.dim[0].size = 25
        self.pow_chunk_msg.layout.dim[1].size = self.data_size_pow_chunk+1
        self.pow_chunk_msg.layout.dim[0].stride = 25*(self.data_size_pow_chunk+1)
        self.pow_chunk_msg.layout.dim[1].stride = 25
        self.pow_chunk_msg.layout.data_offset = 0

        self.pow_chunk_msg_data = np.zeros(25*(self.data_size_pow_chunk + 1), dtype=np.float32)

        
        
        if self.Parm_Sensor_Enable:
            #====================================================#
            ####  Publisher Section                           ####
            #====================================================#
            self.pub_emotiv_insight_eeg= self.create_publisher(Float32MultiArray, 'biosensors/emotiv_insight/eeg', 10) #The raw EEG data from the headset.
            self.pub_emotiv_insight_mot = self.create_publisher(Float32MultiArray, 'biosensors/emotiv_insight/mot', 10) #The motion data from the headset.

            self.pub_emotiv_insight_mot_imu = self.create_publisher(Imu, 'biosensors/emotiv_insight/mot_imu', 10) #The motion data from the headset.
            self.pub_emotiv_insight_mot_mag = self.create_publisher(MagneticField, 'biosensors/emotiv_insight/mot_mag', 10) #The motion data from the headset.

            self.pub_emotiv_insight_pow = self.create_publisher(Float32MultiArray, 'biosensors/emotiv_insight/pow', 10) #The band power of each EEG sensor. It includes the alpha, low beta, high beta, gamma, and theta bands.
            self.pub_emotiv_insight_met = self.create_publisher(Float32MultiArray, 'biosensors/emotiv_insight/met', 10) #The results of the performance metrics detection.
            self.pub_emotiv_insight_dev = self.create_publisher(Float32MultiArray, 'biosensors/emotiv_insight/dev', 10) #It includes the battery level, the wireless signal strength, and the contact quality of each EEG sensor..

            if self.Parm_Chunk_Enable:
                self.pub_emotiv_insight_eeg_chunk= self.create_publisher(Float32MultiArray, 'biosensors/emotiv_insight/eeg_chunk', 10) #The raw EEG data from the headset.
                self.pub_emotiv_insight_pow_chunk= self.create_publisher(Float32MultiArray, 'biosensors/emotiv_insight/pow_chunk', 100) #The raw EEG data from the headset.
                

            # Streaming data
            '''
            #eeg= The raw EEG data from the headset.
            #mot=The motion data from the headset.
            #pow=The band power of each EEG sensor. It includes the alpha, low beta, high beta, gamma, and theta bands.
            #met=The results of the performance metrics detection.
            #com=The results of the mental commands detection. You must load a profile to get meaningful results.
            #fac=The results of the facial expressions detection.
            #sys=The system events. These events are related to the training of the mental commands and facial expressions. See BCI for details.
            '''
            streams = ['eeg', 'mot', 'pow', 'met', 'dev']
            #streams = ['met', 'dev']

            self.sub_request(streams)

    def query_headset(self):
        #print('query headset --------------------------------')        
        query_headset_request = {
            "jsonrpc": "2.0", 
            "id": QUERY_HEADSET_ID,
            "method": "queryHeadsets",
            "params": {}
        }

        self.ws.send(json.dumps(query_headset_request, indent=4))
        result = self.ws.recv()
        result_dic = json.loads(result)

        self.headset_id = result_dic['result'][0]['id']
        if self.debug:
            print('query headset result', json.dumps(result_dic, indent=4))            
            print(self.headset_id)
    def connect_headset(self):
        #print('connect headset --------------------------------')        
        connect_headset_request = {
            "jsonrpc": "2.0", 
            "id": CONNECT_HEADSET_ID,
            "method": "controlDevice",
            "params": {
                "command": "connect",
                "headset": self.headset_id
            }
        }

        self.ws.send(json.dumps(connect_headset_request, indent=4))
        result = self.ws.recv()
        result_dic = json.loads(result)

        if self.debug:
            print('connect headset result', json.dumps(result_dic, indent=4))
    def request_access(self):
        #print('request access --------------------------------')
        request_access_request = {
            "jsonrpc": "2.0", 
            "method": "requestAccess",
            "params": {
                "clientId": self.user['client_id'], 
                "clientSecret": self.user['client_secret']
            },
            "id": REQUEST_ACCESS_ID
        }

        self.ws.send(json.dumps(request_access_request, indent=4))
        result = self.ws.recv()
        result_dic = json.loads(result)

        if self.debug:
            print(json.dumps(result_dic, indent=4))
    def authorize(self):
        #print('authorize --------------------------------')
        authorize_request = {
            "jsonrpc": "2.0",
            "method": "authorize", 
            "params": { 
                "clientId": self.user['client_id'], 
                "clientSecret": self.user['client_secret'], 
                "license": self.user['license'],
                "debit": self.user['debit']
            },
            "id": AUTHORIZE_ID
        }

        if self.debug:
            print('auth request \n', json.dumps(authorize_request, indent=4))

        self.ws.send(json.dumps(authorize_request))
        
        while True:
            result = self.ws.recv()
            result_dic = json.loads(result)
            if 'id' in result_dic:
                if result_dic['id'] == AUTHORIZE_ID:
                    if self.debug:
                        print('auth result \n', json.dumps(result_dic, indent=4))
                    self.auth = result_dic['result']['cortexToken']
                    break
    def create_session(self, auth, headset_id):
        #print('create session --------------------------------')
        create_session_request = { 
            "jsonrpc": "2.0",
            "id": CREATE_SESSION_ID,
            "method": "createSession",
            "params": {
                "cortexToken": self.auth,
                "headset": self.headset_id,
                "status": "active"
            }
        }
        
        if self.debug:
            print('create session request \n', json.dumps(create_session_request, indent=4))

        self.ws.send(json.dumps(create_session_request))
        result = self.ws.recv()
        result_dic = json.loads(result)

        if self.debug:
            print('create session result \n', json.dumps(result_dic, indent=4))

        self.session_id = result_dic['result']['id']
    def close_session(self):
        #print('close session --------------------------------')
        close_session_request = { 
            "jsonrpc": "2.0",
            "id": CREATE_SESSION_ID,
            "method": "updateSession",
            "params": {
                "cortexToken": self.auth,
                "session": self.session_id,
                "status": "close"
            }
        }

        self.ws.send(json.dumps(close_session_request))
        result = self.ws.recv()
        result_dic = json.loads(result)

        if self.debug:
            print('close session result \n', json.dumps(result_dic, indent=4))
    def get_cortex_info(self):
        #print('get cortex version --------------------------------')
        get_cortex_info_request = {
            "jsonrpc": "2.0",
            "method": "getCortexInfo",
            "id":100
        }

        self.ws.send(json.dumps(get_cortex_info_request))        
        result = self.ws.recv()
        if self.debug:
            print(json.dumps(json.loads(result), indent=4))
    def do_prepare_steps(self):
        self.query_headset()
        self.connect_headset()
        self.request_access()
        self.authorize()
        self.create_session(self.auth, self.headset_id)
    def disconnect_headset(self):
        #print('disconnect headset --------------------------------')
        disconnect_headset_request = {
            "jsonrpc": "2.0", 
            "id": DISCONNECT_HEADSET_ID,
            "method": "controlDevice",
            "params": {
                "command": "disconnect",
                "headset": self.headset_id
            }
        }

        self.ws.send(json.dumps(disconnect_headset_request))

        # wait until disconnect completed
        while True:
            time.sleep(1)
            result = self.ws.recv()
            result_dic = json.loads(result)
            
            if self.debug:
                print('disconnect headset result', json.dumps(result_dic, indent=4))

            if 'warning' in result_dic:
                if result_dic['warning']['code'] == 1:
                    break
    def sub_request(self, stream):
        #print('subscribe request --------------------------------')
        sub_request_json = {
            "jsonrpc": "2.0", 
            "method": "subscribe", 
            "params": { 
                "cortexToken": self.auth,
                "session": self.session_id,
                "streams": stream
            }, 
            "id": SUB_REQUEST_ID
        }

        self.ws.send(json.dumps(sub_request_json))
        
        if 'sys' in stream:
            new_data = self.ws.recv()
            #print(json.dumps(new_data, indent=4))
            #print('\n')
        else:
            while rclpy.ok():
                ## For test
                new_data = self.ws.recv()  
                #print(new_data)

                #############
                if new_data.find('jsonrpc') > 0:
                    pass

                elif new_data.find('eeg') > 0:
                    #["COUNTER","INTERPOLATED","AF3","T7","Pz","T8","AF4","RAW_CQ","MARKER_HARDWARE","MARKERS"]
                    eeg_data = new_data[new_data.find('eeg":[')+6:new_data.find(',[]],"sid')-1]
                    eeg_data= np.fromstring(eeg_data, dtype=np.float32, sep=',') 
                    
                    #["COUNTER","INTERPOLATED","AF3","T7","Pz","T8","AF4","RAW_CQ"]
                    eeg_data = eeg_data[0:8]

                    if self.Parm_Chunk_Enable:
                        if eeg_data[0:1] == 0:
                            # start to make a chunk dataset
                            self.bool_act_chunk_eeg = True
                            eeg_data_chunk = eeg_data                        
                        else:
                            if self.bool_act_chunk_eeg == True:
                                eeg_data_chunk = np.vstack((eeg_data_chunk, eeg_data))
                                if len(eeg_data_chunk) > self.data_size_eeg_chunk:
                                    #print(eeg_data_chunk, len(eeg_data_chunk), type(eeg_data_chunk[0,0]))
                                    row_length = self.eeg_chunk_msg.layout.dim[0].size #128
                                    col_length = self.eeg_chunk_msg.layout.dim[1].size #8
                                    dstride1 = self.eeg_chunk_msg.layout.dim[1].stride #8
                                    for i in range(col_length):
                                        for j in range(row_length):
                                            self.eeg_chunk_msg_data[i*row_length + j] = eeg_data_chunk[i,j]
                                    self.eeg_chunk_msg.data = self.eeg_chunk_msg_data.tolist()

                                    self.pub_emotiv_insight_eeg_chunk.publish(self.eeg_chunk_msg)

                                    # Reset chunk data and type
                                    eeg_data_chunk = None
                                    self.bool_act_chunk_eeg = False
                            else:
                                pass

                    self.pub_emotiv_insight_eeg.publish(Float32MultiArray(layout=self.eeg_dimension_layout, data = eeg_data))

                elif new_data.find('mot') > 0:
                    #["COUNTER_MEMS","INTERPOLATED_MEMS","Q0","Q1","Q2","Q3","ACCX","ACCY","ACCZ","MAGX","MAGY","MAGZ"]
                    mot_data = new_data[new_data.find('mot":[')+6:new_data.find(',[]],"sid')-1]
                    mot_data= np.fromstring(mot_data, dtype=np.float32, sep=',')
                    #["Q0","Q1","Q2","Q3","ACCX","ACCY","ACCZ","MAGX","MAGY","MAGZ"]
                    #print(mot_data[2:12])  

                    self.pub_emotiv_insight_mot.publish(Float32MultiArray(data = mot_data[2:12])) # raw mot data

                    #print(mot_data[2:6]) # Quaternions of the gyroscope = Q0, Q1, Q2, Q3                    
                    self.imu_msg.orientation.x = float(mot_data[2:3])
                    self.imu_msg.orientation.y = float(mot_data[3:4])
                    self.imu_msg.orientation.z = float(mot_data[4:5])
                    self.imu_msg.orientation.w = float(mot_data[5:6])
                    
                    #print(mot_data[6:9]) # X, Y, Z axis of the accelerometer
                    self.imu_msg.linear_acceleration.x = float(mot_data[6:7])
                    self.imu_msg.linear_acceleration.y = float(mot_data[7:8])
                    self.imu_msg.linear_acceleration.z = float(mot_data[8:9])
                    self.pub_emotiv_insight_mot_imu.publish(self.imu_msg)
                                                         
                    # Magnetometer Pub.
                    #print(mot_data[9:12]) # X, Y, Z axis of the magnetometer.
                    self.mag_msg.magnetic_field.x = float(mot_data[9:10])
                    self.mag_msg.magnetic_field.y = float(mot_data[10:11])
                    self.mag_msg.magnetic_field.z = float(mot_data[11:12])
                    self.pub_emotiv_insight_mot_mag.publish(self.mag_msg)

                    #eeg_time_data = new_data[new_data.find('time":')+6:new_data.find('}')]

                elif new_data.find('dev') > 0:
                    # For Insight Device
                    #["Battery","Signal",["AF3","T7","Pz","T8","AF4","OVERALL"],"BatteryPercent"]
                    dev_data = new_data[new_data.find('dev":[')+6:new_data.find(',"sid')-1]
                    dev_data = dev_data.split(',')
                    #["Battery","Signal","BatteryPercent"]
                    #Battery = dev_data[0].split('[')
                    Battery = float(dev_data[0])

                    Signal = float(dev_data[1])

                    Pin_signal_AF3 = dev_data[2].split('[')
                    Pin_signal_AF3 = float(Pin_signal_AF3[1])
                    Pin_signal_T7 = float(dev_data[3])
                    Pin_signal_Pz = float(dev_data[4])
                    Pin_signal_T8 = float(dev_data[5])
                    Pin_signal_AF4 = float(dev_data[6])
                    Pin_signal_OVERALL = dev_data[7].split(']')
                    Pin_signal_OVERALL = float(Pin_signal_OVERALL[0])

                    BatteryPercent = dev_data[8].split(']')
                    BatteryPercent = float(BatteryPercent[0])
                    
                    #dev_data = ["Battery","Signal", "AF3_Signal","T7_Signal","Pz_Signal","T8_Signal","AF4_Signal","OVERALL_Signal","BatteryPercent"]
                    dev_data = [Battery, Signal, Pin_signal_AF3, Pin_signal_T7, Pin_signal_Pz, Pin_signal_T8, Pin_signal_AF4, Pin_signal_OVERALL, BatteryPercent]
                    self.pow_dimension_layout
                    self.pub_emotiv_insight_dev.publish(Float32MultiArray(data =dev_data))
                    #dev_time_data = new_data[new_data.find('time":')+6:new_data.find('}')]
                
                elif new_data.find('pow') > 0:
                    pow_data = new_data[new_data.find('pow":[')+6:new_data.find(',"sid')-1]
                    pow_data= np.fromstring(pow_data, dtype=np.float32, sep=',')         
                    #["AF3/theta","AF3/alpha","AF3/betaL","AF3/betaH","AF3/gamma","T7/theta","T7/alpha","T7/betaL","T7/betaH","T7/gamma","Pz/theta","Pz/alpha","Pz/betaL","Pz/betaH","Pz/gamma","T8/theta","T8/alpha","T8/betaL","T8/betaH","T8/gamma","AF4/theta","AF4/alpha","AF4/betaL","AF4/betaH","AF4/gamma"]     

                    #["COUNTER","INTERPOLATED","AF3","T7","Pz","T8","AF4","RAW_CQ"]

                    if self.Parm_Chunk_Enable:
                        if not self.bool_act_chunk_pow:
                            # start to make a chunk dataset
                            self.bool_act_chunk_pow = True
                            pow_data_chunk = pow_data                        
                        else:
                            if self.bool_act_chunk_pow == True:
                                pow_data_chunk = np.vstack((pow_data_chunk, pow_data))
                                #print(len(pow_data_chunk),pow_data_chunk)
                                if len(pow_data_chunk) > self.data_size_pow_chunk:
                                    #print(pow_data_chunk, len(pow_data_chunk), type(pow_data_chunk[0,0]))
                                    row_length = self.pow_chunk_msg.layout.dim[0].size 
                                    col_length = self.pow_chunk_msg.layout.dim[1].size 
                                    dstride1 = self.pow_chunk_msg.layout.dim[1].stride 
                                    #print(row_length, col_length)
                                    for i in range(col_length):
                                        for j in range(row_length):
                                            self.pow_chunk_msg_data[i*row_length + j] = pow_data_chunk[i,j]
                                    self.pow_chunk_msg.data = self.pow_chunk_msg_data.tolist()

                                    self.pub_emotiv_insight_pow_chunk.publish(self.pow_chunk_msg)

                                    # Reset chunk data and type
                                    pow_data_chunk = None
                                    self.bool_act_chunk_pow = False
                            else:
                                pass


                    self.pub_emotiv_insight_pow.publish(Float32MultiArray(layout=self.pow_dimension_layout, data=pow_data))

                    #pow_time_data = new_data[new_data.find('time":')+6:new_data.find('}')]
                
                elif new_data.find('met') > 0:
                    met_data = new_data[new_data.find('met":[')+6:new_data.find(',"sid')-1]
                    #met_time_data = new_data[new_data.find('time":')+6:new_data.find('}')]

                    #["eng.isActive","eng","exc.isActive","exc","lex","str.isActive","str","rel.isActive","rel","int.isActive","int","foc.isActive","foc"]
                    print(met_data, met_data[1], type(met_data[1]))
                    met_data = met_data.split(',')
                    if met_data[1] == "null":
                        met_data = [0, 0, 0, 0, 0, 0, 0]
                    else:                                            
                        Engagement_data = float(met_data[1])
                        Excitement_data = float(met_data[3])
                        Long_term_excitement_data  = float(met_data[4])
                        Stress_Frustration_data  = float(met_data[6])
                        Relaxation_data  = float(met_data[8])
                        Interest_Affinity_data  = float(met_data[10])
                        Focus_data = float(met_data[12])

                        met_data = [Engagement_data, Excitement_data, Long_term_excitement_data, Stress_Frustration_data, Relaxation_data, Interest_Affinity_data, Focus_data]
                        # Decimal number between 0 and 1. 
                        # Zero means "low power".
                        # 1 means "high power". 
                        #Engagement_data, Excitement_data, Long_term_excitement_data, Stress_Frustration_data, Relaxation_data, Interest_Affinity_data, Focus_data
                    self.pub_emotiv_insight_met.publish(Float32MultiArray(data=met_data))
                else:
                    print("Unknown Error")
    def query_profile(self):
        #print('query profile --------------------------------')
        query_profile_json = {
            "jsonrpc": "2.0",
            "method": "queryProfile",
            "params": {
              "cortexToken": self.auth,
            },
            "id": QUERY_PROFILE_ID
        }

        if self.debug:
            print('query profile request \n', json.dumps(query_profile_json, indent=4))
            print('\n')

        self.ws.send(json.dumps(query_profile_json))

        result = self.ws.recv()
        result_dic = json.loads(result)

        print('query profile result\n',result_dic)
        print('\n')

        profiles = []
        for p in result_dic['result']:
            profiles.append(p['name'])

        print('extract profiles name only')        
        print(profiles)
        print('\n')

        return profiles
    def setup_profile(self, profile_name, status):
        #print('setup profile --------------------------------')
        setup_profile_json = {
            "jsonrpc": "2.0",
            "method": "setupProfile",
            "params": {
              "cortexToken": self.auth,
              "headset": self.headset_id,
              "profile": profile_name,
              "status": status
            },
            "id": SETUP_PROFILE_ID
        }
        
        if self.debug:
            print('setup profile json:\n', json.dumps(setup_profile_json, indent=4))
            print('\n')

        self.ws.send(json.dumps(setup_profile_json))

        result = self.ws.recv()
        result_dic = json.loads(result)

        if self.debug:
            print('result \n', json.dumps(result_dic, indent=4))
            print('\n')
    def train_request(self, detection, action, status):
        # print('train request --------------------------------')
        train_request_json = {
            "jsonrpc": "2.0", 
            "method": "training", 
            "params": {
              "cortexToken": self.auth,
              "detection": detection,
              "session": self.session_id,
              "action": action,
              "status": status
            }, 
            "id": TRAINING_ID
        }

        # print('training request:\n', json.dumps(train_request_json, indent=4))
        # print('\n')

        self.ws.send(json.dumps(train_request_json))
        
        if detection == 'mentalCommand':
            start_wanted_result = 'MC_Succeeded'
            accept_wanted_result = 'MC_Completed'

        if detection == 'facialExpression':
            start_wanted_result = 'FE_Succeeded'
            accept_wanted_result = 'FE_Completed'

        if status == 'start':
            wanted_result = start_wanted_result
            print('\n YOU HAVE 8 SECONDS FOR TRAIN ACTION {} \n'.format(action.upper()))

        if status == 'accept':
            wanted_result = accept_wanted_result

        # wait until success
        while True:
            result = self.ws.recv()
            result_dic = json.loads(result)

            print(json.dumps(result_dic, indent=4))

            if 'sys' in result_dic:
                # success or complete, break the wait
                if result_dic['sys'][1]==wanted_result:
                    break
    def create_record(self,
                    record_name,
                    record_description):
        #print('create record --------------------------------')
        create_record_request = {
            "jsonrpc": "2.0", 
            "method": "createRecord",
            "params": {
                "cortexToken": self.auth,
                "session": self.session_id,
                "title": record_name,
                "description": record_description
            }, 

            "id": CREATE_RECORD_REQUEST_ID
        }

        self.ws.send(json.dumps(create_record_request))
        result = self.ws.recv()
        result_dic = json.loads(result)

        if self.debug:
            print('start record request \n',
                    json.dumps(create_record_request, indent=4))
            print('start record result \n',
                    json.dumps(result_dic, indent=4))

        self.record_id = result_dic['result']['record']['uuid']
    def stop_record(self):
        #print('stop record --------------------------------')
        stop_record_request = {
            "jsonrpc": "2.0", 
            "method": "stopRecord",
            "params": {
                "cortexToken": self.auth,
                "session": self.session_id
            }, 

            "id": STOP_RECORD_REQUEST_ID
        }
        
        self.ws.send(json.dumps(stop_record_request))
        result = self.ws.recv()
        result_dic = json.loads(result)

        if self.debug:
            print('stop request \n',
                json.dumps(stop_record_request, indent=4))
            print('stop result \n',
                json.dumps(result_dic, indent=4))
    def export_record(self, 
                    folder, 
                    export_types, 
                    export_format,
                    export_version,
                    record_ids):
        #print('export record --------------------------------')
        export_record_request = {
            "jsonrpc": "2.0",
            "id":EXPORT_RECORD_ID,
            "method": "exportRecord", 
            "params": {
                "cortexToken": self.auth, 
                "folder": folder,
                "format": export_format,
                "streamTypes": export_types,
                "recordIds": record_ids
            }
        }

        # "version": export_version,
        if export_format == 'CSV':
            export_record_request['params']['version'] = export_version

        if self.debug:
            print('export record request \n',
                json.dumps(export_record_request, indent=4))
        
        self.ws.send(json.dumps(export_record_request))

        # wait until export record completed
        while True:
            time.sleep(1)
            result = self.ws.recv()
            result_dic = json.loads(result)

            if self.debug:            
                print('export record result \n',
                    json.dumps(result_dic, indent=4))

            if 'result' in result_dic:
                if len(result_dic['result']['success']) > 0:
                    break
    def inject_marker_request(self, marker):
        #print('inject marker --------------------------------')
        inject_marker_request = {
            "jsonrpc": "2.0",
            "id": INJECT_MARKER_REQUEST_ID,
            "method": "injectMarker", 
            "params": {
                "cortexToken": self.auth, 
                "session": self.session_id,
                "label": marker['label'],
                "value": marker['value'], 
                "port": marker['port'],
                "time": marker['time']
            }
        }

        self.ws.send(json.dumps(inject_marker_request))
        result = self.ws.recv()
        result_dic = json.loads(result)

        if self.debug:
            print('inject marker request \n', json.dumps(inject_marker_request, indent=4))
            print('inject marker result \n',
                json.dumps(result_dic, indent=4))
    def get_mental_command_action_sensitivity(self, profile_name):
        #print('get mental command sensitivity ------------------')
        sensitivity_request = {
            "id": SENSITIVITY_REQUEST_ID,
            "jsonrpc": "2.0",
            "method": "mentalCommandActionSensitivity",
            "params": {
                "cortexToken": self.auth,
                "profile": profile_name,
                "status": "get"
            }
        }

        self.ws.send(json.dumps(sensitivity_request))
        result = self.ws.recv()
        result_dic = json.loads(result)

        if self.debug:
            print(json.dumps(result_dic, indent=4))

        return result_dic
    def set_mental_command_action_sensitivity(self, 
                                            profile_name, 
                                            values):
        #print('set mental command sensitivity ------------------')
        sensitivity_request = {
                                "id": SENSITIVITY_REQUEST_ID,
                                "jsonrpc": "2.0",
                                "method": "mentalCommandActionSensitivity",
                                "params": {
                                    "cortexToken": self.auth,
                                    "profile": profile_name,
                                    "session": self.session_id,
                                    "status": "set",
                                    "values": values
                                }
                            }

        self.ws.send(json.dumps(sensitivity_request))
        result = self.ws.recv()
        result_dic = json.loads(result)

        if self.debug:
            print(json.dumps(result_dic, indent=4))

        return result_dic
    def get_mental_command_active_action(self, profile_name):
        #print('get mental command active action ------------------')
        command_active_request = {
            "id": MENTAL_COMMAND_ACTIVE_ACTION_ID,
            "jsonrpc": "2.0",
            "method": "mentalCommandActiveAction",
            "params": {
                "cortexToken": self.auth,
                "profile": profile_name,
                "status": "get"
            }
        }

        self.ws.send(json.dumps(command_active_request))
        result = self.ws.recv()
        result_dic = json.loads(result)

        if self.debug:
            print(json.dumps(result_dic, indent=4))

        return result_dic
    def get_mental_command_brain_map(self, profile_name):
        #print('get mental command brain map ------------------')
        brain_map_request = {
            "id": MENTAL_COMMAND_BRAIN_MAP_ID,
            "jsonrpc": "2.0",
            "method": "mentalCommandBrainMap",
            "params": {
                "cortexToken": self.auth,
                "profile": profile_name,
                "session": self.session_id
            }
        }

        self.ws.send(json.dumps(brain_map_request))
        result = self.ws.recv()
        result_dic = json.loads(result)

        if self.debug:
            print(json.dumps(result_dic, indent=4))

        return result_dic
    def get_mental_command_training_threshold(self, profile_name):
        #print('get mental command training threshold -------------')
        training_threshold_request = {
            "id": MENTAL_COMMAND_TRAINING_THRESHOLD,
            "jsonrpc": "2.0",
            "method": "mentalCommandTrainingThreshold",
            "params": {
                "cortexToken": self.auth,
                "session": self.session_id
            }
        }

        self.ws.send(json.dumps(training_threshold_request))
        result = self.ws.recv()
        result_dic = json.loads(result)

        if self.debug:
            print(json.dumps(result_dic, indent=4))

        return result_dic


def main(args=None):
    rclpy.init(args=args)
    emotiv_insight_node = ros2_emotiv_insight(user, debug_mode=False)
    try:
        while rclpy.ok():
            rclpy.spin(emotiv_insight_node)
            emotiv_insight_node.disconnect_headset()

    except KeyboardInterrupt:
        print('repeater stopped cleanly')
        
    except BaseException:
        print('exception in repeater:', file=sys.stderr)
        raise

    finally:        
        emotiv_insight_node.destroy_node()
        rclpy.shutdown() 

if __name__ == '__main__':
    main()
