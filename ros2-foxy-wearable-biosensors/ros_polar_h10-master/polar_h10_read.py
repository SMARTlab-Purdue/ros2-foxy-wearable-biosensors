#!/usr/bin/env python
#[SMART LAB] ROS template by Wonse Jo
import roslib
roslib.load_manifest('polar_h10_ros')
import sys
import rospy
from std_msgs.msg import Int32MultiArray, Bool

import time
import math
import pexpect

# count the arguments
IMU_MAC_ADDRESS = "C9:61:FF:AC:8E:23" # add a mac address of your polar H10.
    
class polar_h10_read():
    #############################################################
    # Subscriber Functions
    #############################################################
    def sub_h10_request_state_callback(self,msg): #For a callback function
        self.sensor_status = msg.data
        #print(self.sensor_status)
    #############################################################
    # Main Loop
    #############################################################
    def __init__(self):
        #############################################################
        # Global Variables
        #############################################################
        #example: self.dx_1_read_position_data = 0
        self.sensor_status = 0
        #############################################################
        # Publisher Parts
        #############################################################
        self.pub_h10_hrv_data = rospy.Publisher("polar_h10/hrv", Int32MultiArray, queue_size=10)
        #############################################################
        # Subscriber Parts
        #############################################################
        # Set up your subscriber and define its callback
        self.sub_h10_request_state = rospy.Subscriber("/polar_h10/status", Bool, self.sub_h10_request_state_callback)
        
        wonsu_state = True
        while wonsu_state:
            gatt = pexpect.spawn("gatttool -t random -b " + IMU_MAC_ADDRESS+" --char-write-req --handle=0x0011 --value=0100 --listen" )
            check = gatt.readline()
            if check == "Characteristic value was written successfully\r\n":
                line = gatt.readline()
                wonsu=line.split()
                #print(line)       
                if wonsu[5] == "10":
                    print("[Polar] connecting")
                    wonsu_state = False
            else:
                print("[Polar] fail to hack ble")
                pass
        print("[Polar] connected")

	    #############################################################
        # ROS main loop
        #############################################################
        r = rospy.Rate(1) # 10hz
        while not rospy.is_shutdown():
            line = gatt.readline()
            wonsu=line.split()
            #print(int(wonsu[6],16),int(wonsu[7],16),int(wonsu[8],16))
            ## your codes
            if wonsu[5] == "00":
                #print("wonsu_error")
                pass
            else:
                hrv_data = int(wonsu[6],16)
                ibi_data = int(wonsu[7],16)
                bat_level_data = int(wonsu[8],16)
                arr_h10_data = [hrv_data, ibi_data, bat_level_data]
                
                if self.sensor_status == True:
                    #print("enter")
                    self.pub_h10_hrv_data.publish(Int32MultiArray(data=arr_h10_data))
                else:
                    #print("[Polar] Ready")
                    pass

            r.sleep()
        rospy.spin()    # Spin until ctrl + c

if __name__ == '__main__':
    rospy.init_node('polar_h10_ros', anonymous=True)
    try:
        ic = polar_h10_read()
    except KeyboardInterrupt:
        print("Shutting down")

