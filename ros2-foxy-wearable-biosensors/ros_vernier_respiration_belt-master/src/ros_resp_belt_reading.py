#!/usr/bin/env python3
#[SMART LAB] ROS template by Wonse Jo
import roslib
import sys
import rospy

from std_msgs.msg import Float32MultiArray, Bool

# For Godirect libs.
from gdx import gdx
# define information of Veriner sensors 
sensor_name = 'GDX-RB 0K2002Z5' # change name with your device __name__
sensor_sampling_rate = 10 #unit = milliseconds 

gdx = gdx.gdx()  
gdx.open_ble(sensor_name)
# 1: Force (N), 2: Respiration Rate (bpm), 4: Steps (steps), 5: Step Rate (spm)
gdx.select_sensors([1,2,4,5]) 


class veriner_resp_belt_reading():
    #############################################################
    # Subscriber Functions
    #############################################################
    def sub_respiration_belt_state_callback(self,msg): #For a callback function
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
        self.pub_respiration_belt_data = rospy.Publisher("respiration_belt/data", Float32MultiArray, queue_size=10)
        #############################################################
        # Subscriber Parts
        #############################################################
        # Set up your subscriber and define its callback
        self.sub_h10_request_state = rospy.Subscriber("/respiration_belt/status", Bool, self.sub_respiration_belt_state_callback)


	    #############################################################
        # ROS main loop
        #############################################################
        r = rospy.Rate(sensor_sampling_rate) # 10hz
        while not rospy.is_shutdown():
            if self.sensor_status == True:
                gdx.start(sensor_sampling_rate)                 
                arr_resp_belt_data = gdx.read()
                self.pub_respiration_belt_data.publish(Float32MultiArray(data=arr_resp_belt_data))
            else:
                gdx.stop()
                pass

            r.sleep()
        gdx.close()   

        rospy.spin()    # Spin until ctrl + c

if __name__ == '__main__':
    rospy.init_node('ros_resp_belt_reading', anonymous=True)
    try:
        ic = veriner_resp_belt_reading()
    except KeyboardInterrupt:
        print("Shutting down")

