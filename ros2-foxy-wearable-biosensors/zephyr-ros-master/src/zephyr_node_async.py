#!/usr/bin/env python

import sys
import rospy
import std_msgs.msg
from zephyr_dev import *
#from empatica_e4.src.e4_dev import *
from zephyr.msg import *
from core import BioHarness
import asyncio

class ROSActions(ZephyrDataActions):
    
    def __init__(self):
        super(ROSActions, self).__init__()
        #'topic name', msg type, queue_size = 10 <-standard queue size
        self.acc_pub = rospy.Publisher('acc', Acc3D, queue_size=10)
        self.gsr_pub = rospy.Publisher('gsr', GSR, queue_size=10)
        self.hr_pub = rospy.Publisher('heartrate', Heartrate, queue_size=10)
        self.skinTemp_pub = rospy.Publisher('skintemp', SkinTemp, queue_size=10)
        
        self.acc_time = rospy.Time.now()
        self.gsr_time = rospy.Time.now() 
        self.hr_time = rospy.Time.now() 
        self.skinTemp_time = rospy.Time.now() 
        
                
    def onAcc3D(self, acc):
        self.rate = rospy.Rate(10)
        for i in range(0,len(acc),3):
            msg = Acc3D()
            msg.header = std_msgs.msg.Header()
            msg.header.stamp = rospy.Time.now()
            msg.acc_x = acc[i]
            msg.acc_y = acc[i+1]
            msg.acc_z = acc[i+2]
            self.acc_pub.publish(msg)
            self.rate.sleep()
        
    def onGSR(self, gsr):
        self.rate = rospy.Rate(10)
        for i in range(0,len(gsr),2):
            msg = GSR()
            msg.header = std_msgs.msg.Header()
            msg.header.stamp = rospy.Time.now()
            msg.status = gsr[i]
            msg.resistance = gsr[i+1]
            self.gsr_pub.publish(msg)
            self.rate.sleep()
            
    def onHeartRate(self,heartRate):
        self.rate = rospy.Rate(10)
        
    def onSkinTemp(self, skinTemp):
        #8 msgs per bluetooth payload
        self.rate = rospy.Rate(10)
        
        tm_pts, self.skinTemp_time = self.timeParse(previous_time=self.skinTemp_time,msg_split=8)
        
        for i in range(0,len(skinTemp)):
            msg = SkinTemp()
            msg.header = std_msgs.msg.Header()
            msg.header.stamp = tm_pts[i]
            #msg.header.stamp = rospy.Time.now()
            msg.temp = skinTemp[i]
            self.skinTemp_pub.publish(msg)
            self.rate.sleep()

    @classmethod
    def timeParse(cls, previous_time, msg_split):        
        incoming_time = rospy.Time.now()
        time_step = (incoming_time-previous_time)/(msg_split-1)
        t_dur = int(time_step.nsecs)
        
        t_offset = [rospy.Duration(0,t_dur*i) for i in range(msg_split)]
        t_offset.reverse()
        t_base = [incoming_time for i in range(msg_split)]
        
        return list(map(lambda x,y: x-y, t_base,t_offset)), incoming_time

async def init():
    def __init__(self,addr,actions):
        self.addr = addr
        self.actions = actions
        
    conn = BioHarness(self.addr, port=1, timeout=20)
    
    infos = await conn.get_infos()
    
    for mod in actions.enablers:
        enabler = actions.enablers[mod]
        await enabler(link, nameprefix=args.streamprefix,
                        idprefix=id_prefix, **vars(args))
        
if __name__ == '__main__':
    
    if len(sys.argv) != 2:
        print("Fatal, must pass device address:", sys.argv[0], "<device address="">")
        quit()
    
    addr = sys.argv[1]
    
    try:
        rospy.init_node('zephyr_node', anonymous=True)
        
        actions = ROSActions()
        
        init()
        #conn.notificationSet(conn.zephyr)
        
        
        #notifications = E4Notifications()
        
        while not rospy.is_shutdown():
            if conn.zephyr.waitForNotifications(1.0):
                continue
    
        conn.zephyr.disconnect()

    except rospy.ROSInterruptException:
        pass