#!/usr/bin/env python3

 #"args": ["--address A4:34:F1:EA:1C:A2"]
import sys
import rospy
import std_msgs.msg

from core_z.zephyr_dev_async import *
from zephyr.msg import *


'''
Intended to use 30s windows of data for feature engineering:

Breathing Waveform - Report Fq: 1.008Hz
                     Samples/Report: 18 samples
                     Time between samples: 56ms
                     Data Min Needed: 30 Waveforms
                    
ECG Waveform - Report Fq: 252Hz
               Samples/Report: 63 samples
               Time between samples: 4ms
               Data Min Needed: 119 Waveforms'''

class ROSActions(ZephyrDataActions):
    
    def __init__(self):
        super(ROSActions, self).__init__()
        #'topic name', msg type, queue_size = 10 <-standard queue size
        self.hr_pub = rospy.Publisher('heartrate', Heartrate, queue_size=10)
        self.hrv_pub = rospy.Publisher('hrv', HRV, queue_size=10)
        self.br_pub = rospy.Publisher('br', Breath, queue_size=10)
        
        self.ecg_wv_pub = rospy.Publisher('ecg_chunk', Float32MultiArray, queue_size=10)
        self.br_wv_pub = rospy.Publisher('br_chunk', Float32MultiArray, queue_size=10)
        
        self.rr_wv = []
        self.ecg_wv = []
        
    def onSummary(self,msg):
        self.rate = rospy.Rate(10)
        header = std_msgs.msg.Header()
        stamp = rospy.Time.now()
        content = msg.as_dict()
        
        #Heart Rate 
        hr = Heartrate()
        hr.header = header
        hr.header.stamp = stamp
        hr.rate = content['heart_rate']
        hr.confidence = content['heart_rate_confidence']
        hr.reliable = not(content['heart_rate_unreliable'])
        self.hr_pub.publish(hr)
        
        #Heart Rate Variablity
        hrv = HRV()
        hrv.header = header
        hrv.header.stamp = stamp
        hrv.variation = content['heart_rate_variability']
        hrv.reliable = not(content['hrv_unreliable'])
        self.hrv_pub.publish(hrv)
        
        #Breathing Rate
        br = Breath()
        br.header = header
        br.header.stamp = stamp
        br.rate = content['respiration_rate']
        br.confidence = bool(content['device_worn_confidence'])
        br.reliable = not(content['respiration_rate_unreliable'])
        self.br_pub.publish(br)
        
        self.rate.sleep()
            
    def onECG(self, msg):
        self.ecg_wv_pub.publish(Float32MultiArray(data=msg.waveform))
        #Use 3rd party biosignal library for further analysis - Pyphysio Example Below - Requires import pyphysio as ph
        #self.ecg_wv.append(msg.waveform)
        #if len(self.ecg_wv) >= 119: # Waveforms needed for 30s of data
        #    flat_ecg = [item for elem in self.ecg_wv for item in elem]
        #    ecg = ph.EvenlySignal(values = flat_ecg, sampling_freq = 63, signal_type = 'ecg')
        #    ecg.plot('r')
        #    self.rr_wv.pop(0)
        #    pause(.001) #requires import sys

    def onBreath(self, msg):
        self.br_wv_pub.publish(Float32MultiArray(data=msg.waveform))
        #Use 3rd party biosignal library for further analysis - See above example
        #self.rr_wv.append(msg.waveform)
        #if len(self.ecg_wv) >= 30: # Waveforms needed for 30s of data
        #    pass
        
        
    @classmethod
    def timeParse(cls, previous_time, msg_split):        
        incoming_time = rospy.Time.now()
        time_step = (incoming_time-previous_time)/(msg_split-1)
        t_dur = int(time_step.nsecs)
        
        t_offset = [rospy.Duration(0,t_dur*i) for i in range(msg_split)]
        t_offset.reverse()
        t_base = [incoming_time for i in range(msg_split)]
        
        return list(map(lambda x,y: x-y, t_base,t_offset)), incoming_time
        
link = None
logger = logging.getLogger(__name__)
if __name__ == '__main__':
    rospy.init_node('zephyr_node', anonymous=True)
    rosargs = rospy.myargv(argv=sys.argv)
    myargs = getArgs(rosargs)  
    if len(sys.argv) > 1:
        myargs.address = str(sys.argv[2])
    else:
        myargs.address = 'A4:34:F1:F1:67:8F'
        #myargs.address = 'test'
        
    actions = ROSActions()
    
    asyncio.ensure_future(init(actions,myargs))
    loop = asyncio.get_event_loop()
    try:
        loop.run_forever()
    except KeyboardInterrupt:
        logger.info("Ctrl-C pressed.")