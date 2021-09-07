#!/usr/bin/python
import sys, struct, serial, os
import rospy
import time
from std_msgs.msg import String, Float64, Float32, Bool
import subprocess

os.system("sudo rfcomm bind 0 00:06:66:F2:AF:E9")

sensor_status = False

def wait_for_ack():
    ddata = ""
    ack = struct.pack('B', 0xff)
    while ddata != ack:
        ddata = ser.read(1)
        print "0x%02x" % ord(ddata[0])
        
    return

def sub_sensor_status_callback(msg):
    global sensor_status
    sensor_status = msg.data

## ROS connection
def talker(GSR_ohm, PPG_mv):
    #global is_on
    global sensor_status

    rospy.init_node('shimmer3_read_gar_ppg', anonymous=False)
    pub_gsr = rospy.Publisher('shimmer3/GSR', Float64, queue_size=1)
    pub_ppg = rospy.Publisher('shimmer3/PPG', Float64, queue_size=1)
    sub_sensor_status = rospy.Subscriber('shimmer3/status', Bool, sub_sensor_status_callback, queue_size=1)
    rate = rospy.Rate(30)   # sampling time check

    while not rospy.is_shutdown():
        #if is_on:
        #    pub.publish(Conf)
        if sensor_status == True:
            pub_gsr.publish(GSR_ohm)
            pub_ppg.publish(PPG_mv)
        else:
            pass
        rate.sleep()
        break

if len(sys.argv) < 2:
    #print "no device specified"
    #print "You need to specify the serial port of the device you wish to connect to"
    print "[Shimmer3] example:"
    #print "   aAccel5Hz.py Com12"
    #print "or"
    print "[Shimmer3]    aAccel5Hz.py /dev/rfcomm0"   # To assign rfcoom0, "sudo rfcomm bind 0 <MAC address> <- 00:06:66:F2:AF:E9
    print "[Shimmer3] sudo rfcomm bind 0 00:06:66:F2:AF:E9"
else:
    print "[Shimmer3] connecting:"
    ser = serial.Serial(sys.argv[1], 115200)
    ser.flushInput()
    #print "port opening, done."

    # send the set sensors command
    ser.write(struct.pack('BBBB', 0x08 , 0x04, 0x01, 0x00))  #GSR and PPG
    wait_for_ack()   
   # print "sensor setting, done."

    # Enable the internal expansion board power
    ser.write(struct.pack('BB', 0x5E, 0x01))
    wait_for_ack()
    #print "enable internal expansion board power, done."

    # send the set sampling rate command
    '''
        sampling_freq = 32768 / clock_wait = X Hz
    '''
    sampling_freq = 50
    clock_wait = (2 << 14) / sampling_freq

    ser.write(struct.pack('<BH', 0x05, clock_wait))
    wait_for_ack()

    # send start streaming command
    ser.write(struct.pack('B', 0x07))
    wait_for_ack()
    #print "start command sending, done."

    # read incoming data    
    ddata = ""
    numbytes = 0
    framesize = 8 # 1byte packet type + 3byte timestamp + 2 byte GSR + 2 byte PPG(Int A13)

    ## ROS subscribe
    

    #print "Packet Type\tTimestamp\tGSR\tPPG"
    try:
        print "[Shimmer3] connected:"
        while True:
            while numbytes < framesize:
                ddata += ser.read(framesize)
                numbytes = len(ddata)

            data = ddata[0:framesize]
            ddata = ddata[framesize:]
            numbytes = len(ddata)

            # read basic packet information
            (packettype) = struct.unpack('B', data[0:1])
            (timestamp0, timestamp1, timestamp2) = struct.unpack('BBB', data[1:4])

            # read packet payload
            (PPG_raw, GSR_raw) = struct.unpack('HH', data[4:framesize])

            # get current GSR range resistor value
            Range = ((GSR_raw >> 14) & 0xff)  # upper two bits
            if(Range == 0):
                Rf = 40.2   # kohm
            elif(Range == 1):
                Rf = 287.0  # kohm
            elif(Range == 2):
                Rf = 1000.0 # kohm
            elif(Range == 3):
                Rf = 3300.0 # kohm

            # convert GSR to kohm value
            gsr_to_volts = (GSR_raw & 0x3fff) * (3.0/4095.0)
            GSR_ohm = Rf/( (gsr_to_volts /0.5) - 1.0)

            # convert PPG to milliVolt value
            PPG_mv = PPG_raw * (3000.0/4095.0)

            timestamp = timestamp0 + timestamp1*256 + timestamp2*65536

            # print "0x%02x\t\t%5d,\t%4d,\t%4d" % (packettype[0], timestamp, GSR_ohm, PPG_mv)
            
            # ROS connection and share data
            if __name__ == '__main__':
                try:
                    talker(GSR_ohm, PPG_mv)
                except rospy.ROSInterruptException:
                    pass

    except KeyboardInterrupt:
#send stop streaming command
        print "[Shimmer3] error"

        ser.write(struct.pack('B', 0x20))
        #print
        #print "stop command sent, waiting for ACK_COMMAND"
        wait_for_ack()
        #print "ACK_COMMAND received."
    #close serial port
        ser.close()
        #print "All done"
