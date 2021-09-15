import sys, struct, serial, os  # pip3 install pyserial
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from std_msgs.msg import Float32, Float32MultiArray


class ros2_shimmer3(Node):
    #############################################################
    # Main Loop
    #############################################################
    def __init__(self):
        super().__init__('shimmer3_gsr_node')

        # For the generlized structure of ROS2 Node
        self.declare_parameter('Sensor_Enable', True) # Enable to publish sensor data (true)
        self.Parm_Sensor_Enable = self.get_parameter('Sensor_Enable').value 
        self.declare_parameter('Chunk_Enable', True) # Enable to publish chunk data (true)
        self.Parm_Chunk_Enable = self.get_parameter('Chunk_Enable').value 
        self.declare_parameter('Chunk_Length', 128) # Define the length of the chunk data
        self.Parm_Chunk_Length = self.get_parameter('Chunk_Length').value 

        # For the Veriner Respriation Belt H10 device.
        # Ensure the Bluetooth radio is available by running the '''$ hciconfig''' command.
        # Scan for the Shimmer by running the '''$ hcitool scan''' command.
        self.declare_parameter('Device_Name', "00:06:66:F2:AF:E9") # Should be changed inro your device name
        self.Parm_Device_Name = self.get_parameter('Device_Name').value 
        
        
        os.system("sudo chmod a+rw /dev/rfcomm0")
        os.system("sudo rfcomm bind 0 " + self.Parm_Device_Name)

        self.data_index = 0
        self.gsr_data_index = 0
        self.gsr_chunk_data = [] 
        self.ppg_data_index = 0
        self.ppg_chunk_data = [] 

        self.ser = serial.Serial("/dev/rfcomm0", 115200)
        self.ser.flushInput()
        print ("port opening, done.")

        # send the set sensors command
        self.ser.write(struct.pack('BBBB', 0x08 , 0x04, 0x01, 0x00))  #GSR and PPG
        self.wait_for_ack()   
        print ("sensor setting, done.")

        # Enable the internal expansion board power
        self.ser.write(struct.pack('BB', 0x5E, 0x01))
        self.wait_for_ack()
        print ("enable internal expansion board power, done.")

        # send the set sampling rate command
        # sampling_freq = 32768 / clock_wait = X Hz
        sampling_freq = 50
        clock_wait = (2 << 14) / sampling_freq
        self.ser.write(struct.pack('<BH', 0x05, int(clock_wait)))
        self.wait_for_ack()

        # send start streaming command
        self.ser.write(struct.pack('B', 0x07))
        self.wait_for_ack()
        print ("start command sending, done.")


        # read incoming data
        

        #############################################################
        # Publisher Parts
        #############################################################
        if self.Parm_Sensor_Enable:
            self.pub_shimmer3_gsr_data = self.create_publisher(Float32, "biosensors/shimmer3_gsr/gsr", 10)
            self.pub_shimmer3_ppg_data = self.create_publisher(Float32, "biosensors/shimmer3_gsr/ppg", 10)

            if self.Parm_Chunk_Enable:
                self.pub_shimmer3_gsr_chunk_data = self.create_publisher(Float32MultiArray, "biosensors/shimmer3_gsr/gsr_chunk", 10) 
                self.pub_shimmer3_ppg_chunk_data = self.create_publisher(Float32MultiArray, "biosensors/shimmer3_gsr/ppg_chunk", 10)

        #print ("Packet Type\tTimestamp\tGSR\tPPG")
        
        while rclpy.ok() and self.Parm_Sensor_Enable:
            
            (shimmer_raw_GSR, Shimmer_raw_PPG) = self.reading_shimmer3_data()

            self.pub_shimmer3_gsr_data.publish(Float32(data = shimmer_raw_GSR))
            self.pub_shimmer3_ppg_data.publish(Float32(data = Shimmer_raw_PPG))
            if self.Parm_Chunk_Enable:
                self.gsr_chunk_data.append(shimmer_raw_GSR)
                self.ppg_chunk_data.append(Shimmer_raw_PPG)

                #print(self.bvp_data_index, stream_id.name, timestamp, sample) #For test
                if self.data_index  == self.Parm_Chunk_Length - 1:
                    # publish chunk
                    self.pub_shimmer3_gsr_chunk_data.publish(Float32MultiArray(data = self.gsr_chunk_data))
                    self.pub_shimmer3_ppg_chunk_data.publish(Float32MultiArray(data = self.ppg_chunk_data))
                            
                    #Reset index and ref_array
                    self.gsr_chunk_data = []
                    self.ppg_chunk_data = []
                    self.data_index = 0
                else:
                    self.data_index += 1
            else:
                pass


    def reading_shimmer3_data(self):
        ddata = ""
        numbytes = 0
        framesize = 8 # 1byte packet type + 3byte timestamp + 2 byte GSR + 2 byte PPG(Int A13)

        while numbytes < framesize:
            ddata = self.ser.read(framesize)
            #ddata = ddata + test.decode('utf-8')
            numbytes = len(ddata)
        
        data = ddata[0:framesize]
        ddata = ddata[framesize:]
        numbytes = len(ddata)


        (packettype) = struct.unpack('B', data[0:1])
        #(timestamp0, timestamp1, timestamp2) = struct.unpack('BBB', data[1:4])

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
        #print(GSR_ohm, type(GSR_ohm), PPG_mv, type(PPG_mv))
        return (float(GSR_ohm), float(PPG_mv))

    def wait_for_ack(self):
        ddata = ""
        ack = struct.pack('B', 0xff)
        while ddata != ack:
            ddata = self.ser.read(1)
            #print ("0x%02x" % ord(ddata[0]))
            
        return
        

def main(args=None):
    rclpy.init(args=args)

    ros2_shimmer3_node = ros2_shimmer3()

    try:
        while rclpy.ok():
            pass
            rclpy.spin(ros2_shimmer3_node)
    
    except KeyboardInterrupt:
        print('repeater stopped cleanly')

    except BaseException:
        print('exception in repeater:', file=sys.stderr)
        raise

    finally:        
        ros2_shimmer3_node.destroy_node()
        rclpy.shutdown() 

if __name__ == '__main__':
    main()
