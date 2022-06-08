## Empatica E4 Wristband Node
<img align="right" width="150" src="/media/img/empatica_e4.jpg">
The Empatica E4 is a wristband with an array of biosensors for physiological monitoring: Electrodermal Activity (EDA), Blood Volume Pulse (BVP), Inter-Beat-Interval (IBI), Heart Rate (HR), and Skin Temperature (ST), and behavioral monitoring: 3-axis accelerometer [14]. However, there are a limitation to directly read biosensor data on the Linux environment since the Empatica does not officially provides SDKs and libraries for Linux operating system. Thus, an additional Window machine and Bluetooth dongle (e.g., Bluegiga Bluetooth Smart Dongle) are required in the current version of the biosensor package in order to stream biosensor data using LSL as mentioned on the Empatica E4 website [15]. The main Linux machine having the ROS 2 system converts the LSL data into ROS 2 topics in real-time. 

* Official website: [https://www.empatica.com](https://www.empatica.com)

## Requirments
1) Additional window machine to stream data via [E4 stremaing server](https://developer.empatica.com/windows-streaming-server-usage.html).
2) Install a python library: ```$ pip install open-e4-client```


## Node Informations
1) Node name: empatica_e4_node
2) Parameters: 
  * _Sensor_Enable_ : a boolean data type (i.e., True or False).; default= _True_
  * _Chunk_Enable_ : a boolean data type (i.e., True or False).; default= _True_
  * _Chunk_Length_ : a integer data type to adjust data length per topic.; default= _128_
  * _DeviceID_ : a string data type to connect own device via ble/; for example, _'CC34CD'_
  * _E4_Host_IP_ : a string data type to connect Empatica E4 streaming server.; for example, _'XXX.XXX.XXX.XXX'_
  * _E4_Host_Port_ : a integer data type to adjust data length per topic.; default= _28000_

## Topic Information
### For raw data
1) _biosensors/empatica_e4/bvp_ : 
* type: standard_msg/Float32
* size: 1-by-1 
* detail:the Blood  Volume  Pulse (BVP) signal. 
2) _iosensors/empatica_e4/gsr_ :
* type: standard_msg/Float32
* size: 1-by-1 
* detail: the Electrodermal  Activity  (EDA) signal. 
3) _biosensors/empatica_e4/hr_ : 
* type: standard_msg/Float32
* size: 1-by-1 
* detail: the Heart Rate (HR) signal. 
4) _biosensors/empatica_e4/st_ : 
* type: standard_msg/Float32
* size: 1-by-1 
* detail: Skin Temperature (ST) signal
5) _biosensors/empatica_e4/ibi_ : 
* type: standard_msg/Float32
* size: 1-by-1 
* detail:  the Inter-Beat-Interval (IBI) signal. 
6) _biosensors/empatica_e4/acc_ : 
* type: standard_msg/Float32MultiArray
* size: 1-by-1 
* detail: the accelerometer (ACC) data.
7) _biosensors/empatica_e4/bat_ : 
* type: standard_msg/Float32
* size: 1-by-1 
* detail: the Electrodermal  Activity  (EDA) signal.
8) _biosensors/empatica_e4/tag_ : 
* type: standard_msg/Empty
* size: 1-by-1 
* detail: the Electrodermal  Activity  (EDA) signal. 

### Chunk data
1) _biosensors/empatica_e4/bvp_chunk_ : 
* type: standard_msg/Float32MultiArray
* size: 1-by-128 float array
* detail: the Blood  Volume  Pulse (BVP) signals.
2) _biosensors/empatica_e4/gsr_chunk :
* type: standard_msg/Float32MultiArray
* size: 1-by-128 float array
* detail: the Electrodermal  Activity  (EDA) signals.
3) _biosensors/empatica_e4/st_chunk : 
* type: standard_msg/Float32MultiArray
* size: 1-by-7 float array
* detail: Skin Temperature (ST) signal.

## Test the Node using Launch file

```bash
$ros2 launch empatica_e4 ros2-empatica_E4.launch.py
```

# Example of Published Topic Data
<p align="center">
<img src="/media/img/empatica_e4_data.jpg" width="700" >
</p>
