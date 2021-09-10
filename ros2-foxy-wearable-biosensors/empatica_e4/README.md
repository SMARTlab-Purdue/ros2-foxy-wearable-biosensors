## Empatica E4 Wristband Node
<img align="right" width="200" src="https://github.com/SMARTlab-Purdue/ros2-foxy-wearable-biosensors/blob/master/media/img/empatica_e4.jpg">
The Empatica E4 is a wristband with an array of biosensors for physiological monitoring: Electrodermal Activity (EDA), Blood Volume Pulse (BVP), Inter-Beat-Interval (IBI), Heart Rate (HR), and Skin Temperature (ST), and behavioral monitoring: 3-axis accelerometer [14]. However, there are a limitation to directly read biosensor data on the Linux environment since the Empatica does not officially provides SDKs and libraries for Linux operating system. Thus, an additional Window machine and Bluetooth dongle (e.g., Bluegiga Bluetooth Smart Dongle) are required in the current version of the biosensor package in order to stream biosensor data using LSL as mentioned on the Empatica E4 website [15]. The main Linux machine having the ROS 2 system converts the LSL data into ROS 2 topics in real-time. 


## Requirments
1) Additional window machine to stream data via [E4 stremaing server](https://developer.empatica.com/windows-streaming-server-usage.html).
2) Install a python library: ```$ pip install open-e4-client```


## Node Informations
1) Node name: empatica_e4_node
2) Parameters: 
  * _Sensor_Enable_ : a boolean data type (i.e., True or False).; default= _True_
  * _Chunk_Enable_ : a boolean data type (i.e., True or False).; default= _False_
  * _Chunk_Length_ : a integer data type to adjust data length per topic.; default= _10_
  * _Device_Mac_Address_ : a string data type to connect own device via ble/; for example, _'xx:xx:xx:xx:xx:xx'_

## Topic Information
### Raw data
1) _biosensors/polar_h10/hr_ :
2) _biosensors/polar_h10/ibi_ :
3) _biosensors/polar_h10/dev_ :


## How to run the Node using Launch file

```bash
$ros2 launch ros2-foxy-wearable-biosensors ros2-polar_h10.launch.py
```
