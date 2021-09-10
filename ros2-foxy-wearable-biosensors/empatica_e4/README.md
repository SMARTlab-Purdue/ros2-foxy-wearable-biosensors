## Empatica E4 Wristband Node
<img align="right" width="200" src="https://github.com/SMARTlab-Purdue/ros2-foxy-wearable-biosensors/blob/master/media/img/empatica_e4.jpg">
The Empatica E4 is a wristband with an array of biosensors for physiological monitoring: Electrodermal Activity (EDA), Blood Volume Pulse (BVP), Inter-Beat-Interval (IBI), Heart Rate (HR), and Skin Temperature (ST), and behavioral monitoring: 3-axis accelerometer [14]. However, there are a limitation to directly read biosensor data on the Linux environment since the Empatica does not officially provides SDKs and libraries for Linux operating system. Thus, an additional Window machine and Bluetooth dongle (e.g., Bluegiga Bluetooth Smart Dongle) are required in the current version of the biosensor package in order to stream biosensor data using LSL as mentioned on the Empatica E4 website [15]. The main Linux machine having the ROS 2 system converts the LSL data into ROS 2 topics in real-time. 


## Requirments
1) Additional window machine to stream data via [E4 stremaing server](https://developer.empatica.com/windows-streaming-server-usage.html).
2) Install a python library: ```bash
$ pip install open-e4-client ```


## Node Informations
1) Node name:
2) Parameters: 

## Topic Information
### Raw data
1) d

### Chunk Data
1) d


# How to run the Node using Launch file

```bash
$ros2 launch ros2-foxy-wearable-biosensors XXXXX
```
