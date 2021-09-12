## Emotiv Insight
<img align="right" width="300" src="https://github.com/SMARTlab-Purdue/ros2-foxy-wearable-biosensors/blob/master/media/img/emotiv-insight.jpg">
Emotiv Insight is a wearable headset capable of reading 8 channels Electroencephalography (EEG) signals (e.g., AF3, AF4, T7, T8, and Pz). The sampling rate of each channel is 128 samples per second with 14 bits resolution. It has an 9-axis inertial measurement unit (IMU) sensor to detect head motions. Since it has lightweight and user-friendly design, many affective researchers utilize it to measure the EEG signal from a human body.

## Requirments
1) [Emotiv Pro License](https://www.emotiv.com/emotivpro/)
2) Install [Emotiv App (Ubuntu version)](https://www.emotiv.com/my-account/downloads/)


## Node Informations
1) Node name: _emotiv_insight_node_
2) Parameters:
* _Sensor_Enable_ : 
* _Chunk_Enable_ : 
* Chunk_Length :  

## Topic Information
### Raw data
1) _biosensors/emotiv_insight/eeg_ :
* type: standard_msg/Float32MultiArray
2) _biosensors/emotiv_insight/mot_ :
* type: standard_msg/Float32MultiArray
3) _biosensors/emotiv_insight/met_ :
* type: standard_msg/Float32MultiArray
4) _biosensors/emotiv_insight/mot_ :
* type: standard_msg/Float32MultiArray
5) _biosensors/emotiv_insight/dev_ : 
* type: standard_msg/Float32MultiArray


### Chunk Data
1) _biosensors/emotiv_insight/eeg_chunk_ :
2) _biosensors/emotiv_insight/met_chunk_ :


# How to run the Node using Launch file

```bash
$ros2 launch ros2-foxy-wearable-biosensors ros2-emotiv_insight.launch.py
```
