## Zephyr Bioharness Node
<img align="right" width="200" src="/media/img/zephyr_bioharness.png">
The Zephyr Bioharness is a chest strap sensor designed for dynamic movement activities. The Bioharness is capable of publishing output summary data (e.g., heart rate, acceleration) at 1 Hz as well as raw waveforms (e.g., ECG Waveform), which can be used in more advanced feature engineering

* Official website: [https://www.zephyranywhere.com/](https://www.zephyranywhere.com/)

## Requirments
1) Install a python library: ```$ pip3 install bluepy```


## Node Informations
1) Node name: zephyr_node
2) Parameters: 
  * _Sensor_Enable_ : a boolean data type (i.e., True or False).; default= _True_
  * _Chunk_Enable_ : a boolean data type (i.e., True or False).; default= _True_
  * _Chunk_Length_ : a integer data type to adjust data length per topic.; default= _128_

## Topic Information
### For raw data
1) _/biosensor/zephyr/hr_ : 
* type: standard_msg/Float32
* size: 1-by-1 
* detail:the Heart Rate (HR) signal. 
2) _/biosensor/zephyr/hrv_ :
* type: standard_msg/Float32
* size: 1-by-1 
* detail: (TBD)
3) _/biosensor/zephyr/br_ : 
* type: standard_msg/Float32
* size: 1-by-1 
* detail: (TBD)


### Chunk data
1) _/biosensor/zephyr/ecg_wv : 
* type: standard_msg/Float32MultiArray
* size: 1-by-128 float array
* detail: the Blood  Volume  Pulse (BVP) signals.


## Test the Node using Launch file

```bash
$ros2 launch ros2-foxy-wearable-biosensors ros2-zephyr.launch.py
```

## Example of Published Topic Data
<p align="center">
<img src="/media/img/zephyr_bioharness_data.jpg" width="700" >
</p>
