## Emotiv Insight
<img align="right" width="200" src="/media/img/shimmer3_gsr.png">
The Shimmer3-GSR+ Unit is a wearable biosensor to measure Galvanic Skin Response (GSR) and Photoplethysmography (PPG) signals from the fingers or skins, converting to estimate HR [26].


* Official website: [https://www.shimmersensing.com/products/shimmer3-wireless-gsr-sensor](https://www.shimmersensing.com/products/shimmer3-wireless-gsr-sensor)

## Requirments
1) Emotiv Pro Liences
2) Install a python library: ```$ pip3 install pyserial```

## Node Informations
1) Node name: shimmer3_gsr_node
2) Parameters:
  * _Sensor_Enable_ : a boolean data type (i.e., True or False).; default= _True_
  * _Chunk_Enable_ : a boolean data type (i.e., True or False).; default= _True_
  * _Chunk_Length_ : a integer data type to adjust data length per topic.; default= _128_
  * _Device_Name_ : a string data type to connect own device via ble/; for example, _'00:06:66:F2:AF:E9'_

## Topic Information
### Raw data
1) _biosensors/shimmer3/gsr_ : 
* type: standard_msg/Float32
* size: 1-by-1 
* detail: a Photoplethysmography (PPG) signal.
2) _iosensors/shimmer3/ppg_ :
* type: standard_msg/Float32
* size: 1-by-1 
* detail: a Galvanic Skin Response (GSR) signal. 

### Chunk Data
1) _biosensors/empatica_e4/bvp_chunk_ : 
* type: standard_msg/Float32MultiArray
* size: 1-by-128 float array
* detail: Photoplethysmography (PPG) signals.
2) _biosensors/empatica_e4/gsr_chunk :
* type: standard_msg/Float32MultiArray
* size: 1-by-128 float array
* detail: Galvanic Skin Response (GSR) signals. 

## Test the Node using Launch file

```bash
$ros2 launch shimmer3_gsr_unit ros2-shimmer3_gsr.launch.py
```

## Example of Published Topic Data
<p align="center">
<img src="/media/img/shimmer3_gsr_data.jpg" width="700" >
</p>
