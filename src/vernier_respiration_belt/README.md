## Vernier Respiration Belt
<img align="right" width="300" src="/media/img/vernier_respiration_belt.jpg">
The Vernier Respiration Belt is a wearable biosensor to measures human respiration rate from around the chest via Bluetooth. It is capable of measure from 0 to 50 N with 0.01 N resolution and breaths per minute (BPM) with 50 Hz sample rate.

* Official website: [https://www.vernier.com/product/go-direct-respiration-belt/](https://www.vernier.com/product/go-direct-respiration-belt/)

## Requirments
1) Python Libraries: ```$ pip3 install godirect bleak```

## Node Informations
1) Node name: empatica_e4_node
2) Parameters: 
  * _Sensor_Enable_ : a boolean data type (i.e., True or False).; default= _True_
  * _Chunk_Enable_ : a boolean data type (i.e., True or False).; default= _True_
  * _Chunk_Length_ : a integer data type to adjust data length per topic.; default= _128_
  * _Device_Name_ : a string data type to define a device name; for example, _'GDX-RB 0K2002Z5'_
  * _Device_Sampling_Rate_ : a integer data type to define the sampling rate.; default= _100_

## Topic Information
### For raw data
1) _biosensors/vernier_respiration_belt/bpm_ : 
* type: standard_msg/Float32
* size: 1-by-1 
* detail: raw Respiration Rate (bpm). 
2) _biosensors/vernier_respiration_belt/force_ :
* type: standard_msg/Float32
* size: 1-by-1 
* detail: raw Force (N). 


### Chunk data
1) _biosensors/vernier_respiration_belt/force_chunk_ : 
* type: standard_msg/Float32MultiArray
* size: 1-by-128 float array
* detail: chunk Force (N).


## Test the Node using Launch file

```bash
$ros2 launch vernier_respiration_belt ros2-vernier_respiration_belt.launch.py
```

## Example of Published Topic Data
<p align="center">
<img src="/media/img/vernier_respiration_belt_data.jpg" width="700" >
</p>
