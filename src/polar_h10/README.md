## Polar H10

<img align="right" width="250" src="/media/img/polar_h10.jpg">
The Polar H10 is a wearable heart rate biosensor and attached on the chest. It is mostly used for fitness objectives to read HR with 1Hz sampling time


* Official website: [https://www.polar.com/us-en/products/accessories/h10_heart_rate_sensor](https://www.polar.com/us-en/products/accessories/h10_heart_rate_sensor)

## Requirments
1) Install Python Library: '''$ pip install pexpect'''


## Node Informations
1) Node name: polar_h10_node
2) Parameters:
* _Sensor_Enable_ : a boolean data type (i.e., True or False).; default= _True_
* _Chunk_Enable_ : a boolean data type (i.e., True or False).; default= _True_
* _Chunk_Length_ : a integer data type to adjust data length per topic.; default= _128_
* Device_Mac_Address : a string data type to connect own device via ble/; for example, _'C9:61:FF:AC:8E:23'_

## Topic Information
### For raw data
1) _biosensors/polar_h10/hr_ : 
  * type: standard_msg/Float32
  * size: 1-by-1 
  * detail: the Heart Rate (HR) signal. 


## Test the Node using Launch file

```bash
$ros2 launch polar_h10 ros2-polar_h10.launch.py
```

# Example of Published Topic Data
<p align="center">
<img src="/media/img/polar_h10_data.jpg" width="700" >
</p>
