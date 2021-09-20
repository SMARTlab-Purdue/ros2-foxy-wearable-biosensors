## Emotiv Insight
<img align="right" width="300" src="/media/img/emotiv_insight.jpg">
Emotiv Insight is a wearable headset capable of reading 8 channels Electroencephalography (EEG) signals (e.g., AF3, AF4, T7, T8, and Pz). The sampling rate of each channel is 128 samples per second with 14 bits resolution. It has an 9-axis inertial measurement unit (IMU) sensor to detect head motions. Since it has lightweight and user-friendly design, many affective researchers utilize it to measure the EEG signal from a human body.


* Official website: [https://www.emotiv.com](https://www.emotiv.com)


## Requirments
1) [Emotiv Pro License](https://www.emotiv.com/emotivpro/)
update in user details '''''
# Emotiv License Information (you should have the emotiv license from emotiv website)
user_details = {
	"license" : "your emotivpro license, which could use for third party app",
	"client_id" : "your client id",
	"client_secret" : "your client secret",
	"debit" : 100
}



3) Install [Emotiv App (Ubuntu version)](https://www.emotiv.com/my-account/downloads/)


## Node Informations
1) Node name: _emotiv_insight_node_
2) Parameters:
* _Sensor_Enable_ : boolean data type (i.e., _True_ or _False_), activating the sensor or not.
* _Chunk_Enable_ : boolean data type (i.e., _True_ or _False_), activating the sensor or not.
* _Chunk_Length_ : a integer data type (_default= 128_), adjusting data length per topic.

## Topic Information
### Raw data
1) _biosensors/emotiv_insight/eeg_ :
* type: standard_msg/Float32MultiArray
* size: 1-by-8 float array
* detail: [COUNTER,INTERPOLATED,AF3,T7,Pz,T8,AF4,RAW_CQ]
2) _biosensors/emotiv_insight/pow_ :
* type: standard_msg/Float32MultiArray
* size: 1-by-25 float array
* detail: [AF3/theta, AF3/alpha, AF3/betaL, AF3/betaH, AF3/gamma, T7/theta, T7/alpha, T7/betaL, T7/betaH, T7/gamma, Pz/theta, Pz/alpha, Pz/betaL, Pz/betaH, Pz/gamma,T8/theta, T8/alpha, T8/betaL, T8/betaH, T8/gamma, AF4/theta, AF4/alpha, AF4/betaL, AF4/betaH, AF4/gamma]
3) _biosensors/emotiv_insight/met_ :
* type: standard_msg/Float32MultiArray
* size: 1-by-7 float array
* detail: [Engagement_data, Excitement_data, Long_term_excitement_data, Stress_Frustration_data, Relaxation_data, Interest_Affinity_data, Focus_data]
4) _biosensors/emotiv_insight/mot_ :
* type: standard_msg/Float32MultiArray
* size: 1-by-10 float array
* detail: ["Q0","Q1","Q2","Q3","ACCX","ACCY","ACCZ","MAGX","MAGY","MAGZ"]
5) _biosensors/emotiv_insight/dev_: 
* type: standard_msg/Float32MultiArray
* size: 1-by-9 float array
* detail: [Battery, Signal, AF3_Signal, T7_Signal, Pz_Signal, T8_Signal, AF4_Signal, OVERALL_Signal, BatteryPercent]


### Chunk Data
1) _biosensors/emotiv_insight/eeg_chunk_:
* type: standard_msg/Float32MultiArray
* size: 128-by-8 float array
* detail: [COUNTER,INTERPOLATED,AF3,T7,Pz,T8,AF4,RAW_CQ]; ...
2) _biosensors/emotiv_insight/pow_chunk_ :
* type: standard_msg/Float32MultiArray
* size: 128-by-8 float array
* detail: [AF3/theta, AF3/alpha, AF3/betaL, AF3/betaH, AF3/gamma, T7/theta, T7/alpha, T7/betaL, T7/betaH, T7/gamma, Pz/theta, Pz/alpha, Pz/betaL, Pz/betaH, Pz/gamma,T8/theta, T8/alpha, T8/betaL, T8/betaH, T8/gamma, AF4/theta, AF4/alpha, AF4/betaL, AF4/betaH, AF4/gamma]


# Test the Node using Launch file

```bash
$ros2 launch emotiv_insight ros2-emotiv_insight.launch.py
```

# Example of Published Topic Data
<p align="center">
<img src="https://github.com/SMARTlab-Purdue/ros2-foxy-wearable-biosensors/blob/master/media/img/emotiv_insight_data.jpg" width="700" >
</p>


