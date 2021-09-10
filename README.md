# ros2-foxy-wearable-biosensors
<p align="center">
<img src="https://github.com/SMARTlab-Purdue/ros2-foxy-wearable-biosensors/blob/master/media/img/ros2_biosensor_pkg.png" width="700" >
</p>


Wearable biosensors enable continuous human data capture, facilitating development of real-world Human-Robot Inteaction (HRI) systems. However, a lack of standardized libraries and implementations add extraneous complexity to HRI system designs, and preclude collaboration across disciplines and institutions. Here, we propose a novel wearable biosensor package for the Robot Operating System 2 (ROS 2). The package standardizes biosensor HRI integration, lowers the technical barrier of entry, and expands the biosensor ecosystem in the robotics field. Each biosensor package node follows a generalized node and topic structure concentrated on ease of use. Current package capabilities, listed by biosensor, highlight package standardization. Collected example data demonstrates integration of each biosensor into ROS 2. Standardization of a biosensors package for ROS 2 will greatly simplify use and cross-collaboration across many disciplines.

# Requirements
* ROS2 foxy
* pip install 

# Installation
```bash
$ sudo apt install ros-foxy-desktop 
$ source /opt/ros/foxy/setup.bash
$ mkdir -p ~/ros2_ws/src
$ cd ~/ros2_ws/src
$ git clone https://github.com/SMARTlab-Purdue/ros2-foxy-wearable-biosensors.git
$ cd ~/ros2_ws
$ colcon build --symlink-install
```

# Supported Biosensors (v0.0.1 updated on Sep. 7th 2021)
1) [Empatica E4 wristband](https://github.com/SMARTlab-Purdue/ros2-foxy-wearable-biosensors/tree/master/ros2-foxy-wearable-biosensors/empatica_e4)
2) [Emotiv Insight](https://github.com/SMARTlab-Purdue/ros2-foxy-wearable-biosensors/tree/master/ros2-foxy-wearable-biosensors/emotiv_insight)
3) [Shimmer3-GSR Unit+](https://github.com/SMARTlab-Purdue/ros2-foxy-wearable-biosensors/tree/master/ros2-foxy-wearable-biosensors/ros1_shimmer3_gsr_ppg)
4) [Polar H10](https://github.com/SMARTlab-Purdue/ros2-foxy-wearable-biosensors/tree/master/ros2-foxy-wearable-biosensors/polar_h10)
5) [Vernier Respiration Belt](https://github.com/SMARTlab-Purdue/ros2-foxy-wearable-biosensors/tree/master/ros2-foxy-wearable-biosensors/vernier_respiration_belt)
6) [Zephyr Bioharness](https://github.com/SMARTlab-Purdue/ros2-foxy-wearable-biosensors/tree/master/ros2-foxy-wearable-biosensors/zephyr-ros-master)
7) TBD (will be added new sensors on v0.0.2)


## Contributors
Wonse Jo, Jaeeun Kim, and Dr. Byung-Cheol Min are with [SMART Lab](http://www.smart-laboratory.org/index.html), Department of Computer and Information Technology, [Purdue University](https://www.purdue.edu/), West Lafayette, IN 47907, USA.<br />

<table>
  <tr>
<td align="center"><a href="https://www.purdue.edu/"><img src="http://www.smart-laboratory.org/group/Wonse_Jo.png" width="200px;" alt=""/><br />
         </a><br />
<sub><b>Purdue Univ.</b></sub></a><br />
<sub><b> </b></sub></a><br />
</td>


<td align="center"><a href="https://wonsu0513.github.io/"><img src="https://github.com/SMARTlab-Purdue/ros2-foxy-wearable-biosensors/blob/master/media/contributors/Wonse_Jo.png" width="200px;" alt=""/><br />
          <sub><b><a href="https://wonsu0513.github.io/" title="personal_website">🌍Wonse Jo</b></sub></a><br />
<sub><b>Ph.D. Candidate</b></sub></a><br />
<sub><b><a href="mailto: jow@purdue.edu" title="contact">jow@purdue.edu</a></b></sub></a><br />
</td>



<td align="center"><a href="http://www.smart-laboratory.org/group/Jaeeun_Kim.html"><img src="https://github.com/SMARTlab-Purdue/ros2-foxy-wearable-biosensors/blob/master/media/contributors/Jaeeun_Kim.jpg" width="200px;" alt=""/><br />
              <sub><b><a href="http://www.smart-laboratory.org/group/Jaeeun_Kim.html" title="personal_website">🌍Jaeeun Kim</b></sub></a><br />
<sub><b>Undergradute</b></sub></a><br />
<sub><b><a href="mailto: kim2592@purdue.edu" title="contact">kim2592@purdue.edu </a></b></sub></a><br />
</td>


<td align="center"><a href="http://www.smart-laboratory.org/group/bcm.html"><img src="https://github.com/SMARTlab-Purdue/ros2-foxy-wearable-biosensors/blob/master/media/contributors/Dr_Byung-Cheol_Min.png" width="200px;" alt=""/><br />
  <sub><b><a href="http://www.smart-laboratory.org/group/bcm.html" title="personal_website">🌍Dr. Byung-Cheol Min</b></sub></a><br />
<sub><b>Director</b></sub></a><br />
<sub><b><a href="mailto: minb@purdue.edu" title="contact">minb@purdue.edu</a></b></sub></a><br />
</td>

  </tr>
</table>


Dr. Robert Wilson, and Dr. Steve McGuire are with HARE Lab, Department of Electrical and Computer Engineering, [University of California Santa Cruz](https://www.ucsc.edu/), Santa Cruz, CA 95064, USA.<br />


<table>
  <tr>
<td align="center"><a href="https://www.ucsc.edu/"><img src="http://www.smart-laboratory.org/group/Wonse_Jo.png" width="200px;" alt=""/><br />
         </a><br />
<sub><b>UCSC</b></sub></a><br />
<sub><b> </b></sub></a><br />
</td>

<td align="center"><a href="https://engineering.ucsc.edu/people/smcguire"><img src="https://github.com/SMARTlab-Purdue/ros2-foxy-wearable-biosensors/blob/master/media/contributors/Dr_Steve%20McGuire.jpeg" width="200px;" alt=""/><br />
          <sub><b><a href="http://www.smart-laboratory.org/group/Wonse_Jo.html" title="personal_website">🌍Dr. Robert Wilson</b></sub></a><br />
<sub><b>PostDoc. researcher</b></sub></a><br />
<sub><b><a href="mailto: jow@purdue.edu" title="contact">jow@purdue.edu</a></b></sub></a><br />
</td>


<td align="center"><a href="https://engineering.ucsc.edu/people/smcguire"><img src="https://github.com/SMARTlab-Purdue/ros2-foxy-wearable-biosensors/blob/master/media/contributors/Dr_Steve%20McGuire.jpeg" width="200px;" alt=""/><br />
          <sub><b><a href="https://engineering.ucsc.edu/people/smcguire" title="personal_website">🌍Dr. Steve McGuire</b></sub></a><br />
<sub><b>Director</b></sub></a><br />
<sub><b><a href="mailto: steve.mcguire@ucsc.edu" title="contact">steve.mcguire@ucsc.edu</a></b></sub></a><br />
</td>

  </tr>
</table>

# Acknowledgements
This  material  is  based  upon  work  supported  by  the  Na-tional  Science  Foundation  under  Grant  No.  IIS-1846221and  byUCSC  Grant  number.  Any  opinions,  findings,  andconclusions  or  recommendations  expressed  in  this  materialare those of the author(s) and do not necessarily reflect theviews of the National Science Foundation andUCSC Grantaffiliation.
