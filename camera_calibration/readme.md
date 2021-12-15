# Camera Calibration Experiment

This document contains the instructions to calibrate the Raspberry Pi camera.

Repository for the ROS camera library:

https://github.com/UbiquityRobotics/raspicam_node

Assuming the host core is at IP: 192.168.43.186 \
and the raspberry IP is: 192.168.43.172\
Open the terminal in the core host and execute:

```
source /opt/ros/noetic/setup.zsh
export ROS_IP=192.168.43.186  
export ROS_MASTER_URI=http://192.168.43.186:11311
roscore
```

Open the terminal in the raspberry and type:


```
source /opt/ros/noetic/setup.bash
export ROS_IP=192.168.43.172  
export ROS_MASTER_URI=http://192.168.43.186:11311
```


## Check if image is available

In the raspberry run:

```
roslaunch raspicam_node camerav2_1280x960.launch
```
There are other launch files with other resolutions, check which is more appropriate. \
In the core PC run (another terminal):


```
rosrun rqt_reconfigure rqt_reconfigure
```

In the core PC run (another terminal) to view the output:

```
rosrun rqt_image_view rqt_image_view
```
From this you can config the parameters of the camera. Terminate the execution of those nodes when you check if it works.

## Calibration

You can find the PDF of the calibration checkboard here:

[8x6](http://wiki.ros.org/camera_calibration/Tutorials/StereoCalibration?action=AttachFile&do=view&target=check-108.pdf) and [7x6](http://wiki.ros.org/camera_calibration/Tutorials/StereoCalibration?action=AttachFile&do=view&target=check_7x6_108mm.pdf) 



To publich the raw camera topic, you must run in the Raspberry Pi:
```
roslaunch raspicam_node camerav2_1280x960_10fps.launch enable_raw:=true
```
And in the workstation:

```
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.074 image:=/raspicam_node/image camera:=/raspicam_node
```
The **size** is the number of cross in the checkboard used.\
The **square** is the measure in meters of the edge of each square\
If you want to use multiple checkboards you must specify **size** and **square** for each of them. They must have different dimensions to allow the algorithm to recognize them.

Follow the instruction in this wiki page: \
http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration

