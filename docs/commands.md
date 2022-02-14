# NETWORK COMMANDS

Check wifi IP:

```
ip a
```
Check network devices:

```
nmap -sP 192.168.43.0/24
```
nmap with IP of the network and mask. Find the Raspberry IP.
## ssh connection

```
ssh pi@192.168.43.72
```

## Change Wifi SSID on the raspberry


```
ssh pi@192.168.43.72
sudo vim /etc/wpa
```

The current config is:

```
ssid="ERMES"
psk="ermes_wifi"
```
To change the config open on raspberry terminal this file:
```
sudo vim /etc/wpa_supplicant/wpa_supplicant.conf 
```
And change the parameters accordingly.


## Environment variables and roscore config
Set the environment variables in the console:


```
source /opt/ros/noetic/setup.zsh
export ROS_IP=192.168.43.186  
export ROS_MASTER_URI=http://192.168.43.186:11311
roscore
```


Open the terminal in the raspberry and type:


```
source /opt/ros/noetic/setup.bash
export ROS_IP=192.168.43.177  
export ROS_MASTER_URI=http://192.168.43.186:11311
source ~/ros_catkin_ws/devel_isolated/setup.bash
```

# Workspace

## Compile ROS Workspace
To compile the entire workspace, move to ros_catkin_ws workspace:
```
cd ~/ros_catkin_ws
sudo src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/noetic -j1 -DPYTHON_EXECUTABLE=/usr/bin/python3
```

# Install a new package
Move to src:
```
cd ~/ros_catkin_ws/src
git clone <address-of-repository>
```
Update dependencies:

```
TODO comando update dependencies
```

Recompile entire ros_catkin_ws:

```
sudo src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/noetic -j1 -DPYTHON_EXECUTABLE=/usr/bin/python3
```

Source the workspace:

```
source ~/ros_catkin_ws/devel_isolated/setup.bash
```

<!--Finish the document when useful command must be added-->



# ROS SOURCE
## zsh source
source /opt/ros/noetic/setup.zsh

## setting the ros source for bash
source /opt/ros/noetic/setup.bash


## SETUP devel bash

source devel/setup.bash


## publish static transforms to test:

```
rosrun tf static_transform_publisher 0 0 0 0 0 0 1 map base_link 1000
rosrun tf static_transform_publisher 0 0 0.1 0 1.5708 0 base_link raspicam 1000
```