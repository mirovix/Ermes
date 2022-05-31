# Installation of ROS in Ubuntu 20 server in Raspberry Pi 4

## System Installation and initial config
Download rpi-imager, there is a guide here:
[Canonical Ubuntu guide](https://ubuntu.com/tutorials/how-to-install-ubuntu-on-your-raspberry-pi#1-overview)

You can set the username and password, ssh login, and wifi information directly with rpi-imager.

Update the system:

```
sudo apt update | sudo apt upgrade
```
## Extra: Install zsh

You should install [zsh and implement an Oh-My-zsh theme](https://github.com/ohmyzsh/ohmyzsh/wiki), so the console will be different.


## Install cmake and build-essential

To install run the following:

```
sudo apt install cmake build-essential
```

## Install ROS

Go to the official [wiki page installation ](http://wiki.ros.org/noetic/Installation/Ubuntu) for Ubuntu Focal Fossa.
Follow the guide.

Install Ros-base because you don't need user interfaces in the raspberry.

## Create ROS workspace

ERMES will work with 2 workspaces, one to install from source packages that aren't available in the repository, the other for the ermes project itself.

Create the ros_catkin_ws:

```
mkdir -p ~/ros_catkin_ws/src
cd ~/ros_catkin_ws
catkin_make
```

Add source to the .zshrc file if you prefer, otherwise execute this command every time you need those packages:


```
source ~/ros_catkin_ws/devel/setup.zsh
```
## Install Raspicam node

Download the repository
```
cd ~/ros_catkin_ws/src
git clone https://github.com/UbiquityRobotics/raspicam_node.git
```
Install dependencies: 

```
sudo apt install libraspberrypi0 libraspberrypi-dev pigpio-tools libpigpiod-if-dev ros-noetic-camera-info-manager ros-noetic-compressed-image-transport ros-noetic-diagnostic-updater
```

Compile:

```
catkin_make
```

Test with this command:
```
roslaunch raspicam_node camerav2_1280x960.launch
```
## Throubleshooting 

Section with common problems found during installation


###  "`failed to open vchiq instance`"

This problem is related with permission or camera.

```
sudo usermod -aG video pi
sudo chmod 777 /dev/vchiq
```

Now connect the SD card to the PC, navigate into `/boot` and open the file `config.txt`
Append at the end:
```
start_x=1 
```
Put the SD in the raspberry and restart. Now it should work.


## Install rosserial for arduino


```
sudo apt install ros-noetic-rosserial-arduino ros-noetic-rosserial
```
Install also suggested package:

```
sudo apt install arduino-mk dfu-programmer avrdude-doc gcc-doc
```
To run rosserial node:
```
rosrun rosserial_arduino serial_node.py _port:=/dev/ttyACM0 _baud:=230400
```


## Conclusions
Check the file `commands.md` to know how to run the software