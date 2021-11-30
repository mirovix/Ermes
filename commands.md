#TO SET THE ROS SOURCE, EVERY TIME OR ADD TO ZSHRC
#zsh source
source /opt/ros/noetic/setup.zsh

#setting the ros source for bash
source /opt/ros/noetic/setup.bash


#SETUP devel bash

source devel/setup.bash


#publish position base link:

rosrun tf static_transform_publisher 0 0 0 0 0 0 1 map base_link 10

