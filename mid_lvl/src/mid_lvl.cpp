#include <ros/ros.h>


int main(int argc, char** argv){
  
  ros::init(argc, argv, "mid_lvl");
  ros::NodeHandle n;
  ros::spin();

}