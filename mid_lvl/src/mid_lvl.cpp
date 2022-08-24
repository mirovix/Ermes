#include <ros/ros.h>
#include <vector> 

/*
  "10011001" spo x
  "11001100" spo y
  "00001111" spo z
  "00111100" rot x
  "10010110" rot y
  "01010101" rot z
  "01100110" spo x
  "00110011" spo y
  "11110000" spo z
  "11000011" rot x
  "01101001" rot y
  "10101010" rot z
*/

//[0-5] positive axis [6-11] negative axis

void sendCommand(int axis, int cylces){

}

void controlSequence(){
  //def ranges and weights
  std::vector<double> range_ori = {0.5, 0.5, 0.5};
  std::vector<double> range_pos = {0.5, 0.5};
  double w_ori = 0.1, w_pos = 0.1, w_pos_x = 5;

  //init orientation (alfa, beta, teta)
  //TODO: init

  //check if state is found, [x, y, z, alfa, beta, theta]
  std::vector<double> state;
  
  //check orientation
  for(int pos_ori = 3; pos_ori < 6; pos_ori++){
    if(state[pos_ori] > range_ori[pos_ori-3])
      sendCommand(pos_ori, int(abs(w_ori*state[pos_ori]))+1);
    if(state[pos_ori] < -range_ori[pos_ori-3])  
      sendCommand(pos_ori+6, int(abs(w_ori*state[pos_ori]))+1);
  }

  //check position y and z
  for(int pos = 1; pos < 3; pos++){
    if(state[pos] > range_pos[pos-1])
      sendCommand(pos, int(abs(w_pos*state[pos]))+1);
    if(state[pos] < -range_pos[pos-1])  
      sendCommand(pos+6, int(abs(w_pos*state[pos]))+1);
  }

  //decrease position along x
  sendCommand(0, int(w_pos_x));

}

int main(int argc, char** argv){
  ros::init(argc, argv, "mid_lvl");
  controlSequence();
  // ros::NodeHandle n;
  // ros::spin();
}