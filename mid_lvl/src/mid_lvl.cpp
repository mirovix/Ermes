#include <ros/ros.h>
#include <vector> 
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <async_comm/serial.h>
#include <async_comm/util/message_handler_ros.h>

//[0-5] positive axis [6-11] negative axis

geometry_msgs::Pose state_cam;
const uint32_t INTERVAL_ROS_MSG = (uint32_t) 1.f/10.f*1000; //10Hz
char serialPortFilename[] = "/dev/ttyUSB0";
int fd = open(serialPortFilename, O_RDWR | O_NOCTTY | O_SYNC);

int setInterfaceAttribs (int fd, int speed, int parity){
  struct termios tty;
  if (tcgetattr (fd, &tty) != 0)
  {
    ROS_INFO ("error %d from tcgetattr", errno);
    return -1;
  }

  cfsetospeed (&tty, speed);
  cfsetispeed (&tty, speed);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
  // disable IGNBRK for mismatched speed tests; otherwise receive break
  // as \000 chars
  tty.c_iflag &= ~IGNBRK;         // disable break processing
  tty.c_lflag = 0;                // no signaling chars, no echo,
                                  // no canonical processing
  tty.c_oflag = 0;                // no remapping, no delays
  tty.c_cc[VMIN]  = 0;            // read doesn't block
  tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

  tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
  tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
  tty.c_cflag |= parity;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  if (tcsetattr (fd, TCSANOW, &tty) != 0)
  {
    ROS_INFO ("error %d from tcsetattr", errno);
    return -1;
  }
  return 0;
}

void setBlocking (int fd, int should_block){
  struct termios tty;
  memset (&tty, 0, sizeof tty);
  if (tcgetattr (fd, &tty) != 0)
    return;

  tty.c_cc[VMIN]  = should_block ? 1 : 0;
  tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

  if (tcsetattr (fd, TCSANOW, &tty) != 0)
    ROS_INFO("error %d setting term attributes", errno);
}

void sendCommand(int axis, int cylces){

  ROS_INFO("axis %d cycles %d", axis, cylces);

  size_t n_cycles = 3, n_axis = 2;
  std::ostringstream ss_cycles, ss_axis;
  
  ss_cycles << std::setw(n_cycles) << std::setfill('0') << std::to_string(cylces);
  ss_axis << std::setw(n_axis) << std::setfill('0') << std::to_string(axis);

  std::string s = ss_axis.str() + "01" + ss_cycles.str();
  char input[s.length() + 1];
  strcpy(input, s.c_str());

  write (fd, input, sizeof(input));
  //std::this_thread::sleep_for(std::chrono::milliseconds((cylces*INTERVAL_ROS_MSG)+85));
  sleep((cylces*INTERVAL_ROS_MSG/1000)+1);
}

void controlSequence(){
  //def ranges and weights
  std::vector<double> range_ori = {0.5, 0.5, 0.5};
  std::vector<double> range_pos = {0.5, 0.5};
  double w_ori = 2, w_pos = 2, w_pos_x = 5;

  //init orientation (alfa, beta, teta)
  //TODO: init

  //check if state is found, [x, y, z, alfa, beta, theta]
  std::vector<double> state;
  tf::Quaternion q(
        state_cam.orientation.x,
        state_cam.orientation.y,
        state_cam.orientation.z,
        state_cam.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  state.push_back(state_cam.position.x);
  state.push_back(state_cam.position.y);
  state.push_back(state_cam.position.z);
  state.push_back(0.0);
  state.push_back(pitch);
  state.push_back(0.0);

  for(int i=0; i<6; i++)
    ROS_INFO("I %d heard: [%f]", i, state[i]);
  
  //check orientation
  for(int pos_ori = 3; pos_ori < 6; pos_ori++){
    if(state[pos_ori] > range_ori[pos_ori-3]){
      sendCommand(pos_ori, int(abs(w_ori*state[pos_ori]))+1);
      ROS_INFO("value %f", state[pos_ori]);
      return;
    }
    if(state[pos_ori] < -range_ori[pos_ori-3]){ 
      sendCommand(pos_ori+6, int(abs(w_ori*state[pos_ori]))+1);
      ROS_INFO("value %f", state[pos_ori]);
      return;
    }
  }

  //check position y and z
  for(int pos = 1; pos < 3; pos++){
    if(state[pos] > range_pos[pos-1]){
      sendCommand(pos, int(abs(w_pos*state[pos]))+1);
      ROS_INFO("value %f", state[pos]);
      return;
    }
    if(state[pos] < -range_pos[pos-1]){
      sendCommand(pos+6, int(abs(w_pos*state[pos]))+1);
      ROS_INFO("value %f", state[pos]);
      return;
    }
  }

  //decrease position along x
  sendCommand(0, int(w_pos_x));

}

void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
  //ROS_INFO("I heard: [%f]", msg->pose.pose.position.x);
  state_cam = msg->pose.pose;
  //ROS_INFO("I heard: [%f]", state_cam.position.x);
  controlSequence();
}

int main(int argc, char** argv){
  ros::init(argc, argv, "mid_lvl");
  ros::NodeHandle cam;

  setInterfaceAttribs (fd, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
  setBlocking (fd, 0);                    // set no blocking
  if (fd < 0){
      ROS_INFO("error %d opening %s: %s", errno, serialPortFilename, strerror (errno));
      exit(0);
  }
  ros::Subscriber sub = cam.subscribe("/chaser/sensors/pose_from_tag_bundle", 1000, poseCallback);
  ros::spin();
}