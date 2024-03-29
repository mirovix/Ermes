#include <ros/ros.h>
#include <vector> 
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <chrono>
#include <thread>
#include <sys/stat.h>
//[0-5] positive axis [6-11] negative axis

geometry_msgs::Pose state_cam;
int sequence = -1, current_sequence = -1;

size_t n_cycles = 3, n_axis = 2;
int time_acc = 150, time_dec = 65;
double secs, max_time_opened_thrusters = 5.0;
float x_tof = 1000;
int count_x = 0;
const uint32_t INTERVAL_ROS_MSG = (uint32_t) 1.f/(200.f/13.f)*1000; //65ms
//def ranges and weights
std::vector<double> range_ori, range_pos, defualt_ori, default_pos;
double w_ori, w_pos, w_pos_x, min_distance;
int cylces_dec; 

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
  tty.c_cc[VTIME] = 1;            // 0.5 seconds read timeout

  if (tcsetattr (fd, TCSANOW, &tty) != 0)
    ROS_INFO("error %d setting term attributes", errno);
}

void sendCommand(int axis, int cylces, int fd, int time){

  if(cylces < 2)cylces = 1;

  std::ostringstream ss_cycles, ss_axis;

  ROS_INFO("axis %d cycles %d", axis, cylces);
  
  ss_cycles << std::setw(n_cycles) << std::setfill('0') << std::to_string(cylces);
  ss_axis << std::setw(n_axis) << std::setfill('0') << std::to_string(axis);

  std::string s = ss_axis.str() + "01" + ss_cycles.str();
  char input[s.length() + 1];
  strcpy(input, s.c_str());
  write (fd, input, sizeof(input));
  std::this_thread::sleep_for(std::chrono::milliseconds((cylces*INTERVAL_ROS_MSG)+time));
}

void sendDecCommand(int axis, int fd, int time){
  std::ostringstream ss_axis_dec, ss_cycles_dec;
  if(axis > 5) axis -= 6;else axis += 6;
  if(cylces_dec < 2)cylces_dec = 1;
  
  ROS_INFO("axis %d cycles %d", axis, cylces_dec);
  
  ss_axis_dec << std::setw(n_axis) << std::setfill('0') << std::to_string(axis);
  ss_cycles_dec << std::setw(n_cycles) << std::setfill('0') << std::to_string(cylces_dec);
  
  std::string s_dec = ss_axis_dec.str() + "01" + ss_cycles_dec.str();
  char input_dec[s_dec.length() + 1];
  strcpy(input_dec, s_dec.c_str());
  write (fd, input_dec, sizeof(input_dec));
  std::this_thread::sleep_for(std::chrono::milliseconds((cylces_dec*INTERVAL_ROS_MSG)+time));
}

void controlSequence(int fd){
  if(current_sequence == sequence && (ros::Time::now().toSec() - secs) > max_time_opened_thrusters)
    return;
  current_sequence = sequence;

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
  state.push_back(roll);
  state.push_back(pitch);
  state.push_back(yaw);

  float value_to_check = 0.085;
  if(x_tof < value_to_check){
    //use tof
    state[0] = x_tof;
    ROS_INFO("TOF ON, value %f", x_tof);
  }

  //check if the values are not found
  for(int i=0; i<3; i++)
    if(state[i] == 0.0)
      return;
  for(int i=3; i<6; i++)
    if(isnan(state[i]))
      return;

  for(int i=0; i<6; i++)
    ROS_INFO("I %d heard: [%f]", i, state[i]);
  
  //for(int pos_ori = 3; pos_ori < 5; pos_ori++){
    //std::cout << defualt_ori[pos_ori]+range_ori[pos_ori-3] << std::endl;
    //std::cout << state[pos_ori] << std::endl;
  if(state[4] > defualt_ori[1]+range_ori[1]){
      sendCommand(10, int(abs(w_ori)), fd, time_acc);
      //sendDecCommand(4, fd, time_dec);
      ROS_INFO("value %f", state[4]);
      return;
  }
  else if(state[4] < defualt_ori[1]-range_ori[1]){ 
      sendCommand(4, int(abs(w_ori)), fd, time_acc);
      //sendDecCommand(10, fd, time_dec);
      ROS_INFO("value %f", state[4]);
      return;
  }

  if(abs(state[5]) < abs(defualt_ori[2]) - abs(range_ori[2])){
    int pos_ori_yaw;
    if(state[5] > 0)
      pos_ori_yaw = 5;
    else
      pos_ori_yaw = 11;
    sendCommand(pos_ori_yaw, int(abs(w_ori)), fd, time_acc);
    //sendDecCommand(pos_ori_yaw, fd, time_dec);
    ROS_INFO("value %f", state[5]);
    return;
  }

  //check position y and z
  for(int pos = 1; pos < 3; pos++){
    if(state[pos] > default_pos[pos-1]+range_pos[pos-1]){
      sendCommand(pos, int(abs(w_pos)), fd, time_acc);
      ROS_INFO("value %f", state[pos]);
      return;
    }
    if(state[pos] < default_pos[pos-1]-range_pos[pos-1]){
      sendCommand(pos+6, int(abs(w_pos)), fd, time_acc);
      ROS_INFO("value %f", state[pos]);
      return;
    }
  }

  //decrease position along x
  if(state[0] < min_distance){
     ROS_INFO("***DONE***");
     exit(0);
  }

  /*if(count_x == 2){
    sendDecCommand(0, fd, time_acc);
    count_x=0;
    return;
  }*/

  if(state[0] < value_to_check){
    //count_x++;
    sendCommand(0, (int(w_pos_x)*0.35), fd, time_acc);
  }
  else if(state[0] < 0.14){
    //count_x++;
    sendCommand(0, (int(w_pos_x)*0.65), fd, time_acc);
  }
  else 
    sendCommand(0, int(w_pos_x), fd, time_acc);

}

//const nav_msgs::Odometry::ConstPtr& msg
void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
  //ROS_INFO("I heard: [%f]", msg->pose.pose.position.x);
  state_cam = msg->pose.pose;
  sequence = msg->header.seq;
  //ROS_INFO("I heard: [%f]", state_cam.position.x); 
}

void tofCallback(const geometry_msgs::PointStamped::ConstPtr& msg){
  x_tof = msg->point.x/100;
  //ROS_INFO("I TOFFFF: [%f]", x_tof); 
}

int main(int argc, char* argv[]){
  //args USB, range orientation, range position, w_orientation, w_position, w_x, default orientation, default position,
  // USB0 range_ori_roll range_ori_pitch range_ori_yaw range_pos_y range_pos_z w_ori w_pos w_pos_x #
  //1 0.5 0.5 0.02 0.02 0.3 1 3.5 1.57 -0.13 -3.14 0.020 -0.05
  ros::init(argc, argv, "mid_lvl");
  ros::NodeHandle mid_lvl;
  if(argc < 2)
    return 0;
  else if(argc > 2){
    range_ori = {std::atof(argv[2]), std::atof(argv[3]), std::atof(argv[4])};
    range_pos = {std::atof(argv[5]), std::atof(argv[6])};
    defualt_ori = {std::atof(argv[10]), std::atof(argv[11]), std::atof(argv[12])};
    default_pos = {std::atof(argv[13]), std::atof(argv[14])};
    w_ori = std::atof(argv[7]); w_pos = std::atof(argv[8]); w_pos_x = std::atof(argv[9]);
    //cylces_dec = std::atoi(argv[15]);
  }
  else{
    range_ori = {1, 0.5, 0.5};
    range_pos = {0.02, 0.02};
    defualt_ori = {1.57, -0.13, -3.14};
    default_pos = {0.020, -0.05};
    w_ori = 0.3; w_pos = 1; w_pos_x = 3.5; 
    //cylces_dec = 1;
  }
  min_distance = 0.04;
  char serialPortFilename[] = "/dev/tty";
  strcat(serialPortFilename, argv[1]);
  int c_mod = chmod(serialPortFilename, S_IRWXU|S_IRWXG|S_IROTH|S_IWOTH);
  int fd = open(serialPortFilename, O_RDWR | O_NOCTTY | O_SYNC);
  setInterfaceAttribs (fd, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
  setBlocking (fd, 0);                    // set no blocking
  if (fd < 0){
    ROS_INFO("error %d opening %s: %s", errno, serialPortFilename, strerror (errno));
    exit(0);
  }
  secs = ros::Time::now().toSec();
  ros::Subscriber sub = mid_lvl.subscribe("/chaser/sensors/pose_from_tag_bundle", 1000, poseCallback);
  ros::Subscriber sub_tof = mid_lvl.subscribe("/chaser/sensors/tof", 1000, tofCallback);
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    ros::spinOnce();
    controlSequence(fd);
    loop_rate.sleep();
  }
  
}