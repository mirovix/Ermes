#include <Adafruit_VL6180X.h>
#include <ros.h>
#include <geometry_msgs/PointStamped.h>

const uint8_t INTERVAL_ROS_MSG = (uint32_t) 1.f/(200.f/13.f)*1000; //65ms

ros::NodeHandle nh;
geometry_msgs::PointStamped tof_msg;
ros::Publisher tof_pub("/chaser/sensors/tof", &tof_msg);

Adafruit_VL6180X vl1, vl2, vl3;

unsigned int lastRefreshTimeTof = 0;


void setup() {

  vl1 = Adafruit_VL6180X();
  vl2 = Adafruit_VL6180X();
  vl3 = Adafruit_VL6180X();

  Serial.begin(115200);
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(tof_pub);
  Wire.begin();

  initTCA9548A_PS();

  TCA9548A_PS(1);
  tofSetup(vl1);

  TCA9548A_PS(2);
  tofSetup(vl2);

  TCA9548A_PS(3);
  tofSetup(vl3);
}

void tofSetup(Adafruit_VL6180X &vl){
  if (!vl.begin())
    while (1);
}

void loop() { 
  TCA9548A_PS(1);
  uint8_t range1 = vl1.readLux(VL6180X_ALS_GAIN_5);
  TCA9548A_PS(2);
  uint8_t range2 = vl2.readLux(VL6180X_ALS_GAIN_5);
  TCA9548A_PS(3);
  uint8_t range3 = vl3.readLux(VL6180X_ALS_GAIN_5);
  
  pubtof(range1, range2, range3, lastRefreshTimeTof);
}

void pubtof(uint8_t &range1, uint8_t &range2, uint8_t &range3, unsigned int &lastRefreshTime){
  if (millis() - lastRefreshTime >= INTERVAL_ROS_MSG){
    lastRefreshTime += INTERVAL_ROS_MSG;
    tof_msg.header.frame_id = "tof";
    tof_msg.header.stamp = nh.now();
    int div = 3;

    if(range1 < 0){
      range1 = 0;
      div--;
    }
    if(range2 < 0){
      range2 = 0;
      div--;
    }
    if(range3 < 0){
      range3 = 0;  
      div--;
    } 
    if(div == 0)
      return;

    tof_msg.point.x = ((range1+range2+range3)/div);
    tof_msg.point.y = 0;
    tof_msg.point.z = 0;

    tof_pub.publish(&tof_msg);
    nh.spinOnce();
  }
}

void initTCA9548A_PS(){
  pinMode(1, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
}

void TCA9548A_PS(uint8_t bus){
  Wire.beginTransmission(0x70);
  Wire.write(1 << bus);
  Wire.endTransmission();
}
