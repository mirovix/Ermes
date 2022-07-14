#include <Ixmatix_LSM9DS1.h>
#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <QueueArray.h>

//GLOBAL VARIABLES
//[0-5] positive axis [6-11] negative axis
//input(6 elements) >>> stringa (asse, accelerazione, cicli) (e.g. '0' '14' '5') 
// 4 bit + 4 bit + 8 bit (+ 1 bit) (asse, accelerazione, cicli, controllo) N.B. MAX cicli con 10Hz >> 220
const int num_input = 6;
const int num_ev = 8;
const int dof = 6;
int count = 0, length_input = 0;
int i = 0;
int count_cycles = 0, cycles = -1;
bool flag_thruster_open = false;
char input_serial[80];
char log_msg[50];

struct chars{
  char input_char[8];
};
QueueArray<chars> inputs;


//pins for swithing imus
int pinA = 12, pinB = 13;

//from 4 to 16 bit PWM
int equalize[16] = {0,83,94,103,111,121,132,144,157,172,187,206,223,244,250,255};

//matrix (2(sign) X 6(dof)) X 8(ev) for trhsruster configuration
char *ev_config[2*dof];

//time synchronization 
unsigned long start,loop_start = 0;

const uint32_t INTERVAL_ROS_MSG = (uint32_t) 1.f/10.f*1000; // 10Hz
unsigned long lastRefreshTime1 = 0, lastRefreshTime2 = 0;

/*
 * Ros variables
 */
ros::NodeHandle nh;
sensor_msgs::Imu imu1_msg, imu2_msg;
ros::Publisher imu1_pub("/chaser/sensors/imu1", &imu1_msg);
ros::Publisher imu2_pub("/chaser/sensors/imu2", &imu2_msg);

Ixmatix_LSM9DS1 imu1,imu2;

// deltat variables
int32_t lastUpdate1, lastUpdate2 = 0; 
uint32_t now1 = 0, now2 = 0;

/*
 * Calibration variables
 */

// Magnetic Mapping
// Soft iron error compensation matrix
float mag_softiron_matrix[3][3] = { { +0.990, -0.002, -0.005 },
                                    { -0.002, +0.989, -0.075 },
                                    { -0.005, -0.075, +1.027 }
                                  };

// xl = accelerometer ----- g  = gyroscope
float xl_offsets[3] = { +0.04f, -0.10f, -0.01f  };
float g_offsets[3]  = { -0.05f, +0.35f, -0.73f  };

// Magnetic calibration data - obtaied from Motion Sensor Calibration Tool
// link - https://www.pjrc.com/store/prop_shield.html
// Magnetic offset
float mag_offsets[3]            = { -5.63/1000., 16.78/1000., -6.8/1000. };

/*
 * LSM9DS1 variables
 */
float aRes1, aRes2;
float gRes1, gRes2;
float mRes1, mRes2;

float ax1, ay1, az1, ax2, ay2, az2;
float gx1, gy1, gz1, gx2, gy2, gz2;
float mx1, my1, mz1, mx2, my2, mz2;

uint8_t statusXL_G1, statusXL_G2;
uint8_t statusM1, statusM2;

uint8_t newXLData1, newXLData2;
uint8_t newGData1, newGData2;
uint8_t newMData1, newMData2;

int16_t sensorValues1[3], sensorValues2[3];

char name_imu1[] = "imu1";
char name_imu2[] = "imu2";

void inputCallback(const std_msgs::String& msg){
  chars input;
  sprintf(input.input_char,"%s", msg.data);
  inputs.enqueue(input);
  log_msg[0] = '\0';
  sprintf(log_msg,"New input recived :%s", msg.data);
  nh.loginfo(log_msg);
}

ros::Subscriber<std_msgs::String> sub("input", &inputCallback);

void setup() {

  Serial.begin(230400);
  nh.getHardware()->setBaud(230400);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(imu1_pub);
  nh.advertise(imu2_pub);
  randomSeed(analogRead(0));

  Wire.begin();

  //chipselect between imu1 and imu2
  pinMode(pinB, OUTPUT);
  pinMode(pinA, OUTPUT);
  digitalWrite(pinB, LOW);
  digitalWrite(pinA, HIGH);

  //setup imu1
  imuSetup(imu1, aRes1, gRes1, mRes1);

  digitalWrite(pinA, LOW);
  digitalWrite(pinB, HIGH);

  //setup imu2
  imuSetup(imu2, aRes2, gRes2, mRes2);

  digitalWrite(pinB, LOW);
  digitalWrite(pinA, HIGH);

  //leds
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);

  //ev configuration
  ev_config[0] = "10011001"; //spo x
  ev_config[1] = "11001100"; //spo y
  ev_config[2] = "00001111"; //spo z
  ev_config[3] = "00111100"; //rot x
  ev_config[4] = "10010110"; //rot y
  ev_config[5] = "01010101"; //rot z
  ev_config[6] = "01100110"; //-spo x
  ev_config[7] = "00110011"; //-spo y
  ev_config[8] = "11110000"; //-spo z
  ev_config[9] = "11000011"; //-rot x
  ev_config[10] = "01101001"; //-rot y
  ev_config[11] = "10101010"; //-rot z

}

void loop() {
  if(!flag_thruster_open){//!flag_thruster_open
    if(!inputs.isEmpty()){

      flag_thruster_open = true;  
      input_serial[0] = '\0';
      sprintf(input_serial,"%s", inputs.dequeue().input_char);    
      
      log_msg[0] = '\0';
      sprintf(log_msg,"Current input :%s", input_serial);
      nh.loginfo(log_msg);
        
      cycles = (input_serial[4] - '0') * 100 + (input_serial[5] - '0') * 10 + input_serial[6] - '0';

      start = millis();
      OpenThrusters(input_serial);
        
      //print_info(input_serial);
    }
  }
  if(((millis() - start) > cycles*INTERVAL_ROS_MSG) and cycles != -1){
    CloseThrusters(); 
    log_msg[0] = '\0';
    sprintf(log_msg,"Effective time :%d", (millis()-start));
    nh.loginfo(log_msg);
  }

  digitalWrite(pinB, LOW);
  digitalWrite(pinA, HIGH);

  //imus updating and sending information
  imuUpdate(imu1, aRes1, gRes1, mRes1, statusXL_G1, statusM1, newXLData1, newGData1, newMData1, sensorValues1,
  ax1, ay1, az1, gx1, gy1, gz1, mx1, my1, mz1, lastRefreshTime1, lastUpdate1, now1);
  imuPublish(imu1, gRes1, mRes1, ax1, ay1, az1, gx1, gy1, gz1, imu1_msg, imu1_pub, lastRefreshTime1, name_imu1);
  
  digitalWrite(pinA, LOW);
  digitalWrite(pinB, HIGH);

  imuUpdate(imu2, aRes2, gRes2, mRes2, statusXL_G2, statusM2, newXLData2, newGData2, newMData2, sensorValues2,
  ax2, ay2, az2, gx2, gy2, gz2, mx2, my2, mz2, lastRefreshTime2, lastUpdate2, now2);   
  imuPublish(imu2, gRes2, mRes2, ax2, ay2, az2, gx2, gy2, gz2, imu2_msg, imu2_pub, lastRefreshTime2, name_imu2);

  nh.spinOnce();
}

void OpenThrusters(char *input){
  nh.loginfo("Thrusters opened:");
  for(i=0;i<num_ev;i++)
    if(ev_config[((input[0] - '0') * 10 + input[1] - '0')][i]-'0')
      OpenSingleThruster(equalize[((input[2] - '0') * 10 + input[3] - '0')]);
}

void OpenSingleThruster(uint8_t acc){
  digitalWrite(i+2, HIGH);
  log_msg[0] = '\0';
  sprintf(log_msg,"--- Thruster:%d Acc:%d",i,acc);
  nh.loginfo(log_msg);
}

void CloseThrusters(){
  for(i=0;i<num_ev;i++)
    digitalWrite(i+2, LOW);
  count_cycles = 0;
  cycles = -1;
  flag_thruster_open = false;
  nh.loginfo("Thruster closed");
}

// Print the input recived
void print_info(char *info){
  log_msg[0] = '\0';
  sprintf(log_msg,"Axis: %d%d",info[0],info[1]);
  sprintf(log_msg," --- Acceleration: %d%d",info[2],info[3]);
  sprintf(log_msg," --- Time[ms]: %d",(cycles*INTERVAL_ROS_MSG));
  nh.loginfo(log_msg);
}

// Function used to print the error
void print_error(char *err){
  log_msg[0] = '\0';
  sprintf(log_msg,"\n\n ERROR: %s \n\n",err);
	nh.loginfo(log_msg);
}

void imuSetup(Ixmatix_LSM9DS1 &imu, float &aRes, float &gRes, float &mRes){
  
  // Wait for sensor
  while (!imu.isLSM9DS1Ready());

  imu.setConfig();

  // Mahony variables
  imu.Kp = (2.0f * 6.0f);
  imu.Ki = (2.0f * 0.0f);
  
  // Madgwick variables
  imu.updateBeta(15.f);
  
  imu.initLSM9DS1();                // Setting sensor registers to get data
  imu.getRes(&aRes, &gRes, &mRes);  // Get resolution to convert raw data to real data
}

void imuUpdate(Ixmatix_LSM9DS1 &imu, float &aRes, float &gRes, float &mRes, 
uint8_t &statusXL_G, uint8_t &statusM, 
uint8_t &newXLData, uint8_t &newGData, uint8_t &newMData, int16_t *sensorValues,
float &ax, float &ay, float &az, float &gx, float &gy, float &gz, float &mx, float &my, float &mz,
unsigned long &lastRefreshTime, int32_t &lastUpdate, uint32_t &now){
  
  // Read accelerometer and gyroscope register status from sensor
  imu.readI2C(LSM9DS1_ADDR_XG, LSM9DS1_R_XG_STATUS_REG, 1, &statusXL_G);
  // Get a boolean that indicates if there's new data available from status register
  newXLData = statusXL_G & 0x01;
  newGData  = statusXL_G & 0x02;
  // Read magnetometer register status from sensor
  imu.readI2C(LSM9DS1_ADDR_M, LSM9DS1_R_M_STATUS_REG_M, 1, &statusM);
  // Get a boolean that indicates if there's new data available from status register
  newMData  = statusM & 0x08;

  // is accelerometer data available?
  if (newXLData) {
    imu.readSensorData(LSM9DS1_ADDR_XG, LSM9DS1_R_XG_OUT_X_L_XL, sensorValues); // get sensor values
    
    // The data that is returned from the sensor is
    // raw data obtained by the ADC (Analog to Digital Converter) as LSB units
    // we need to multiply this value to its resolution converter
    // to obtain real data. Check page 12 from datasheet
    ax = (float)sensorValues[0] * aRes - xl_offsets[0]; // offset or bias is subtracted to get calibrated data
    ay = (float)sensorValues[1] * aRes - xl_offsets[1];
    az = (float)sensorValues[2] * aRes - xl_offsets[2];
  }
  
  // is gyoscope data available?
  if (newGData) {
    imu.readSensorData(LSM9DS1_ADDR_XG, LSM9DS1_R_XG_OUT_X_L_G, sensorValues); // get sensor values
    
    gx = (float)sensorValues[0] * gRes - g_offsets[0];  // offset or bias is subtracted to get calibrated data
    gy = (float)sensorValues[1] * gRes - g_offsets[1];
    gz = (float)sensorValues[2] * gRes - g_offsets[2];
  }
  
  // is accelerometer data available?
  if (newMData) {
    imu.readSensorData(LSM9DS1_ADDR_M, LSM9DS1_R_M_OUT_X_L_M, sensorValues); // get sensor values

    // Compensating for Hard-Iron Effects
    float mx_temp = (float)sensorValues[0] * mRes - mag_offsets[0];  // offset or bias is subtracted to get calibrated data
    float my_temp = (float)sensorValues[1] * mRes - mag_offsets[1];
    float mz_temp = (float)sensorValues[2] * mRes - mag_offsets[2];
    
    // Compensating for Soft-Iron Effects
    mx = mx_temp * mag_softiron_matrix[0][0] + my_temp * mag_softiron_matrix[0][1] + mz_temp * mag_softiron_matrix[0][2];
    my = mx_temp * mag_softiron_matrix[1][0] + my_temp * mag_softiron_matrix[1][1] + mz_temp * mag_softiron_matrix[1][2];
    mz = mx_temp * mag_softiron_matrix[2][0] + my_temp * mag_softiron_matrix[2][1] + mz_temp * mag_softiron_matrix[2][2];
  }

  // Get dt expressed in sec for madgwick/mahony algorithms
  now = micros();
  imu.deltat = ((now - lastUpdate) / 1000000.0f);
  lastUpdate = now;

  // Algorithm filter
  // mx is sent as the negative of mx due to axis coordinate difference
  // check datasheet page 10
  // y-axis is sent as the negative of y-axis to correct quaternion orientation
  // Madgwick and Mahony algorithms require rad/seg for gyroscope values
  // 1 rad/seg = 1 deg * (PI/ 180)
  //                          (                 |                   |                   |   |    |   |    |    |   )
  imu.updateMadgwickQuaternion(gx * PI / 180.0f , -gy * PI / 180.0f , gz * PI / 180.0f  , ax, -ay, az, -mx, -my, mz);
  //imu.updateMahonyQuaternion(  gx*PI/180.0f     , -gy*PI/180.0f     , gz*PI/180.0f      , ax, -ay, az, -mx, -my, mz);
}

void imuPublish(Ixmatix_LSM9DS1 &imu, float &gRes, float &mRes, 
float &ax, float &ay, float &az, float &gx, float &gy, float &gz,
sensor_msgs::Imu &imu_msg, ros::Publisher &imu_pub,
unsigned long &lastRefreshTime, char *name)
{
  if (millis() - lastRefreshTime >= INTERVAL_ROS_MSG){
  
    // sensor_msgs/Imu Message
    // http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/Imu.html
    // you can define frame_id with a representative name for your robot
    imu_msg.header.frame_id =  name;
    imu_msg.header.stamp = nh.now();
    imu_msg.orientation.x = imu.q[1];
    imu_msg.orientation.y = imu.q[2];
    imu_msg.orientation.z = imu.q[3];
    imu_msg.orientation.w = imu.q[0];

    // Imu message requires rad/sec for rotational velocity
    // LSM9DS1 gyroscope sensor gives deg/seg
    // 1 rad/seg = 1 deg * (PI/ 180)
    imu_msg.angular_velocity.x = gx * PI / 180.0f;
    imu_msg.angular_velocity.y = gy * PI / 180.0f;
    imu_msg.angular_velocity.z = gz * PI / 180.0f;

    // Imu message requires m/s^2 (not g's) for linear acceleration
    // LSM9DS1 accelerometer sensor gives mg data
    // 1 m/s^2 = 1 mg*(1g / 1000mg)*(9.81 m/s^2 / 1g) = 0.00981 m/s^2
    imu_msg.linear_acceleration.x = ax * 0.00981;
    imu_msg.linear_acceleration.y = ay * 0.00981;
    imu_msg.linear_acceleration.z = az * 0.00981;
    
    // Gyroscope bias is the standard deviation
    // variance is just the square of it.
    imu_msg.angular_velocity_covariance[0] = g_offsets[0]*g_offsets[0];
    imu_msg.angular_velocity_covariance[4] = g_offsets[1]*g_offsets[1];;
    imu_msg.angular_velocity_covariance[8] = g_offsets[2]*g_offsets[2];
    
    // Accelerometer bias is the standard deviation
    // variance is just the square of it
    imu_msg.linear_acceleration_covariance[0] = xl_offsets[0]*xl_offsets[0];
    imu_msg.linear_acceleration_covariance[4] = xl_offsets[1]*xl_offsets[1];
    imu_msg.linear_acceleration_covariance[8] = xl_offsets[2]*xl_offsets[2];
    
    // orientation_covariance is filled with zeros to
    // indicate a "covariance unknown"

    // send imu_msg to ros
    imu_pub.publish(&imu_msg);
    //Serial.println(millis()-lastRefreshTime);
    lastRefreshTime = millis();
    nh.spinOnce();
  }
}
