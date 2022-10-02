


/*
    2 LSM9DS1 sensors communication using chipset  with ROS
    and use of Madgwick filter
    
    Original Creation Date: 10/01/19

    Based on Kris Winer' Code
    More info: https://github.com/kriswiner/LSM9DS1
*/

#include <Ixmatix_LSM9DS1.h>
#include <Adafruit_VL6180X.h>
#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PointStamped.h>
//#include <TCA9548A.h>

int count = 0, length_input = 0;
int i = 0;

//time synchronization 
unsigned long start,loop_start = 0;

const uint32_t INTERVAL_ROS_MSG = (uint32_t) 1.f/(200.f/13.f)*1000; //65ms

ros::NodeHandle nh;
sensor_msgs::Imu imu_msg1, imu_msg2;
geometry_msgs::PointStamped tof_msg;
ros::Publisher imu_pub("/chaser/sensors/imu0", &imu_msg1);
ros::Publisher tof_pub("/chaser/sensors/tof", &tof_msg);
Ixmatix_LSM9DS1 imu1, imu2;

Adafruit_VL6180X vl1 = Adafruit_VL6180X();
Adafruit_VL6180X vl2 = Adafruit_VL6180X();
Adafruit_VL6180X vl3 = Adafruit_VL6180X();
//tofs values
float lux1, lux2, lux3;
uint8_t range1, range2, range3, status1, status2, status3;
//TCA9548A I2CMux; 

//pin on/off for tof and imus definitions
uint8_t pin_imu1 = 0, pin_imu2 = -1, pin_tof1 = 1, pin_tof2 = 2, pin_tof3 = 3;

unsigned long lastRefreshTime1 = 0, lastRefreshTime2 = 0, lastRefreshTimeTof = 0;
int32_t lastUpdate1, lastUpdate2 = 0; 
uint32_t now1 = 0, now2 = 0;

// xl = accelerometer ----- g  = gyroscope
float xl_offsets[3] = { +0.04f, -0.10f, -0.01f  };
float g_offsets[3]  = { -0.05f, +0.35f, -0.73f  };

// Magnetic calibration data - obtaied from Motion Sensor Calibration Tool
// link - https://www.pjrc.com/store/prop_shield.html
// Magnetic offset
float mag_offsets[3]            = { -5.63/1000., 16.78/1000., -6.8/1000. };

// Magnetic Mapping
// Soft iron error compensation matrix
float mag_softiron_matrix[3][3] = { { +0.990, -0.002, -0.005 },
                                    { -0.002, +0.989, -0.075 },
                                    { -0.005, -0.075, +1.027 }
                                  };

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
uint8_t newXLData1, newXLData2;
uint8_t newGData1, newGData2;
uint8_t statusM1, statusM2;
uint8_t newMData1, newMData2;

int16_t sensorValues1[3], sensorValues2[3];

char name_imu1[] = "imu0";
char name_imu2[] = "imu1";
char name_tof[] = "tof";


void setup() {

  Serial.begin(115200);
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(imu_pub);
  nh.advertise(tof_pub);
  Wire.begin();

  //chipselect between imus and tofs
  initTCA9548A_PS();

  //chipselect between imu1 and imu2
  //TCA9548A_PS(pin_imu1);

  //setup the frist imu
  //imuSetup(imu1, aRes1, gRes1, mRes1);

  //TCA9548A_PS(pin_imu2);

  //setup the second imu
  //imuSetup(imu2, aRes2, gRes2);

  //setup the first tof
  TCA9548A_PS(pin_tof1);
  tofSetup(vl1);

  //setup the second tof
  TCA9548A_PS(pin_tof2);
  tofSetup(vl2);

  //setup the third tof
  TCA9548A_PS(pin_tof3);
  tofSetup(vl3);
}

void imuSetup(Ixmatix_LSM9DS1 &imu, float &aRes, float &gRes, float &mRes){
  //Serial.println("Waiting for LSM9DS1");
  
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

void tofSetup(Adafruit_VL6180X &vl){
  if (!vl.begin()) {
    Serial.println("Failed to find sensor");
    while (1);
  }
  Serial.println("Sensor found!");
}

void loop() { 
  
  //TCA9548A_PS(pin_imu1);
  
  //loop for the frist imu
  //imuLoop(imu_msg1, imu1, aRes1, gRes1, mRes1, statusXL_G1, newXLData1, newGData1, statusM1, newMData1,
  //sensorValues1, ax1, ay1, az1, gx1, gy1, gz1, mx1, my1, mz1, name_imu1, lastRefreshTime1, lastUpdate1, now1);

  //TCA9548A_PS(pin_imu2);

  //loop for the second imu
  ///imuLoop(imu2, aRes2, gRes2, statusXL_G2, newXLData2, newGData2, sensorValues2,
  //ax2, ay2, az2, gx2, gy2, gz2, ax_2, ay_2, az_2, gx_2, gy_2, gz_2, name_imu2,
  //lastRefreshTime2, lastUpdate2, now2);

  //loop for the first tof
  TCA9548A_PS(pin_tof1);
  tofLoop(vl1, lux1, range1);

  //loop for the second tof
  TCA9548A_PS(pin_tof2);
  tofLoop(vl2, lux2, range2);

  //loop for the third tof
  TCA9548A_PS(pin_tof3);
  tofLoop(vl3, lux3, range3);

  pubtof(range1, range2, range3, name_tof, lastRefreshTimeTof);

}

void imuLoop(sensor_msgs::Imu imu_msg, Ixmatix_LSM9DS1 &imu, float &aRes, float &gRes, float &mRes,
uint8_t &statusXL_G, uint8_t &newXLData,
uint8_t &newGData, uint8_t &statusM, uint8_t newMData,
int16_t *sensorValues,
float &ax, float &ay, float &az, float &gx, float &gy, float &gz, float &mx, float &my, float &mz,
char *name, unsigned long &lastRefreshTime, int32_t &lastUpdate, uint32_t &now){
  
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
  
  // is it time to send data to ROS?
  if (millis() - lastRefreshTime >= INTERVAL_ROS_MSG) {
    lastRefreshTime += INTERVAL_ROS_MSG;
    
    // sensor_msgs/Imu Message
    // http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/Imu.html
    // you can define frame_id with a representative name for your robot
    imu_msg.header.frame_id = name;
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

    imu_pub.publish(&imu_msg);
    nh.spinOnce();
  }
}

void tofLoop(Adafruit_VL6180X &vl, float &lux, uint8_t &range){
  range = vl.readLux(VL6180X_ALS_GAIN_5);
}

void pubtof(uint8_t &range1, uint8_t &range2, uint8_t &range3, char *name, unsigned long &lastRefreshTime){
  if (millis() - lastRefreshTime >= INTERVAL_ROS_MSG){
    lastRefreshTime += INTERVAL_ROS_MSG;
    tof_msg.header.frame_id = name;
    tof_msg.header.stamp = nh.now();
    int div = 3;
    //TODO >> combinazione dei tre randge
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

// Function used to print the error
void print_error(char *err)
{
	Serial.print("\n\n ERROR: ");
  Serial.print(err);
  Serial.print("\n\n");
	return;
}

void initTCA9548A_PS(){
  pinMode(pin_imu1, OUTPUT);
  pinMode(pin_imu2, OUTPUT);
  pinMode(pin_tof1, OUTPUT);
  pinMode(pin_tof2, OUTPUT);
  pinMode(pin_tof3, OUTPUT);
}

void TCA9548A_PS(uint8_t bus) //function of TCA9548A
{
  Wire.beginTransmission(0x70);  // TCA9548A address is 0x70
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();
}
