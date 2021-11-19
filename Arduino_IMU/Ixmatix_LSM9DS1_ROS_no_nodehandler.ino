/*
    LSM9DS1 sensor communication with ROS
    and use of Madgwick filter
    
    Original Creation Date: 10/01/19

    Based on Kris Winer' Code
    More info: https://github.com/kriswiner/LSM9DS1
*/

#include <Ixmatix_LSM9DS1.h>


/*
 * Ros variables
 */
// ROS Message Interval
const uint32_t INTERVAL_ROS_MSG = (uint32_t) 1.f/30.f*1000; // 30Hz
unsigned long lastRefreshTime = 0;


Ixmatix_LSM9DS1 imu;

// deltat variables
int32_t lastUpdate = 0; 
uint32_t now = 0;

/*
 * Calibration variables
 */

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
float aRes;
float gRes;
float mRes;

float axVar;
float ayVar;
float azVar;

float gxVar;
float gyVar;
float gzVar;

float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;

int16_t ax_, ay_, az_;
int16_t gx_, gy_, gz_;

uint8_t statusXL_G;
uint8_t statusM;
uint8_t newXLData;
uint8_t newGData;
uint8_t newMData;

int16_t sensorValues[3];


void setup() {
  Serial.begin(230400);
  Serial.println("Waiting for LSM9DS1");
  
  
  
  // Wait for sensor
  Wire.begin();
  while (!imu.isLSM9DS1Ready());

  // sensor is ready

  Serial.println("LSM9DS1: Ready...");
  Serial.println("LSM9DS1: Setting Gyro-Accel Ctrl Registers");

  imu.setConfig();

  // Mahony variables
  imu.Kp = (2.0f * 6.0f);
  imu.Ki = (2.0f * 0.0f);
  
  // Madgwick variables
  imu.updateBeta(15.f);
  
  Serial.print("Measurement error: ");
  Serial.println(imu.measErrorDeg);
  
  imu.initLSM9DS1();                // Setting sensor registers to get data
  imu.getRes(&aRes, &gRes, &mRes);  // Get resolution to convert raw data to real data
  
  Serial.print("aRes: ");
  Serial.println(aRes, 8);
  Serial.print("gRes: ");
  Serial.println(gRes, 8);
  Serial.print("mRes: ");
  Serial.println(mRes, 8);
  Serial.println("");

  Serial.print("aOff: ");
  Serial.print(xl_offsets[0], 3); Serial.print(" ");
  Serial.print(xl_offsets[1], 3); Serial.print(" ");
  Serial.println(xl_offsets[2], 3);
  Serial.print("gOff: ");
  Serial.print(g_offsets[0], 3); Serial.print(" ");
  Serial.print(g_offsets[1], 3); Serial.print(" ");
  Serial.println(g_offsets[2], 3);
  Serial.print("mOff: ");
  Serial.print(mag_offsets[0], 8); Serial.print(" ");
  Serial.print(mag_offsets[1], 8); Serial.print(" ");
  Serial.print(mag_offsets[2], 8); Serial.print(" ");
  
  Serial.print(mag_softiron_matrix[0][0], 3); Serial.print(" ");
  Serial.print(mag_softiron_matrix[0][1], 3); Serial.print(" ");
  Serial.print(mag_softiron_matrix[0][2], 3); Serial.print(" ");
  Serial.print(mag_softiron_matrix[1][0], 3); Serial.print(" ");
  Serial.print(mag_softiron_matrix[1][1], 3); Serial.print(" ");
  Serial.print(mag_softiron_matrix[1][2], 3); Serial.print(" ");
  Serial.print(mag_softiron_matrix[2][0], 3); Serial.print(" ");
  Serial.print(mag_softiron_matrix[2][1], 3); Serial.print(" ");
  Serial.println(mag_softiron_matrix[2][2], 3);
  
  
  axVar = xl_offsets[0]*xl_offsets[0];
  ayVar = xl_offsets[1]*xl_offsets[1];
  azVar = xl_offsets[2]*xl_offsets[2];
  
  gxVar = g_offsets[0]*g_offsets[0];
  gyVar = g_offsets[1]*g_offsets[1];
  gzVar = g_offsets[2]*g_offsets[2];
  
}


void loop() {
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
    ax_ = sensorValues[0];
    ay_ = sensorValues[1];
    az_ = sensorValues[2];
  }
  
  // is gyoscope data available?
  if (newGData) {
    imu.readSensorData(LSM9DS1_ADDR_XG, LSM9DS1_R_XG_OUT_X_L_G, sensorValues); // get sensor values
    
    gx = (float)sensorValues[0] * gRes - g_offsets[0];  // offset or bias is subtracted to get calibrated data
    gy = (float)sensorValues[1] * gRes - g_offsets[1];
    gz = (float)sensorValues[2] * gRes - g_offsets[2];
    gx_ = sensorValues[0];
    gy_ = sensorValues[1];
    gz_ = sensorValues[2];
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
    
    Serial.print("Imu: ");
    // orientation
    //*
    Serial.print(imu.q[0],7);
    Serial.print(' ');
    Serial.print(imu.q[1],7);
    Serial.print(' ');
    Serial.print(imu.q[2],7);
    Serial.print(' ');
    Serial.print(imu.q[3],7);
    Serial.print(' ');
    // angular_velocity
    Serial.print(gx_);
    Serial.print(' ');
    Serial.print(gy_);
    Serial.print(' ');
    Serial.print(gz_);
    Serial.print(' ');
    // angular_velocity_covariance 
    Serial.print(gxVar);
    Serial.print(' ');
    Serial.print(gyVar);
    Serial.print(' ');
    Serial.print(gzVar);
    Serial.print(' ');
    //*/
    // linear_acceleration
    Serial.print(ax_);
    Serial.print(' ');
    Serial.print(ay_);
    Serial.print(' ');
    Serial.print(az_);
    Serial.print(' ');
    //Serial.println("");
    //*
    // linear_acceleration_covariance
    Serial.print(axVar);
    Serial.print(' ');
    Serial.print(ayVar);
    Serial.print(' ');
    Serial.print(azVar);
    Serial.println("");
    //*/
  }
}
