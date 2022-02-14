/*
    LSM9DS1 sensor communication with ROS
    and use of Madgwick filter
    
    Original Creation Date: 10/01/19

    Based on Kris Winer' Code
    More info: https://github.com/kriswiner/LSM9DS1
*/

#include <Ixmatix_LSM9DS1.h>
#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>


/*
 * Ros variables
 */
ros::NodeHandle nh;
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu/data", &imu_msg);

geometry_msgs::TransformStamped tfs_msg;
tf::TransformBroadcaster tfbroadcaster;

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

float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;

uint8_t statusXL_G;
uint8_t statusM;
uint8_t newXLData;
uint8_t newGData;
uint8_t newMData;

int16_t sensorValues[3];


void setup() {
  // Initialize ROS node handler, imu publisher and tf broadcaster
  Serial.begin(230400);
  nh.getHardware()->setBaud(230400);
  nh.initNode();
  nh.advertise(imu_pub);
  tfbroadcaster.init(nh);
  
  // Wait for sensor
  Wire.begin();
  while (!imu.isLSM9DS1Ready());

  // sensor is ready
  imu.setConfig();

  // Mahony variables
  imu.Kp = (2.0f * 6.0f);
  imu.Ki = (2.0f * 0.0f);
  
  // Madgwick variables
  imu.updateBeta(15.f);

  imu.initLSM9DS1();                // Setting sensor registers to get data
  imu.getRes(&aRes, &gRes, &mRes);  // Get resolution to convert raw data to real data
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
    imu_msg.header.frame_id =  "imu_second_floor_link";
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

    // This will be useful for ROS to determine
    // where the IMU sensor is with respect to the base_link
    tfs_msg.header.stamp    = nh.now();
    tfs_msg.header.frame_id = "base_link";
    tfs_msg.child_frame_id  = "imu_second_floor_link";
    tfs_msg.transform.rotation.x = imu.q[1];
    tfs_msg.transform.rotation.y = imu.q[2];
    tfs_msg.transform.rotation.z = imu.q[3];
    tfs_msg.transform.rotation.w = imu.q[0];
    tfs_msg.transform.translation.x = 0.;
    tfs_msg.transform.translation.y = 0.;
    tfs_msg.transform.translation.z = 0.10;
    
    // send transform to ROS
    tfbroadcaster.sendTransform(tfs_msg);

    nh.spinOnce();
  }
}
