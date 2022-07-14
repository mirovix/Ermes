


/*
    2 LSM9DS1 sensors communication using chipset  with ROS
    and use of Madgwick filter
    
    Original Creation Date: 10/01/19

    Based on Kris Winer' Code
    More info: https://github.com/kriswiner/LSM9DS1
*/

#include <Ixmatix_LSM9DS1.h>
#include <ros.h>
#include <TCA9548A.h>

const int num_input = 6;
const int num_ev = 8;
const int dof = 6;
int count = 0, length_input = 0;
int i = 0;
int count_cycles = 0, cycles = -1;
bool flag_thruster_open = false;
char input_serial[8];


//from 4 to 16 bit PWM
int equalize[16] = {0,83,94,103,111,121,132,144,157,172,187,206,223,244,250,255};

//matrix (2(sign) X 6(dof)) X 8(ev) for trhsruster configuration
char *ev_config[2*dof];

//time synchronization 
unsigned long start,loop_start = 0;

const uint32_t INTERVAL_ROS_MSG = (uint32_t) 1.f/10.f*1000; // 10Hz

Ixmatix_LSM9DS1 imu1;
Ixmatix_LSM9DS1 imu2;
TCA9548A I2CMux; 

//pin on/off for tof and imus definitions
uint8_t pin_imu1 = 7, pin_imu2 = 1, pin_tof1 = 10, pin_tof2 = 11, pin_tof3 = 12;

unsigned long lastRefreshTime1 = 0, lastRefreshTime2 = 0;
int32_t lastUpdate1, lastUpdate2 = 0; 
uint32_t now1 = 0, now2 = 0;

// xl = accelerometer ----- g  = gyroscope
float xl_offsets[3] = { +0.04f, -0.10f, -0.01f  };
float g_offsets[3]  = { -0.05f, +0.35f, -0.73f  };

/*
 * LSM9DS1 variables
 */
float aRes1, aRes2;
float gRes1, gRes2;

float ax1, ay1, az1, ax2, ay2, az2;
float gx1, gy1, gz1, gx2, gy2, gz2;

int16_t ax_1, ay_1, az_1, ax_2, ay_2, az_2;
int16_t gx_1, gy_1, gz_1, gx_2, gy_2, gz_2;

uint8_t statusXL_G1, statusXL_G2;
uint8_t newXLData1, newXLData2;
uint8_t newGData1, newGData2;

int16_t sensorValues1[3], sensorValues2[3];

char name_imu1[] = "Imu1: ";
char name_imu2[] = "Imu2: ";


void setup() {

  Serial.begin(115200);
  Wire.begin();

  CloseThrusters();

  //chipselect between imu1 and imu2
  TCA9548A_PS(pin_imu1);

  //setup the frist imu
  imuSetup(imu1, aRes1, gRes1);

  TCA9548A_PS(pin_imu2);

  //setup the second imu
  imuSetup(imu2, aRes2, gRes2);

  Serial.print("aOff: ");
  Serial.print(xl_offsets[0], 3); Serial.print(" ");
  Serial.print(xl_offsets[1], 3); Serial.print(" ");
  Serial.println(xl_offsets[2], 3);
  Serial.print("gOff: ");
  Serial.print(g_offsets[0], 3); Serial.print(" ");
  Serial.print(g_offsets[1], 3); Serial.print(" ");
  Serial.println(g_offsets[2], 3);

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

void imuSetup(Ixmatix_LSM9DS1 &imu, float &aRes, float &gRes){
  Serial.println("Waiting for LSM9DS1");
  
  // Wait for sensor
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
  float mRes;
  imu.getRes(&aRes, &gRes, &mRes);  // Get resolution to convert raw data to real data
  
  Serial.print("aRes: ");
  Serial.println(aRes, 8);
  Serial.print("gRes: ");
  Serial.println(gRes, 8);
}

void loop() {
 
  if(!flag_thruster_open){
    if(Serial.available()){
      flag_thruster_open = true;      
      length_input = Serial.readBytes(input_serial, 8);
      input_serial[7] = '\0';
      if(length_input>0){
        
        Serial.print("Input:");
        Serial.println(input_serial);
        
        cycles = (input_serial[4] - '0') * 100 + (input_serial[5] - '0') * 10 + input_serial[6] - '0';
      
        start = millis();
        OpenThrusters(input_serial);
        
        print_info(input_serial);
      }
    }
  }
  //else
  //  count_cycles++;
  
  //Serial.println(millis()-loop_start);  
  
  if(((millis() - start) > cycles*INTERVAL_ROS_MSG) and cycles != -1){
    CloseThrusters(); 
    Serial.print("Effective time:");
    Serial.println(millis()-start);
  }

  TCA9548A_PS(pin_imu1);
  
  //loop for the frist imu
  imuLoop(imu1, aRes1, gRes1, statusXL_G1, newXLData1, newGData1, sensorValues1,
  ax1, ay1, az1, gx1, gy1, gz1, ax_1, ay_1, az_1, gx_1, gy_1, gz_1, name_imu1,
  lastRefreshTime1, lastUpdate1, now1);

  TCA9548A_PS(pin_imu2);

  //loop for the second imu
  imuLoop(imu2, aRes2, gRes2, statusXL_G2, newXLData2, newGData2, sensorValues2,
  ax2, ay2, az2, gx2, gy2, gz2, ax_2, ay_2, az_2, gx_2, gy_2, gz_2, name_imu2,
  lastRefreshTime2, lastUpdate2, now2);

}

void imuLoop(Ixmatix_LSM9DS1 &imu, float &aRes, float &gRes, uint8_t &statusXL_G, uint8_t &newXLData,
uint8_t &newGData, int16_t *sensorValues,
float &ax, float &ay, float &az, float &gx, float &gy, float &gz,
int16_t &ax_, int16_t &ay_, int16_t &az_, int16_t &gx_, int16_t &gy_, int16_t &gz_, char *name,
unsigned long &lastRefreshTime, int32_t &lastUpdate, uint32_t &now){
  
  // Read accelerometer and gyroscope register status from sensor
  imu.readI2C(LSM9DS1_ADDR_XG, LSM9DS1_R_XG_STATUS_REG, 1, &statusXL_G);
  // Get a boolean that indicates if there's new data available from status register
  newXLData = statusXL_G & 0x01;
  newGData  = statusXL_G & 0x02;

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
  float m1,m2,m3;
  imu.updateMadgwickQuaternion(gx * PI / 180.0f , -gy * PI / 180.0f , gz * PI / 180.0f  , ax, -ay, az, m1, m2, m3);
  //imu.updateMahonyQuaternion(  gx*PI/180.0f     , -gy*PI/180.0f     , gz*PI/180.0f      , ax, -ay, az, -mx, -my, mz);
  
  // is it time to send data to ROS?
  if (millis() - lastRefreshTime >= INTERVAL_ROS_MSG) {
    lastRefreshTime += INTERVAL_ROS_MSG;
    
    Serial.print(name);
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
    //*/
    // linear_acceleration
    Serial.print(ax_);
    Serial.print(' ');
    Serial.print(ay_);
    Serial.print(' ');
    Serial.print(az_);
    Serial.print(' ');
    Serial.println("");
    //*/
  }
}

void OpenThrusters(char *input){
  Serial.print("Thrusters opened:");
  for(i=0;i<num_ev;i++)
    if(ev_config[((input[0] - '0') * 10 + input[1] - '0')][i]-'0')
      OpenSingleThruster(equalize[((input[2] - '0') * 10 + input[3] - '0')]);
   Serial.println();
}

void OpenSingleThruster(uint8_t acc){
  digitalWrite(i+2, HIGH);
  Serial.print("--- Thruster:");
  Serial.print(i);
  Serial.print(" Acc:");
  Serial.print(acc);
}

void CloseThrusters(){
  for(i=0;i<num_ev;i++)
    digitalWrite(i+2, LOW);
  count_cycles = 0;
  cycles = -1;
  flag_thruster_open = false;
  Serial.println("Thruster closed");
}

// Print the input recived
void print_info(char *info){
  Serial.print("Axis:");
  Serial.print(info[0]);
  Serial.print(info[1]);
  Serial.print(" --- Acceleration:");
  Serial.print(info[2]);
  Serial.print(info[3]);
  Serial.print(" --- Time[ms]:");
  Serial.println(((info[4] - '0') * 100.0 + (info[5] - '0') * 10.0 + info[6] - '0')*(INTERVAL_ROS_MSG));
}

// Function used to print the error
void print_error(char *err)
{
	Serial.print("\n\n ERROR: ");
  Serial.print(err);
  Serial.print("\n\n");
	return;
}

void TCA9548A_PS(uint8_t bus) //function of TCA9548A
{
  Wire.beginTransmission(0x70);  // TCA9548A address is 0x70
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();
}
