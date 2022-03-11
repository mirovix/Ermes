
//GLOBAL VARIABLES
//[0-5] positive axis [6-11] negative axis
//input(6 elements) >>> stringa (asse, accelerazione, cicli) (e.g. '0' '14' '5') 
// 4 bit + 4 bit + 8 bit (+ 1 bit) (asse, accelerazione, cicli, controllo) N.B. MAX cicli con 10Hz >> 220
const int num_input = 6;
const int num_ev = 8;
const int dof = 6;
uint8_t count = 0, length_input = 0;
uint8_t i = 0;
uint8_t count_cycles = 0, cycles = -1;
bool flag_thruster_open = false;
char input_serial[8];
uint8_t axes = 0, acceleration = 0;
bool error_check = false;

//from 4 to 16 bit PWM
int equalize[16] = {0,83,94,103,111,121,132,144,157,172,187,206,223,244,250,255};

//matrix (2(sign) X 6(dof)) X 8(ev) for trhsruster configuration
char *ev_config[2*dof];

//time synchronization 
unsigned long start,loop_start = 0;

const uint32_t INTERVAL_ROS_MSG = (uint32_t) 1.f/10.f*1000; // 10Hz



void setup() {

  Serial.begin(230400);

  TCCR2B = (TCCR2B & 0b11111000) | 0x07; //30.65 Hz

  randomSeed(analogRead(0));

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
  ev_config[1] = "01100110"; //-spo x

  ev_config[2] = "11001100"; //spo y
  ev_config[3] = "00110011"; //-spo y

  ev_config[4] = "00001111"; //spo z
  ev_config[5] = "11110000"; //-spo z

  ev_config[6] = "00111100"; //rot x
  ev_config[7] = "11000011"; //-rot x

  ev_config[8] = "10010110"; //rot y
  ev_config[9] = "01101001"; //-rot y

  ev_config[10] = "01010101"; //rot z
  ev_config[11] = "10101010"; //-rot z

}

void loop() {
  loop_start = millis();
  if(!flag_thruster_open){
    if(Serial.available()){    
      input_serial[0] = '\0';  
      length_input = Serial.readBytes(input_serial, 8);
      input_serial[7] = '\0';
      if(length_input == 8 and print_check_info(input_serial)){
        flag_thruster_open = true;
        Serial.print("Input:");
        Serial.println(input_serial);
        start = millis();
        OpenThrusters(input_serial); 
      }
    }
  }
  else
    count_cycles++;
  
  if(count_cycles > cycles and cycles != -1){
    CloseThrusters(); 
    Serial.print("Effective time:");
    Serial.println(millis()-start);
  }

  delay(INTERVAL_ROS_MSG-(millis()-loop_start));
}

void OpenThrusters(char *input){
  Serial.print("Thrusters opened:");
  for(i=0;i<num_ev;i++)
    if(ev_config[axes][i]-'0')
      OpenSingleThruster(equalize[acceleration]);
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

// Print and check the input recived
bool print_check_info(char *info){
  
  error_check = true;

  Serial.print("Ax:");
  axes = (info[0] - '0') * 10.0 + info[1] - '0';
  Serial.print(axes);
  if(axes > 11 or axes < 0)
    error_check = print_error("Axes is not between 11 and 0");
  
  Serial.print(" --- Acceleration:");
  acceleration = (info[2] - '0') * 10 + info[3] - '0';
  Serial.print(acceleration);
  if(acceleration > 15 or acceleration < 0)
    error_check = print_error("Acceleration is not between 15 and 0");

  Serial.print(" --- Time[ms]:");
  cycles = (info[4] - '0') * 100.0 + (info[5] - '0') * 10.0 + info[6] - '0';
  Serial.println(cycles*INTERVAL_ROS_MSG);
  if(cycles > 220 or cycles < 1){
    error_check = print_error("Cycles are over/under limit");
    cycles = -1;
  }
  else
    cycles--;

  if(error_check)
    return true;
  else
    return false;
}

// Function used to print the error
bool print_error(char *err)
{
	Serial.print("\n\n ERROR: ");
  Serial.print(err);
  Serial.print("\n\n");
	return false;
}
