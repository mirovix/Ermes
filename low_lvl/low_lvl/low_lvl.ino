
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
char input_serial[8];

//from 4 to 16 bit PWM
int equalize[16] = {0,83,94,103,111,121,132,144,157,172,187,206,223,244,250,255};

//matrix (2(sign) X 6(dof)) X 8(ev) for trhsruster configuration
char *ev_config[2*dof];

//time synchronization 
unsigned long start,loop_start = 0;

const uint32_t INTERVAL_ROS_MSG = (uint32_t) 1.f/65.f*1000; // 10Hz


void setup() {

  Serial.begin(115200);

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
  ev_config[0] = "11001100"; //spo x
  ev_config[1] = "11110000"; //spo y
  ev_config[2] = "01100110"; //spo z
  ev_config[3] = "10010110"; //rot x
  ev_config[4] = "10101010"; //rot y
  ev_config[5] = "11000011"; //rot z
  ev_config[6] = "00110011"; //-spo x
  ev_config[7] = "00001111"; //-spo y
  ev_config[8] = "10011001"; //-spo z
  ev_config[9] = "01101001"; //-rot x
  ev_config[10] = "01010101"; //-rot y
  ev_config[11] = "00111100"; //-rot z

}

void loop() {
  loop_start = millis();
  if(!flag_thruster_open){
    if(Serial.available()){
      flag_thruster_open = true;      
      length_input = Serial.readBytes(input_serial, 8);
      input_serial[7] = '\0';
      if(length_input>0){
        
        //Serial.print("Input:");
        //Serial.println(input_serial);
        
        cycles = (input_serial[4] - '0') * 100 + (input_serial[5] - '0') * 10 + input_serial[6] - '0';
        cycles--;

        start = millis();
        OpenThrusters(input_serial);
        
        //print_info(input_serial);
      }
    }
  }
  else
    count_cycles++;
  
  //Serial.println(millis()-loop_start);  
  
  if(count_cycles > cycles and cycles != -1){
    CloseThrusters(); 
    //Serial.print("Effective time:");
    //Serial.println(millis()-start);
  }
  
  delay(INTERVAL_ROS_MSG-(millis()-loop_start));
}

void OpenThrusters(char *input){
  //Serial.print("Thrusters opened:");
  for(i=0;i<num_ev;i++)
    if(ev_config[((input[0] - '0') * 10 + input[1] - '0')][i]-'0')
      OpenSingleThruster(equalize[((input[2] - '0') * 10 + input[3] - '0')]);
  //Serial.println();
}

void OpenSingleThruster(uint8_t acc){
  digitalWrite(i+2, HIGH);
  //Serial.print("--- Thruster:");
  //Serial.print(i);
  //Serial.print(" Acc:");
  //Serial.print(acc);
}

void CloseThrusters(){
  for(i=0;i<num_ev;i++)
    digitalWrite(i+2, LOW);
  count_cycles = 0;
  cycles = -1;
  flag_thruster_open = false;
  //Serial.println("Thruster closed");
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
