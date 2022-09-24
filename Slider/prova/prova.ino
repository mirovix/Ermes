#define PIN_POS1 6
#define PIN_POS2 3
//#define PIN_POS3 4
#define PIN_HOME 5
//#define PIN_START 6
//#define PIN_PAUSE 7
#define PIN_POSCOMP 8
//#define PIN_HCOMP 9

#define PIN_EM 9

void setup() {
Serial.begin(9600);
    pinMode(PIN_POS1, OUTPUT);
  pinMode(PIN_POS2, OUTPUT);
  pinMode(PIN_HOME, OUTPUT);
  pinMode(PIN_EM, OUTPUT);

  pinMode(PIN_POSCOMP, INPUT);
  

}

void loop() {

  //Serial.println("H");
  digitalWrite(PIN_POS1, LOW);

Serial.println("on");
  delay(3000);

  //digitalWrite(PIN_POS1, HIGH);
    delay(3000);
    Serial.println("off");
  
  
  //Serial.println("L");
  //digitalWrite(PIN_POS1, LOW);

}
