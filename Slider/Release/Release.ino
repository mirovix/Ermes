/*
  Blink

  Turns an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the UNO, MEGA and ZERO
  it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN is set to
  the correct LED pin independent of which board is used.
  If you want to know what pin the on-board LED is connected to on your Arduino
  model, check the Technical Specs of your board at:
  https://www.arduino.cc/en/Main/Products

  modified 8 May 2014
  by Scott Fitzgerald
  modified 2 Sep 2016
  by Arturo Guadalupi
  modified 8 Sep 2016
  by Colby Newman

  This example code is in the public domain.

  https://www.arduino.cc/en/Tutorial/BuiltInExamples/Blink
*/
#define PIN_POS1 2
#define PIN_POS2 3
#define PIN_POS3 4
//#define PIN_HOME 5
//#define PIN_START 6
//#define PIN_PAUSE 7
//#define PIN_POSCOMP 8
//#define PIN_HCOMP 9

//#define PIN_EM 9


// the setup function runs once when you press reset or power the board
void setup()
{

  pinMode(PIN_POS1, OUTPUT);
  pinMode(PIN_POS2, OUTPUT);
  pinMode(PIN_POS3, OUTPUT);
 // pinMode(PIN_HOME, OUTPUT);
 // pinMode(PIN_START, OUTPUT);
 // pinMode(PIN_PAUSE, OUTPUT);
  //pinMode(PIN_POSCOMP, OUTPUT);
  //pinMode(PIN_HCOMP, OUTPUT);
 // pinMode(PIN_EM, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT); 

  digitalWrite(PIN_POS1, HIGH);
  digitalWrite(LED_BUILTIN, HIGH);
  //digitalWrite(PIN_HOME, HIGH);
  //digitalWrite(PIN_POS1, HIGH);
  //digitalWrite(PIN_POS2, HIGH);
  delay(15000);
  digitalWrite(PIN_POS1, LOW);
  digitalWrite(LED_BUILTIN, LOW);
}

// the loop function runs over and over again forever
void loop()
{




}
