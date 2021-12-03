/*
   -------------------------------------------------------------------------------------
   HX711_ADC
   Arduino library for HX711 24-Bit Analog-to-Digital Converter for Weight Scales
   Olav Kallhovd sept2017
   -------------------------------------------------------------------------------------
*/
/*
   Settling time (number of samples) and data filtering can be adjusted in the config.h file
   For calibration and storing the calibration value in eeprom, see example file "Calibration.ino"
   The update() function checks for new data and starts the next conversion. In order to acheive maximum effective
   sample rate, update() should be called at least as often as the HX711 sample rate; >10Hz@10SPS, >80Hz@80SPS.
   This example shows how call the update() function from an ISR with interrupt on the dout pin.
   Try this if you experince longer settling time due to time consuming code in the loop(),
   i.e. if you are refreshing an graphical LCD, etc.
   The pin used for dout must be external interrupt capable.
*/

/**
 * @file load_cell_reader.ino
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-12-01
 * 
 * @copyright Copyright (c) 2021
 * 
 * 
 * PINMAP arduino mega
 * https://www.arduino.cc/en/Hacking/PinMapping2560
 */
#define PWM0 0
#define PWM1 1
#define PWM2 2
#define PWM_MASK 0x03
#define PWM_SHIFT 0

//#define EXPERIMENT_CONTROL
//#define TARE_CONTROL

#include <HX711_ADC.h>

const int HX711_dout = 3; // mcu > HX711 dout pin, must be external interrupt capable!
const int HX711_sck = 5;  // mcu > HX711 sck pin

// HX711 constructor:
HX711_ADC LoadCell(HX711_dout, HX711_sck);

const int calVal_eepromAdress = 0;
unsigned long t = 0;
volatile boolean newDataReady;

void setup()
{
  pinMode(PWM0, INPUT);
  pinMode(PWM1, INPUT);
  pinMode(PWM2, INPUT);
  Serial.begin(57600);
  delay(10);
  Serial.println();
  Serial.println("Starting...");

  float calibrationValue;   // calibration value
  calibrationValue = 696.0; // uncomment this if you want to set this value in the sketch
  LoadCell.begin();
  // LoadCell.setReverseOutput();
  unsigned long stabilizingtime = 2000; // tare preciscion can be improved by adding a few seconds of stabilizing time
  boolean _tare = true;                 // set this to false if you don't want tare to be performed in the next step
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag())
  {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    while (1)
      ;
  }
  else
  {
    LoadCell.setCalFactor(calibrationValue); // set calibration value (float)
    Serial.println("Startup is complete");
  }

  attachInterrupt(digitalPinToInterrupt(HX711_dout), dataReadyISR, FALLING);
}

// interrupt routine:
void dataReadyISR()
{
  if (LoadCell.update())
  {
    newDataReady = 1;
  }
}

void loop()
{
  const int serialPrintInterval = 0; // increase value to slow down serial print activity

#ifndef(EXPERIMENT_CONTROL)
  byte inExperiment = digitalRead(PIN_EXPERIMENT);
#else
  byte inExperiment = 1;
#endif
  // get smoothed value from the dataset:
  if (newDataReady && inExperiment)
  {
    if (millis() > t + serialPrintInterval)
    {
      float i = LoadCell.getData();
      byte pwm_setted = PIND & PWM_MASK >> PWM_SHIFT;
      newDataReady = 0;
      Serial.print(pwm_setted);
      Serial.print(" ");
      Serial.print(i);
      Serial.println("");
      // Serial.print("  ");
      // Serial.println(millis() - t);
      t = millis();
    }
  }
#ifdef(TARE_CONTROL)
  // receive command from serial terminal, send 't' to initiate tare operation:
  if (Serial.available() > 0)
  {
    char inByte = Serial.read();
    if (inByte == 't')
      LoadCell.tareNoDelay();
  }

  // check if last tare operation is complete
  if (LoadCell.getTareStatus() == true)
  {
    Serial.println("Tare complete");
  }
#endif
}