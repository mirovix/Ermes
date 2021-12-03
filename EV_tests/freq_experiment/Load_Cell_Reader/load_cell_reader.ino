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

#define PRINT_PWM
#ifdef PRINT_PWM
#define PWM0 37
#define PWM1 36
#define PWM2 35
#define PWM3 34
#define PWM_MASK 0x0F
//#define PWM_SHIFT 0

#endif

#define EXPERIMENT_CONTROL
//#define TARE_CONTROL

#define PIN_EXPERIMENT 4

#include <HX711_ADC.h>

const int HX711_dout = 6; // mcu > HX711 dout pin, must be external interrupt capable!
const int HX711_sck = 7;  // mcu > HX711 sck pin

// HX711 constructor:
HX711_ADC LoadCell(HX711_dout, HX711_sck);

unsigned long t = 0;
boolean newDataReady;

void setup()
{
#ifdef PRINT_PWM
  pinMode(PWM0, INPUT);
  pinMode(PWM1, INPUT);
  pinMode(PWM2, INPUT);
  pinMode(PWM3, INPUT);
#endif
  pinMode(PIN_EXPERIMENT, INPUT);
  Serial.begin(115200);
  delay(10);
  Serial.println();
  Serial.println("Starting...");

  float calibrationValue; // calibration value
  calibrationValue = 1.0; // uncomment this if you want to set this value in the sketch
  LoadCell.begin();
  // LoadCell.setReverseOutput();
  unsigned long stabilizingtime = 2000; // tare preciscion can be improved by adding a few seconds of stabilizing time
  boolean _tare = false;                // set this to false if you don't want tare to be performed in the next step
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
}

void loop()
{
  const int serialPrintInterval = 15; // increase value to slow down serial print activity

#ifdef EXPERIMENT_CONTROL
  byte inExperiment = digitalRead(PIN_EXPERIMENT);
#else
  byte inExperiment = 1;
#endif
  if (LoadCell.update())
    newDataReady = true;
  // get smoothed value from the dataset:
  if (newDataReady && inExperiment)
  {
    if (millis() > t + serialPrintInterval)
    {
      float i = LoadCell.getData();
// Serial.print("Load_cell output val: ");
#ifdef PRINT_PWM
#ifdef PWM_SHIFT
      byte pwm = PINC & PWM_MASK >> PWM_SHIFT;
#else
      byte pwm = PINC & PWM_MASK;
#endif
      Serial.print(pwm);
      Serial.print(" ");
#endif
      Serial.println(i);
      newDataReady = 0;
      t = millis();
    }
  }
}
