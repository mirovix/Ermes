/**
 * @file calibration.ino
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2021-11-29
 *
 * @copyright Copyright (c) 2021
 *
 * https://github.com/olkal/HX711_ADC
 */
#include <HX711_ADC.h>
const int HX711_dout = 6; // mcu > HX711 dout pin
const int HX711_sck = 7;  // mcu > HX711 sck pin

// HX711 constructor:
HX711_ADC LoadCell(HX711_dout, HX711_sck);

unsigned long t = 0;

void setup()
{
    Serial.begin(57600);
    delay(10);
    Serial.println();
    Serial.println("Starting...");

    float calibrationValue;   // calibration value
    calibrationValue = 1.0; // uncomment this if you want to set this value in the sketch
    LoadCell.begin();
    // LoadCell.setReverseOutput();
    unsigned long stabilizingtime = 2000; // tare preciscion can be improved by adding a few seconds of stabilizing time
    boolean _tare = false;                 // set this to false if you don't want tare to be performed in the next step
    LoadCell.start(stabilizingtime, _tare);
    if (LoadCell.getTareTimeoutFlag())
    {
        Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    }
    else
    {
        LoadCell.setCalFactor(calibrationValue); // set calibration factor (float)
        Serial.println("Startup is complete");
    }

    while (!LoadCell.update())
        ;
    Serial.print("Calibration value: ");
    Serial.println(LoadCell.getCalFactor());
    Serial.print("HX711 measured conversion time ms: ");
    Serial.println(LoadCell.getConversionTime());
    Serial.print("HX711 measured sampling rate HZ: ");
    Serial.println(LoadCell.getSPS());
    Serial.print("HX711 measured settlingtime ms: ");
    Serial.println(LoadCell.getSettlingTime());
    Serial.println("Note that the settling time may increase significantly if you use delay() in your sketch!");
    if (LoadCell.getSPS() < 7)
    {
        Serial.println("!!Sampling rate is lower than specification, check MCU>HX711 wiring and pin designations");
    }
    else if (LoadCell.getSPS() > 100)
    {
        Serial.println("!!Sampling rate is higher than specification, check MCU>HX711 wiring and pin designations");
    }
}

void loop()
{
    static boolean newDataReady = 0;
    const int serialPrintInterval = 20; // increase value to slow down serial print activity

    // check for new data/start next conversion:
    if (LoadCell.update())
        newDataReady = true;

    // get smoothed value from the dataset:
    if (newDataReady)
    {
        if (millis() > t + serialPrintInterval)
        {
            float i = LoadCell.getData();
            //Serial.print("Load_cell output val: ");
            Serial.println(i);
            newDataReady = 0;
            t = millis();
        }
    }

    // receive command from serial terminal, send 't' to initiate tare operation:
    if (Serial.available() > 0)
    {
        char inByte = Serial.read();
        if (inByte == 't')
            LoadCell.tareNoDelay();
    }

    // check if last tare operation is complete:
    if (LoadCell.getTareStatus() == true)
    {
        Serial.println("Tare complete");
    }
}