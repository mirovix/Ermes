//#define HIGH_FREQUENCY
//#define DEBUG_LOG

#define PIN_OUTPUT 3

#ifdef HIGH_FREQUENCY
#define DELAY 10000
#define PWM_VALUE 128
#endif

#ifndef HIGH_FREQUENCY
#define PWM_DELAY_STEP_MS 32 // 8 ms delay step = 8*2^bits ms period
#define PWM_MAX 7
#define PWM_SET 1 // 0 to 7 setpoints
#endif
#include <Arduino.h>

void setup()
{
#if defined(HIGH_FREQUENCY)
    //TCCR2B = (TCCR2B & 0b11111000) | 0x05;   // 245.10 [Hz]
    //TCCR2B = (TCCR2B & 0b11111000) | 0x06;   // 122.55 [Hz]
    TCCR2B = (TCCR2B & 0b11111000) | 0x07; // 30.64 [Hz]
#endif
    pinMode(PIN_OUTPUT, OUTPUT);
}

void loop()
{
#if !defined(HIGH_FREQUENCY)

    if (PWM_SET > 0)
        digitalWrite(PIN_OUTPUT, HIGH);
    for (size_t i = 0; i < PWM_MAX; i++)
    {
        if (i == PWM_SET)
        {
            digitalWrite(PIN_OUTPUT, LOW);
        }
        delay(PWM_DELAY_STEP_MS);
    }
#else
    analogWrite(PIN_OUTPUT, PWM_VALUE); // half opened
    delay(DELAY);
    analogWrite(PIN_OUTPUT, 0); // half opened
    delay(DELAY);
#endif // HIGH_FREQUENCY
}