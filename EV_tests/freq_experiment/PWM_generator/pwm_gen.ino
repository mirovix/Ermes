//#define HIGH_FREQUENCY
//#define DEBUG_LOG
#define PRINT_PWM

#define PIN_OUTPUT 3
#define PIN_EXPERIMENT 2

#ifdef HIGH_FREQUENCY
#define DELAY 1000
#define PWM_VALUE 128
#else
#define PWM_DELAY_STEP_MS 6 // 8 ms delay step = 8*2^bits ms period
#define PWM_MAX 7
#define PWM_SET 5 // 0 to 7 setpoints
#define LOOP_COUNT 100
#define INITIAL_DELAY 10000
#endif
#include <Arduino.h>

void setup()
{
    pinMode(PIN_OUTPUT, OUTPUT);
    pinMode(PIN_EXPERIMENT, OUTPUT);
    delay(INITIAL_DELAY);

#if defined(HIGH_FREQUENCY)
    TCCR2B = (TCCR2B & 0b11111000) | 0x05; // 245.10 [Hz]
    // TCCR2B = (TCCR2B & 0b11111000) | 0x06;   // 122.55 [Hz]
    // TCCR2B = (TCCR2B & 0b11111000) | 0x07; // 30.64 [Hz]
#else

    digitalWrite(PIN_EXPERIMENT, HIGH);
    for (size_t cycle = 0; cycle < LOOP_COUNT; cycle++)
    {

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
        /* code */
    }
    digitalWrite(PIN_EXPERIMENT, LOW);
    digitalWrite(PIN_OUTPUT, LOW);
#endif
}

void loop()
{
#if !defined(HIGH_FREQUENCY)
    // if (PWM_SET > 0)
    //         digitalWrite(PIN_OUTPUT, HIGH);
    //     for (size_t i = 0; i < PWM_MAX; i++)
    //     {
    //         if (i == PWM_SET)
    //         {
    //             digitalWrite(PIN_OUTPUT, LOW);
    //         }
    //         delay(PWM_DELAY_STEP_MS);
    //     }

#else
    analogWrite(PIN_OUTPUT, PWM_VALUE); // half opened
    delay(DELAY);
    analogWrite(PIN_OUTPUT, 0); // half opened
    delay(DELAY);
#endif // HIGH_FREQUENCY
}