#define HIGH_FREQUENCY
//#define DEBUG_LOG
#include <Arduino.h>
#define PRINT_PWM

#ifdef PRINT_PWM
#define PWM0 4
#define PWM1 5
#define PWM2 6
#define PWM3 7
#define PWM4 8
#define PWM5 9
#define PWM6 10
#define PWM7 11
#define PWM_MASK_L 0xF0
#define PWM_MASK_H 0x0F
#define PWM_SHIFT 4
byte portd_supp;
#define writePWMData(pwm)                                 \
  portd_supp = PORTD;                                     \
  portd_supp &= ~PWM_MASK_L;                              \
  PORTD = portd_supp | ((pwm << PWM_SHIFT) & PWM_MASK_L); \
  PORTB = PORTB & PWM_MASK_L | ((pwm >> PWM_SHIFT) & PWM_MASK_H);
#endif

#define PIN_OUTPUT 3
#define PIN_EXPERIMENT 2

#ifdef HIGH_FREQUENCY
#define DELAY 500
byte pwm_real;
#else
#define PWM_DELAY_STEP_MS 32 // 8 ms delay step = 8*2^bits ms period
#define PWM_MAX 7
#define PWM_SET 1 // 0 to 7 setpoints
#define LOOP_COUNT 100
#endif
#define INITIAL_DELAY 10000

void setup()
{
#ifdef PRINT_PWM
  pinMode(PWM0, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);
#ifdef PWM3
  pinMode(PWM3, OUTPUT);
#endif
  pinMode(PWM4, OUTPUT);
  pinMode(PWM5, OUTPUT);
  pinMode(PWM6, OUTPUT);
  pinMode(PWM7, OUTPUT);
#endif
  pinMode(PIN_OUTPUT, OUTPUT);
  pinMode(PIN_EXPERIMENT, OUTPUT);

  delay(INITIAL_DELAY);

#if defined(HIGH_FREQUENCY) // HIGH FREQUENCY EXPERIMENT SECTION
  // TCCR2B = (TCCR2B & 0b11111000) | 0x05; // 245.10 [Hz]

   TCCR2B = (TCCR2B & 0b11111000) | 0x06;   // 122.55 [Hz]
  // TCCR2B = (TCCR2B & 0b11111000) | 0x07; // 30.64 [Hz]
  digitalWrite(PIN_EXPERIMENT, HIGH);
  // for (size_t i = 0, pwm_real = 210; pwm_real < 256; ++i, pwm_real += 3)
  size_t i = 0;
  for (i = 0; i < 255; i++)
  {
    writePWMData(i);
    analogWrite(PIN_OUTPUT, i);
    delay(DELAY);
  }
  writePWMData(i);
  analogWrite(PIN_OUTPUT, i);
  delay(DELAY);
  for (i = 255; i > 0; --i)
  {
    writePWMData(i);
    analogWrite(PIN_OUTPUT, i);
    delay(DELAY);
  }
  writePWMData(i);
  analogWrite(PIN_OUTPUT, i);
  delay(DELAY);
  analogWrite(PIN_OUTPUT, 0);
  digitalWrite(PIN_EXPERIMENT, LOW);

#else // LOW FREQUENCY EXEPERIMENT SECTION

  writePWMData(PWM_SET);

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
  // analogWrite(PIN_OUTPUT, PWM_VALUE); // half opened
  // delay(DELAY);
  // analogWrite(PIN_OUTPUT, 0); // half opened
  // delay(DELAY);
#endif // HIGH_FREQUENCY
}
