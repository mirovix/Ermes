#define HIGH_FREQUENCY
//#define DEBUG_LOG
#include <Arduino.h>
#define PRINT_PWM

#ifdef PRINT_PWM
#define PWM0 4
#define PWM1 5
#define PWM2 6
#define PWM3 7
#define PWM_MASK 0xF0
#define PWM_SHIFT 4
byte portd_supp;
#define writePWMData(pwm)  \
  portd_supp = PORTD;      \
  portd_supp &= ~PWM_MASK; \
  PORTD = portd_supp | ((pwm << PWM_SHIFT) & PWM_MASK);
#endif

#define PIN_OUTPUT 3
#define PIN_EXPERIMENT 2

#ifdef HIGH_FREQUENCY
#define DELAY 2000
//30Hz
//const byte  pwm_reals[] = {0,31,47,48+15,64+15,80+15,96+15,112+15,128+15,144+15,160+15,176+15,192+15,208+15,224+15,255};
//122Hz
const byte pwm_reals[] = {0,224,227,228,229,230,231,232,233,234,235,236,239,242,245,255};
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
#endif
  pinMode(PIN_OUTPUT, OUTPUT);
  pinMode(PIN_EXPERIMENT, OUTPUT);

  delay(INITIAL_DELAY);

#if defined(HIGH_FREQUENCY) // HIGH FREQUENCY EXPERIMENT SECTION
  // TCCR2B = (TCCR2B & 0b11111000) | 0x05; // 245.10 [Hz]

   TCCR2B = (TCCR2B & 0b11111000) | 0x06;   // 122.55 [Hz]
  //TCCR2B = (TCCR2B & 0b11111000) | 0x07; // 30.64 [Hz]
  digitalWrite(PIN_EXPERIMENT, HIGH);
  // for (size_t i = 0, pwm_real = 210; pwm_real < 256; ++i, pwm_real += 3)
  for (size_t i = 0; i < 16; ++i)
  {
    writePWMData(i);
    analogWrite(PIN_OUTPUT, pwm_reals[i]);
    delay(DELAY);
  }
  for (size_t i = 15; i >= 0; --i)
  {
    analogWrite(PIN_OUTPUT, pwm_reals[i]);
    writePWMData(i);
    delay(DELAY);
    if (i == 0)
      break;
  }
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
