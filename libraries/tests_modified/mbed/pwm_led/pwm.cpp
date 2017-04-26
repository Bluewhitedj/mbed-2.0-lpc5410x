#include "mbed.h"

#if defined(TARGET_K64F)
#define TEST_LED D9

#elif defined(TARGET_NUCLEO_F030R8) || \
      defined(TARGET_NUCLEO_F070RB) || \
      defined(TARGET_NUCLEO_F072RB) || \
      defined(TARGET_NUCLEO_F091RC) || \
      defined(TARGET_NUCLEO_F103RB) || \
      defined(TARGET_NUCLEO_F302R8) || \
      defined(TARGET_NUCLEO_F303RE) || \
      defined(TARGET_NUCLEO_F334R8) || \
      defined(TARGET_NUCLEO_F401RE) || \
      defined(TARGET_NUCLEO_F411RE) || \
      defined(TARGET_NUCLEO_L053R8) || \
      defined(TARGET_NUCLEO_L073RZ) || \
      defined(TARGET_NUCLEO_L152RE)
#define TEST_LED D3

#elif defined (TARGET_K22F) || \
      defined (TARGET_LPC824)
#define TEST_LED LED_GREEN

#elif defined (TARGET_MAXWSNENV)
#define TEST_LED LED_GREEN

#elif defined (TARGET_DISCO_F407VG) 
#define TEST_LED LED1

#elif defined(TARGET_SAMR21G18A) || defined(TARGET_SAMD21J18A)
#define TEST_LED LED1

#elif defined (TARGET_LPC5410X)
#define TEST_LED LED_GREEN
#else
#error This test is not supported on this target.
#endif

PwmOut led(TEST_LED);
PwmOut pwmCh2(P0_9);
PwmOut pwmCh0(P0_7);
PwmOut pwmCh1(P0_8);
PwmOut pwmCh4(P1_10);
PwmOut pwmCh5(P1_15);
PwmOut pwmCh6(P0_5);
PwmOut pwmCh7(P1_14);

 float delta = 0.025;
 
float valueChange(float crt)
{

      crt = crt + delta;
    if (crt > 1) {
        crt = 1;
        delta = -delta;
    }
    else if (crt < 0) {
        crt = 0;
        delta = -delta;
    }
    return crt;
}
int main() {
    float crt3 = 0.4, crt2 = 0.3;
    int32_t cnt = 0;
    led.period_ms(20); // 50Hz
    while (true) {
        led.write(crt3);
        pwmCh2.write(crt2);
        pwmCh0.write(0.1);
        pwmCh1.write(0.2);
        pwmCh4.write(0.5);
        pwmCh5.write(0.6);
        pwmCh6.write(0.7);
        pwmCh7.write(0.8);
        wait_ms(50);
//        crt3 = valueChange(crt3);
//        crt2 = valueChange(crt2);
        cnt++;
        if(cnt == 20)
        {
          led.period_ms(10);
        }
    }
}
