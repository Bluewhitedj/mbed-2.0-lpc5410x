#include "mbed.h"

DigitalOut myled(LED1);

int main() {
    while(1) {
        myled = 1;
        wait(0.1);
        myled = 0;
        wait(0.1);
        printf("Rst Status = 0x%X\n", LPC_SYSCON->SYSRSTSTAT);
    }
}
