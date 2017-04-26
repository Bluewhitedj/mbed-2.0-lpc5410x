#include "mbed.h"

DigitalOut led3(LED3);
DigitalOut led2(LED2);

Serial computer(STDIO_UART_TX, STDIO_UART_RX);

uint8_t buffer[100];
uint8_t cnt = 0;
uint8_t flag = 0;
// This function is called when a character goes into the TX buffer.
void txCallback() {
    led3 = !led3;
}

// This function is called when a character goes into the RX buffer.
void rxCallback() {
  
    led2 = !led2;
    flag = 1;
    while(computer.readable())
    {
//         computer.putc(computer.getc());
         buffer[cnt++] =  computer.getc();
    }
//    computer.putc(cnt);
}

int main() {
  uint8_t num;
  
    printf("start test\n");
 //   computer.format(8,SerialBase::None,2);
//    computer.attach(&txCallback, Serial::TxIrq);
    computer.attach(&rxCallback, Serial::RxIrq);
    while (true) {
//        wait(1);
      if (flag == 1)
      {
        num = cnt;
        cnt = 0;
        flag = 0;
        for(char i = 0; i<num; i++)
        {
            computer.putc(buffer[i]);
        }
        computer.putc(num);
      }
    }
}
