/* mbed Microcontroller Library
 * Copyright (c) 2006-2013 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
// math.h required for floating point operations for baud rate calculation
#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "serial_api.h"
#include "cmsis.h"
#include "pinmap.h"
#include "mbed_error.h"

/******************************************************************************
 * INITIALIZATION
 ******************************************************************************/
static const PinMap PinMap_UART_TX[] = {
    {P0_1,  UART_0, 1},
    {P0_6,  UART_1, 1},
    {P0_9, UART_2, 1},
    {P0_12, UART_1, 2},
    {P0_18, UART_3, 1},
    {P0_21, UART_0, 2},
    {P1_10 , UART_1, 2},
    {P1_13 , UART_3, 2},
    {NC   , NC    , 0}
};

static const PinMap PinMap_UART_RX[] = {
    {P0_0 , UART_0, 1},
    {P0_5 , UART_1, 1},
    {P0_8, UART_2, 1},
    {P0_11, UART_1, 2},
    {P0_20, UART_3, 1},
    {P0_22 , UART_0, 2},
    {P1_12 , UART_3, 2},
    {P1_14, UART_2, 2},
    {NC   , NC    , 0}
};

#define UART_NUM    4
#define RXRDY         (0x01<<0)
#define TXRDY         (0x01<<2)

#define TXBRKEN       (0x01<<1)
#define CTSEN         (0x01<<9)

#define SERIAL_FIFO_ENABLE 1

#if (SERIAL_FIFO_ENABLE == 1)

/* FIFO Globe Control Register bit definition */
#define FIFOCTLUSART_RXPAUSE   (1 << 0)
#define FIFOCTLUSART_RXPAUSED  (1 << 1)
#define FIFOCTLUSART_RXEMPTY   (1 << 2)
#define FIFOCTLUSART_TXPAUSE   (1 << 8)
#define FIFOCTLUSART_TXPAUSED  (1 << 9)
#define FIFOCTLUSART_TXEMPTY   (1 << 10)


#define FIFOCFGUSART_RXSIZE    0
#define FIFOCFGUSART_TXSIZE    8

#define FIFOUPDATEUSART_RXUPDATE 0
#define FIFOUPDATEUSART_TXUPDATE 16

#define CFGUSART_TIMEOUT_BASE  8
#define CFGUSART_TIMEOUT_VALUE 12
#define CFGUSART_RX_TRESH      16
#define CFGUSART_TX_TRESH      24

#define STATUSART_RXTH         (1 << 0)
#define STATUSART_TXTH         (1 << 1)
#define STATUSART_RX_TIMEOUT   (1 << 4)
#define STATUSART_RXEMPTY      (1 << 8)
#define STATUSART_TXEMPTY      (1 << 9)
#define STATUSART_RXCOUNT      16
#define STATUSART_TXCOUNT      24



#endif
static uint32_t serial_irq_ids[UART_NUM] = {0};
static uart_irq_handler irq_handler;

int stdio_uart_inited = 0;
serial_t stdio_uart;

void serial_init(serial_t *obj, PinName tx, PinName rx) {
    int is_stdio_uart = 0;
	uint8_t uart_n;
    uint32_t fifo_ctl_status;
	
    // determine the UART to use
    UARTName uart_tx = (UARTName)pinmap_peripheral(tx, PinMap_UART_TX);
    UARTName uart_rx = (UARTName)pinmap_peripheral(rx, PinMap_UART_RX);
    UARTName uart = (UARTName)pinmap_merge(uart_tx, uart_rx);
    MBED_ASSERT((int)uart != NC);
    
    obj->uart = (LPC_USART_T *)uart;
    
    switch (uart) {
        case UART_0: uart_n = 0; break;
        case UART_1: uart_n = 1; break;
        case UART_2: uart_n = 2; break;
        case UART_3: uart_n = 3; break;
    }
    obj->index = uart_n;

    /* disable uart interrupts */
    NVIC_DisableIRQ((IRQn_Type)(UART0_IRQn + uart_n));

	// Enable UART clock 
	LPC_ASYNC_SYSCON->ASYNCAPBCLKCTRLSET = 1 << (uart_n + 1);

	// Peripheral reset control to UART, then clear, bring it out of reset.
	LPC_ASYNC_SYSCON->ASYNCPRESETCTRLSET = 1 << (uart_n + 1);
	LPC_ASYNC_SYSCON->ASYNCPRESETCTRLCLR = 1 << (uart_n + 1);

	/****************************************************************/
    /*******          enable fifos and default rx trigger level             ***********/
	/****************************************************************/
#if (SERIAL_FIFO_ENABLE == 1)
	
    //Enable clock to FIFO, and clear the reset, enable uart in Sys FIFO Control
	LPC_SYSCON->AHBCLKCTRLSET[1] = (1 << 9);
	LPC_SYSCON->PRESETCTRLCLR[1] = (1 << 9);
	LPC_SYSCON->FIFOCTRL |= (1 << (8 + uart_n) | (1 << uart_n));

	//Power WWDT osc to enable VFIFO timeout timer
	LPC_SYSCON->PDRUNCFGCLR = (1 << 20);
	
	//Pause uart Rx and Tx in FIFO Control Register
	LPC_FIFO->common.FIFOCTLUSART = (FIFOCTLUSART_RXPAUSE | FIFOCTLUSART_TXPAUSE);
	//Wait for uart paused and empty
	fifo_ctl_status = (FIFOCTLUSART_RXPAUSED | FIFOCTLUSART_RXEMPTY | FIFOCTLUSART_TXEMPTY | FIFOCTLUSART_TXPAUSED);
	while(!(LPC_FIFO->common.FIFOCTLUSART & fifo_ctl_status == fifo_ctl_status));

	//Config the FIFO Tx and Rx size as 4B in FIFO Config Register
	LPC_FIFO->common.FIFOCFGUSART[uart_n] = (0x3 << FIFOCFGUSART_RXSIZE) | (0x3 << FIFOCFGUSART_TXSIZE);
    //Update FIFO config
	LPC_FIFO->common.FIFOUPDATEUSART = (0xF << FIFOUPDATEUSART_RXUPDATE) | (0xF << FIFOUPDATEUSART_TXUPDATE);
    //Unpause the uart Rx and Tx in FIFO Control Register
    LPC_FIFO->common.FIFOCTLUSART &= ~(FIFOCTLUSART_RXPAUSE | FIFOCTLUSART_TXPAUSE);

	//Config UART timeout value and threshold, set threshold as 2B, timeout = 2 * 2^10 clock
	LPC_FIFO->usart[uart_n].CFG = (0xA << CFGUSART_TIMEOUT_BASE) | (2 << CFGUSART_TIMEOUT_VALUE) \
	                               | (2 << CFGUSART_RX_TRESH) | (2 << CFGUSART_TX_TRESH);

#endif

    
    // set default baud rate and format
    serial_baud  (obj, 115200);
    serial_format(obj, 8, ParityNone, 1);
    
    // pinout the chosen uart
    pinmap_pinout(tx, PinMap_UART_TX);
    pinmap_pinout(rx, PinMap_UART_RX);
    
    /* Clear all status bits. */
    obj->uart->STAT = ((1 << 5) | (1 << 8) | (0x3F << 11));
	
#if (SERIAL_FIFO_ENABLE == 1)
    //Clear Uart FIFO status
    LPC_FIFO->usart[uart_n].STAT = (STATUSART_RXTH | STATUSART_TXTH | STATUSART_RX_TIMEOUT);
#endif
    
    /* enable uart interrupts */
    NVIC_EnableIRQ((IRQn_Type)(UART0_IRQn + uart_n));
    
    /* Enable UART */
    obj->uart->CFG |= (1 << 0);
    
    is_stdio_uart = ((tx == STDIO_UART_TX) && (rx == STDIO_UART_RX));
 
    if (is_stdio_uart) {
        stdio_uart_inited = 1;
        memcpy(&stdio_uart, obj, sizeof(serial_t));
    }
}

void serial_free(serial_t *obj) {
    serial_irq_ids[obj->index] = 0;
}

// serial_baud
// set the baud rate, taking in to account the current SystemFrequency
void serial_baud(serial_t *obj, int baudrate) {
        /* Integer divider:
         BRG = UARTSysClk/(Baudrate * 16) - 1
       
       Frational divider:
         FRG = ((UARTSysClk / (Baudrate * 16 * (BRG + 1))) - 1)
       
       where
         FRG = (LPC_SYSCON->UARTFRDADD + 1) / (LPC_SYSCON->UARTFRDSUB + 1)
       
       (1) The easiest way is set SUB value to 256, -1 encoded, thus SUB
           register is 0xFF.
       (2) In ADD register value, depending on the value of UartSysClk,
           baudrate, BRG register value, and SUB register value, be careful
           about the order of multiplier and divider and make sure any
           multiplier doesn't exceed 32-bit boundary and any divider doesn't get
           down below one(integer 0).
       (3) ADD should be always less than SUB.
    */

	//Set UART over sample rate as 16
	obj->uart->OSR = 0xF; 

    //Set BRG value according to baud rate
    obj->uart->BRG = ASYNC_CLOCK_FREQ / 16 / baudrate - 1;
    
    // To use of the fractional baud rate generator, you must write 0xFF to the DIV
    // value to yield a denominator value of 256. All other values are not supported.

	//enable the clock to the FRG
	LPC_ASYNC_SYSCON->ASYNCAPBCLKCTRLSET = (1 << 15);
	//Reset and clear reset to FRG
	LPC_ASYNC_SYSCON->ASYNCPRESETCTRLSET = (1 << 15);
	LPC_ASYNC_SYSCON->ASYNCPRESETCTRLCLR = (1 << 15);
	
    LPC_ASYNC_SYSCON->FRGCTRL = 0xFF;
    
    LPC_ASYNC_SYSCON->FRGCTRL |= ( ( ((ASYNC_CLOCK_FREQ / 16) * (0xFF + 1)) /
                                (baudrate * (obj->uart->BRG + 1))
                              ) - (0xFF + 1) ) << 8;

}

void serial_format(serial_t *obj, int data_bits, SerialParity parity, int stop_bits) {
    MBED_ASSERT((stop_bits == 1) || (stop_bits == 2)); // 0: 1 stop bits, 1: 2 stop bits
    MBED_ASSERT((data_bits > 6) && (data_bits < 10)); // 0: 7 data bits ... 2: 9 data bits
    MBED_ASSERT((parity == ParityNone) || (parity == ParityOdd) || (parity == ParityEven));

    stop_bits -= 1;
    data_bits -= 7;
    
    int paritysel;
    switch (parity) {
        case ParityNone: paritysel = 0; break;
        case ParityEven: paritysel = 2; break;
        case ParityOdd : paritysel = 3; break;
        default:
            break;
    }

    // First disable the the usart as described in documentation and then enable while updating CFG

    // 24.6.1 USART Configuration register
    // Remark: If software needs to change configuration values, the following sequence should
    // be used: 1) Make sure the USART is not currently sending or receiving data. 2) Disable
    // the USART by writing a 0 to the Enable bit (0 may be written to the entire register). 3)
    // Write the new configuration value, with the ENABLE bit set to 1.
    obj->uart->CFG &= ~(1 << 0);

    obj->uart->CFG = (data_bits << 2)
                   | (paritysel << 4)
                   | (stop_bits << 6);
	
	obj->uart->CFG |= (1 << 0); //enable uart

}

/******************************************************************************
 * INTERRUPTS HANDLING
 ******************************************************************************/
static inline void uart_irq(SerialIrq irq_type, uint32_t index) {

	if (serial_irq_ids[index] != 0)
		irq_handler(serial_irq_ids[index], irq_type);
}

#if (SERIAL_FIFO_ENABLE == 1)

void uart0_irq() {uart_irq((LPC_FIFO->usart[0].INTSTAT & (STATUSART_RXTH | STATUSART_RX_TIMEOUT)) ? RxIrq : TxIrq, 0);}
void uart1_irq() {uart_irq((LPC_FIFO->usart[1].INTSTAT & (STATUSART_RXTH | STATUSART_RX_TIMEOUT)) ? RxIrq : TxIrq, 1);}
void uart2_irq() {uart_irq((LPC_FIFO->usart[2].INTSTAT & (STATUSART_RXTH | STATUSART_RX_TIMEOUT)) ? RxIrq : TxIrq, 2);}
void uart3_irq() {uart_irq((LPC_FIFO->usart[3].INTSTAT & (STATUSART_RXTH | STATUSART_RX_TIMEOUT)) ? RxIrq : TxIrq, 3);}


#else

void uart0_irq() {uart_irq((LPC_USART0->INTSTAT & 1) ? RxIrq : TxIrq, 0);}
void uart1_irq() {uart_irq((LPC_USART1->INTSTAT & 1) ? RxIrq : TxIrq, 1);}
void uart2_irq() {uart_irq((LPC_USART2->INTSTAT & 1) ? RxIrq : TxIrq, 2);}
void uart3_irq() {uart_irq((LPC_USART3->INTSTAT & 1) ? RxIrq : TxIrq, 3);}
#endif


void serial_irq_handler(serial_t *obj, uart_irq_handler handler, uint32_t id) {
	irq_handler = handler;
	serial_irq_ids[obj->index] = id;
}

void serial_irq_set(serial_t *obj, SerialIrq irq, uint32_t enable) {
	IRQn_Type irq_n = (IRQn_Type)0;
	uint32_t vector = 0;
	switch ((int)obj->uart) {
		case LPC_USART0_BASE: irq_n=UART0_IRQn; vector = (uint32_t)&uart0_irq; break;
		case LPC_USART1_BASE: irq_n=UART1_IRQn; vector = (uint32_t)&uart1_irq; break;
		case LPC_USART2_BASE: irq_n=UART2_IRQn; vector = (uint32_t)&uart2_irq; break;
		case LPC_USART3_BASE: irq_n=UART3_IRQn; vector = (uint32_t)&uart3_irq; break;
	}
	
	if (enable) {
		NVIC_DisableIRQ(irq_n);
#if (SERIAL_FIFO_ENABLE == 1)

       LPC_FIFO->usart[obj->index].CTLSET |= ((irq == RxIrq) ? (STATUSART_RXTH | STATUSART_RX_TIMEOUT) : STATUSART_TXTH);
#else

       obj->uart->INTENSET |= (1 << ((irq == RxIrq) ? 0 : 2));
#endif
		NVIC_SetVector(irq_n, vector);
		NVIC_EnableIRQ(irq_n);
	} else { // disable
		int all_disabled = 0;
		SerialIrq other_irq = (irq == RxIrq) ? (TxIrq) : (RxIrq);
		
#if (SERIAL_FIFO_ENABLE == 1)

       LPC_FIFO->usart[obj->index].CTLCLR |= ((irq == RxIrq) ? (STATUSART_RXTH | STATUSART_RX_TIMEOUT) : STATUSART_TXTH);
       all_disabled = (LPC_FIFO->usart[obj->index].CTLSET \
	   	               &((other_irq == RxIrq) ? (STATUSART_RXTH | STATUSART_RX_TIMEOUT) : STATUSART_TXTH)) == 0;
#else
       obj->uart->INTENCLR |= (1 << ((irq == RxIrq) ? 0 : 2)); // disable the interrupt
       all_disabled = (obj->uart->INTENSET & (1 << ((other_irq == RxIrq) ? 0 : 2))) == 0;
#endif
		if (all_disabled)
			NVIC_DisableIRQ(irq_n);
	}
}

/******************************************************************************
 * READ/WRITE
 ******************************************************************************/
int serial_getc(serial_t *obj) {
    while (!serial_readable(obj));
	
#if (SERIAL_FIFO_ENABLE == 1)
    return LPC_FIFO->usart[obj->index].RXDAT;
#else
    return obj->uart->RXDAT;
#endif
}

void serial_putc(serial_t *obj, int c) {
   while (!serial_writable(obj));
#if (SERIAL_FIFO_ENABLE == 1)
	   LPC_FIFO->usart[obj->index].TXDAT = c;
#else
	   obj->uart->TXDAT = c;
#endif

}

int serial_readable(serial_t *obj) {
#if (SERIAL_FIFO_ENABLE == 1)
    return !(LPC_FIFO->usart[obj->index].STAT & STATUSART_RXEMPTY);
#else
    return obj->uart->STAT & RXRDY;
#endif
}

int serial_writable(serial_t *obj) {
#if (SERIAL_FIFO_ENABLE == 1)
		return (LPC_FIFO->usart[obj->index].STAT & STATUSART_TXEMPTY);
#else
		return obj->uart->STAT & TXRDY;
#endif

}

void serial_clear(serial_t *obj) {
    
}

void serial_pinout_tx(PinName tx) {
    pinmap_pinout(tx, PinMap_UART_TX);
}

void serial_break_set(serial_t *obj) {
    obj->uart->CTL |= TXBRKEN;
}

void serial_break_clear(serial_t *obj) {
    obj->uart->CTL &= ~TXBRKEN;
}

