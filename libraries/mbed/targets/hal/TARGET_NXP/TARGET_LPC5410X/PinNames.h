/* mbed Microcontroller Library
 * Copyright (c) 2006-2014 ARM Limited
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
#ifndef MBED_PINNAMES_H
#define MBED_PINNAMES_H

#include "cmsis.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    PIN_INPUT,
    PIN_OUTPUT
} PinDirection;

typedef enum {
    // LPC Pin Names
    P0_0 = 0,
    P0_1, P0_2, P0_3, P0_4, P0_5, P0_6, P0_7, P0_8, P0_9, P0_10, P0_11, P0_12, P0_13, P0_14, P0_15, P0_16, P0_17, P0_18, P0_19, P0_20, P0_21, P0_22, P0_23, P0_24, P0_25, P0_26, P0_27, P0_28, P0_29, P0_30, P0_31,
    P1_0, P1_1, P1_2, P1_3, P1_4, P1_5, P1_6, P1_7, P1_8, P1_9, P1_10, P1_11, P1_12, P1_13, P1_14, P1_15, P1_16, P1_17,
    
    LED_RED = P0_29,
    LED_GREEN = P0_30,
    LED_BLUE = P0_31,
    
    // mbed original LED naming
    LED1 = LED_RED,
    LED2 = LED_GREEN,
    LED3 = LED_BLUE,
    LED4 = LED_BLUE,
    
    
    // Arduino Shield Receptacles Names
    D0 = P1_12,
    D1 = P1_13,
    D2 = P0_2,
    D3 = P0_18,
    D4 = P0_8,
    D5 = P0_7, 
    D6 = P1_16,
    D7 = P0_6,
    D8 = P0_4,
    D9 = P0_3,
    D10= P1_15,
    D11= P1_7,
    D12= P1_14,
    D13= P1_6, 
    D14= P0_24, // same port as SDA
    D15= P0_23, // same port as SCL

    A0 = P1_0,
    A1 = P1_1,
    A2 = P1_8,
    A3 = P1_2,
    A4 = P1_4, 
    A5 = P1_5, 
    SDA= P0_24, // same port as D14
    SCL= P0_23, // same port as D15
    
    // Not connected
    NC = (int)0xFFFFFFFF,
} PinName;

typedef enum {
    PullUp = 2,
    PullDown = 1,
    PullNone = 0,
    Repeater = 3,
    OpenDrain = 4,
    PullDefault = PullUp
} PinMode;

#define STDIO_UART_TX     P0_1
#define STDIO_UART_RX     P0_0


#ifdef __cplusplus
}
#endif

#endif
