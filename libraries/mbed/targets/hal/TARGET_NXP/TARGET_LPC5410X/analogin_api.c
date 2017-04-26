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
#include "mbed_assert.h"
#include "analogin_api.h"
#include "cmsis.h"
#include "pinmap.h"
#include "PeripheralNames.h"

#if DEVICE_ANALOGIN

#define ANALOGIN_MEDIAN_FILTER      1

#define ADC_RANGE    0xFFF

#define RAMBLOCK_H   60

#define CTRL_RESOL   9

#define SEQ_CTRL_START 26
#define SEQ_CTRL_ENA   31

#define SEQ_GDAT_VALID 31

static const PinMap PinMap_ADC[] = {
    {P0_29 , ADC_0, 0},
    {P0_30 , ADC_1, 0},
    {P0_31, ADC_2, 0},
    {P1_0, ADC_3, 0},
    {P1_1, ADC_4, 0},
    {P1_2, ADC_5, 0},
    {P1_3, ADC_6, 0},
    {P1_4, ADC_7, 0},
    {P1_5, ADC_8, 0},
    {P1_6, ADC_9, 0},
    {P1_7, ADC_10,0},
    {P1_8 , ADC_11,0},
    {NC,    NC,   0}
};

void analogin_init(analogin_t *obj, PinName pin)
{
    uint8_t adc_num;
	
    obj->adc = (ADCName)pinmap_peripheral(pin, PinMap_ADC);
    MBED_ASSERT(obj->adc != (ADCName)NC);
    adc_num = (uint8_t)obj->adc;
	
    // pin enable
    if(obj->adc < ADC_3)
   	{
   	  //Analog is in, GPIO function, Filter off is set
	  LPC_IOCON->PIO[0][adc_num + 29] = (1 << 8);
	  //Set the GPIO direction as input
	  LPC_GPIO->DIR[0] &= ~(1 << (adc_num + 29));
	}
	else
	{
	  LPC_IOCON->PIO[1][adc_num - 3] = (1 << 8);
	  LPC_GPIO->DIR[1] &= ~(1 << (adc_num - 3));
	}
	
    //Power on ADC 
    LPC_SYSCON->PDRUNCFGCLR = (1 << 10);
	//Enable the clock for ADC
    LPC_SYSCON->AHBCLKCTRLSET[0] = (1 << 27);
	//Clear Peripheral Reset for ADC
	LPC_SYSCON->PRESETCTRLCLR[0] = (1 << 27);

    __IO LPC_ADC_T *adc_reg = LPC_ADC;

    // determine the system clock divider for a 48Mhz ADC clock during calibration, 48MHz / 15 clock = 3.2M sample rate
    uint32_t clkdiv = (MAIN_CLOCK_FREQ / 48000000) - 1;

    // 12bit resolution
    adc_reg->CTRL = (3 << CTRL_RESOL) | (clkdiv & 0xFF);
	
    //Start ADC calibration, using ROM api 
    uint32_t  start_of_ram_block0[ RAMBLOCK_H ];
    ADC_HANDLE_T*  adc_handle;
    ADC_CONFIG_T adc_set;
	int size_in_bytes;
	size_in_bytes =  LPC_ADCD_API->adc_get_mem_size() ;
    if (RAMBLOCK_H < (size_in_bytes /4)) 
	{
      return;
    }
	adc_handle = LPC_ADCD_API->adc_setup(LPC_ADC_BASE, (uint8_t *)start_of_ram_block0);
	adc_set.system_clock = MAIN_CLOCK_FREQ;
	LPC_ADCD_API->adc_calibration(adc_handle, &adc_set);
    
}

static inline uint32_t adc_read(analogin_t *obj)
{
    uint32_t channels;
    __IO LPC_ADC_T *adc_reg = LPC_ADC;

    channels = (obj->adc & 0x1F);

    // select channel
    adc_reg->SEQ_CTRL[0] &= ~(0xFFF);
    adc_reg->SEQ_CTRL[0] |= (1UL << channels);

    // start conversion and sequence enable
    adc_reg->SEQ_CTRL[0] |= ((1UL << SEQ_CTRL_START) | (1UL << SEQ_CTRL_ENA));

    // Repeatedly get the sample data until DONE bit
    volatile uint32_t data;
    do {
        data = adc_reg->SEQ_GDAT[0];
    } while ((data & (1UL << SEQ_GDAT_VALID)) == 0);

    // Stop conversion
    adc_reg->SEQ_CTRL[0] &= ~(1UL << SEQ_CTRL_ENA);

    return ((data >> 4) & ADC_RANGE);
}

static inline void order(uint32_t *a, uint32_t *b)
{
    if (*a > *b) {
        uint32_t t = *a;
        *a = *b;
        *b = t;
    }
}

static inline uint32_t adc_read_u32(analogin_t *obj)
{
    uint32_t value;
#if ANALOGIN_MEDIAN_FILTER
    uint32_t v1 = adc_read(obj);
    uint32_t v2 = adc_read(obj);
    uint32_t v3 = adc_read(obj);
    order(&v1, &v2);
    order(&v2, &v3);
    order(&v1, &v2);
    value = v2;
#else
    value = adc_read(obj);
#endif
    return value;
}

uint16_t analogin_read_u16(analogin_t *obj)
{
    uint32_t value = adc_read_u32(obj);
    return (value << 4) | ((value >> 8) & 0x000F); // 12 bit
}

float analogin_read(analogin_t *obj)
{
    uint32_t value = adc_read_u32(obj);
    return (float)value * (1.0f / (float)ADC_RANGE);
}

#endif
