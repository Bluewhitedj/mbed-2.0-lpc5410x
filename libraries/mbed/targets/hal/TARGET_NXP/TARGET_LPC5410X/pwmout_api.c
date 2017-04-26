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
#include "pwmout_api.h"
#include "cmsis.h"
#include "pinmap.h"
#include "mbed_error.h"

#if DEVICE_PWMOUT

#define MAX_PWM_NUM  8

/****      SCT0 Config Register           *******/

#define CFG_UNIFY        0
#define CFG_CLKMODE      1

#define CFG_AUTOLIMIT    17

/****     SCT0 Control Register          ****/

#define CTRL_DOWN_L      0
#define CTRL_STOP_L      1
#define CTRL_HALT_L      2
#define CTRL_CLRCTR_L    3
#define CTRL_BIDIR_L     4
#define CTRL_PRE_L       5

/****     SCT0 Event Control Register          ****/

#define EVENT_CTRL_COMBMODE  12
#define EVENT_CTRL_MATCHSEL  0

static const PinMap PinMap_PWM[] = {
    {P0_7, PWM_0, 2},
    {P0_18, PWM_0, 2},
    {P0_1, PWM_1, 5},
    {P0_8, PWM_1, 2},
    {P0_19, PWM_1, 2},
    {P0_9, PWM_2, 2},
    {P0_29, PWM_2, 2},
    {P0_0, PWM_3, 5},
    {P0_10, PWM_3, 2},
    {P0_30, PWM_3, 2},
    {P0_13, PWM_4, 2},
    {P1_1, PWM_4, 3},
    {P1_10, PWM_4, 3},
    {P0_14, PWM_5, 2},
    {P1_2, PWM_5, 3},
    {P1_15, PWM_5, 2},
    {P0_5, PWM_6, 2},
    {P1_3, PWM_6, 3},
    {P1_4, PWM_7, 3},
    {P1_14, PWM_7, 3},
    {NC   , NC   , 0}
};

static uint32_t pwm_used;

void pwmout_init(pwmout_t* obj, PinName pin)
{
    uint32_t pwm_n;
	
	// determine the PWM channel to use
    PWMName pwm = (PWMName)pinmap_peripheral(pin, PinMap_PWM);
    MBED_ASSERT(pwm != (PWMName)NC);
    pinmap_pinout(pin, PinMap_PWM);
	pwm_n = (uint32_t)pwm;
	pwm_used |= (1 << pwm_n);
		
    obj->pwm =  (LPC_SCT_T*)LPC_SCT_BASE;
    obj->pwm_ch = pwm_n;

    // Enable the SCT clock
    LPC_SYSCON->AHBCLKCTRLSET[1] = (1 << 2);

    // Clear peripheral reset the SCT:
    LPC_SYSCON->PRESETCTRLCLR[1] = (1 << 2);

    // Unified 32-bit counter, autolimit
    obj->pwm->CONFIG |= ((0x3 << CFG_AUTOLIMIT) | (0x1 << CFG_UNIFY));

    // halt and clear the counter
    obj->pwm->CTRL_U |= (1 << CTRL_HALT_L) | (1 << CTRL_CLRCTR_L);

    // Set Precaler to divide System Clock to 1MHz
    obj->pwm->CTRL_U &= ~(0xFF << CTRL_PRE_L);
    obj->pwm->CTRL_U |= (((MAIN_CLOCK_FREQ/1000000 - 1) & 0xFF) << CTRL_PRE_L);
	
	//Attach output channel n to event n+1 and evnet 0
    obj->pwm->OUT[pwm_n].SET = (1 << 0); //Event 0 set the output
    obj->pwm->OUT[pwm_n].CLR = (1 << (pwm_n+ 1));

	//Set Match only event, and associate event num n with match num n //Event0 associate with match0, controls the pwm period
    obj->pwm->EVENT[0].CTRL  = (1 << EVENT_CTRL_COMBMODE) | (0 << EVENT_CTRL_MATCHSEL);                                                           
    obj->pwm->EVENT[0].STATE = 0xFFFFFFFF;
    obj->pwm->EVENT[(pwm_n + 1)].CTRL  = (1 << EVENT_CTRL_COMBMODE) | ((pwm_n + 1) << EVENT_CTRL_MATCHSEL);	                                                            
    obj->pwm->EVENT[(pwm_n + 1)].STATE = 0xFFFFFFFF;

    // unhalt the counter:
    obj->pwm->CTRL_U &= ~(1 << CTRL_HALT_L);

    // default preriod to 20ms / 50Hz, duty cycle as 0
    pwmout_period_ms(obj, 20);
    pwmout_write    (obj, 0);
}

void pwmout_free(pwmout_t* obj)
{
    pwm_used &= ~(1 << obj->pwm_ch);
	//Set match0 as 0
	obj->pwm->MATCHREL[obj->pwm_ch + 1].U = 0;

	if(!pwm_used)
	{
	  //Halt and clear the SCT counter
	  obj->pwm->CTRL_U = (1 << CTRL_HALT_L) | (1 << CTRL_CLRCTR_L);
      // Disable the SCT clock
      LPC_SYSCON->AHBCLKCTRLSET[1] &= ~(1 << 2);
	  
	}
	
}

void pwmout_write(pwmout_t* obj, float value)
{
    if (value < 0.0f) {
        value = 0.0;
    } else if (value > 1.0f) {
        value = 1.0;
    }
    uint32_t t_on = (uint32_t)((float)(obj->pwm->MATCHREL[0].U) * value);
    obj->pwm->MATCHREL[obj->pwm_ch + 1].U = t_on;
}

float pwmout_read(pwmout_t* obj)
{
    uint32_t t_off = obj->pwm->MATCHREL[0].U;
    uint32_t t_on  = obj->pwm->MATCHREL[obj->pwm_ch + 1].U;
    float v = (float)t_on/(float)t_off;
    return (v > 1.0f) ? (1.0f) : (v);
}

void pwmout_period(pwmout_t* obj, float seconds)
{
    pwmout_period_us(obj, seconds * 1000000.0f);
}

void pwmout_period_ms(pwmout_t* obj, int ms)
{
    pwmout_period_us(obj, ms * 1000);
}

// Set the PWM period, keeping the duty cycle the same.
void pwmout_period_us(pwmout_t* obj, int us)
{
    uint8_t i;
	uint32_t t_on;
	float v;
	uint32_t t_off = obj->pwm->MATCHREL[0].U;

	//All Pwm channel have the same period, find the registered pwm channel, set the duty cycle according to the period
	for (i = 0; i < MAX_PWM_NUM; i++)
	{
      if(pwm_used & (1 << i))
      {
        t_on  = obj->pwm->MATCHREL[i + 1].U;
		//calculate the duty cycle
		v = (float)t_on/(float)t_off;
		//change the value to keep the duty cycle the same
		obj->pwm->MATCHREL[i + 1].U = (uint32_t)((float)us * (float)v);
	  }
	}
	
    obj->pwm->MATCHREL[0].U = (uint32_t)us;
}

void pwmout_pulsewidth(pwmout_t* obj, float seconds)
{
    pwmout_pulsewidth_us(obj, seconds * 1000000.0f);
}

void pwmout_pulsewidth_ms(pwmout_t* obj, int ms)
{
    pwmout_pulsewidth_us(obj, ms * 1000);
}

void pwmout_pulsewidth_us(pwmout_t* obj, int us)
{
    obj->pwm->MATCHREL[obj->pwm_ch + 1].U = (uint32_t)us;
}

#endif
