/**************************************************************************//**
 * @file     system_LPC5410x.h
 * @brief    CMSIS Cortex-M4 Device System Header File for
 *           NXP LPC5410x Device Series
 * @version  V1.00
 * @date     19. July 2013
 *
 * @note
 * Copyright (C) 2013 ARM Limited. All rights reserved.
 *
 * @par
 * ARM Limited (ARM) is supplying this software for use with Cortex-M
 * processor based microcontrollers.  This file can be freely distributed
 * within development tools that are supporting such ARM based processors.
 *
 * @par
 * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 * ARM SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 ******************************************************************************/


#ifndef __SYSTEM_LPC5410x_H
#define __SYSTEM_LPC5410x_H

#ifdef __cplusplus
extern "C" {
#endif

/* System initialization options */
#define CFG_PIN_SETUP             1 /* Configure pins during initialization */
#define CFG_CLOCK_SETUP           1 /* Configure PPL clocks during initialization */
#define IRC_CLOCK_FREQ            12000000 /* Define IRC clock Frequence 12MHz */
#define ASYNC_CLOCK_FREQ          12000000 /* Define ASYNC clock Frequence 12MHz */
#if (CFG_CLOCK_SETUP == 1)
  #define MAIN_CLOCK_FREQ 96000000
#else
  #define MAIN_CLOCK_FREQ IRC_CLOCK_FREQ
#endif

/** @addtogroup LPC5410x_System
 * @{
 */

#if defined(__FPU_PRESENT) && __FPU_PRESENT == 1

  extern void fpuInit(void);
#endif


extern uint32_t SystemCoreClock;     /*!< System Clock Frequency (Core Clock)  */

/**
 * Initialize the system
 *
 * @param  none
 * @return none
 *
 * @brief  Setup the microcontroller system.
 *         Initialize the System.
 */
extern void SystemInit (void);

/**
 * Update SystemCoreClock variable
 *
 * @param  none
 * @return none
 *
 * @brief  Updates the SystemCoreClock with current core Clock
 *         retrieved from cpu registers.
 */
extern void SystemCoreClockUpdate (void);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* __SYSTEM_LPC15xx_H */
