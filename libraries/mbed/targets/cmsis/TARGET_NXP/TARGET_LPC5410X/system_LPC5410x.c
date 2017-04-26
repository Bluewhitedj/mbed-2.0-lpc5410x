/**************************************************************************//**
 * @file     system_LPC5410x.c
 * @brief    CMSIS Cortex-M4 Device System Source File for
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


#include <stdint.h>
#include <stdbool.h>
#include "LPC5410x.h"

/*********************************************************************************//*
*
*                        Pin Mux Setup Definition
*
*********************************************************************************/

#if (CFG_PIN_SETUP == 1)


#define SYSCON_CLOCK_IOCON 13



/* Pin muxing table, only items that need changing from their default pin
   state are in this table. Not every pin is mapped. */
static const PINMUX_GRP_T pinmuxing[] = {
	/* UART0 */
	{0, 0,  (IOCON_FUNC1 | IOCON_MODE_INACT | IOCON_DIGITAL_EN)},	/* UART0 RX */
	{0, 1,  (IOCON_FUNC1 | IOCON_MODE_INACT | IOCON_DIGITAL_EN)},	/* UART0 TX */

	/* UART1 */
	{0, 5,  (IOCON_FUNC1 | IOCON_MODE_INACT | IOCON_DIGITAL_EN)},	/* UART1 RX */
	{0, 25, (IOCON_FUNC2 | IOCON_MODE_INACT | IOCON_DIGITAL_EN)},	/* UART1 CTS */
	{1, 10, (IOCON_FUNC2 | IOCON_MODE_INACT | IOCON_DIGITAL_EN)},	/* UART1 TX */
	{1, 11, (IOCON_FUNC2 | IOCON_MODE_INACT | IOCON_DIGITAL_EN)},	/* UART1 RTS */

	/* UART3 */
	{1, 12, (IOCON_FUNC2 | IOCON_MODE_INACT | IOCON_DIGITAL_EN)},	/* UART3 TX */
	{1, 13, (IOCON_FUNC2 | IOCON_MODE_INACT | IOCON_DIGITAL_EN)},	/* UART3 RX */

	/* SPI0 (bridge) */
	{0, 12, (IOCON_FUNC1 | IOCON_MODE_PULLDOWN | IOCON_DIGITAL_EN)},/* BRIDGE_MOSI (SPI MOSI) */
	{0, 13, (IOCON_FUNC1 | IOCON_MODE_PULLDOWN | IOCON_DIGITAL_EN)},/* BRIDGE_MISO (MISO) */
	/* 0, 14 BRIDGE_SSEL is configured in ConfigureBridgeSSEL() */
	{1, 3,  (IOCON_FUNC5 | IOCON_MODE_PULLDOWN | IOCON_DIGITAL_EN)},/* BRIDGE_SCK (SCK) */
	{0, 19, (IOCON_FUNC0 | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN)},	/* BRIDGE_INTR (GPIO) */
	{0, 20, (IOCON_FUNC0 | IOCON_MODE_PULLDOWN | IOCON_DIGITAL_EN)},/* BRIDGE_GPIO (GPIO) */

	/* SPI1 (master) */
	{1, 6,  (IOCON_FUNC2 | IOCON_MODE_PULLDOWN | IOCON_DIGITAL_EN)},/* SPI1_SCK */
	{1, 7,  (IOCON_FUNC2 | IOCON_MODE_PULLDOWN | IOCON_DIGITAL_EN)},/* SPI1_MOSI */
	{1, 14, (IOCON_FUNC4 | IOCON_MODE_PULLDOWN | IOCON_DIGITAL_EN)},/* SPI1_MISO */
	{1, 15, (IOCON_FUNC4 | IOCON_MODE_PULLDOWN | IOCON_DIGITAL_EN)},/* SPI1_SSEL0 */

	/* I2C0 standard/fast (master) */
	{0, 23, (IOCON_FUNC1 | IOCON_MODE_INACT | IOCON_DIGITAL_EN | IOCON_STDI2C_EN)},	/* I2C0_SCL (SCL) */
	{0, 24, (IOCON_FUNC1 | IOCON_MODE_INACT | IOCON_DIGITAL_EN | IOCON_STDI2C_EN)},	/* I2C0_SDA-WAKEUP (SDA) */

	/* I2C1 standard/fast (bridge) */
	{0, 27, (IOCON_FUNC1 | IOCON_MODE_INACT | IOCON_DIGITAL_EN | IOCON_STDI2C_EN)},	/* BRIDGE_SCL (SCL) */
	{0, 28, (IOCON_FUNC1 | IOCON_MODE_INACT | IOCON_DIGITAL_EN | IOCON_STDI2C_EN)},	/* BRIDGE_SDA (SDA) */

	/* ADC inputs */
	{1, 0,  (IOCON_FUNC0 | IOCON_MODE_INACT)},	/* ADC3 */
	{1, 1,  (IOCON_FUNC0 | IOCON_MODE_INACT)},	/* ADC4 */
	{1, 2,  (IOCON_FUNC0 | IOCON_MODE_INACT)},	/* ADC5 */
	{1, 4,  (IOCON_FUNC0 | IOCON_MODE_INACT)},	/* ADC7 */
	{1, 5,  (IOCON_FUNC0 | IOCON_MODE_INACT)},	/* ADC8 */
	{1, 8,  (IOCON_FUNC0 | IOCON_MODE_INACT)},	/* ADC11 */

	/* Misc */
	{0, 2,  (IOCON_FUNC0 | IOCON_MODE_INACT | IOCON_DIGITAL_EN)},	/* ARDUINO_INT */
	{0, 3,  (IOCON_FUNC0 | IOCON_MODE_INACT | IOCON_DIGITAL_EN)},	/* CT32B1_MAT3 */
	{0, 6,  (IOCON_FUNC0 | IOCON_MODE_INACT | IOCON_DIGITAL_EN)},	/* CT32B0_MAT1 */
	{0, 7,  (IOCON_FUNC0 | IOCON_MODE_INACT | IOCON_DIGITAL_EN)},	/* CT32B0_MAT2 */
	{0, 8,  (IOCON_FUNC0 | IOCON_MODE_INACT | IOCON_DIGITAL_EN)},	/* CT32B0_MAT3 */
	{0, 9,  (IOCON_FUNC0 | IOCON_MODE_INACT | IOCON_DIGITAL_EN)},	/* DMIC_DATA */
	{0, 10, (IOCON_FUNC0 | IOCON_MODE_INACT | IOCON_DIGITAL_EN)},	/* BTLE_CONN */
	{0, 11, (IOCON_FUNC0 | IOCON_MODE_INACT | IOCON_DIGITAL_EN)},	/* DMIC_CLKIN */
	{0, 21, (IOCON_FUNC0 | IOCON_MODE_INACT | IOCON_DIGITAL_EN)},	/* CLKOUT-CT32B3_MAT0 */
	{0, 26, (IOCON_FUNC0 | IOCON_MODE_INACT | IOCON_DIGITAL_EN)},	/* I2C1_SDA-CT32B0_CAP3 */
	{1, 9,  (IOCON_FUNC0 | IOCON_MODE_INACT | IOCON_DIGITAL_EN)},	/* BTLE_CMD_DAT */
	{1, 16, (IOCON_FUNC0 | IOCON_MODE_INACT | IOCON_DIGITAL_EN)},	/* CT32B0_MAT0 */
	{1, 17, (IOCON_FUNC0 | IOCON_MODE_INACT | IOCON_DIGITAL_EN)},	/* IR_LEARN_EN */

	/* Debugger signals */
	{0, 15, (IOCON_FUNC2 | IOCON_MODE_INACT | IOCON_DIGITAL_EN)},	/* SWO */
#if 0
	/* Not setup by SystemInit(), since default state after reset is already correct */
	{0, 16, (IOCON_FUNC5 | IOCON_MODE_INACT | IOCON_DIGITAL_EN)},	/* SWCLK_TCK */
	{0, 17, (IOCON_FUNC5 | IOCON_MODE_INACT | IOCON_DIGITAL_EN)},	/* SWDIO */
#endif

	/* Sensor related */
	{0, 4,  (IOCON_FUNC0 | IOCON_MODE_PULLDOWN | IOCON_DIGITAL_EN)},/* GYR_INT1 (GPIO input) */
	{0, 18, (IOCON_FUNC0 | IOCON_MODE_INACT | IOCON_DIGITAL_EN)},	/* CT32B0_MAT0-ACCL_INT1 */
	{0, 22, (IOCON_FUNC0 | IOCON_MODE_PULLDOWN | IOCON_DIGITAL_EN)},/* MAG_DRDY_INT (GPIO input) */

	/* LEDs on P0.29, P0.30, and P0.31 are set as part of Board_LED_Init(), left in GPIO state */
};

#endif /* #if  (CFG_PIN_SETUP == 1) */


/*********************************************************************************//*
*
*                        Clock Setup Definition
*
*********************************************************************************/


/**
 * Power control definition bits (0 = powered, 1 = powered down)
 */
#define SYSCON_PDRUNCFG_PD_IRC_OSC       (1 << 3)		/*!< IRC oscillator output */
#define SYSCON_PDRUNCFG_PD_IRC           (1 << 4)		/*!< IRC oscillator */
#define SYSCON_PDRUNCFG_PD_SYS_PLL       (1 << 22)		/*!< PLL0 */


/**
 * Clock sources for system PLLs
 */
typedef enum CHIP_SYSCON_PLLCLKSRC {
	SYSCON_PLLCLKSRC_IRC = 0,		/*!< Internal oscillator */
	SYSCON_PLLCLKSRC_CLKIN,			/*!< External clock input pin */
	SYSCON_PLLCLKSRC_WDTOSC,		/*!< WDT oscillator */
	SYSCON_PLLCLKSRC_RTC,			/*!< RTC 32KHz oscillator */
} CHIP_SYSCON_PLLCLKSRC_T;

/**
 * Clock sources for main system clock. This is a mix of both main clock A
 * and B selections.
 */
typedef enum {
	SYSCON_MAINCLKSRC_IRC = 0,				/*!< Internal oscillator */
	SYSCON_MAINCLKSRC_CLKIN,				/*!< Crystal (main) oscillator in */
	SYSCON_MAINCLKSRC_WDTOSC,				/*!< Watchdog oscillator rate */
	SYSCON_MAINCLKSRC_PLLIN = 5,			/*!< System PLL input */
	SYSCON_MAINCLKSRC_PLLOUT,				/*!< System PLL output */
	SYSCON_MAINCLKSRC_RTC					/*!< RTC oscillator 32KHz output */
} CHIP_SYSCON_MAINCLKSRC_T;

/**
 * Clock source selections for the asynchronous APB clock
 */
typedef enum {
	SYSCON_ASYNC_IRC = 0,			/*!< IRC input */
	SYSCON_ASYNC_WDTOSC,			/*!< Watchdog oscillator */
	SYSCON_ASYNC_MAINCLK = 4,		/*!< Main clock */
	SYSCON_ASYNC_CLKIN,				/*!< external CLK input */
	SYSCON_ASYNC_SYSPLLOUT			/*!< System PLL output */
} CHIP_ASYNC_SYSCON_SRC_T;


/* SYS PLL related bit fields */
#define SYS_PLL_BYPASS_PLL(d)    (d<<15)
#define SYS_PLL_BYPASS_FBDIV2(d) (d<<16)
#define SYS_PLL_LIMUPOFF(d) (d<<17)
#define SYS_PLL_BANDSEL(d)  (d<<18)
#define SYS_PLL_DIRECTI(d)  (d<<19)
#define SYS_PLL_DIRECTO(d)  (d<<20)
#define SYS_PLL_INSELR(d)   ((d&0xf)<<0)      
#define SYS_PLL_INSELI(d)   ((d&0x3f)<<4) 
#define SYS_PLL_INSELP(d)   ((d&0x1f)<<10) 


/**
 * @brief FLASH Access time definitions
 */
typedef enum {
	SYSCON_FLASH_1CYCLE = 0,	/*!< Flash accesses use 1 CPU clock */
	FLASHTIM_20MHZ_CPU = SYSCON_FLASH_1CYCLE,
	SYSCON_FLASH_2CYCLE,		/*!< Flash accesses use 2 CPU clocks */
	SYSCON_FLASH_3CYCLE,		/*!< Flash accesses use 3 CPU clocks */
	SYSCON_FLASH_4CYCLE,		/*!< Flash accesses use 4 CPU clocks */
	SYSCON_FLASH_5CYCLE,		/*!< Flash accesses use 5 CPU clocks */
	SYSCON_FLASH_6CYCLE,		/*!< Flash accesses use 6 CPU clocks */
	SYSCON_FLASH_7CYCLE,		/*!< Flash accesses use 7 CPU clocks */
	SYSCON_FLASH_8CYCLE			/*!< Flash accesses use 8 CPU clocks */
} SYSCON_FLASHTIM_T;


/*********************************************************************************//*
*
*                        FPU Definition
*
*********************************************************************************/

#if defined(__FPU_PRESENT) && __FPU_PRESENT == 1
  /* FPU declarations */
  #define LPC_CPACR	          0xE000ED88

  #define SCB_MVFR0           0xE000EF40
  #define SCB_MVFR0_RESET     0x10110021

  #define SCB_MVFR1           0xE000EF44
  #define SCB_MVFR1_RESET     0x11000011

#endif


static void Chip_Clock_SetupSystemPLL(uint32_t multiply_by, uint32_t input_freq);
static void Chip_SetupIrcClocking(uint32_t iFreq);
static void Chip_SystemInit(void);

static void Board_SetupMuxing(void);
static void Board_SystemInit(void);



void Board_SetupMuxing(void)
{

#if (CFG_PIN_SETUP == 1)

  uint32_t ix;
  uint8_t port, pin;
  uint32_t mode;
  
  /* Enable IOCON clock */
  LPC_SYSCON->AHBCLKCTRLSET[0] = (1 << SYSCON_CLOCK_IOCON);
  
  /* Set up pin muxing */
  for(ix=0; ix < (sizeof(pinmuxing)/sizeof(PINMUX_GRP_T)); ix++)
  {
    port = pinmuxing[ix].port;
	pin = pinmuxing[ix].pin;
	mode = pinmuxing[ix].modefunc;
    LPC_IOCON->PIO[port][pin] = mode;
  }

#endif
	
}


void Chip_Clock_SetupSystemPLL(uint32_t multiply_by, uint32_t input_freq)
{
	uint32_t cco_freq = input_freq * multiply_by;
	uint32_t pdec = 1;
	uint32_t selr;
	uint32_t seli;
	uint32_t selp;
	uint32_t mdec, ndec;
	
	uint32_t directo = 1;

	while (cco_freq < 75000000) {
		multiply_by <<= 1; // double value in each iteration
		pdec <<= 1;        // correspondingly double pdec to cancel effect of double msel
		cco_freq = input_freq * multiply_by;
	};
	selr = 0;
	seli = (multiply_by & 0x3c) + 4;
	selp = (multiply_by>>1) + 1;	

	if (pdec > 1) {
		directo = 0; // use post divider
		pdec = pdec/2; // account for minus 1 encoding
		//  Translate P value
		pdec = (pdec == 1)  ? 0x62 :        //1  * 2
           (pdec == 2)  ? 0x42 :        //2  * 2
           (pdec == 4)  ? 0x02 :        //4  * 2
           (pdec == 8)  ? 0x0b :        //8  * 2
           (pdec == 16) ? 0x11 :        //16 * 2
           (pdec == 32) ? 0x08 : 0x08;  //32 * 2
	}                   

	mdec = 0x7fff>>(16 - (multiply_by -1)) ; // we only support values of 2 to 16 (to keep driver simple)
	ndec = 0x202;  // pre divide by 2 (hardcoded)
	
	LPC_SYSCON->SYSPLLCTRL        =  SYS_PLL_BANDSEL(1) |SYS_PLL_DIRECTI(0) |  SYS_PLL_DIRECTO(directo) | SYS_PLL_INSELR(selr) | SYS_PLL_INSELI(seli) | SYS_PLL_INSELP(selp); // tbd
	LPC_SYSCON->SYSPLLPDEC        = pdec    | (1<<7); // set Pdec value and assert preq
	LPC_SYSCON->SYSPLLNDEC        = ndec    | (1<<10); // set Pdec value and assert preq
	LPC_SYSCON->SYSPLLSSCTRL[0] = (1<<18) | (1<<17) | mdec;  // select non sscg MDEC value, assert mreq and select mdec value  
	return;
}


void Chip_SetupIrcClocking(uint32_t iFreq)
{

  uint32_t tmp,delay;
  
#if (iFreq > 96000000)
	if (LPC_SYSCON->DEVICE_ID1 < V4_UID) {
		/* Older version of chips does not support > 96MHz (see errata) */
		iFreq = 96000000;
	}
#endif

  /* Turn on the IRC by clearing the power down bit */
  LPC_SYSCON->PDRUNCFGCLR = (SYSCON_PDRUNCFG_PD_IRC_OSC | SYSCON_PDRUNCFG_PD_IRC);

  /* Select the PLL input to the IRC */
  LPC_SYSCON->SYSPLLCLKSEL = SYSCON_PLLCLKSRC_IRC;

  /* Set main clock source to IRC, which is also the source of PLLIN */
  LPC_SYSCON->MAINCLKSELB = (SYSCON_MAINCLKSRC_PLLIN - 4);

  /* Setup FLASH access */
  tmp = LPC_SYSCON->FLASHCFG;
  tmp &= ~(0xF << 12);
  
  if(iFreq < 20000000)	
  {
    LPC_SYSCON->FLASHCFG = tmp | ((uint32_t)SYSCON_FLASH_1CYCLE << 12);
  }
  else if (iFreq < 48000000)
  {
    LPC_SYSCON->FLASHCFG = tmp | ((uint32_t)SYSCON_FLASH_2CYCLE << 12);
  }
  else if(iFreq < 72000000)
  {
    LPC_SYSCON->FLASHCFG = tmp | ((uint32_t)SYSCON_FLASH_3CYCLE << 12);
  }
  else
  {
    LPC_SYSCON->FLASHCFG = tmp | ((uint32_t)SYSCON_FLASH_4CYCLE << 12);
  }

  if(iFreq > 12000000)
 {
   /*Setup Vd voltage according to core frequency*/
    LPC_PWRD_API->set_vd_level(VD1, V1100, FINE_V_NONE);
    LPC_PWRD_API->set_vd_level(VD8, V1100, FINE_V_NONE);
 }

  /* Power down PLL to change the PLL divider ratio */
  LPC_SYSCON->PDRUNCFGSET = SYSCON_PDRUNCFG_PD_SYS_PLL;

#if (CFG_CLOCK_SETUP == 1)


#if 1
  /* set PLL settings to generate much higher frequecy first */
  Chip_Clock_SetupSystemPLL(16, IRC_CLOCK_FREQ);
  LPC_SYSCON->PDRUNCFGCLR = SYSCON_PDRUNCFG_PD_SYS_PLL;
  /* set PLL to generate requested frequency. Use the delay need to calculate the PLL setting as the min delay needed to
		   update settings. */
  Chip_Clock_SetupSystemPLL(iFreq / IRC_CLOCK_FREQ, IRC_CLOCK_FREQ);
  delay = 100;
  while (delay--) {	// wait for lock to be relevant for new settings
  }
#else
  /* First parameter is the multiplier, the second parameter is the input frequency in Hz */
  Chip_Clock_SetupSystemPLL(iFreq/IRC_CLOCK_FREQ, IRC_CLOCK_FREQ);

  /* Turn on the PLL by clearing the power down bit */
  LPC_SYSCON->PDRUNCFGCLR = SYSCON_PDRUNCFG_PD_SYS_PLL;
  
#endif
  /* Wait for PLL to lock */
  while (!((bool)((LPC_SYSCON->SYSPLLSTAT & 1) != 0))) {}

  /* Set system clock divider to 1 */
  LPC_SYSCON->AHBCLKDIV = 1;

  /* Set main clock source to the system PLL. This will drive main clock as the system clock */
  LPC_SYSCON->MAINCLKSELB = (SYSCON_MAINCLKSRC_PLLOUT - 4);

#else

  /* Set main clock source to IRC, which is also the source of PLLIN */
  LPC_SYSCON->MAINCLKSELB = (SYSCON_MAINCLKSRC_PLLIN - 4);

#endif

  /* ASYSNC SYSCON needs to be on or all serial peripheral won't work.
     Be careful if PLL is used or not, ASYNC_SYSCON source needs to be
     selected carefully. */
     
  /* Enable APB Bridge  */
  LPC_SYSCON->ASYNCAPBCTRL = 0x01;

  /* Set APB clock divider to 1  */
  LPC_ASYNC_SYSCON->ASYNCCLKDIV = 0x01;

  /* Set APB clock source A as IRC */
  LPC_ASYNC_SYSCON->ASYNCAPBCLKSELA = SYSCON_ASYNC_IRC;

  /* Set APB clock source B as A */
  LPC_ASYNC_SYSCON->ASYNCAPBCLKSELB = 3;

}

void Chip_SystemInit(void)
{
  Chip_SetupIrcClocking(MAIN_CLOCK_FREQ);
}
void Board_SystemInit(void)
{

#if (CFG_PIN_SETUP == 1)
  Board_SetupMuxing();
#endif

  
}

#if defined(__FPU_PRESENT) && __FPU_PRESENT == 1
/*
 * fpuInit() - Early initialization of the FPU
 */
void fpuInit(void)
{
   /*
    * from ARM TRM manual:
    *   ; CPACR is located at address 0xE000ED88
    *   LDR.W R0, =0xE000ED88
    *   ; Read CPACR
    *   LDR R1, [R0]
    *   ; Set bits 20-23 to enable CP10 and CP11 coprocessors
    *   ORR R1, R1, #(0xF << 20)
    *   ; Write back the modified value to the CPACR
    *   STR R1, [R0]
    */

    volatile uint32_t *regCpacr = (uint32_t *) LPC_CPACR;
    volatile uint32_t *regMvfr0 = (uint32_t *) SCB_MVFR0;
    volatile uint32_t *regMvfr1 = (uint32_t *) SCB_MVFR1;
    volatile uint32_t Cpacr;
    volatile uint32_t Mvfr0;
    volatile uint32_t Mvfr1;
    char vfpPresent = 0;

    Mvfr0 = *regMvfr0;
    Mvfr1 = *regMvfr1;

    vfpPresent = ((SCB_MVFR0_RESET == Mvfr0) && (SCB_MVFR1_RESET == Mvfr1));

    if (vfpPresent) {
        Cpacr = *regCpacr;
        Cpacr |= (0xF << 20);
        *regCpacr = Cpacr;  /* enable CP10 and CP11 for full access */
    }
}
#endif /* defined(__FPU_PRESENT) && __FPU_PRESENT == 1 */

/*----------------------------------------------------------------------------
  Clock Variable definitions
 *----------------------------------------------------------------------------*/
uint32_t SystemCoreClock = MAIN_CLOCK_FREQ;      /* System Clock Frequency     */

void SystemInit(void)
{
#if defined(CORE_M4)

    /* Initialize vector table in flash */
#if defined(__ARMCC_VERSION)
    extern void *__Vectors;

    SCB->VTOR = (unsigned int) &__Vectors;
#elif defined(__IAR_SYSTEMS_ICC__)
    extern void *__vector_table;

    SCB->VTOR = (unsigned int) &__vector_table;
#elif defined(TOOLCHAIN_GCC_ARM)
    extern void *__isr_vector;

    SCB->VTOR = (unsigned int) &__isr_vector;
#else /* defined(__GNUC__) and others */
    extern void *g_pfnVectors;

    SCB->VTOR = (unsigned int) &g_pfnVectors;
#endif

#if defined(__FPU_PRESENT) && __FPU_PRESENT == 1
    /* Initialize floating point */
    fpuInit();
#endif

#endif //#if defined (CORE_M4)

  Chip_SystemInit();
  Board_SystemInit();
  
}
