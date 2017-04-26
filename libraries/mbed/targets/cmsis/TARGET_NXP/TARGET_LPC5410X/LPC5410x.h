/*
  * @brief LPC5410x  MCU header
  *
  * @note
  * Copyright(C) NXP Semiconductors, 2014
  * All rights reserved.
  *
  * @par
  * Software that is described herein is for illustrative purposes only
  * which provides customers with programming information regarding the
  * LPC products.  This software is supplied "AS IS" without any warranties of
  * any kind, and NXP Semiconductors and its licensor disclaim any and
  * all warranties, express or implied, including all implied warranties of
  * merchantability, fitness for a particular purpose and non-infringement of
  * intellectual property rights.  NXP Semiconductors assumes no responsibility
  * or liability for the use of the software, conveys no license or rights under any
  * patent, copyright, mask work right, or any other intellectual property rights in
  * or to any products. NXP Semiconductors reserves the right to make changes
  * in the software without notification. NXP Semiconductors also makes no
  * representation or warranty that such application will be suitable for the
  * specified use without further testing or modification.
  *
  * @par
  * Permission to use, copy, modify, and distribute this software and its
  * documentation is hereby granted, under NXP Semiconductors' and its
  * licensor's relevant copyrights in the software, without fee, provided that it
  * is used in conjunction with NXP Semiconductors microcontrollers.  This
  * copyright, permission, and disclaimer notice must appear in all copies of
  * this code.
  */

#ifndef __LPC5410X_H
#define __LPC5410X_H

#ifdef __cplusplus
extern "C" {
#endif

/* Treat __CORE_Mx as CORE_Mx */
#if defined(__CORTEX_M0P) && !defined(CORE_M0PLUS)
  #define CORE_M0PLUS
#endif
#if defined(__CORTEX_M3) && !defined(CORE_M3)
  #define CORE_M3
#endif
/* Default to M4 core if no core explicitly declared */
#if !defined(CORE_M0PLUS) && !defined(CORE_M3)
  #define CORE_M4
#endif

#if defined(__ARMCC_VERSION)
// Kill warning "#pragma push with no matching #pragma pop"
  #pragma diag_suppress 2525
  #pragma push
  #pragma anon_unions
#elif defined(__CWCC__)
  #pragma push
  #pragma cpp_extensions on
#elif defined(__GNUC__)
/* anonymous unions are enabled by default */
#elif defined(__IAR_SYSTEMS_ICC__)
//  #pragma push // FIXME not usable for IAR
  #pragma language=extended
#else
  #error Not supported compiler type
#endif

/*
 * ==========================================================================
 * ---------- Interrupt Number Definition -----------------------------------
 * ==========================================================================
 */

#if defined(CORE_M4)

/** @defgroup CMSIS_5410X_M4_IRQ CHIP_5410X: LPC5410X M4 core peripheral interrupt numbers
 * @{
 */

typedef enum {
	/******  Cortex-M4 Processor Exceptions Numbers ***************************************************/
	Reset_IRQn                    = -15,	/*!< 1  Reset Vector, invoked on Power up and warm reset */
	NonMaskableInt_IRQn           = -14,	/*!< 2  Non maskable Interrupt, cannot be stopped or preempted */
	HardFault_IRQn                = -13,	/*!< 3  Hard Fault, all classes of Fault */
	MemoryManagement_IRQn         = -12,	/*!< 4  Memory Management, MPU mismatch, including Access Violation and No Match */
	BusFault_IRQn                 = -11,	/*!< 5  Bus Fault, Pre-Fetch-, Memory Access Fault, other address/memory related Fault */
	UsageFault_IRQn               = -10,	/*!< 6  Usage Fault, i.e. Undef Instruction, Illegal State Transition */
	SVCall_IRQn                   =  -5,	/*!< 11  System Service Call via SVC instruction */
	DebugMonitor_IRQn             =  -4,	/*!< 12  Debug Monitor                    */
	PendSV_IRQn                   =  -2,	/*!< 14  Pendable request for system service */
	SysTick_IRQn                  =  -1,	/*!< 15  System Tick Timer                */

	/******  LPC5410X Specific Interrupt Numbers ********************************************************/
	WDT_IRQn                      = 0,		/*!< WWDT                                             */
	BOD_IRQn                      = 1,		/*!< BOD                                              */
	Reserved2_IRQn                = 2,		/*!< Reserved Interrupt                               */
	DMA_IRQn                      = 3,		/*!< DMA                                              */
	GINT0_IRQn                    = 4,		/*!< GINT0                                            */
	PIN_INT0_IRQn                 = 5,		/*!< PININT0                                          */
	PIN_INT1_IRQn                 = 6,		/*!< PININT1                                          */
	PIN_INT2_IRQn                 = 7,		/*!< PININT2                                          */
	PIN_INT3_IRQn                 = 8,		/*!< PININT3                                          */
	UTICK_IRQn                    = 9,		/*!< Micro-tick Timer interrupt                       */
	MRT_IRQn                      = 10,		/*!< Multi-rate timer interrupt                       */
	CT32B0_IRQn                   = 11,		/*!< CTMR0                                            */
	CT32B1_IRQn                   = 12,		/*!< CTMR1                                            */
	CT32B2_IRQn                   = 13,		/*!< CTMR2                                            */
	CT32B3_IRQn                   = 14,		/*!< CTMR3                                            */
	CT32B4_IRQn                   = 15,		/*!< CTMR4                                            */
	SCT0_IRQn                     = 16,		/*!< SCT                                              */
	UART0_IRQn                    = 17,		/*!< UART0                                            */
	UART1_IRQn                    = 18,		/*!< UART1                                            */
	UART2_IRQn                    = 19,		/*!< UART2                                            */
	UART3_IRQn                    = 20,		/*!< UART3                                            */
	I2C0_IRQn                     = 21,		/*!< I2C0                                             */
	I2C1_IRQn                     = 22,		/*!< I2C1                                             */
	I2C2_IRQn                     = 23,		/*!< I2C2                                             */
	SPI0_IRQn                     = 24,		/*!< SPI0                                             */
	SPI1_IRQn                     = 25,		/*!< SPI1                                             */
	ADC_SEQA_IRQn                 = 26,		/*!< ADC0 sequence A completion                       */
	ADC_SEQB_IRQn                 = 27,		/*!< ADC0 sequence B completion                       */
	ADC_THCMP_IRQn                = 28,		/*!< ADC0 threshold compare and error                 */
	RTC_IRQn                      = 29,		/*!< RTC alarm and wake-up interrupts                 */
	Reserved30_IRQn               = 30,		/*!< Reserved Interrupt                               */
	MAILBOX_IRQn                  = 31,		/*!< Mailbox                                          */
	GINT1_IRQn                    = 32,		/*!< GINT1                                            */
	PIN_INT4_IRQn                 = 33,		/*!< External Interrupt 4                             */
	PIN_INT5_IRQn                 = 34,		/*!< External Interrupt 5                             */
	PIN_INT6_IRQn                 = 35,		/*!< External Interrupt 6                             */
	PIN_INT7_IRQn                 = 36,		/*!< External Interrupt 7                             */
	Reserved37_IRQn               = 37,		/*!< Reserved Interrupt                               */
	Reserved38_IRQn               = 38,		/*!< Reserved Interrupt                               */
	Reserved39_IRQn               = 39,		/*!< Reserved Interrupt                               */
	RIT_IRQn                      = 40,		/*!< Repetitive Interrupt Timer                       */
	Reserved41_IRQn               = 41,		/*!< Reserved Interrupt                               */
	Reserved42_IRQn               = 42,		/*!< Reserved Interrupt                               */
	Reserved43_IRQn               = 43,		/*!< Reserved Interrupt                               */
	Reserved44_IRQn               = 44,		/*!< Reserved Interrupt                               */
} IRQn_Type;

/**
 * @}
 */

/*
 * ==========================================================================
 * ----------- Processor and Core Peripheral Section ------------------------
 * ==========================================================================
 */

/** @defgroup CMSIS_5410X_M4_COMMON CHIP: LPC5410X M4 core Cortex CMSIS definitions
 * @{
 */

/* Configuration of the Cortex-M4 Processor and Core Peripherals */
#define __CM4_REV                 0x0001	/*!< Cortex-M4 Core Revision                          */
#define __MPU_PRESENT             1			/*!< MPU present or not                               */
#define __NVIC_PRIO_BITS          3			/*!< Number of Bits used for Priority Levels          */
#define __Vendor_SysTickConfig    0			/*!< Set to 1 if different SysTick Config is used     */
#define __FPU_PRESENT             1

/**
 * @}
 */

/**
 * @}
 */


#include "core_cm4.h"                        /* Cortex-M4 processor and core peripherals */

#elif defined(CORE_M0PLUS)

/** @defgroup CMSIS_5410X_M0_IRQ CHIP_5410X: LPC5410X M0 core peripheral interrupt numbers
 * @{
 */

typedef enum {
	/******  Cortex-M0 Processor Exceptions Numbers ***************************************************/
	Reset_IRQn                    = -15,	/*!< 1 Reset Vector, invoked on Power up and warm reset */
	NonMaskableInt_IRQn           = -14,	/*!< 2 Non Maskable Interrupt                           */
	HardFault_IRQn                = -13,	/*!< 3 Cortex-M0 Hard Fault Interrupt                   */
	SVCall_IRQn                   = -5,		/*!< 11 Cortex-M0 SV Call Interrupt                     */
	PendSV_IRQn                   = -2,		/*!< 14 Cortex-M0 Pend SV Interrupt                     */
	SysTick_IRQn                  = -1,		/*!< 15 Cortex-M0 System Tick Interrupt                 */

	/******  LPC5410X Specific Interrupt Numbers ********************************************************/
	WDT_IRQn                      = 0,		/*!< WWDT                                             */
	BOD_IRQn                      = 1,		/*!< BOD                                              */
	Reserved2_IRQn                = 2,		/*!< Reserved Interrupt                               */
	DMA_IRQn                      = 3,		/*!< DMA                                              */
	GINT0_IRQn                    = 4,		/*!< GINT0                                            */
	PIN_INT0_IRQn                 = 5,		/*!< PININT0                                          */
	PIN_INT1_IRQn                 = 6,		/*!< PININT1                                          */
	PIN_INT2_IRQn                 = 7,		/*!< PININT2                                          */
	PIN_INT3_IRQn                 = 8,		/*!< PININT3                                          */
	UTICK_IRQn                    = 9,		/*!< Micro-tick Timer interrupt                       */
	MRT_IRQn                      = 10,		/*!< Multi-rate timer interrupt                       */
	CT32B0_IRQn                   = 11,		/*!< CTMR0                                            */
	CT32B1_IRQn                   = 12,		/*!< CTMR1                                            */
	CT32B2_IRQn                   = 13,		/*!< CTMR2                                            */
	CT32B3_IRQn                   = 14,		/*!< CTMR3                                            */
	CT32B4_IRQn                   = 15,		/*!< CTMR4                                            */
	SCT0_IRQn                     = 16,		/*!< SCT                                              */
	UART0_IRQn                    = 17,		/*!< UART0                                            */
	UART1_IRQn                    = 18,		/*!< UART1                                            */
	UART2_IRQn                    = 19,		/*!< UART2                                            */
	UART3_IRQn                    = 20,		/*!< UART3                                            */
	I2C0_IRQn                     = 21,		/*!< I2C0                                             */
	I2C1_IRQn                     = 22,		/*!< I2C1                                             */
	I2C2_IRQn                     = 23,		/*!< I2C2                                             */
	SPI0_IRQn                     = 24,		/*!< SPI0                                             */
	SPI1_IRQn                     = 25,		/*!< SPI1                                             */
	ADC_SEQA_IRQn                 = 26,		/*!< ADC0 sequence A completion                       */
	ADC_SEQB_IRQn                 = 27,		/*!< ADC0 sequence B completion                       */
	ADC_THCMP_IRQn                = 28,		/*!< ADC0 threshold compare and error                 */
	RTC_IRQn                      = 29,		/*!< RTC alarm and wake-up interrupts                 */
	Reserved30_IRQn               = 30,		/*!< Reserved Interrupt                               */
	MAILBOX_IRQn                  = 31,		/*!< Mailbox                                          */
} IRQn_Type;

/**
 * @}
 */

/*
 * ==========================================================================
 * ----------- Processor and Core Peripheral Section ------------------------
 * ==========================================================================
 */

/** @defgroup CMSIS_5410X_M0_COMMON CHIP: LPC5410X M0 core Cortex CMSIS definitions
 * @{
 */

/* Configuration of the Cortex-M0+ Processor and Core Peripherals */
#define __CM0PLUS_REV             0x0001	/*!< Cortex-M0PLUS Core Revision                      */
#define __MPU_PRESENT             0			/*!< MPU present or not                               */
#define __NVIC_PRIO_BITS          2			/*!< Number of Bits used for Priority Levels          */
#define __Vendor_SysTickConfig    0			/*!< Set to 1 if different SysTick Config is used     */
#define __VTOR_PRESENT            1

/**
 * @}
 */

/**
 * @}
 */


#include "core_cm0plus.h"                        /* Cortex-M4 processor and core peripherals */
#else
#error Please #define CORE_M0PLUS or CORE_M4
#endif

#include "system_LPC5410x.h"

/** @defgroup ROMAPI_5410X CHIP: LPC5410X ROM API declarations and functions
 * @ingroup CHIP_5410X_DRIVERS
 * @romapi_5410x.h
 * @{
 */


/* Pointer to ROM IAP entry functions */
#define IAP_ENTRY_LOCATION        0x03000205

/* IAP_ENTRY API function type */
typedef void (*IAP_ENTRY_T)(unsigned int[5], unsigned int[4]);

/**
 * @brief LPC5410x IAP_ENTRY API function type
 */
static __INLINE void iap_entry(unsigned int cmd_param[5], unsigned int status_result[4])
{
	((IAP_ENTRY_T) IAP_ENTRY_LOCATION)(cmd_param, status_result);
}




/** @defgroup PWRD_412X CHIP: LPC412X Power ROM API declarations and functions
 * @ingroup CHIP_412X_Drivers
 * @{
 */


typedef enum  domain_name {
       VD1 = 0x0,
       VD2 = 0x1,
       VD3 = 0x2,
       VD8 = 0x3
} domain_t;

typedef enum  voltage_level{
       V0650 = 0x0,
       V0700 = 0x1,
       V0750 = 0x2,
       V0800 = 0x3,
       V0850 = 0x4,
       V0900 = 0x5,
       V0950 = 0x6,
       V1000 = 0x7,
       V1050 = 0x8,
       V1100 = 0x9,
       V1150 = 0xA,
       V1200 = 0xB,
       V1250 = 0xC,
       V1300 = 0xD,
       V1350 = 0xE,
       V1400 = 0xF
}  voltageLevel_t;

typedef enum  fine_voltage_level
{
       FINE_V_NONE = 0x0,
       FINE_V_M025 = 0x1,
       FINE_V_P025 = 0x3
} fineVoltageLevel_t;



typedef	struct _PWRD_API {
  void (*set_pll)(unsigned int cmd[], unsigned int resp[]);
  void (*set_power)(unsigned int cmd[], unsigned int resp[]);
  void (*power_mode_configure)(unsigned int power_mode, unsigned int peripheral_ctrl, unsigned int gint_flag);
  void (*clear_flash_data_latch)(void);
  void (*set_vd_level)(uint32_t domain, uint32_t level, uint32_t flevel);
  void (*set_lpvd_level)(uint32_t vd1LpLevel, uint32_t vd2LpLevel, uint32_t vd3LpLevel, uint32_t vd8LpLevel,
		uint32_t flevel, uint32_t vd1_lp_full_poweren, uint32_t vd2_lp_full_poweren, uint32_t vd3_lp_full_poweren , uint32_t vd8_lp_full_poweren);
  void (*set_clamp_level)(uint32_t domain, uint32_t low_level, uint32_t high_level, uint32_t enable_low_clamp, uint32_t enable_high_clamp);
  void (*enter_RBB)( void );
  void (*enter_NBB)( void );
  void (*enter_FBB)( void );
  void (*set_aclkgate)(unsigned aclkgate);
  unsigned (*get_aclkgate)(void);
}  PWRD_API_T;

/**
 * @brief LPC540XX High level ROM API structure
 */

typedef void*  ADC_HANDLE_T;
typedef void  (*ADC_SEQ_CALLBK_T) (ADC_HANDLE_T handle); //define callback func TYPE
typedef void  (*ADC_CALLBK_T) (uint32_t err_code, uint32_t n); //define callback func TYPE

typedef struct {
	uint32_t system_clock;
	uint32_t adc_clock;
	uint32_t adc_ctrl;
	uint32_t input_sel;
	uint32_t seqa_ctrl;
	uint32_t seqb_ctrl;
	uint32_t thrsel;
	uint32_t thr0_low;
	uint32_t thr0_high;
	uint32_t thr1_low;
	uint32_t thr1_high;
	uint32_t error_en;
	uint32_t thcmp_en;
	uint32_t channel_num;  // Total number channels for the sequence 
} ADC_CONFIG_T;

typedef struct{
	uint32_t       dma_adc_num;   // DMA request num., peripheral to memory for ADC conversion
	uint32_t       dma_pinmux_num;
	uint32_t       dma_handle;		// DMA handle
	ADC_CALLBK_T   dma_done_callback_pt; // DMA completion callback function
} ADC_DMA_CFG_T;

//typedef ErrorCode_t  (*ADC_DMA_SETUP_T) (ADC_HANDLE_T handle, ADC_DMA_CFG_T *dma_cfg);


typedef struct {		 // params passed to adc driver function
  uint32_t          *buffer      ;  // Considering supporting DMA and non-DMA mode, 32-bit buffer is needed for DMA
  uint32_t          driver_mode  ;  // 0x00: Polling mode, function is blocked until transfer is finished.
                                    // 0x01: Interrupt mode, function exit immediately, callback function is invoked when transfer is finished.
																		// 0x02: DMA mode, in case DMA block is available, data transferred by ADC is processed by DMA, 
																		//       and max buffer size is the total number ADC channels, DMA req function is called for ADC DMA 
																		//       channel setup, then SEQx completion also used as DMA callback function when that ADC conversion/DMA transfer 
																		//       is finished.
  uint32_t          seqa_hwtrig  ;
  uint32_t          seqb_hwtrig  ;
  ADC_CONFIG_T      *adc_cfg     ;
  uint32_t          comp_flags   ;
  uint32_t          overrun_flags;
  uint32_t          thcmp_flags  ;
  ADC_DMA_CFG_T     *dma_cfg;
  ADC_SEQ_CALLBK_T  seqa_callback_pt; 	  // SEQA callback function/the same callback on DMA completion if DMA is used for ADCx.
  ADC_SEQ_CALLBK_T  seqb_callback_pt; 	  // SEQb callback function/the same callback on DMA completion if DMA is used for ADCx.
  ADC_CALLBK_T      overrun_callback_pt; 	// Overrun callback function
  ADC_CALLBK_T      thcmp_callback_pt; 	  // THCMP callback function
//  ADC_DMA_SETUP_T   dma_setup_func_pt; 	  // ADC DMA channel setup function
} ADC_PARAM_T;


typedef struct  ADCD_API {	   // index of all the SPI driver functions
	uint32_t (*adc_get_mem_size)(void);
	ADC_HANDLE_T (*adc_setup)(uint32_t base_addr, uint8_t *ram);
	void (*adc_calibration)(ADC_HANDLE_T handle, ADC_CONFIG_T *set);
	void (*adc_init)(ADC_HANDLE_T handle, ADC_CONFIG_T *set);
	uint32_t (*adc_seqa_read)(ADC_HANDLE_T handle, ADC_PARAM_T * param);
	uint32_t (*adc_seqb_read)(ADC_HANDLE_T handle, ADC_PARAM_T * param);
	//--interrupt functions--//
	void (*adc_seqa_isr)(ADC_HANDLE_T handle);
	void (*adc_seqb_isr)(ADC_HANDLE_T handle);
	void (*adc_ovr_isr)(ADC_HANDLE_T handle);
	void (*adc_thcmp_isr)(ADC_HANDLE_T handle);
	uint32_t  (*adc_get_firmware_version)( void );
} ADCD_API_T  ;	// end of structure ************************************


typedef struct {
	const uint32_t reserved0;				/*!< Reserved */
	const uint32_t reserved1;				/*!< Reserved */
	const uint32_t reserved2;				/*!< Reserved */
	const PWRD_API_T *pPWRD;				/*!< Power API function table base address */
	const uint32_t reserved3;				/*!< Reserved */
	const uint32_t reserved4;				/*!<  */
	const uint32_t reserved5;				/*!<  */
	const uint32_t reserved6;				/*!< */
	const ADCD_API_T *pADCD;				/*!< ADC driver API function table base address */
	const uint32_t reserved7;				/*!<  */
	const uint32_t reserved8;				/*!< Reserved */
	const uint32_t reserved9;				/*!< Reserved */
} LPC_ROM_API_T;


/* Pointer to ROM API function address */
#define LPC_ROM_API_BASE_LOC    0x03000200UL
#define LPC_ROM_API     (*(LPC_ROM_API_T * *) LPC_ROM_API_BASE_LOC)

/* Pointer to @ref PWRD_API_T functions in ROM */
#define LPC_PWRD_API	((LPC_ROM_API)->pPWRD)

/* Pointer to @ref ADCD_API_T functions in ROM for ADC */
#define LPC_ADCD_API	((LPC_ROM_API)->pADCD)

/**
 * @}
 */

/**
 * @}
 */

/************************************** System Configuration******************************/

/** @defgroup SYSCON_5410X CHIP: LPC5410X System and Control Driver
 * @ingroup CHIP_5410X_DRIVERS
 * @{
 * @syscon_5410x.h
 */

/**
 * @brief LPC5410X Main system configuration register block structure
 */
typedef struct {
	__IO uint32_t SYSMEMREMAP;			/*!< System Remap register */
	__I  uint32_t RESERVED0[4];
	__IO uint32_t SYSTCKCAL;			/*!< System Tick Calibration register */
	__I  uint32_t RESERVED1[1];
	__IO uint32_t NMISRC;				/*!< NMI Source select register */
	__IO uint32_t ASYNCAPBCTRL;			/*!< Asynch APB chiplet control register */
	__I  uint32_t RESERVED2[7];
	__IO uint32_t SYSRSTSTAT;			/*!< System Reset Stat register */
	__IO uint32_t PRESETCTRL[2];		/*!< Peripheral Reset Ctrl register */
	__IO uint32_t PRESETCTRLSET[2];		/*!< Peripheral Reset Ctrl Set register */
	__IO uint32_t PRESETCTRLCLR[2];		/*!< Peripheral Reset Ctrl Clr register */
	__IO uint32_t PIOPORCAP[2];			/*!< PIO Power-On Reset Capture register */
	__I  uint32_t RESERVED3[1];
	__IO uint32_t PIORESCAP[2];			/*!< PIO Pad Reset Capture register */
	__I  uint32_t RESERVED4[4];
	__IO uint32_t MAINCLKSELA;			/*!< Main Clk sel Source Sel A register */
	__IO uint32_t MAINCLKSELB;			/*!< Main Clk sel Source Sel B register */
	__I  uint32_t RESERVED5;
	__IO uint32_t ADCCLKSEL;			/*!< ADC Async Clk Sel register */
	__I  uint32_t RESERVED6;
	__IO uint32_t CLKOUTSELA;			/*!< Clk Out Sel Source A register */
	__IO uint32_t CLKOUTSELB;			/*!< Clk Out Sel Source B register */
	__I  uint32_t RESERVED7;
	__IO uint32_t SYSPLLCLKSEL;			/*!< System PLL Clk Selregister */
	__I  uint32_t RESERVED8[7];
	__IO uint32_t AHBCLKCTRL[2];		/*!< AHB Peripheral Clk Enable register */
	__IO uint32_t AHBCLKCTRLSET[2];		/*!< AHB Peripheral Clk Enable Set register */
	__IO uint32_t AHBCLKCTRLCLR[2];		/*!< AHB Peripheral Clk Enable Clr register */
	__I  uint32_t RESERVED9[2];
	__IO uint32_t SYSTICKCLKDIV;		/*!< Systick Clock divider register */
	__I  uint32_t RESERVED10[7];
	__IO uint32_t AHBCLKDIV;			/*!< Main Clk Divider register */
	__IO uint32_t RESERVED11;
	__IO uint32_t ADCCLKDIV;			/*!< ADC Async Clk Divider register */
	__IO uint32_t CLKOUTDIV;			/*!< Clk Out Divider register */
	__I  uint32_t RESERVED12[4];
	__IO uint32_t FREQMECTRL;			/*!< Frequency Measure Control register */
	__IO uint32_t FLASHCFG;				/*!< Flash Config register */
	__I  uint32_t RESERVED13[8];
	__IO uint32_t FIFOCTRL;				/*!< VFIFO control register */
	__I  uint32_t RESERVED14[14];
	__I  uint32_t RESERVED15[1];
	__I  uint32_t RESERVED16[2];
	__IO uint32_t RTCOSCCTRL;			/*!< RTC Oscillator Control register */
	__I  uint32_t RESERVED17[7];
	__IO uint32_t SYSPLLCTRL;			/*!< System PLL control register */
	__IO uint32_t SYSPLLSTAT;			/*!< PLL status register */
	__IO uint32_t SYSPLLNDEC;			/*!< PLL N decoder register */
	__IO uint32_t SYSPLLPDEC;			/*!< PLL P decoder register */
	__IO uint32_t SYSPLLSSCTRL[2];	/*!< Spread Spectrum control registers */
	__I  uint32_t RESERVED18[18];
	__IO uint32_t PDRUNCFG;				/*!< Power Down Run Config register */
	__IO uint32_t PDRUNCFGSET;			/*!< Power Down Run Config Set register */
	__IO uint32_t PDRUNCFGCLR;			/*!< Power Down Run Config Clr register */
	__I  uint32_t RESERVED19[9];
	__IO uint32_t STARTERP[2];			/*!< Start Signal Enable Register */
	__IO uint32_t STARTERSET[2];		/*!< Start Signal Enable Set Register */
	__IO uint32_t STARTERCLR[2];		/*!< Start Signal Enable Clr Register */
	__I  uint32_t RESERVED20[42];
	__I  uint32_t RESERVED20A[4];
	__I  uint32_t RESERVED21[57];
	__IO uint32_t JTAG_IDCODE;
	__IO uint32_t DEVICE_ID0;			/*!< Boot ROM and die revision register */
	__IO uint32_t DEVICE_ID1;			/*!< Boot ROM and die revision register */
} LPC_SYSCON_T;

/**
 * @brief LPC5410X Asynchronous system configuration register block structure
 */
typedef struct {
	__IO uint32_t AYSNCPRESETCTRL;		/*!< peripheral reset register */
	__IO uint32_t ASYNCPRESETCTRLSET;	/*!< peripheral reset Set register */
	__IO uint32_t ASYNCPRESETCTRLCLR;	/*!< peripheral reset Clr register */
	__I  uint32_t RESERVED0;
	__IO uint32_t ASYNCAPBCLKCTRL;		/*!< clk enable register */
	__IO uint32_t ASYNCAPBCLKCTRLSET;	/*!< clk enable Set register */
	__IO uint32_t ASYNCAPBCLKCTRLCLR;	/*!< clk enable Clr register */
	__I  uint32_t RESERVED1;
	__IO uint32_t ASYNCAPBCLKSELA;		/*!< clk source mux A register */
	__IO uint32_t ASYNCAPBCLKSELB;		/*!< clk source mux B register */
	__IO uint32_t ASYNCCLKDIV;			/*!< clk div register */
	__I  uint32_t RESERVED2;
	__IO uint32_t FRGCTRL;				/*!< Fraction Rate Generator Ctrl register */
} LPC_ASYNC_SYSCON_T;

/**
 * System memory remap modes used to remap interrupt vectors
 */
typedef enum CHIP_SYSCON_BOOT_MODE_REMAP {
	REMAP_BOOT_LOADER_MODE,	/*!< Interrupt vectors are re-mapped to Boot ROM */
	REMAP_USER_RAM_MODE,	/*!< Interrupt vectors are re-mapped to user Static RAM */
	REMAP_USER_FLASH_MODE	/*!< Interrupt vectors are not re-mapped and reside in Flash */
} CHIP_SYSCON_BOOT_MODE_REMAP_T;

/** @brief V4 CHIP PART ID */
#define V4_UID              (0x08C1FECE)
/**
 * @}
 */


/** @defgroup PMU_5410X CHIP: LPC5410X Power Management declarations and functions
 * @ingroup CHIP_5410X_DRIVERS
 * @{
 * @pmu_5410x.h
 */

/**
 * @brief PMU register block structure
 * @note Most of the PMU support is handled by the PMU library.
 */
typedef struct {
	__I  uint32_t RESERVED0[4];
	__I  uint32_t RESERVED1[4];
	__I  uint32_t RESERVED2[4];
	__I  uint32_t RESERVED3[4];
	__I  uint32_t RESERVED4;
	__IO uint32_t BODCTRL;
	__I  uint32_t RESERVED5;
	__I  uint32_t RESERVED6;
	__IO uint32_t DPDWAKESRC;
} LPC_PMU_T;

/**
 * Brown-out detector reset level
 */
typedef enum {
	PMU_BODRSTLVL_0,	/*!< Brown-out reset at ~1.5v */
	PMU_BODRSTLVL_1_50V = PMU_BODRSTLVL_0,
	PMU_BODRSTLVL_1,	/*!< Brown-out reset at ~1.85v */
	PMU_BODRSTLVL_1_85V = PMU_BODRSTLVL_1,
	PMU_BODRSTLVL_2,	/*!< Brown-out reset at ~2.0v */
	PMU_BODRSTLVL_2_00V = PMU_BODRSTLVL_2,
	PMU_BODRSTLVL_3,	/*!< Brown-out reset at ~2.3v */
	PMU_BODRSTLVL_2_30V = PMU_BODRSTLVL_3
} CHIP_PMU_BODRSTLVL_T;

/**
 * Brown-out detector interrupt level
 */
typedef enum CHIP_PMU_BODRINTVAL {
	PMU_BODINTVAL_LVL0,	/*!< Brown-out interrupt at ~2.05v */
	PMU_BODINTVAL_2_05v = PMU_BODINTVAL_LVL0,
	PMU_BODINTVAL_LVL1,	/*!< Brown-out interrupt at ~2.45v */
	PMU_BODINTVAL_2_45v = PMU_BODINTVAL_LVL1,
	PMU_BODINTVAL_LVL2,	/*!< Brown-out interrupt at ~2.75v */
	PMU_BODINTVAL_2_75v = PMU_BODINTVAL_LVL2,
	PMU_BODINTVAL_LVL3,	/*!< Brown-out interrupt at ~3.05v */
	PMU_BODINTVAL_3_05v = PMU_BODINTVAL_LVL3
} CHIP_PMU_BODRINTVAL_T;

/**
 * brown-out detection reset status (in BODCTRL register)
 */
#define PMU_BOD_RST     (1 << 6)
/**
 * brown-out detection interrupt status (in BODCTRL register)
 */
#define PMU_BOD_INT     (1 << 7)

/**
 * @}
 */
 
/** @defgroup IOCON_5410X CHIP: LPC5410X IOCON register block and driver
 * @ingroup CHIP_5410X_DRIVERS
  * @iocon_5410x.h
 * @{
 */

/**
 * @brief LPC5410X IO Configuration Unit register block structure
 */
typedef struct {			/*!< LPC5410X IOCON Structure */
	__IO uint32_t  PIO[2][32];
} LPC_IOCON_T;

/**
 * @brief Array of IOCON pin definitions passed to Chip_IOCON_SetPinMuxing() must be in this format
 */
typedef struct {
	uint32_t port : 8;			/* Pin port */
	uint32_t pin : 8;			/* Pin number */
	uint32_t modefunc : 16;		/* Function and mode */
} PINMUX_GRP_T;

/**
 * IOCON function and mode selection definitions
 * See the User Manual for specific modes and functions supported by the
 * various LPC15XX pins.
 */
#define IOCON_FUNC0             0x0				/*!< Selects pin function 0 */
#define IOCON_FUNC1             0x1				/*!< Selects pin function 1 */
#define IOCON_FUNC2             0x2				/*!< Selects pin function 2 */
#define IOCON_FUNC3             0x3				/*!< Selects pin function 3 */
#define IOCON_FUNC4             0x4				/*!< Selects pin function 4 */
#define IOCON_FUNC5             0x5				/*!< Selects pin function 5 */
#define IOCON_FUNC6             0x6				/*!< Selects pin function 6 */
#define IOCON_FUNC7             0x7				/*!< Selects pin function 7 */
#define IOCON_MODE_INACT        (0x0 << 3)		/*!< No addition pin function */
#define IOCON_MODE_PULLDOWN     (0x1 << 3)		/*!< Selects pull-down function */
#define IOCON_MODE_PULLUP       (0x2 << 3)		/*!< Selects pull-up function */
#define IOCON_MODE_REPEATER     (0x3 << 3)		/*!< Selects pin repeater function */
#define IOCON_HYS_EN            (0x1 << 5)		/*!< Enables hysteresis */
#define IOCON_GPIO_MODE         (0x1 << 5)		/*!< GPIO Mode */
#define IOCON_I2C_SLEW          (0x1 << 5)		/*!< I2C Slew Rate Control */
#define IOCON_INV_EN            (0x1 << 6)		/*!< Enables invert function on input */
#define IOCON_ANALOG_EN         (0x0 << 7)		/*!< Enables analog function by setting 0 to bit 7 */
#define IOCON_DIGITAL_EN        (0x1 << 7)		/*!< Enables digital function by setting 1 to bit 7(default) */
#define IOCON_STDI2C_EN         (0x1 << 8)		/*!< I2C standard mode/fast-mode */
#define IOCON_FASTI2C_EN        (0x3 << 8)		/*!< I2C Fast-mode Plus and high-speed slave */
#define IOCON_INPFILT_OFF       (0x1 << 8)		/*!< Input filter Off for GPIO pins */
#define IOCON_INPFILT_ON        (0x0 << 8)		/*!< Input filter On for GPIO pins */
#define IOCON_OPENDRAIN_EN      (0x1 << 10)		/*!< Enables open-drain function */
#define IOCON_S_MODE_0CLK       (0x0 << 11)		/*!< Bypass input filter */
#define IOCON_S_MODE_1CLK       (0x1 << 11)		/*!< Input pulses shorter than 1 filter clock are rejected */
#define IOCON_S_MODE_2CLK       (0x2 << 11)		/*!< Input pulses shorter than 2 filter clock2 are rejected */
#define IOCON_S_MODE_3CLK       (0x3 << 11)		/*!< Input pulses shorter than 3 filter clock2 are rejected */
#define IOCON_S_MODE(clks)      ((clks) << 11)	/*!< Select clocks for digital input filter mode */
#define IOCON_CLKDIV(div)       ((div) << 13)	/*!< Select peripheral clock divider for input filter sampling clock, 2^n, n=0-6 */

/**
 * @}
 */

/** @defgroup PININT_5410X CHIP: LPC5410X Pin Interrupt and Pattern Match driver
 * @ingroup CHIP_5410X_DRIVERS
 * @{
 */

/**
 * @brief LPC5410X Pin Interrupt and Pattern Match register block structure
 */
typedef struct {			/*!< PIN_INT Structure */
	__IO uint32_t ISEL;		/*!< Pin Interrupt Mode register */
	__IO uint32_t IENR;		/*!< Pin Interrupt Enable (Rising) register */
	__IO uint32_t SIENR;	/*!< Set Pin Interrupt Enable (Rising) register */
	__IO uint32_t CIENR;	/*!< Clear Pin Interrupt Enable (Rising) register */
	__IO uint32_t IENF;		/*!< Pin Interrupt Enable Falling Edge / Active Level register */
	__IO uint32_t SIENF;	/*!< Set Pin Interrupt Enable Falling Edge / Active Level register */
	__IO uint32_t CIENF;	/*!< Clear Pin Interrupt Enable Falling Edge / Active Level address */
	__IO uint32_t RISE;		/*!< Pin Interrupt Rising Edge register */
	__IO uint32_t FALL;		/*!< Pin Interrupt Falling Edge register */
	__IO uint32_t IST;		/*!< Pin Interrupt Status register */
	__IO uint32_t PMCTRL;	/*!< GPIO pattern match interrupt control register          */
	__IO uint32_t PMSRC;	/*!< GPIO pattern match interrupt bit-slice source register */
	__IO uint32_t PMCFG;	/*!< GPIO pattern match interrupt bit slice configuration register */
} LPC_PIN_INT_T;

/**
 * LPC5410X Pin Interrupt and Pattern match engine register
 * bit fields and macros
 */

/* PININT Interrupt Mode Mask */
#define PININT_ISEL_PMODE_MASK   ((uint32_t) 0x00FF)

/* PININT Pattern Match Control Register Mask */
#define PININT_PMCTRL_MASK       ((uint32_t) 0xFF000003)

/* PININT interrupt control register */
#define PININT_PMCTRL_PMATCH_SEL (1 << 0)
#define PININT_PMCTRL_RXEV_ENA   (1 << 1)

/* PININT Bit slice source register bits */
#define PININT_SRC_BITSOURCE_START  8
#define PININT_SRC_BITSOURCE_MASK   7

/* PININT Bit slice configuration register bits */
#define PININT_SRC_BITCFG_START  8
#define PININT_SRC_BITCFG_MASK   7

/**
 * LPC5410X Pin Interrupt channel values
 */
#define PININTCH0         (1 << 0)
#define PININTCH1         (1 << 1)
#define PININTCH2         (1 << 2)
#define PININTCH3         (1 << 3)
#define PININTCH4         (1 << 4)
#define PININTCH5         (1 << 5)
#define PININTCH6         (1 << 6)
#define PININTCH7         (1 << 7)
#define PININTCH(ch)      (1 << (ch))

/**
 * LPC5410X Pin Interrupt select enum values
 */
typedef enum Chip_PININT_SELECT {
	PININTSELECT0 = 0,
	PININTSELECT1 = 1,
	PININTSELECT2 = 2,
	PININTSELECT3 = 3,
	PININTSELECT4 = 4,
	PININTSELECT5 = 5,
	PININTSELECT6 = 6,
	PININTSELECT7 = 7
} Chip_PININT_SELECT_T;

/**
 * LPC5410X Pin Matching Interrupt bit slice enum values
 */
typedef enum Chip_PININT_BITSLICE {
	PININTBITSLICE0 = 0,	/*!< PININT Bit slice 0 */
	PININTBITSLICE1 = 1,	/*!< PININT Bit slice 1 */
	PININTBITSLICE2 = 2,	/*!< PININT Bit slice 2 */
	PININTBITSLICE3 = 3,	/*!< PININT Bit slice 3 */
	PININTBITSLICE4 = 4,	/*!< PININT Bit slice 4 */
	PININTBITSLICE5 = 5,	/*!< PININT Bit slice 5 */
	PININTBITSLICE6 = 6,	/*!< PININT Bit slice 6 */
	PININTBITSLICE7 = 7		/*!< PININT Bit slice 7 */
} Chip_PININT_BITSLICE_T;

/**
 * LPC5410X Pin Matching Interrupt bit slice configuration enum values
 */
typedef enum Chip_PININT_BITSLICE_CFG {
	PININT_PATTERNCONST1           = 0x0,	/*!< Contributes to product term match */
	PININT_PATTERNRISING           = 0x1,	/*!< Rising edge */
	PININT_PATTERNFALLING          = 0x2,	/*!< Falling edge */
	PININT_PATTERNRISINGORFALLING  = 0x3,	/*!< Rising or Falling edge */
	PININT_PATTERNHIGH             = 0x4,	/*!< High level */
	PININT_PATTERNLOW              = 0x5,	/*!< Low level */
	PININT_PATTERNCONST0           = 0x6,	/*!< Never contributes for match */
	PININT_PATTERNEVENT            = 0x7	/*!< Match occurs on event */
} Chip_PININT_BITSLICE_CFG_T;

/**
 * @}
 */

/** @defgroup INMUX_5410X CHIP: LPC5410X Input Mux Registers and Driver
 * @ingroup CHIP_5410X_DRIVERS
 * @ inmux_5410x.h
 * @{
 */

/**
 * @brief LPC5410X Input Mux Register Block Structure
 */
typedef struct {						/*!< INMUX Structure */
	__IO uint32_t RESERVED0[6];
	__I  uint32_t RESERVED1[42];
	__IO uint32_t PINTSEL[8];			/*!< Pin interrupt select registers */
	__IO uint32_t DMA_ITRIG_INMUX[22];	/*!< Input mux register for DMA trigger inputs */
	__I  uint32_t RESERVED2[2];
	__IO uint32_t DMA_OTRIG_INMUX[4];	/*!< Input mux register for DMA trigger inputs */
	__I  uint32_t RESERVED3[4];
	__IO uint32_t FREQMEAS_REF;			/*!< Clock selection for frequency measurement ref clock */
	__IO uint32_t FREQMEAS_TARGET;		/*!< Clock selection for frequency measurement target clock */
} LPC_INMUX_T;

/**
 * @}
 */
 

/** @defgroup CRC_5410X CHIP: LPC5410X Cyclic Redundancy Check Engine driver
 * @ingroup CHIP_5410X_DRIVERS
  * @crc_5410x.h
 * @{
 */

/**
 * @brief CRC register block structure
 */
typedef struct {					/*!< CRC Structure */
	__IO    uint32_t    MODE;		/*!< CRC Mode Register */
	__IO    uint32_t    SEED;		/*!< CRC SEED Register */
	union {
		__I     uint32_t    SUM;	/*!< CRC Checksum Register. */
		__O     uint32_t    WRDATA32;	/*!< CRC Data Register: write size 32-bit*/
		__O     uint16_t    WRDATA16;	/*!< CRC Data Register: write size 16-bit*/
		__O     uint8_t     WRDATA8;	/*!< CRC Data Register: write size 8-bit*/
	};

} LPC_CRC_T;

/*
 * @brief CRC MODE register description
 */
#define CRC_MODE_POLY_BITMASK   ((0x03))	/** CRC polynomial Bit mask */
#define CRC_MODE_POLY_CCITT     (0x00)		/** Select CRC-CCITT polynomial */
#define CRC_MODE_POLY_CRC16     (0x01)		/** Select CRC-16 polynomial */
#define CRC_MODE_POLY_CRC32     (0x02)		/** Select CRC-32 polynomial */
#define CRC_MODE_WRDATA_BITMASK (0x03 << 2)	/** CRC WR_Data Config Bit mask */
#define CRC_MODE_WRDATA_BIT_RVS (1 << 2)	/** Select Bit order reverse for WR_DATA (per byte) */
#define CRC_MODE_WRDATA_CMPL    (1 << 3)	/** Select One's complement for WR_DATA */
#define CRC_MODE_SUM_BITMASK    (0x03 << 4)	/** CRC Sum Config Bit mask */
#define CRC_MODE_SUM_BIT_RVS    (1 << 4)	/** Select Bit order reverse for CRC_SUM */
#define CRC_MODE_SUM_CMPL       (1 << 5)	/** Select One's complement for CRC_SUM */

#define MODE_CFG_CCITT          (0x00)	/** Pre-defined mode word for default CCITT setup */
#define MODE_CFG_CRC16          (0x15)	/** Pre-defined mode word for default CRC16 setup */
#define MODE_CFG_CRC32          (0x36)	/** Pre-defined mode word for default CRC32 setup */

#define CRC_SEED_CCITT          (0x0000FFFF)/** Initial seed value for CCITT mode */
#define CRC_SEED_CRC16          (0x00000000)/** Initial seed value for CRC16 mode */
#define CRC_SEED_CRC32          (0xFFFFFFFF)/** Initial seed value for CRC32 mode */

/**
 * @brief CRC polynomial
 */
typedef enum IP_CRC_001_POLY {
	CRC_POLY_CCITT = CRC_MODE_POLY_CCITT,	/**< CRC-CCIT polynomial */
	CRC_POLY_CRC16 = CRC_MODE_POLY_CRC16,	/**< CRC-16 polynomial */
	CRC_POLY_CRC32 = CRC_MODE_POLY_CRC32,	/**< CRC-32 polynomial */
	CRC_POLY_LAST,
} CRC_POLY_T;

/**
 * @}
 */

/** @defgroup GPIO_5410X CHIP: LPC5410X GPIO driver
 * @ingroup CHIP_5410X_DRIVERS
 * @ gpio_5410x.h
 * @{
 */

/**
 * @brief  GPIO port register block structure
 */
typedef struct {				/*!< GPIO_PORT Structure */
	__IO uint8_t B[128][32];	/*!< Offset 0x0000: Byte pin registers ports 0 to n; pins PIOn_0 to PIOn_31 */
	__IO uint32_t W[32][32];	/*!< Offset 0x1000: Word pin registers port 0 to n */
	__IO uint32_t DIR[32];		/*!< Offset 0x2000: Direction registers port n */
	__IO uint32_t MASK[32];		/*!< Offset 0x2080: Mask register port n */
	__IO uint32_t PIN[32];		/*!< Offset 0x2100: Portpin register port n */
	__IO uint32_t MPIN[32];		/*!< Offset 0x2180: Masked port register port n */
	__IO uint32_t SET[32];		/*!< Offset 0x2200: Write: Set register for port n Read: output bits for port n */
	__O  uint32_t CLR[32];		/*!< Offset 0x2280: Clear port n */
	__O  uint32_t NOT[32];		/*!< Offset 0x2300: Toggle port n */
} LPC_GPIO_T;

/**
 * @}
 */


/** @defgroup FIFO_5410X CHIP: LPC5410X System FIFO chip driver
 * @ingroup CHIP_5410X_DRIVERS
  * @ fifo_5410x.h
 * This driver provides basic functionality for the system FIFO
 * and can be used to increase the amount of FIFO space available
 * to the UART and SPI peripherals. If using the system FIFO with the
 * UART or SPI drivers, the standard UART and SPI transfer handlers
 * cannot be used and buffer/stream management and status checking
 * must occur in the user application.
 * @{
 */

/** Maximum USART peripherals */
#define LPC_FIFO_USART_MAX      (4)

/** Maximum SPI peripherals */
#define LPC_FIFO_SPI_MAX        (2)

/**
 * @brief LPC5410X System FIFO USART register block structure
 */
typedef struct {
	__IO uint32_t CFG;			/*!< USART configuration Register */
	__IO uint32_t STAT;			/*!< USART status Register */
	__IO uint32_t INTSTAT;		/*!< USART interrupt status Register */
	__IO uint32_t CTLSET;		/*!< USART control read and set Register */
	__IO uint32_t CTLCLR;		/*!< USART control clear Register */
	__IO uint32_t RXDAT;		/*!< USART received data Register */
	__IO uint32_t RXDATSTAT;	/*!< USART received data with status Register */
	__IO uint32_t TXDAT;		/*!< USART transmit data Register */
	__I uint32_t  RESERVED[0x38];
} LPC_FIFO_USART_T;

/**
 * @brief LPC5410X System FIFO SPI register block structure
 */
typedef struct {
	__IO uint32_t CFG;			/*!< SPI configuration Register */
	__IO uint32_t STAT;			/*!< SPI status Register */
	__IO uint32_t INTSTAT;		/*!< SPI interrupt status Register */
	__IO uint32_t CTLSET;		/*!< SPI control read and set Register */
	__IO uint32_t CTLCLR;		/*!< SPI control clear Register */
	__I  uint32_t RXDAT;		/*!< SPI received data Register */
	union {
		__O uint32_t TXDATSPI;	/*!< SPI transmit data and control Register */
		struct {
			__O uint16_t TXDATSPI_DATA;	/*!< SPI transmit data Register */
			__O uint16_t TXDATSPI_CTRL;	/*!< SPI transmit control Register */
		};

	};

	__I  uint32_t RESERVED[0x39];
} LPC_FIFO_SPI_T;

/**
 * @brief LPC5410X System FIFO common register block structure
 */
typedef struct {
	__I  uint32_t reserved0[0x40];
	__IO uint32_t FIFOCTLUSART;			/*!< USART FIFO global control Register */
	__O  uint32_t FIFOUPDATEUSART;		/*!< USART FIFO global update Register */
	__I  uint32_t reserved1[0x2];
	__IO uint32_t FIFOCFGUSART[LPC_FIFO_USART_MAX];	/*!< USART FIFO configuration Registers */
	__I  uint32_t reserved2[0x38];
	__IO uint32_t FIFOCTLSPI;			/*!< SPI FIFO global control Register */
	__O  uint32_t FIFOUPDATESPI;		/*!< SPI FIFO global update Register */
	__I  uint32_t reserved3[0x2];
	__IO uint32_t FIFOCFGSPI[LPC_FIFO_SPI_MAX];		/*!< SPI FIFO configuration Registers */
	__I  uint32_t reserved4[0x3A];
	__I  uint32_t reserved5[((0x1000 - 0x300) / sizeof(uint32_t))];
} LPC_FIFO_CMN_T;

/**
 * @brief LPC5410X Complete system FIFO register block structure
 */
typedef struct {
	LPC_FIFO_CMN_T      common;
	LPC_FIFO_USART_T    usart[LPC_FIFO_USART_MAX];
	__I uint32_t        reserved0[((0x2000 - 0x1400) / sizeof(uint32_t))];
	LPC_FIFO_SPI_T      spi[LPC_FIFO_SPI_MAX];
} LPC_FIFO_T;

/**
 * @}
 */

/** @defgroup MRT_5410X CHIP: LPC5410X Multi-Rate Timer driver
 * @ingroup CHIP_5410X_DRIVERS
 * @ mrt_5410x.h
 * @{
 */

/**
 * @brief LPC5410X MRT chip configuration
 */
#define MRT_CHANNELS_NUM      (4)
#define MRT_NO_IDLE_CHANNEL   (0x40)

/**
 * @brief MRT register block structure
 */
typedef struct {
	__IO uint32_t INTVAL;	/*!< Timer interval register */
	__O  uint32_t TIMER;	/*!< Timer register */
	__IO uint32_t CTRL;		/*!< Timer control register */
	__IO uint32_t STAT;		/*!< Timer status register */
} LPC_MRT_CH_T;

/**
 * @brief MRT register block structure
 */
typedef struct {
	LPC_MRT_CH_T CHANNEL[MRT_CHANNELS_NUM];
	uint32_t unused[44];
	__IO uint32_t MODCFG;
	__O  uint32_t IDLE_CH;
	__IO uint32_t IRQ_FLAG;
} LPC_MRT_T;

/**
 * @brief MRT Interrupt Modes enum
 */
typedef enum MRT_MODE {
	MRT_MODE_REPEAT =  (0 << 1),	/*!< MRT Repeat interrupt mode */
	MRT_MODE_ONESHOT = (1 << 1)		/*!< MRT One-shot interrupt mode */
} MRT_MODE_T;

/**
 * @brief MRT register bit fields & masks
 */
/* MRT Time interval register bit fields */
#define MRT_INTVAL_IVALUE        (0x7FFFFFFF)	/* Maximum interval load value and mask */
#define MRT_INTVAL_LOAD          (0x80000000UL)	/* Force immediate load of timer interval register bit */

/* MRT Control register bit fields & masks */
#define MRT_CTRL_INTEN_MASK      (0x01)
#define MRT_CTRL_MODE_MASK       (0x06)

/* MRT Status register bit fields & masks */
#define MRT_STAT_INTFLAG         (0x01)
#define MRT_STAT_RUNNING         (0x02)

/* Pointer to individual MR register blocks */
#define LPC_MRT_CH0         ((LPC_MRT_CH_T *) &LPC_MRT->CHANNEL[0])
#define LPC_MRT_CH1         ((LPC_MRT_CH_T *) &LPC_MRT->CHANNEL[1])
#define LPC_MRT_CH2         ((LPC_MRT_CH_T *) &LPC_MRT->CHANNEL[2])
#define LPC_MRT_CH3         ((LPC_MRT_CH_T *) &LPC_MRT->CHANNEL[3])
#define LPC_MRT_CH(ch)      ((LPC_MRT_CH_T *) &LPC_MRT->CHANNEL[(ch)])

/* Global interrupt flag register interrupt mask/clear values */
#define MRT0_INTFLAG        (1)
#define MRT1_INTFLAG        (2)
#define MRT2_INTFLAG        (4)
#define MRT3_INTFLAG        (8)
#define MRTn_INTFLAG(ch)    (1 << (ch))

/**
 * @}
 */

/** @defgroup LPC_WWDT CHIP: LPC5410X Windowed Watchdog driver
 * @ingroup CHIP_5410X_DRIVERS
  * @ wwdt_5410x.h
 * @{
 */

/**
 * @brief Windowed Watchdog register block structure
 */
typedef struct {				/*!< WWDT Structure         */
	__IO uint32_t  MOD;			/*!< Watchdog mode register. This register contains the basic mode and status of the Watchdog Timer. */
	__IO uint32_t  TC;			/*!< Watchdog timer constant register. This register determines the time-out value. */
	__O  uint32_t  FEED;		/*!< Watchdog feed sequence register. Writing 0xAA followed by 0x55 to this register reloads the Watchdog timer with the value contained in WDTC. */
	__I  uint32_t  TV;			/*!< Watchdog timer value register. This register reads out the current value of the Watchdog timer. */
	__I  uint32_t  RESERVED0;
	__IO uint32_t  WARNINT;		/*!< Watchdog warning interrupt register. This register contains the Watchdog warning interrupt compare value. */
	__IO uint32_t  WINDOW;		/*!< Watchdog timer window register. This register contains the Watchdog window value. */
} LPC_WWDT_T;

/**
 * @brief Watchdog Mode register definitions
 */
/** Watchdog Mode Bitmask */
#define WWDT_WDMOD_BITMASK          ((uint32_t) 0x3F)
/** WWDT enable bit */
#define WWDT_WDMOD_WDEN             ((uint32_t) (1 << 0))
/** WWDT reset enable bit */
#define WWDT_WDMOD_WDRESET          ((uint32_t) (1 << 1))
/** WWDT time-out flag bit */
#define WWDT_WDMOD_WDTOF            ((uint32_t) (1 << 2))
/** WWDT warning interrupt flag bit */
#define WWDT_WDMOD_WDINT            ((uint32_t) (1 << 3))
/** WWDT Protect flag bit */
#define WWDT_WDMOD_WDPROTECT        ((uint32_t) (1 << 4))
/** WWDT lock bit */
#define WWDT_WDMOD_LOCK             ((uint32_t) (1 << 5))


/**
 * @}
 */

/** @defgroup SCT_5410X CHIP: LPC5410X State Configurable Timer driver
 * @ingroup CHIP_5410X_DRIVERS
  * @ sct_5410x.h
 * @{
 */

/*                         match/cap registers,     events,           states,         inputs,         outputs
 *
 * @brief SCT Module configuration
 */
#define CONFIG_SCT_nEV   (13)			/*!< Number of events */
#define CONFIG_SCT_nRG   (13)			/*!< Number of match/compare registers */
#define CONFIG_SCT_nOU   (8)			/*!< Number of outputs */
#define CONFIG_SCT_nIN   (8)			/*!< Number of outputs */

/**
 * @brief State Configurable Timer register block structure
 */
typedef struct {
	__IO  uint32_t CONFIG;				/*!< configuration Register (offset (0x000) */
	union {
		__IO uint32_t CTRL_U;			/*!< control Register */
		struct {
			__IO uint16_t CTRL_L;		/*!< low control register */
			__IO uint16_t CTRL_H;		/*!< high control register */
		};

	};

	union {
		__IO uint32_t LIMIT_U;			/*!< limit Register */
		struct {
			__IO uint16_t LIMIT_L;		/*!< limit register for counter L */
			__IO uint16_t LIMIT_H;		/*!< limit register for counter H */
		};

	};

	union {
		__IO uint32_t HALT_U;			/*!< halt Register */
		struct {
			__IO uint16_t HALT_L;		/*!< halt register for counter L */
			__IO uint16_t HALT_H;		/*!< halt register for counter H */
		};

	};

	union {
		__IO uint32_t STOP_U;			/*!< stop Register */
		struct {
			__IO uint16_t STOP_L;		/*!< stop register for counter L */
			__IO uint16_t STOP_H;		/*!< stop register for counter H */
		};

	};

	union {
		__IO uint32_t START_U;			/*!< start Register */
		struct {
			__IO uint16_t START_L;		/*!< start register for counter L */
			__IO uint16_t START_H;		/*!< start register for counter H */
		};

	};

	uint32_t RESERVED1[10];				/*!< 0x018 - 0x03C reserved */

	union {
		__IO uint32_t COUNT_U;			/*!< counter register (offset 0x040)*/
		struct {
			__IO uint16_t COUNT_L;		/*!< counter register for counter L */
			__IO uint16_t COUNT_H;		/*!< counter register for counter H */
		};

	};

	union {
		__IO uint32_t STATE_U;			/*!< State register */
		struct {
			__IO uint16_t STATE_L;		/*!< state register for counter L */
			__IO uint16_t STATE_H;		/*!< state register for counter H */
		};

	};

	__I  uint32_t INPUT;				/*!< input register */
	union {
		__IO uint32_t REGMODE_U;		/*!< RegMode register */
		struct {
			__IO uint16_t REGMODE_L;	/*!< match - capture registers mode register L */
			__IO uint16_t REGMODE_H;	/*!< match - capture registers mode register H */
		};

	};

	__IO uint32_t OUTPUT;				/*!< output register */
	__IO uint32_t OUTPUTDIRCTRL;		/*!< output counter direction Control Register */
	__IO uint32_t RES;					/*!< conflict resolution register */
	__IO uint32_t DMAREQ0;				/*!< DMA0 Request Register */
	__IO uint32_t DMAREQ1;				/*!< DMA1 Request Register */
	uint32_t RESERVED2[35];
	__IO uint32_t EVEN;		/*!< event enable register */
	__IO uint32_t EVFLAG;		/*!< event flag register */
	__IO uint32_t CONEN;	/*!< conflict enable register */
	__IO uint32_t CONFLAG;		/*!< conflict flag register */

	union {

		__IO union {	/*!< ... Match / Capture value */
			uint32_t U;		/*!<       SCTMATCH[i].U  Unified 32-bit register */

			struct {
				uint16_t L;		/*!<       SCTMATCH[i].L  Access to L value */
				uint16_t H;		/*!<       SCTMATCH[i].H  Access to H value */
			};

		} MATCH[CONFIG_SCT_nRG];

		__I union {
			uint32_t U;		/*!<       SCTCAP[i].U  Unified 32-bit register */

			struct {
				uint16_t L;		/*!<       SCTCAP[i].L  Access to L value */
				uint16_t H;		/*!<       SCTCAP[i].H  Access to H value */
			};

		} CAP[CONFIG_SCT_nRG];

	};

	uint32_t RESERVED3[48 + (16 - CONFIG_SCT_nRG)];

	union {

		__IO union {	/* 0x200-... Match Reload / Capture Control value */
			uint32_t U;		/*       SCTMATCHREL[i].U  Unified 32-bit register */

			struct {
				uint16_t L;		/*       SCTMATCHREL[i].L  Access to L value */
				uint16_t H;		/*       SCTMATCHREL[i].H  Access to H value */
			};

		} MATCHREL[CONFIG_SCT_nRG];

		__IO union {
			uint32_t U;		/*       SCTCAPCTRL[i].U  Unified 32-bit register */

			struct {
				uint16_t L;		/*       SCTCAPCTRL[i].L  Access to H value */
				uint16_t H;		/*       SCTCAPCTRL[i].H  Access to H value */
			};

		} CAPCTRL[CONFIG_SCT_nRG];

	};

	uint32_t RESERVED6[48 + (16 - CONFIG_SCT_nRG)];

	__IO struct {		/* 0x300-0x3FC  SCTEVENT[i].STATE / SCTEVENT[i].CTRL*/
		uint32_t STATE;		/* Event State Register */
		uint32_t CTRL;		/* Event Control Register */
	} EVENT[CONFIG_SCT_nEV];

	uint32_t RESERVED9[128 - 2 * CONFIG_SCT_nEV];		/*!< ...-0x4FC reserved */

	__IO struct {		/*!< 0x500-0x57C  SCTOUT[i].SET / SCTOUT[i].CLR */
		uint32_t SET;		/*!< Output n Set Register */
		uint32_t CLR;		/*!< Output n Clear Register */
	} OUT[CONFIG_SCT_nOU];

	uint32_t RESERVED10[191 - 2 * CONFIG_SCT_nOU];		/*!< ...-0x7F8 reserved */
	__I uint32_t MODULECONTENT;		/*!< 0x7FC Module Content */
} LPC_SCT_T;

/**
 * @brief Macro defines for SCT configuration register
 */
#define SCT_CONFIG_16BIT_COUNTER        0x00000000	/*!< Operate as 2 16-bit counters */
#define SCT_CONFIG_32BIT_COUNTER        0x00000001	/*!< Operate as 1 32-bit counter */

#define SCT_CONFIG_CLKMODE_BUSCLK       (0x0 << 1)	/*!< Bus clock */
#define SCT_CONFIG_CLKMODE_SCTCLK       (0x1 << 1)	/*!< SCT clock */
#define SCT_CONFIG_CLKMODE_INCLK        (0x2 << 1)	/*!< Input clock selected in CLKSEL field */
#define SCT_CONFIG_CLKMODE_INEDGECLK    (0x3 << 1)	/*!< Input clock edge selected in CLKSEL field */

#define SCT_CONFIG_CLKMODE_SYSCLK               (0x0 << 1)	/*!< System clock */
#define SCT_CONFIG_CLKMODE_PRESCALED_SYSCLK     (0x1 << 1)	/*!< Prescaled system clock */
#define SCT_CONFIG_CLKMODE_SCT_INPUT            (0x2 << 1)	/*!< Input clock/edge selected in CKSEL field */
#define SCT_CONFIG_CLKMODE_PRESCALED_SCT_INPUT  (0x3 << 1)	/*!< Prescaled input clock/edge selected in CKSEL field */

#define SCT_CONFIG_CKSEL_RISING_IN_0    (0x0UL << 3)
#define SCT_CONFIG_CKSEL_FALLING_IN_0   (0x1UL << 3)
#define SCT_CONFIG_CKSEL_RISING_IN_1    (0x2UL << 3)
#define SCT_CONFIG_CKSEL_FALLING_IN_1   (0x3UL << 3)
#define SCT_CONFIG_CKSEL_RISING_IN_2    (0x4UL << 3)
#define SCT_CONFIG_CKSEL_FALLING_IN_2   (0x5UL << 3)
#define SCT_CONFIG_CKSEL_RISING_IN_3    (0x6UL << 3)
#define SCT_CONFIG_CKSEL_FALLING_IN_3   (0x7UL << 3)
#define SCT_CONFIG_CKSEL_RISING_IN_4    (0x8UL << 3)
#define SCT_CONFIG_CKSEL_FALLING_IN_4   (0x9UL << 3)
#define SCT_CONFIG_CKSEL_RISING_IN_5    (0xAUL << 3)
#define SCT_CONFIG_CKSEL_FALLING_IN_5   (0xBUL << 3)
#define SCT_CONFIG_CKSEL_RISING_IN_6    (0xCUL << 3)
#define SCT_CONFIG_CKSEL_FALLING_IN_6   (0xDUL << 3)
#define SCT_CONFIG_CKSEL_RISING_IN_7    (0xEUL << 3)
#define SCT_CONFIG_CKSEL_FALLING_IN_7   (0xFUL << 3)
#define SCT_CONFIG_NORELOADL_U          (0x1 << 7)	/*!< Operate as 1 32-bit counter */
#define SCT_CONFIG_NORELOADH            (0x1 << 8)	/*!< Operate as 1 32-bit counter */
#define SCT_CONFIG_AUTOLIMIT_U          (0x1UL << 17)
#define SCT_CONFIG_AUTOLIMIT_L          (0x1UL << 17)
#define SCT_CONFIG_AUTOLIMIT_H          (0x1UL << 18)

/**
 * @brief Macro defines for SCT control register
 */
#define COUNTUP_TO_LIMIT_THEN_CLEAR_TO_ZERO     0			/*!< Direction for low or unified counter */
#define COUNTUP_TO LIMIT_THEN_COUNTDOWN_TO_ZERO 1

#define SCT_CTRL_STOP_L                 (1 << 1)				/*!< Stop low counter */
#define SCT_CTRL_HALT_L                 (1 << 2)				/*!< Halt low counter */
#define SCT_CTRL_CLRCTR_L               (1 << 3)				/*!< Clear low or unified counter */
#define SCT_CTRL_BIDIR_L(x)             (((x) & 0x01) << 4)		/*!< Bidirectional bit */
#define SCT_CTRL_PRE_L(x)               (((x) & 0xFF) << 5)		/*!< Prescale clock for low or unified counter */

#define COUNTUP_TO_LIMIT_THEN_CLEAR_TO_ZERO     0			/*!< Direction for high counter */
#define COUNTUP_TO LIMIT_THEN_COUNTDOWN_TO_ZERO 1
#define SCT_CTRL_STOP_H                 (1 << 17)				/*!< Stop high counter */
#define SCT_CTRL_HALT_H                 (1 << 18)				/*!< Halt high counter */
#define SCT_CTRL_CLRCTR_H               (1 << 19)				/*!< Clear high counter */
#define SCT_CTRL_BIDIR_H(x)             (((x) & 0x01) << 20)
#define SCT_CTRL_PRE_H(x)               (((x) & 0xFF) << 21)	/*!< Prescale clock for high counter */

#define SCT_EV_CTRL_MATCHSEL(reg)               (reg << 0)
#define SCT_EV_CTRL_HEVENT_L                    (0UL << 4)
#define SCT_EV_CTRL_HEVENT_H                    (1UL << 4)
#define SCT_EV_CTRL_OUTSEL_INPUT                (0UL << 5)
#define SCT_EV_CTRL_OUTSEL_OUTPUT               (0UL << 5)
#define SCT_EV_CTRL_IOSEL(signal)               (signal << 6)

#define SCT_EV_CTRL_IOCOND_LOW                  (0UL << 10)
#define SCT_EV_CTRL_IOCOND_RISE                 (0x1UL << 10)
#define SCT_EV_CTRL_IOCOND_FALL                 (0x2UL << 10)
#define SCT_EV_CTRL_IOCOND_HIGH                 (0x3UL << 10)
#define SCT_EV_CTRL_COMBMODE_OR                 (0x0UL << 12)
#define SCT_EV_CTRL_COMBMODE_MATCH              (0x1UL << 12)
#define SCT_EV_CTRL_COMBMODE_IO                 (0x2UL << 12)
#define SCT_EV_CTRL_COMBMODE_AND                (0x3UL << 12)
#define SCT_EV_CTRL_STATELD                     (0x1UL << 14)
#define SCT_EV_CTRL_STATEV(x)                   (x << 15)
#define SCT_EV_CTRL_MATCHMEM                    (0x1UL << 20)
#define SCT_EV_CTRL_DIRECTION_INDEPENDENT       (0x0UL << 21)
#define SCT_EV_CTRL_DIRECTION_UP                (0x1UL << 21)
#define SCT_EV_CTRL_DIRECTION_DOWN              (0x2UL << 21)

/**
 * @brief Macro defines for SCT Conflict resolution register
 */
#define SCT_RES_NOCHANGE                (0)
#define SCT_RES_SET_OUTPUT              (1)
#define SCT_RES_CLEAR_OUTPUT            (2)
#define SCT_RES_TOGGLE_OUTPUT           (3)

/**
 * SCT Match register values enum
 */
typedef enum CHIP_SCT_MATCH_REG {
	SCT_MATCH_0 = 0,	/*!< SCT Match register 0 */
	SCT_MATCH_1,
	SCT_MATCH_2,
	SCT_MATCH_3,
	SCT_MATCH_4,
	SCT_MATCH_5,
	SCT_MATCH_6,
	SCT_MATCH_7,
	SCT_MATCH_8,
	SCT_MATCH_9,
	SCT_MATCH_10,
	SCT_MATCH_11,
	SCT_MATCH_12,
	SCT_MATCH_13,
	SCT_MATCH_14,
	SCT_MATCH_15
} CHIP_SCT_MATCH_REG_T;

/**
 * SCT Event values enum
 */
typedef enum CHIP_SCT_EVENT {
	SCT_EVT_0 = (1 << 0),		/*!< Event 0 */
	SCT_EVT_1 = (1 << 1),		/*!< Event 1 */
	SCT_EVT_2 = (1 << 2),		/*!< Event 2 */
	SCT_EVT_3 = (1 << 3),		/*!< Event 3 */
	SCT_EVT_4 = (1 << 4),		/*!< Event 4 */
	SCT_EVT_5 = (1 << 5),		/*!< Event 5 */
	SCT_EVT_6 = (1 << 6),		/*!< Event 6 */
	SCT_EVT_7 = (1 << 7),		/*!< Event 7 */
	SCT_EVT_8 = (1 << 8),		/*!< Event 8 */
	SCT_EVT_9 = (1 << 9),		/*!< Event 9 */
	SCT_EVT_10 = (1 << 10),		/*!< Event 10 */
	SCT_EVT_11 = (1 << 11),		/*!< Event 11 */
	SCT_EVT_12 = (1 << 12),		/*!< Event 12 */
	SCT_EVT_13 = (1 << 13),		/*!< Event 13 */
	SCT_EVT_14 = (1 << 14),		/*!< Event 14 */
	SCT_EVT_15 = (1 << 15)		/*!< Event 15 */
} CHIP_SCT_EVENT_T;

/**
 * @}
 */


/** @defgroup RTC_5410X CHIP: LPC5410X Real Time clock
 * @ingroup CHIP_5410X_DRIVERS
 * @ rtc_5410x.h
 * @{
 */

/**
 * @brief LPC5410X Real Time clock register block structure
 */
typedef struct {			/*!< RTC */
	__IO uint32_t CTRL;		/*!< RTC control register */
	__IO uint32_t MATCH;	/*!< PRTC match (alarm) register */
	__IO uint32_t COUNT;	/*!< RTC counter register */
	__IO uint32_t WAKE;		/*!< RTC high-resolution/wake-up timer control register */
} LPC_RTC_T;

/* CTRL register defniitions */
#define RTC_CTRL_SWRESET        (1 << 0)	/*!< Apply reset to RTC */
#define RTC_CTRL_OFD            (1 << 1)	/*!< Oscillator fail detect status (failed bit) */
#define RTC_CTRL_ALARM1HZ       (1 << 2)	/*!< RTC 1 Hz timer alarm flag status (match) bit */
#define RTC_CTRL_WAKE1KHZ       (1 << 3)	/*!< RTC 1 kHz timer wake-up flag status (timeout) bit */
#define RTC_CTRL_ALARMDPD_EN    (1 << 4)	/*!< RTC 1 Hz timer alarm for Deep power-down enable bit */
#define RTC_CTRL_WAKEDPD_EN     (1 << 5)	/*!< RTC 1 kHz timer wake-up for Deep power-down enable bit */
#define RTC_CTRL_RTC1KHZ_EN     (1 << 6)	/*!< RTC 1 kHz clock enable bit */
#define RTC_CTRL_RTC_EN         (1 << 7)	/*!< RTC enable bit */
#define RTC_CTRL_MASK           ((uint32_t) 0xF1)	/*!< RTC Control register Mask for reserved and status bits */


/**
 * @}
 */

/** @defgroup TIMER_5410X CHIP: LPC5410X 32-bit Timer driver
 * @ingroup CHIP_5410X_DRIVERS
 * @ timer_5410x.h
 * @{
 */

/**
 * @brief 32-bit Standard timer register block structure
 */
typedef struct {					/*!< TIMERn Structure       */
	__IO uint32_t IR;				/*!< Interrupt Register. The IR can be written to clear interrupts. The IR can be read to identify which of eight possible interrupt sources are pending. */
	__IO uint32_t TCR;				/*!< Timer Control Register. The TCR is used to control the Timer Counter functions. The Timer Counter can be disabled or reset through the TCR. */
	__IO uint32_t TC;				/*!< Timer Counter. The 32 bit TC is incremented every PR+1 cycles of PCLK. The TC is controlled through the TCR. */
	__IO uint32_t PR;				/*!< Prescale Register. The Prescale Counter (below) is equal to this value, the next clock increments the TC and clears the PC. */
	__IO uint32_t PC;				/*!< Prescale Counter. The 32 bit PC is a counter which is incremented to the value stored in PR. When the value in PR is reached, the TC is incremented and the PC is cleared. The PC is observable and controllable through the bus interface. */
	__IO uint32_t MCR;				/*!< Match Control Register. The MCR is used to control if an interrupt is generated and if the TC is reset when a Match occurs. */
	__IO uint32_t MR[4];			/*!< Match Register. MR can be enabled through the MCR to reset the TC, stop both the TC and PC, and/or generate an interrupt every time MR matches the TC. */
	__IO uint32_t CCR;				/*!< Capture Control Register. The CCR controls which edges of the capture inputs are used to load the Capture Registers and whether or not an interrupt is generated when a capture takes place. */
	__IO uint32_t CR[4];			/*!< Capture Register. CR is loaded with the value of TC when there is an event on the CAPn.0 input. */
	__IO uint32_t EMR;				/*!< External Match Register. The EMR controls the external match pins MATn.0-3 (MAT0.0-3 and MAT1.0-3 respectively). */
	__I  uint32_t RESERVED0[12];
	__IO uint32_t CTCR;				/*!< Count Control Register. The CTCR selects between Timer and Counter mode, and in Counter mode selects the signal and edge(s) for counting. */
	__IO uint32_t PWMC;
} LPC_TIMER_T;

/** Macro to clear interrupt pending */
#define TIMER_IR_CLR(n)         _BIT(n)

/** Macro for getting a timer match interrupt bit */
#define TIMER_MATCH_INT(n)      (_BIT((n) & 0x0F))
/** Macro for getting a capture event interrupt bit */
#define TIMER_CAP_INT(n)        (_BIT((((n) & 0x0F) + 4)))

/** Timer/counter enable bit */
#define TIMER_ENABLE            ((uint32_t) (1 << 0))
/** Timer/counter reset bit */
#define TIMER_RESET             ((uint32_t) (1 << 1))
/** Timer Control register Mask */
#define TIMER_CTRL_MASK         ((uint32_t) 0x03)

/** Bit location for interrupt on MRx match, n = 0 to 3 */
#define TIMER_INT_ON_MATCH(n)   (_BIT(((n) * 3)))
/** Bit location for reset on MRx match, n = 0 to 3 */
#define TIMER_RESET_ON_MATCH(n) (_BIT((((n) * 3) + 1)))
/** Bit location for stop on MRx match, n = 0 to 3 */
#define TIMER_STOP_ON_MATCH(n)  (_BIT((((n) * 3) + 2)))
/** Match Control register Mask */
#define TIMER_MCR_MASK          ((uint32_t) 0x0FFF)

/** Bit location for CAP.n on CRx rising edge, n = 0 to 3 */
#define TIMER_CAP_RISING(n)     (_BIT(((n) * 3)))
/** Bit location for CAP.n on CRx falling edge, n = 0 to 3 */
#define TIMER_CAP_FALLING(n)    (_BIT((((n) * 3) + 1)))
/** Bit location for CAP.n on CRx interrupt enable, n = 0 to 3 */
#define TIMER_INT_ON_CAP(n)     (_BIT((((n) * 3) + 2)))
/** Capture Control register Mask */
#define TIMER_CCR_MASK          ((uint32_t) 0x0FFF)
/** External Match register Mask */
#define TIMER_EMR_MASK          ((uint32_t) 0x0FFF)
/** Counter Control register Mask */
#define TIMER_CTCR_MASK          ((uint32_t) 0x0F)


/**
 * @}
 */

/** @defgroup RITIMER_5410X CHIP: LPC5410X Repetitive Interrupt Timer driver
 * @ingroup CHIP_5410X_DRIVERS
 * @ ritimer_5410x.h
 * @{
 */

/**
 * @brief Repetitive Interrupt Timer register block structure
 */
typedef struct {				/*!< RITIMER Structure */
	__IO uint32_t  COMPVAL;		/*!< Compare register */
	__IO uint32_t  MASK;		/*!< Mask register */
	__IO uint32_t  CTRL;		/*!< Control register */
	__IO uint32_t  COUNTER;		/*!< 32-bit counter */
	__IO uint32_t  COMPVAL_H;	/*!< Compare register, upper 16-bits */
	__IO uint32_t  MASK_H;		/*!< Compare register, upper 16-bits */
	__I  uint32_t  reserved;
	__IO uint32_t  COUNTER_H;	/*!< Counter register, upper 16-bits */
} LPC_RITIMER_T;

/*
 * @brief RITIMER register support bitfields and mask
 */

/*
 * RIT control register
 */
/**	Set by H/W when the counter value equals the masked compare value */
#define RIT_CTRL_INT    ((uint32_t) (1))
/** Set timer enable clear to 0 when the counter value equals the masked compare value  */
#define RIT_CTRL_ENCLR  ((uint32_t) _BIT(1))
/** Set timer enable on debug */
#define RIT_CTRL_ENBR   ((uint32_t) _BIT(2))
/** Set timer enable */
#define RIT_CTRL_TEN    ((uint32_t) _BIT(3))

/**
 * @}
 */


/** @defgroup UTICK_5410X CHIP: LPC5410X Micro Tick driver
 * @ingroup CHIP_5410X_DRIVERS
  * @ utick_5410x.h
 * @{
 */

/**
 * @brief Micro Tick register block structure
 */
typedef struct {
	__IO uint32_t CTRL;				/*!< UTick Control register */
	__IO uint32_t STATUS;			/*!< UTick Status register */
} LPC_UTICK_T;

/**
 * @brief UTick register definitions
 */
/** UTick repeat delay bit */
#define UTICK_CTRL_REPEAT           ((uint32_t) 1UL << 31)
/** UTick Delay Value Mask */
#define UTICK_CTRL_DELAY_MASK       ((uint32_t) 0x7FFFFFFF)
/** UTick Interrupt Status bit */
#define UTICK_STATUS_INTR           ((uint32_t) 1 << 0)
/** UTick Active Status bit */
#define UTICK_STATUS_ACTIVE         ((uint32_t) 1 << 1)
/** UTick Status Register Mask */
#define UTICK_STATUS_MASK           ((uint32_t) 0x03)

/**
 * @}
 */

/** @defgroup GPIOGP_5410X CHIP: LPC5410X GPIO group driver
 * @ingroup CHIP_5410X_DRIVERS
  * @ gpiogroup_5410x.h
 * @{
 */

/**
 * @brief GPIO grouped interrupt register block structure
 */
typedef struct {					/*!< GPIO_GROUP_INTn Structure */
	__IO uint32_t  CTRL;			/*!< GPIO grouped interrupt control register */
	__I  uint32_t  RESERVED0[7];
	__IO uint32_t  PORT_POL[8];		/*!< GPIO grouped interrupt port polarity register */
	__IO uint32_t  PORT_ENA[8];		/*!< GPIO grouped interrupt port m enable register */
	uint32_t       RESERVED1[4072];
} LPC_GPIOGROUPINT_T;

/**
 * LPC5410x GPIO group bit definitions
 */
#define GPIOGR_INT       (1 << 0)	/*!< GPIO interrupt pending/clear bit */
#define GPIOGR_COMB      (1 << 1)	/*!< GPIO interrupt OR(0)/AND(1) mode bit */
#define GPIOGR_TRIG      (1 << 2)	/*!< GPIO interrupt edge(0)/level(1) mode bit */

/**
 * @}
 */

/** @defgroup MAILBOX_5410X CHIP: LPC5410X Mailbox M4/M0+ driver
 * @ingroup CHIP_5410X_DRIVERS
  * @ mailbox_5410x.h
 * @{
 */

/* Mailbox indexes */
typedef enum {
	MAILBOX_CM0PLUS = 0,
	MAILBOX_CM4
} MBOX_IDX_T;
#define MAILBOX_AVAIL       (MAILBOX_CM4 + 1)	/* Number of available mailboxes */

/** Individual mailbox IRQ structure */
typedef struct {
	__IO    uint32_t        IRQ;		/*!< Mailbox data */
	__O     uint32_t        IRQSET;		/*!< Mailbox data set bits only */
	__O     uint32_t        IRQCLR;		/*!< Mailbox dataclearset bits only */
	__I     uint32_t        RESERVED;
} LPC_MBOXIRQ_T;

/** Mailbox register structure */
typedef struct {						/*!< Mailbox register structure */
	LPC_MBOXIRQ_T           BOX[MAILBOX_AVAIL];	/*!< Mailbox, offset 0 = M0+, offset 1 = M4 */
	LPC_MBOXIRQ_T           RESERVED1[15 - MAILBOX_AVAIL];
	__I     uint32_t        RESERVED2[2];
	__IO    uint32_t        MUTEX;		/*!< Mutex */
} LPC_MBOX_T;

/**
 * @}
 */

/** @defgroup ADC_5410X CHIP:  LPC5410X A/D conversion driver
 * @ingroup CHIP_5410X_DRIVERS
 *  @ adc_5410x.h
 * @{
 */

/** Sequence index enumerations, used in various parts of the code for
    register indexing and sequencer selection */
typedef enum {
	ADC_SEQA_IDX = 0,
	ADC_SEQB_IDX
} ADC_SEQ_IDX_T;

/**
 * @brief ADC register block structure
 */
typedef struct {	/*!< ADCn Structure */
	__IO uint32_t CTRL;
	__O uint32_t RESERVED0;
	__IO uint32_t SEQ_CTRL[2];
	__IO uint32_t SEQ_GDAT[2];
	__IO uint32_t RESERVED1[2];
	__IO uint32_t DAT[12];
	__IO uint32_t THR_LOW[2];
	__IO uint32_t THR_HIGH[2];
	__IO uint32_t CHAN_THRSEL;
	__IO uint32_t INTEN;
	__IO uint32_t FLAGS;
	__IO uint32_t STARTUP;
	__IO uint32_t CALIBR;
} LPC_ADC_T;

/** Maximum sample rate in Hz (12-bit conversions) */
#define ADC_MAX_SAMPLE_RATE 48000000
#define ADC_MAX_CHANNEL_NUM 12

/**
 * @brief ADC register support bitfields and mask
 */
/** ADC Control register bit fields */
#define ADC_CR_CLKDIV_MASK      (0xFF << 0)				/*!< Mask for Clock divider value */
#define ADC_CR_CLKDIV_BITPOS    (0)						/*!< Bit position for Clock divider value */
#define ADC_CR_ASYNC_MODE       (1 << 8)				/*!< Asynchronous mode enable bit */
#define ADC_CR_RESOL(n)         ((n) << 9)				/*!< 2-bits, 6(0x0),8(0x1),10(0x2),12(0x3)-bit mode enable bit */
#define ADC_CR_LPWRMODEBIT      (1 << 10)				/*!< Low power mode enable bit */
#define ADC_CR_BYPASS           (1 << 11)				/*!< Bypass mode */
#define ADC_CR_TSAMP(n)         ((n) << 12)				/*!< 3-bits, 2.5(0x0),3.5(0x1),4.5(0x2),5.5(0x3),6.5(0x4),7.5(0x5),8.5(0x6),9.5(0x7) ADC clocks sampling time */
#define ADC_CR_CALMODEBIT       (1 << 30)				/*!< Self calibration cycle enable bit */
#define ADC_CR_BITACC(n)        ((((n) & 0x1) << 9))	/*!< 12-bit or 10-bit ADC accuracy */
#define ADC_CR_CLKDIV(n)        ((((n) & 0xFF) << 0))	/*!< The APB clock (PCLK) is divided by (this value plus one) to produce the clock for the A/D */
#define ADC_SAMPLE_RATE_CONFIG_MASK (ADC_CR_CLKDIV(0xFF) | ADC_CR_BITACC(0x01))

/** ADC Sequence Control register bit fields */
#define ADC_SEQ_CTRL_CHANSEL(n)             (1 << (n))		/*!< Channel select macro */
#define ADC_SEQ_CTRL_CHANSEL_BITPOS(n)      ((n) << 0)				/*!< Channel select macro */
#define ADC_SEQ_CTRL_CHANSEL_MASK           (0xFFF)				/*!< Channel select mask */

/**
 * @brief ADC sampling time bits 12, 13 and 14
 */
typedef enum _ADC_TSAMP_T {
	ADC_TSAMP_2CLK5 = 0,
	ADC_TSAMP_3CLK5,
	ADC_TSAMP_4CLK5,
	ADC_TSAMP_5CLK5,
	ADC_TSAMP_6CLK5,
	ADC_TSAMP_7CLK5,
	ADC_TSAMP_8CLK5,
	ADC_TSAMP_9CLK5,
} ADC_TSAMP_T;

/** SEQ_CTRL register bit fields */
// #define ADC_SEQ_CTRL_TRIGGER(n)          ((n)<<12)
#define ADC_SEQ_CTRL_HWTRIG_POLPOS       (1 << 18)		/*!< HW trigger polarity - positive edge */
#define ADC_SEQ_CTRL_HWTRIG_SYNCBYPASS   (1 << 19)		/*!< HW trigger bypass synchronisation */
#define ADC_SEQ_CTRL_START               (1 << 26)		/*!< Start conversion enable bit */
#define ADC_SEQ_CTRL_BURST               (1 << 27)		/*!< Repeated conversion enable bit */
#define ADC_SEQ_CTRL_SINGLESTEP          (1 << 28)		/*!< Single step enable bit */
#define ADC_SEQ_CTRL_LOWPRIO             (1 << 29)		/*!< High priority enable bit (regardless of name) */
#define ADC_SEQ_CTRL_MODE_EOS            (1 << 30)		/*!< Mode End of sequence enable bit */
#define ADC_SEQ_CTRL_SEQ_ENA             (1UL << 31)	/*!< Sequence enable bit */

/** ADC global data register bit fields */
#define ADC_SEQ_GDAT_RESULT_MASK         (0xFFF << 4)	/*!< Result value mask */
#define ADC_SEQ_GDAT_RESULT_BITPOS       (4)			/*!< Result start bit position */
#define ADC_SEQ_GDAT_THCMPRANGE_MASK     (0x3 << 16)	/*!< Comparion range mask */
#define ADC_SEQ_GDAT_THCMPRANGE_BITPOS   (16)			/*!< Comparison range bit position */
#define ADC_SEQ_GDAT_THCMPCROSS_MASK     (0x3 << 18)	/*!< Comparion cross mask */
#define ADC_SEQ_GDAT_THCMPCROSS_BITPOS   (18)			/*!< Comparison cross bit position */
#define ADC_SEQ_GDAT_CHAN_MASK           (0xF << 26)	/*!< Channel number mask */
#define ADC_SEQ_GDAT_CHAN_BITPOS         (26)			/*!< Channel number bit position */
#define ADC_SEQ_GDAT_OVERRUN             (1 << 30)		/*!< Overrun bit */
#define ADC_SEQ_GDAT_DATAVALID           (1UL << 31)	/*!< Data valid bit */

/** ADC Data register bit fields */
#define ADC_DR_RESULT_BITPOS       (4)			/*!< Result start bit position */
#define ADC_DR_RESULT(n)           ((((n) >> 4) & 0xFFF))	/*!< Macro for getting the ADC data value */
#define ADC_DR_THCMPRANGE_MASK     (0x3 << 16)			/*!< Comparion range mask */
#define ADC_DR_THCMPRANGE_BITPOS   (16)					/*!< Comparison range bit position */
#define ADC_DR_THCMPRANGE(n)       (((n) >> ADC_DR_THCMPRANGE_BITPOS) & 0x3)
#define ADC_DR_THCMPCROSS_MASK     (0x3 << 18)			/*!< Comparion cross mask */
#define ADC_DR_THCMPCROSS_BITPOS   (18)					/*!< Comparison cross bit position */
#define ADC_DR_THCMPCROSS(n)       (((n) >> ADC_DR_THCMPCROSS_BITPOS) & 0x3)
#define ADC_DR_CHAN_MASK           (0xF << 26)			/*!< Channel number mask */
#define ADC_DR_CHAN_BITPOS         (26)					/*!< Channel number bit position */
#define ADC_DR_CHANNEL(n)          (((n) >> ADC_DR_CHAN_BITPOS) & 0xF)	/*!< Channel number bit position */
#define ADC_DR_OVERRUN             (1 << 30)			/*!< Overrun bit */
#define ADC_DR_DATAVALID           (1UL << 31)			/*!< Data valid bit */
#define ADC_DR_DONE(n)             (((n) >> 31))

/** ADC low/high Threshold register bit fields */
#define ADC_THR_VAL_MASK            (0xFFF << 4)		/*!< Threshold value bit mask */
#define ADC_THR_VAL_POS             (4)					/*!< Threshold value bit position */

/** ADC Threshold select register bit fields */
#define ADC_THRSEL_CHAN_SEL_THR1(n) (1 << (n))			/*!< Select THR1 register for channel n */

/** ADC Interrupt Enable register bit fields */
#define ADC_INTEN_SEQA_ENABLE       (1 << 0)			/*!< Sequence A Interrupt enable bit */
#define ADC_INTEN_SEQB_ENABLE       (1 << 1)			/*!< Sequence B Interrupt enable bit */
#define ADC_INTEN_SEQN_ENABLE(seq)  (1 << (seq))		/*!< Sequence A/B Interrupt enable bit */
#define ADC_INTEN_OVRRUN_ENABLE     (1 << 2)			/*!< Overrun Interrupt enable bit */
#define ADC_INTEN_CMP_DISBALE       (0)					/*!< Disable comparison interrupt value */
#define ADC_INTEN_CMP_OUTSIDETH     (1)					/*!< Outside threshold interrupt value */
#define ADC_INTEN_CMP_CROSSTH       (2)					/*!< Crossing threshold interrupt value */
#define ADC_INTEN_CMP_MASK          (3)					/*!< Comparison interrupt value mask */
#define ADC_INTEN_CMP_ENABLE(isel, ch) (((isel) & ADC_INTEN_CMP_MASK) << ((2 * (ch)) + 3))	/*!< Interrupt selection for channel */

/** ADC Flags register bit fields */
#define ADC_FLAGS_THCMP_MASK(ch)    (1 << (ch))		/*!< Threshold comparison status for channel */
#define ADC_FLAGS_OVRRUN_MASK(ch)   (1 << (12 + (ch)))	/*!< Overrun status for channel */
#define ADC_FLAGS_SEQA_OVRRUN_MASK  (1 << 24)			/*!< Seq A Overrun status */
#define ADC_FLAGS_SEQB_OVRRUN_MASK  (1 << 25)			/*!< Seq B Overrun status */
#define ADC_FLAGS_SEQN_OVRRUN_MASK(seq) (1 << (24 + (seq)))	/*!< Seq A/B Overrun status */
#define ADC_FLAGS_SEQA_INT_MASK     (1 << 28)			/*!< Seq A Interrupt status */
#define ADC_FLAGS_SEQB_INT_MASK     (1 << 29)			/*!< Seq B Interrupt status */
#define ADC_FLAGS_SEQN_INT_MASK(seq) (1 << (28 + (seq)))/*!< Seq A/B Interrupt status */
#define ADC_FLAGS_THCMP_INT_MASK    (1 << 30)			/*!< Threshold comparison Interrupt status */
#define ADC_FLAGS_OVRRUN_INT_MASK   (1UL << 31)			/*!< Overrun Interrupt status */

/** ADC Startup register bit fields */
#define ADC_STARTUP_ENABLE       (0x1 << 0)
#define ADC_STARTUP_INIT         (0x1 << 1)

/* ADC Calibration register definition */
#define ADC_CALIB                       (0x1 << 0)
#define ADC_CALREQD                     (0x1 << 1)

/**
 * @}
 */
 


/** @defgroup DMALEG_5410X CHIP: LPC5410X DMA Engine driver (legacy)
 * @ingroup CHIP_5410X_DRIVERS
  * @ dma_5410x.h
 * @{
 */
/**
 * @brief DMA Controller shared registers structure
 */
typedef struct {					/*!< DMA shared registers structure */
	__IO uint32_t  ENABLESET;		/*!< DMA Channel Enable read and Set for all DMA channels */
	__I  uint32_t  RESERVED0;
	__O  uint32_t  ENABLECLR;		/*!< DMA Channel Enable Clear for all DMA channels */
	__I  uint32_t  RESERVED1;
	__I  uint32_t  ACTIVE;			/*!< DMA Channel Active status for all DMA channels */
	__I  uint32_t  RESERVED2;
	__I  uint32_t  BUSY;			/*!< DMA Channel Busy status for all DMA channels */
	__I  uint32_t  RESERVED3;
	__IO uint32_t  ERRINT;			/*!< DMA Error Interrupt status for all DMA channels */
	__I  uint32_t  RESERVED4;
	__IO uint32_t  INTENSET;		/*!< DMA Interrupt Enable read and Set for all DMA channels */
	__I  uint32_t  RESERVED5;
	__O  uint32_t  INTENCLR;		/*!< DMA Interrupt Enable Clear for all DMA channels */
	__I  uint32_t  RESERVED6;
	__IO  uint32_t  INTA;			/*!< DMA Interrupt A status for all DMA channels */
	__I  uint32_t  RESERVED7;
	__IO  uint32_t  INTB;			/*!< DMA Interrupt B status for all DMA channels */
	__I  uint32_t  RESERVED8;
	__O  uint32_t  SETVALID;		/*!< DMA Set ValidPending control bits for all DMA channels */
	__I  uint32_t  RESERVED9;
	__O  uint32_t  SETTRIG;			/*!< DMA Set Trigger control bits for all DMA channels */
	__I  uint32_t  RESERVED10;
	__O  uint32_t  ABORT;			/*!< DMA Channel Abort control for all DMA channels */
} LPC_DMA_COMMON_T;

/**
 * @brief DMA Controller shared registers structure
 */
typedef struct {					/*!< DMA channel register structure */
	__IO  uint32_t  CFG;				/*!< DMA Configuration register */
	__I   uint32_t  CTLSTAT;			/*!< DMA Control and status register */
	__IO  uint32_t  XFERCFG;			/*!< DMA Transfer configuration register */
	__I   uint32_t  RESERVED;
} LPC_DMA_CHANNEL_T;

/* On LPC540XX, Max DMA channel is 22 */
#define MAX_DMA_CHANNEL         (22)

/**
 * @brief DMA Controller register block structure
 */
typedef struct {					/*!< DMA Structure */
	__IO uint32_t  CTRL;			/*!< DMA control register */
	__I  uint32_t  INTSTAT;			/*!< DMA Interrupt status register */
	__IO uint32_t  SRAMBASE;		/*!< DMA SRAM address of the channel configuration table */
	__I  uint32_t  RESERVED2[5];
	LPC_DMA_COMMON_T DMACOMMON[1];	/*!< DMA shared channel (common) registers */
	__I  uint32_t  RESERVED0[225];
	LPC_DMA_CHANNEL_T DMACH[MAX_DMA_CHANNEL];	/*!< DMA channel registers */
} LPC_DMA_T;

/* DMA interrupt status bits (common) */
#define DMA_INTSTAT_ACTIVEINT       0x2		/*!< Summarizes whether any enabled interrupts are pending */
#define DMA_INTSTAT_ACTIVEERRINT    0x4		/*!< Summarizes whether any error interrupts are pending */

/* Support macro for DMA_CHDESC_T */
#define DMA_ADDR(addr)      ((uint32_t) (addr))

/* Support definitions for setting the configuration of a DMA channel. You
   will need to get more information on these options from the User manual. */
#define DMA_CFG_PERIPHREQEN     (1 << 0)	/*!< Enables Peripheral DMA requests */
#define DMA_CFG_HWTRIGEN        (1 << 1)	/*!< Use hardware triggering via imput mux */
#define DMA_CFG_TRIGPOL_LOW     (0 << 4)	/*!< Hardware trigger is active low or falling edge */
#define DMA_CFG_TRIGPOL_HIGH    (1 << 4)	/*!< Hardware trigger is active high or rising edge */
#define DMA_CFG_TRIGTYPE_EDGE   (0 << 5)	/*!< Hardware trigger is edge triggered */
#define DMA_CFG_TRIGTYPE_LEVEL  (1 << 5)	/*!< Hardware trigger is level triggered */
#define DMA_CFG_TRIGBURST_SNGL  (0 << 6)	/*!< Single transfer. Hardware trigger causes a single transfer */
#define DMA_CFG_TRIGBURST_BURST (1 << 6)	/*!< Burst transfer (see UM) */
#define DMA_CFG_BURSTPOWER_1    (0 << 8)	/*!< Set DMA burst size to 1 transfer */
#define DMA_CFG_BURSTPOWER_2    (1 << 8)	/*!< Set DMA burst size to 2 transfers */
#define DMA_CFG_BURSTPOWER_4    (2 << 8)	/*!< Set DMA burst size to 4 transfers */
#define DMA_CFG_BURSTPOWER_8    (3 << 8)	/*!< Set DMA burst size to 8 transfers */
#define DMA_CFG_BURSTPOWER_16   (4 << 8)	/*!< Set DMA burst size to 16 transfers */
#define DMA_CFG_BURSTPOWER_32   (5 << 8)	/*!< Set DMA burst size to 32 transfers */
#define DMA_CFG_BURSTPOWER_64   (6 << 8)	/*!< Set DMA burst size to 64 transfers */
#define DMA_CFG_BURSTPOWER_128  (7 << 8)	/*!< Set DMA burst size to 128 transfers */
#define DMA_CFG_BURSTPOWER_256  (8 << 8)	/*!< Set DMA burst size to 256 transfers */
#define DMA_CFG_BURSTPOWER_512  (9 << 8)	/*!< Set DMA burst size to 512 transfers */
#define DMA_CFG_BURSTPOWER_1024 (10 << 8)	/*!< Set DMA burst size to 1024 transfers */
#define DMA_CFG_BURSTPOWER(n)   ((n) << 8)	/*!< Set DMA burst size to 2^n transfers, max n=10 */
#define DMA_CFG_SRCBURSTWRAP    (1 << 14)	/*!< Source burst wrapping is enabled for this DMA channel */
#define DMA_CFG_DSTBURSTWRAP    (1 << 15)	/*!< Destination burst wrapping is enabled for this DMA channel */
#define DMA_CFG_CHPRIORITY(p)   ((p) << 16)	/*!< Sets DMA channel priority, min 0 (highest), max 3 (lowest) */

/* DMA channel control and status register definitions */
#define DMA_CTLSTAT_VALIDPENDING    (1 << 0)	/*!< Valid pending flag for this channel */
#define DMA_CTLSTAT_TRIG            (1 << 2)	/*!< Trigger flag. Indicates that the trigger for this channel is currently set */

/* DMA channel transfer configuration registers definitions */
#define DMA_XFERCFG_CFGVALID        (1 << 0)	/*!< Configuration Valid flag */
#define DMA_XFERCFG_RELOAD          (1 << 1)	/*!< Indicates whether the channels control structure will be reloaded when the current descriptor is exhausted */
#define DMA_XFERCFG_SWTRIG          (1 << 2)	/*!< Software Trigger */
#define DMA_XFERCFG_CLRTRIG         (1 << 3)	/*!< Clear Trigger */
#define DMA_XFERCFG_SETINTA         (1 << 4)	/*!< Set Interrupt flag A for this channel to fire when descriptor is complete */
#define DMA_XFERCFG_SETINTB         (1 << 5)	/*!< Set Interrupt flag B for this channel to fire when descriptor is complete */
#define DMA_XFERCFG_WIDTH_8         (0 << 8)	/*!< 8-bit transfers are performed */
#define DMA_XFERCFG_WIDTH_16        (1 << 8)	/*!< 16-bit transfers are performed */
#define DMA_XFERCFG_WIDTH_32        (2 << 8)	/*!< 32-bit transfers are performed */
#define DMA_XFERCFG_SRCINC_0        (0 << 12)	/*!< DMA source address is not incremented after a transfer */
#define DMA_XFERCFG_SRCINC_1        (1 << 12)	/*!< DMA source address is incremented by 1 (width) after a transfer */
#define DMA_XFERCFG_SRCINC_2        (2 << 12)	/*!< DMA source address is incremented by 2 (width) after a transfer */
#define DMA_XFERCFG_SRCINC_4        (3 << 12)	/*!< DMA source address is incremented by 4 (width) after a transfer */
#define DMA_XFERCFG_DSTINC_0        (0 << 14)	/*!< DMA destination address is not incremented after a transfer */
#define DMA_XFERCFG_DSTINC_1        (1 << 14)	/*!< DMA destination address is incremented by 1 (width) after a transfer */
#define DMA_XFERCFG_DSTINC_2        (2 << 14)	/*!< DMA destination address is incremented by 2 (width) after a transfer */
#define DMA_XFERCFG_DSTINC_4        (3 << 14)	/*!< DMA destination address is incremented by 4 (width) after a transfer */
#define DMA_XFERCFG_XFERCOUNT(n)    ((n - 1) << 16)	/*!< DMA transfer count in 'transfers', between (0)1 and (1023)1024 */

/* DMA channel mapping - each channel is mapped to an individual peripheral
   and direction or a DMA imput mux trigger */
typedef enum {
	DMAREQ_UART0_RX = 0,				/*!< UART00 receive DMA channel */
	DMA_CH0 = DMAREQ_UART0_RX,
	DMAREQ_UART0_TX,					/*!< UART0 transmit DMA channel */
	DMA_CH1 = DMAREQ_UART0_TX,
	DMAREQ_UART1_RX,					/*!< UART1 receive DMA channel */
	DMA_CH2 = DMAREQ_UART1_RX,
	DMAREQ_UART1_TX,					/*!< UART1 transmit DMA channel */
	DMA_CH3 = DMAREQ_UART1_TX,
	DMAREQ_UART2_RX,					/*!< UART2 receive DMA channel */
	DMA_CH4 = DMAREQ_UART2_RX,
	DMAREQ_UART2_TX,					/*!< UART2 transmit DMA channel */
	DMA_CH5 = DMAREQ_UART2_TX,
	DMAREQ_UART3_RX,					/*!< UART3 receive DMA channel */
	DMA_CH6 = DMAREQ_UART3_RX,
	DMAREQ_UART3_TX,					/*!< UART3 transmit DMA channel */
	DMA_CH7 = DMAREQ_UART3_TX,
	DMAREQ_SPI0_RX,					/*!< SPI0 receive DMA channel */
	DMA_CH8 = DMAREQ_SPI0_RX,
	DMAREQ_SPI0_TX,					/*!< SPI0 transmit DMA channel */
	DMA_CH9 = DMAREQ_SPI0_TX,
	DMAREQ_SPI1_RX,					/*!< SPI1 receive DMA channel */
	DMA_CH10 = DMAREQ_SPI1_RX,
	DMAREQ_SPI1_TX,					/*!< SPI1 transmit DMA channel */
	DMA_CH11 = DMAREQ_SPI1_TX,
	DMAREQ_I2C0_SLAVE,					/*!< I2C0 Slave DMA channel */
	DMA_CH12 = DMAREQ_I2C0_SLAVE,
	DMAREQ_I2C0_MASTER,					/*!< I2C0 Master DMA channel */
	DMA_CH13 = DMAREQ_I2C0_MASTER,
	DMAREQ_I2C1_SLAVE,					/*!< I2C1 Slave DMA channel */
	DMA_CH14 = DMAREQ_I2C1_SLAVE,
	DMAREQ_I2C1_MASTER,					/*!< I2C1 Master DMA channel */
	DMA_CH15 = DMAREQ_I2C1_MASTER,
	DMAREQ_I2C2_SLAVE,					/*!< I2C2 Slave DMA channel */
	DMA_CH16 = DMAREQ_I2C2_SLAVE,
	DMAREQ_I2C2_MASTER,					/*!< I2C2 Master DMA channel */
	DMA_CH17 = DMAREQ_I2C2_MASTER,
	DMAREQ_I2C0_MONITOR,					/*!< I2C0 Monitor DMA channel */
	DMA_CH18 = DMAREQ_I2C0_MONITOR,
	DMAREQ_I2C1_MONITOR,					/*!< I2C1 Monitor DMA channel */
	DMA_CH19 = DMAREQ_I2C1_MONITOR,
	DMAREQ_I2C2_MONITOR,					/*!< I2C2 Monitor DMA channel */
	DMA_CH20 = DMAREQ_I2C2_MONITOR,
	RESERVED_SPARE_DMA,
	DMA_CH21 = RESERVED_SPARE_DMA
} DMA_CHID_T;

/**
 * @}
 */

/** @defgroup UART_5410X CHIP: LPC5410X UART Driver
 * @ingroup CHIP_5410X_DRIVERS
  * @ uart_5410x.h
 * @{
 */

/* UART Status Register bits */
#define UART_RXRDY           (1 << 0)	/* Receive data ready */
#define UART_RXIDLE          (1 << 1)	/* Receiver Idle */
#define UART_TXRDY           (1 << 2)	/* Transmitter ready */
#define UART_TXIDLE          (1 << 3)	/* Transmitter Idle */
#define UART_RXDERR          (0xF100)	/* overrun err, frame err, parity err, RxNoise err */
#define UART_TXDERR          (0x0200)	/* underrun err */
#define UART_START           (0x1000)

/* UART Interrupt register bits */
#define UART_INT_RXRDY          (1 << 0)
#define UART_INT_TXRDY          (1 << 2)
#define UART_INT_TXIDLE         (1 << 3)
#define UART_INT_CTS            (1 << 5)
#define UART_INT_TXDIS          (1 << 6)
#define UART_INT_OVR            (1 << 8)
#define UART_INT_BREAK          (1 << 11)
#define UART_INT_START          (1 << 12)
#define UART_INT_FRMERR         (1 << 13)
#define UART_INT_PARERR         (1 << 14)
#define UART_INT_RXNOISE        (1 << 15)
#define UART_INT_ABAUDERR       (1 << 16)

/* Configuration register bits */
#define UARTEN      1

#define UART_CTL_TXDIS          (1UL << 6)
#define UART_CTL_TXBRKEN        (1UL << 1)
#define UART_CTL_AUTOBAUD       (1UL << 16)
#define UART_CFG_RES            (2UL | (1UL << 10) | (1UL << 13) | (1UL << 17) | (0xFFUL << 24))
#define UART_CFG_ENABLE          1
#define UART_PAR_MASK           (3 << 4)
#define UART_DATA_MASK          (3 << 2)
#define UART_CTL_RES            (1UL | (7UL << 3) | (1UL << 7) | (0x3FUL << 10) | (0x7FFFUL << 17))
#define UART_IDLE_MASK          (1 << 3)
#define UART_STAT_CTS           (1 << 4)
#define UART_STAT_BREAK         (1 << 10)
#define UART_STAT_RXIDLE        (1 << 1)

/*******************
 * EXPORTED MACROS  *
 ********************/
#define     ECHO_EN             1
#define     ECHO_DIS            0

/*********************
 * EXPORTED TYPEDEFS  *
 **********************/

typedef struct {		/* UART registers Structure          */
	__IO uint32_t CFG;				/*!< Offset: 0x000 Configuration register  */
	__IO uint32_t CTL;				/*!< Offset: 0x004 Control register */
	__IO uint32_t STAT;				/*!< Offset: 0x008 Status register */
	__IO uint32_t INTENSET;			/*!< Offset: 0x00C Interrupt Enable Read and Set register */
	__O  uint32_t INTENCLR;			/*!< Offset: 0x010 Interrupt Enable Clear register */
	__I  uint32_t RXDAT;		/*!< Offset: 0x014 Receiver Data register */
	__I  uint32_t RXDATSTAT;	/*!< Offset: 0x018 Rx Data with status */
	__O  uint32_t TXDAT;			/*!< Offset: 0x01C Transmitter Data Register */
	__IO uint32_t BRG;				/*!< Offset: 0x020 Baud Rate Generator register */
	__I  uint32_t INTSTAT;	/*!< Offset: 0x024 Interrupt Status register */
	__IO uint32_t OSR;				/*!< Offset: 0x028 Oversampling register */
	__IO uint32_t ADR;				/*!< Offset: 0x02C Address register (for automatic address matching) */
} LPC_USART_T;

/**
 * @brief UART CFG register definitions
 */
// #define UART_CFG_ENABLE         (0x01 << 0)
#define UART_CFG_DATALEN_7      (0x00 << 2)		/*!< UART 7 bit length mode */
#define UART_CFG_DATALEN_8      (0x01 << 2)		/*!< UART 8 bit length mode */
#define UART_CFG_DATALEN_9      (0x02 << 2)		/*!< UART 9 bit length mode */
#define UART_CFG_PARITY_NONE    (0x00 << 4)		/*!< No parity */
#define UART_CFG_PARITY_EVEN    (0x02 << 4)		/*!< Even parity */
#define UART_CFG_PARITY_ODD     (0x03 << 4)		/*!< Odd parity */
#define UART_CFG_STOPLEN_1      (0x00 << 6)		/*!< UART One Stop Bit Select */
#define UART_CFG_STOPLEN_2      (0x01 << 6)		/*!< UART Two Stop Bits Select */
#define UART_CFG_MODE32K        (0x01 << 7)		/*!< UART 32K MODE */
#define UART_CFG_LINMODE        (0x01 << 8)		/*!< UART LIN MODE */
#define UART_CFG_CTSEN          (0x01 << 9)		/*!< CTS enable bit */
#define UART_CFG_SYNCEN         (0x01 << 11)	/*!< Synchronous mode enable bit */
#define UART_CFG_CLKPOL         (0x01 << 12)	/*!< Un_RXD rising edge sample enable bit */
#define UART_CFG_SYNCMST        (0x01 << 14)	/*!< Select master mode (synchronous mode) enable bit */
#define UART_CFG_LOOP           (0x01 << 15)	/*!< Loopback mode enable bit */

/**
 * @brief UART CTRL register definitions
 */
#define UART_CTRL_TXBRKEN       (0x01 << 1)		/*!< Continuous break enable bit */
#define UART_CTRL_ADDRDET       (0x01 << 2)		/*!< Address detect mode enable bit */
#define UART_CTRL_TXDIS         (0x01 << 6)		/*!< Transmit disable bit */
#define UART_CTRL_CC            (0x01 << 8)		/*!< Continuous Clock mode enable bit */
#define UART_CTRL_CLRCC         (0x01 << 9)		/*!< Clear Continuous Clock bit */
#define UART_CTRL_AUTOBAUD      (0x01 << 16)	/*!< Auto baud bit */

/**
 * @brief UART STAT register definitions
 */
#define UART_STAT_RXRDY         (0x01 << 0)			/*!< Receiver ready */
// #define UART_STAT_RXIDLE        (0x01 << 1)			/*!< Receiver idle */
#define UART_STAT_TXRDY         (0x01 << 2)			/*!< Transmitter ready for data */
#define UART_STAT_TXIDLE        (0x01 << 3)			/*!< Transmitter idle */
// #define UART_STAT_CTS           (0x01 << 4)			/*!< Status of CTS signal */
#define UART_STAT_DELTACTS      (0x01 << 5)			/*!< Change in CTS state */
#define UART_STAT_TXDISINT      (0x01 << 6)			/*!< Transmitter disabled */
#define UART_STAT_OVERRUNINT    (0x01 << 8)			/*!< Overrun Error interrupt flag. */
#define UART_STAT_RXBRK         (0x01 << 10)		/*!< Received break */
#define UART_STAT_DELTARXBRK    (0x01 << 11)		/*!< Change in receive break detection */
#define UART_STAT_START         (0x01 << 12)		/*!< Start detected */
#define UART_STAT_FRM_ERRINT    (0x01 << 13)		/*!< Framing Error interrupt flag */
#define UART_STAT_PAR_ERRINT    (0x01 << 14)		/*!< Parity Error interrupt flag */
#define UART_STAT_RXNOISEINT    (0x01 << 15)		/*!< Received Noise interrupt flag */
#define UART_STAT_ABERR         (0x01 << 16)		/*!< Auto baud error flag */

/**
 * @brief UART INTENSET/INTENCLR register definitions
 */
#define UART_INTEN_RXRDY        (0x01 << 0)			/*!< Receive Ready interrupt */
#define UART_INTEN_TXRDY        (0x01 << 2)			/*!< Transmit Ready interrupt */
#define UART_INTEN_DELTACTS     (0x01 << 5)			/*!< Change in CTS state interrupt */
#define UART_INTEN_TXDIS        (0x01 << 6)			/*!< Transmitter disable interrupt */
#define UART_INTEN_OVERRUN      (0x01 << 8)			/*!< Overrun error interrupt */
#define UART_INTEN_DELTARXBRK   (0x01 << 11)		/*!< Change in receiver break detection interrupt */
#define UART_INTEN_START        (0x01 << 12)		/*!< Start detect interrupt */
#define UART_INTEN_FRAMERR      (0x01 << 13)		/*!< Frame error interrupt */
#define UART_INTEN_PARITYERR    (0x01 << 14)		/*!< Parity error interrupt */
#define UART_INTEN_RXNOISE      (0x01 << 15)		/*!< Received noise interrupt */

/**
 * @brief	UART Baud rate calculation structure
 * @note
 * Use oversampling (@a ovr) value other than 16, only if the difference
 * between the actual baud and desired baud has an unacceptable error percentage.
 * Smaller @a ovr values can cause the sampling position within the data-bit
 * less accurate an may potentially cause more noise errors or incorrect data
 * set ovr to < 10 only when there is no other higher values suitable. Note that
 * the UART OSR and BRG are -1 encoded i.e., when writing to register BRG @a div must
 * be (div - 1) and when writing to OSR @a ovr must be (ovr - 1)
 */
typedef struct {
	uint32_t clk;	/*!< IN: Base clock to fractional divider; OUT: "Base clock rate for UART" */
	uint32_t baud;	/*!< IN: Required baud rate; OUT: Actual baud rate */
	uint8_t ovr;	/*!< IN: Number of desired over samples [0-auto detect or values 5 to 16]; OUT: Auto detected over samples [unchanged if IN is not 0] */
	uint8_t mul;	/*!< IN: 0 - calculate MUL, 1 - do't calculate (@a clk) has UART base clock; OUT: MUL value to be set in FRG register */
	uint16_t div;	/*!< OUT: Integer divider to divide the "Base clock rate for UART" */
} UART_BAUD_T;

/**
 * @}
 */

/** @defgroup SPI_COMMON_5410X CHIP: LPC5410X SPI driver
 * @ingroup CHIP_5410X_DRIVERS
  * @ spi_common_5410x.h
 * @{
 */

/**
 * @brief SPI register block structure
 */
typedef struct {					/*!< SPI Structure */
	__IO uint32_t  CFG;				/*!< SPI Configuration register */
	__IO uint32_t  DLY;				/*!< SPI Delay register */
	__IO uint32_t  STAT;			/*!< SPI Status register */
	__IO uint32_t  INTENSET;		/*!< SPI Interrupt Enable Set register */
	__O uint32_t  INTENCLR;		/*!< SPI Interrupt Enable Clear register */
	__I  uint32_t  RXDAT;			/*!< SPI Receive Data register */
	__IO uint32_t  TXDATCTL;		/*!< SPI Transmit Data with Control register */
	__O  uint32_t  TXDAT;			/*!< SPI Transmit Data register */
	__IO uint32_t  TXCTRL;			/*!< SPI Transmit Control register */
	__IO uint32_t  DIV;				/*!< SPI clock Divider register */
	__I  uint32_t  INTSTAT;			/*!< SPI Interrupt Status register */
} LPC_SPI_T;

/**
 * Macro defines for SPI Configuration register
 */
#define SPI_CFG_BITMASK         (0xFBD)						/** SPI register bit mask */
#define SPI_CFG_SPI_EN          (1 << 0)					/** SPI Slave Mode Select */
#define SPI_CFG_SLAVE_EN        (0 << 0)					/** SPI Master Mode Select */
#define SPI_CFG_MASTER_EN       (1 << 2)					/** SPI MSB First mode enable */
#define SPI_CFG_MSB_FIRST_EN    (0 << 3)					/** SPI LSB First mode enable */
#define SPI_CFG_LSB_FIRST_EN    (1 << 3)					/** SPI Clock Phase Select */
#define SPI_CFG_CPHA_FIRST      (0 << 4)					/** Capture data on the first edge, Change data on the following edge */
#define SPI_CFG_CPHA_SECOND     (1 << 4)					/** SPI Clock Polarity Select */
#define SPI_CFG_CPOL_LO         (0 << 5)					/** The rest state of the clock (between frames) is low. */
#define SPI_CFG_CPOL_HI         (1 << 5)					/** The rest state of the clock (between frames) is high. */
#define SPI_CFG_LBM_EN          (1 << 7)					/** SPI control 1 loopback mode enable */
#define SPI_CFG_SPOL_LO         (0 << 8)					/** SPI SSEL0 Polarity Select */
#define SPI_CFG_SPOL_HI         (1 << 8)					/** SSEL0 is active High */
#define SPI_CFG_SPOLNUM_HI(n)   (1 << ((n) + 8))			/** SSELN is active High, selects 0 - 3 */

/**
 * Macro defines for SPI Delay register
 */
#define  SPI_DLY_BITMASK            (0xFFFF)				/** SPI DLY Register Mask */
#define  SPI_DLY_PRE_DELAY(n)       (((n) & 0x0F) << 0)		/** Time in SPI clocks between SSEL assertion and the beginning of a data frame */
#define  SPI_DLY_POST_DELAY(n)      (((n) & 0x0F) << 4)		/** Time in SPI clocks between the end of a data frame and SSEL deassertion. */
#define  SPI_DLY_FRAME_DELAY(n)     (((n) & 0x0F) << 8)		/** Minimum time in SPI clocks between adjacent data frames. */
#define  SPI_DLY_TRANSFER_DELAY(n)  (((n) & 0x0F) << 12)	/** Minimum time in SPI clocks that the SSEL is deasserted between transfers. */

/**
 * Macro defines for SPI Status register
 */
#define SPI_STAT_BITMASK            (0x1FF)					/** SPI STAT Register BitMask */
#define SPI_STAT_RXRDY              (1 << 0)				/** Receiver Ready Flag */
#define SPI_STAT_TXRDY              (1 << 1)				/** Transmitter Ready Flag */
#define SPI_STAT_RXOV               (1 << 2)				/** Receiver Overrun interrupt flag */
#define SPI_STAT_TXUR               (1 << 3)				/** Transmitter Underrun interrupt flag (In Slave Mode only) */
#define SPI_STAT_SSA                (1 << 4)				/** Slave Select Assert */
#define SPI_STAT_SSD                (1 << 5)				/** Slave Select Deassert */
#define SPI_STAT_STALLED            (1 << 6)				/** Stalled status flag */
#define SPI_STAT_EOT                (1 << 7)				/** End Transfer flag */
#define SPI_STAT_MSTIDLE            (1 << 8)				/** Idle status flag */

/**
 * Macro defines for SPI Interrupt Enable read and Set register
 */
#define SPI_INTENSET_BITMASK        (0x3F)					/** SPI INTENSET Register BitMask */
#define SPI_INTENSET_RXDYEN         (1 << 0)				/** Enable Interrupt when receiver data is available */
#define SPI_INTENSET_TXDYEN         (1 << 1)				/** Enable Interrupt when the transmitter holding register is available. */
#define SPI_INTENSET_RXOVEN         (1 << 2)				/**  Enable Interrupt when a receiver overrun occurs */
#define SPI_INTENSET_TXUREN         (1 << 3)				/**  Enable Interrupt when a transmitter underrun occurs (In Slave Mode Only)*/
#define SPI_INTENSET_SSAEN          (1 << 4)				/**  Enable Interrupt when the Slave Select is asserted.*/
#define SPI_INTENSET_SSDEN          (1 << 5)				/**  Enable Interrupt when the Slave Select is deasserted..*/

/**
 * Macro defines for SPI Interrupt Enable Clear register
 */
#define SPI_INTENCLR_BITMASK        (0x3F)					/** SPI INTENCLR Register BitMask */
#define SPI_INTENCLR_RXDYEN         (1 << 0)				/** Disable Interrupt when receiver data is available */
#define SPI_INTENCLR_TXDYEN         (1 << 1)				/** Disable Interrupt when the transmitter holding register is available. */
#define SPI_INTENCLR_RXOVEN         (1 << 2)				/** Disable Interrupt when a receiver overrun occurs */
#define SPI_INTENCLR_TXUREN         (1 << 3)				/** Disable Interrupt when a transmitter underrun occurs (In Slave Mode Only) */
#define SPI_INTENCLR_SSAEN          (1 << 4)				/** Disable Interrupt when the Slave Select is asserted. */
#define SPI_INTENCLR_SSDEN          (1 << 5)				/** Disable Interrupt when the Slave Select is deasserted.. */

/**
 * Macro defines for SPI Receiver Data register
 */
#define SPI_RXDAT_BITMASK           (0x1FFFFF)				/** SPI RXDAT Register BitMask */
#define SPI_RXDAT_DATA(n)           ((n) & 0xFFFF)			/** Receiver Data  */
#define SPI_RXDAT_RXSSELN_ACTIVE    (0 << 16)				/** The state of SSEL pin is active */
#define SPI_RXDAT_RXSSELN_INACTIVE  ((1 << 16)				/** The state of SSEL pin is inactive */
#define SPI_RXDAT_RXSSELNUM_INACTIVE(n) (1 << ((n) + 16))	/** The state of SSELN pin is inactive */
#define SPI_RXDAT_SOT               (1 << 20)				/** Start of Transfer flag  */

/**
 * Macro defines for SPI Transmitter Data and Control register
 */
#define SPI_TXDATCTL_BITMASK        (0xF7FFFFF)				/** SPI TXDATCTL Register BitMask */
#define SPI_TXDATCTL_DATA(n)        ((n) & 0xFFFF)			/** SPI Transmit Data */
#define SPI_TXDATCTL_CTRLMASK       (0xF7F0000)				/** SPI TXDATCTL Register BitMask for control bits only */
#define SPI_TXDATCTL_ASSERT_SSEL    (0 << 16)				/** Assert SSEL0 pin */
#define SPI_TXDATCTL_DEASSERT_SSEL  (1 << 16)				/** Deassert SSEL0 pin */
#define SPI_TXDATCTL_DEASSERTNUM_SSEL(n)    (1 << ((n) + 16))	/** Deassert SSELN pin */
#define SPI_TXDATCTL_DEASSERT_ALL   (0xF << 16)				/** Deassert all SSEL pins */
#define SPI_TXDATCTL_EOT            (1 << 20)				/** End of Transfer flag (TRANSFER_DELAY is applied after sending the current frame) */
#define SPI_TXDATCTL_EOF            (1 << 21)				/** End of Frame flag (FRAME_DELAY is applied after sending the current part) */
#define SPI_TXDATCTL_RXIGNORE       (1 << 22)				/** Receive Ignore Flag */
#define SPI_TXDATCTL_FLEN(n)        (((n) & 0x0F) << 24)	/** Frame length - 1 */

/**
 * Macro defines for SPI Transmitter Data Register
 */
#define SPI_TXDAT_DATA(n)           ((n) & 0xFFFF)			/** SPI Transmit Data */

/**
 * Macro defines for SPI Transmitter Control register
 */
#define SPI_TXCTL_BITMASK           (0xF7F0000)				/** SPI TXDATCTL Register BitMask */
#define SPI_TXCTL_ASSERT_SSEL       (0 << 16)				/** Assert SSEL0 pin */
#define SPI_TXCTL_DEASSERT_SSEL     (1 << 16)				/** Deassert SSEL0 pin */
#define SPI_TXCTL_DEASSERTNUM_SSEL(n)   (1 << ((n) + 16))	/** Deassert SSELN pin */
#define SPI_TXDATCTL_DEASSERT_ALL   (0xF << 16)				/** Deassert all SSEL pins */
#define SPI_TXCTL_EOT               (1 << 20)				/** End of Transfer flag (TRANSFER_DELAY is applied after sending the current frame) */
#define SPI_TXCTL_EOF               (1 << 21)				/** End of Frame flag (FRAME_DELAY is applied after sending the current part) */
#define SPI_TXCTL_RXIGNORE          (1 << 22)				/** Receive Ignore Flag */
#define SPI_TXCTL_FLEN(n)           ((((n) - 1) & 0x0F) << 24)	/** Frame length, 0 - 16 */
#define SPI_TXCTL_FLENMASK          (0xF << 24)				/** Frame length mask */

/**
 * Macro defines for SPI Divider register
 */
#define SPI_DIV_VAL(n)          ((n) & 0xFFFF)				/** Rate divider value mask (In Master Mode only)*/

/**
 * Macro defines for SPI Interrupt Status register
 */
#define SPI_INTSTAT_BITMASK         (0x3F)					/** SPI INTSTAT Register Bitmask */
#define SPI_INTSTAT_RXRDY           (1 << 0)				/** Receiver Ready Flag */
#define SPI_INTSTAT_TXRDY           (1 << 1)				/** Transmitter Ready Flag */
#define SPI_INTSTAT_RXOV            (1 << 2)				/** Receiver Overrun interrupt flag */
#define SPI_INTSTAT_TXUR            (1 << 3)				/** Transmitter Underrun interrupt flag (In Slave Mode only) */
#define SPI_INTSTAT_SSA             (1 << 4)				/** Slave Select Assert */
#define SPI_INTSTAT_SSD             (1 << 5)				/** Slave Select Deassert */

/** @brief SPI Clock Mode*/
typedef enum {
	ROM_SPI_CLOCK_CPHA0_CPOL0 = 0,						/**< CPHA = 0, CPOL = 0 */
	ROM_SPI_CLOCK_MODE0 = ROM_SPI_CLOCK_CPHA0_CPOL0,	/**< Alias for CPHA = 0, CPOL = 0 */
	ROM_SPI_CLOCK_CPHA1_CPOL0 = 1,						/**< CPHA = 0, CPOL = 1 */
	ROM_SPI_CLOCK_MODE1 = ROM_SPI_CLOCK_CPHA1_CPOL0,	/**< Alias for CPHA = 0, CPOL = 1 */
	ROM_SPI_CLOCK_CPHA0_CPOL1 = 2,						/**< CPHA = 1, CPOL = 0 */
	ROM_SPI_CLOCK_MODE2 = ROM_SPI_CLOCK_CPHA0_CPOL1,	/**< Alias for CPHA = 1, CPOL = 0 */
	ROM_SPI_CLOCK_CPHA1_CPOL1 = 3,						/**< CPHA = 1, CPOL = 1 */
	ROM_SPI_CLOCK_MODE3 = ROM_SPI_CLOCK_CPHA1_CPOL1,	/**< Alias for CPHA = 1, CPOL = 1 */
} ROM_SPI_CLOCK_MODE_T;

/**
 * Macro defines for SPI Configuration register
 */
#define SPI_CFG_BITMASK         (0xFBD)						/** SPI register bit mask */
#define SPI_CFG_SPI_EN          (1 << 0)					/** SPI Slave Mode Select */
#define SPI_CFG_SLAVE_EN        (0 << 0)					/** SPI Master Mode Select */
#define SPI_CFG_MASTER_EN       (1 << 2)					/** SPI MSB First mode enable */
#define SPI_CFG_MSB_FIRST_EN    (0 << 3)					/** SPI LSB First mode enable */
#define SPI_CFG_LSB_FIRST_EN    (1 << 3)					/** SPI Clock Phase Select */
#define SPI_CFG_CPHA_FIRST      (0 << 4)					/** Capture data on the first edge, Change data on the following edge */
#define SPI_CFG_CPHA_SECOND     (1 << 4)					/** SPI Clock Polarity Select */
#define SPI_CFG_CPOL_LO         (0 << 5)					/** The rest state of the clock (between frames) is low. */
#define SPI_CFG_CPOL_HI         (1 << 5)					/** The rest state of the clock (between frames) is high. */
#define SPI_CFG_LBM_EN          (1 << 7)					/** SPI control 1 loopback mode enable */
#define SPI_CFG_SPOL_LO         (0 << 8)					/** SPI SSEL0 Polarity Select */
#define SPI_CFG_SPOL_HI         (1 << 8)					/** SSEL0 is active High */
#define SPI_CFG_SPOLNUM_HI(n)   (1 << ((n) + 8))			/** SSELN is active High, selects 0 - 3 */

/**
 * Macro defines for SPI Delay register
 */
#define  SPI_DLY_BITMASK            (0xFFFF)				/** SPI DLY Register Mask */
#define  SPI_DLY_PRE_DELAY(n)       (((n) & 0x0F) << 0)		/** Time in SPI clocks between SSEL assertion and the beginning of a data frame */
#define  SPI_DLY_POST_DELAY(n)      (((n) & 0x0F) << 4)		/** Time in SPI clocks between the end of a data frame and SSEL deassertion. */
#define  SPI_DLY_FRAME_DELAY(n)     (((n) & 0x0F) << 8)		/** Minimum time in SPI clocks between adjacent data frames. */
#define  SPI_DLY_TRANSFER_DELAY(n)  (((n) & 0x0F) << 12)	/** Minimum time in SPI clocks that the SSEL is deasserted between transfers. */

/**
 * Macro defines for SPI Status register
 */
#define SPI_STAT_BITMASK            (0x1FF)					/** SPI STAT Register BitMask */
#define SPI_STAT_RXRDY              (1 << 0)				/** Receiver Ready Flag */
#define SPI_STAT_TXRDY              (1 << 1)				/** Transmitter Ready Flag */
#define SPI_STAT_RXOV               (1 << 2)				/** Receiver Overrun interrupt flag */
#define SPI_STAT_TXUR               (1 << 3)				/** Transmitter Underrun interrupt flag (In Slave Mode only) */
#define SPI_STAT_SSA                (1 << 4)				/** Slave Select Assert */
#define SPI_STAT_SSD                (1 << 5)				/** Slave Select Deassert */
#define SPI_STAT_STALLED            (1 << 6)				/** Stalled status flag */
#define SPI_STAT_EOT                (1 << 7)				/** End Transfer flag */
#define SPI_STAT_MSTIDLE            (1 << 8)				/** Idle status flag */

/**
 * Macro defines for SPI Interrupt Enable read and Set register
 */
#define SPI_INTENSET_BITMASK        (0x3F)					/** SPI INTENSET Register BitMask */
#define SPI_INTENSET_RXDYEN         (1 << 0)				/** Enable Interrupt when receiver data is available */
#define SPI_INTENSET_TXDYEN         (1 << 1)				/** Enable Interrupt when the transmitter holding register is available. */
#define SPI_INTENSET_RXOVEN         (1 << 2)				/**  Enable Interrupt when a receiver overrun occurs */
#define SPI_INTENSET_TXUREN         (1 << 3)				/**  Enable Interrupt when a transmitter underrun occurs (In Slave Mode Only)*/
#define SPI_INTENSET_SSAEN          (1 << 4)				/**  Enable Interrupt when the Slave Select is asserted.*/
#define SPI_INTENSET_SSDEN          (1 << 5)				/**  Enable Interrupt when the Slave Select is deasserted..*/

/**
 * Macro defines for SPI Interrupt Enable Clear register
 */
#define SPI_INTENCLR_BITMASK        (0x3F)					/** SPI INTENCLR Register BitMask */
#define SPI_INTENCLR_RXDYEN         (1 << 0)				/** Disable Interrupt when receiver data is available */
#define SPI_INTENCLR_TXDYEN         (1 << 1)				/** Disable Interrupt when the transmitter holding register is available. */
#define SPI_INTENCLR_RXOVEN         (1 << 2)				/** Disable Interrupt when a receiver overrun occurs */
#define SPI_INTENCLR_TXUREN         (1 << 3)				/** Disable Interrupt when a transmitter underrun occurs (In Slave Mode Only) */
#define SPI_INTENCLR_SSAEN          (1 << 4)				/** Disable Interrupt when the Slave Select is asserted. */
#define SPI_INTENCLR_SSDEN          (1 << 5)				/** Disable Interrupt when the Slave Select is deasserted.. */

/**
 * Macro defines for SPI Receiver Data register
 */
#define SPI_RXDAT_BITMASK           (0x1FFFFF)				/** SPI RXDAT Register BitMask */
#define SPI_RXDAT_DATA(n)           ((n) & 0xFFFF)			/** Receiver Data  */
#define SPI_RXDAT_RXSSELN_ACTIVE    (0 << 16)				/** The state of SSEL pin is active */
#define SPI_RXDAT_RXSSELN_INACTIVE  ((1 << 16)				/** The state of SSEL pin is inactive */
#define SPI_RXDAT_RXSSELNUM_INACTIVE(n) (1 << ((n) + 16))	/** The state of SSELN pin is inactive */
#define SPI_RXDAT_SOT               (1 << 20)				/** Start of Transfer flag  */

/**
 * Macro defines for SPI Transmitter Data and Control register
 */
#define SPI_TXDATCTL_BITMASK        (0xF7FFFFF)				/** SPI TXDATCTL Register BitMask */
#define SPI_TXDATCTL_DATA(n)        ((n) & 0xFFFF)			/** SPI Transmit Data */
#define SPI_TXDATCTL_CTRLMASK       (0xF7F0000)				/** SPI TXDATCTL Register BitMask for control bits only */
#define SPI_TXDATCTL_ASSERT_SSEL    (0 << 16)				/** Assert SSEL0 pin */
#define SPI_TXDATCTL_DEASSERT_SSEL  (1 << 16)				/** Deassert SSEL0 pin */
#define SPI_TXDATCTL_DEASSERTNUM_SSEL(n)    (1 << ((n) + 16))	/** Deassert SSELN pin */
#define SPI_TXDATCTL_DEASSERT_ALL   (0xF << 16)				/** Deassert all SSEL pins */
#define SPI_TXDATCTL_EOT            (1 << 20)				/** End of Transfer flag (TRANSFER_DELAY is applied after sending the current frame) */
#define SPI_TXDATCTL_EOF            (1 << 21)				/** End of Frame flag (FRAME_DELAY is applied after sending the current part) */
#define SPI_TXDATCTL_RXIGNORE       (1 << 22)				/** Receive Ignore Flag */
#define SPI_TXDATCTL_FLEN(n)        (((n) & 0x0F) << 24)	/** Frame length - 1 */

/**
 * Macro defines for SPI Transmitter Data Register
 */
#define SPI_TXDAT_DATA(n)           ((n) & 0xFFFF)			/** SPI Transmit Data */

/**
 * Macro defines for SPI Transmitter Control register
 */
#define SPI_TXCTL_BITMASK           (0xF7F0000)				/** SPI TXDATCTL Register BitMask */
#define SPI_TXCTL_ASSERT_SSEL       (0 << 16)				/** Assert SSEL0 pin */
#define SPI_TXCTL_DEASSERT_SSEL     (1 << 16)				/** Deassert SSEL0 pin */
#define SPI_TXCTL_DEASSERTNUM_SSEL(n)   (1 << ((n) + 16))	/** Deassert SSELN pin */
#define SPI_TXDATCTL_DEASSERT_ALL   (0xF << 16)				/** Deassert all SSEL pins */
#define SPI_TXCTL_EOT               (1 << 20)				/** End of Transfer flag (TRANSFER_DELAY is applied after sending the current frame) */
#define SPI_TXCTL_EOF               (1 << 21)				/** End of Frame flag (FRAME_DELAY is applied after sending the current part) */
#define SPI_TXCTL_RXIGNORE          (1 << 22)				/** Receive Ignore Flag */
#define SPI_TXCTL_FLEN(n)           ((((n) - 1) & 0x0F) << 24)	/** Frame length, 0 - 16 */
#define SPI_TXCTL_FLENMASK          (0xF << 24)				/** Frame length mask */

/**
 * Macro defines for SPI Divider register
 */
#define SPI_DIV_VAL(n)          ((n) & 0xFFFF)				/** Rate divider value mask (In Master Mode only)*/

/**
 * Macro defines for SPI Interrupt Status register
 */
#define SPI_INTSTAT_BITMASK         (0x3F)					/** SPI INTSTAT Register Bitmask */
#define SPI_INTSTAT_RXRDY           (1 << 0)				/** Receiver Ready Flag */
#define SPI_INTSTAT_TXRDY           (1 << 1)				/** Transmitter Ready Flag */
#define SPI_INTSTAT_RXOV            (1 << 2)				/** Receiver Overrun interrupt flag */
#define SPI_INTSTAT_TXUR            (1 << 3)				/** Transmitter Underrun interrupt flag (In Slave Mode only) */
#define SPI_INTSTAT_SSA             (1 << 4)				/** Slave Select Assert */
#define SPI_INTSTAT_SSD             (1 << 5)				/** Slave Select Deassert */

/**
 * @}
 */

/** @defgroup I2C_5410X CHIP: LPC5410x I2C driver
 * @ingroup CHIP_5410X_DRIVERS
  * @ i2c_common_5410x.h
 * @{
 */

/**
 * @brief I2C register block structure
 */
typedef struct {					/* I2C0 Structure         */
	__IO uint32_t CFG;				/*!< I2C Configuration Register common for Master, Slave and Monitor */
	__IO uint32_t STAT;				/*!< I2C Status Register common for Master, Slave and Monitor */
	__IO uint32_t INTENSET;			/*!< I2C Interrupt Enable Set Register common for Master, Slave and Monitor */
	__O  uint32_t INTENCLR;			/*!< I2C Interrupt Enable Clear Register common for Master, Slave and Monitor */
	__IO uint32_t TIMEOUT;			/*!< I2C Timeout value Register */
	__IO uint32_t CLKDIV;			/*!< I2C Clock Divider Register */
	__IO uint32_t INTSTAT;			/*!< I2C Interrupt Status Register */
	__I  uint32_t RESERVED0;
	__IO uint32_t MSTCTL;			/*!< I2C Master Control Register */
	__IO uint32_t MSTTIME;			/*!< I2C Master Time Register for SCL */
	__IO uint32_t MSTDAT;			/*!< I2C Master Data Register */
	__I  uint32_t RESERVED1[5];
	__IO uint32_t SLVCTL;			/*!< I2C Slave Control Register */
	__IO uint32_t SLVDAT;			/*!< I2C Slave Data Register */
	__IO uint32_t SLVADR[4];		/*!< I2C Slave Address Registers */
	__IO uint32_t SLVQUAL0;			/*!< I2C Slave Address Qualifier 0 Register */
	__I  uint32_t RESERVED2[9];
	__IO uint32_t MONRXDAT;			/*!< I2C Monitor Data Register */
} LPC_I2C_T;

/*
 * @brief I2C Configuration register Bit definition
 */
#define I2C_CFG_MSTEN             (1 << 0)			/*!< Master Enable/Disable Bit */
#define I2C_CFG_SLVEN             (1 << 1)			/*!< Slave Enable/Disable Bit */
#define I2C_CFG_MONEN             (1 << 2)			/*!< Monitor Enable/Disable Bit */
#define I2C_CFG_TIMEOUTEN         (1 << 3)			/*!< Timeout Enable/Disable Bit */
#define I2C_CFG_MONCLKSTR         (1 << 4)			/*!< Monitor Clock Stretching Bit */
#define I2C_CFG_MASK              ((uint32_t) 0x1F)	/*!< Configuration Register Mask */

/*
 * @brief I2C Status register Bit definition
 */
#define I2C_STAT_MSTPENDING       (1 << 0)		/*!< Master Pending Status Bit */
#define I2C_STAT_MSTSTATE         (0x7 << 1)	/*!< Master State Code */
#define I2C_STAT_MSTRARBLOSS      (1 << 4)		/*!< Master Arbitration Loss Bit */
#define I2C_STAT_MSTSTSTPERR      (1 << 6)		/*!< Master Start Stop Error Bit */
#define I2C_STAT_SLVPENDING       (1 << 8)		/*!< Slave Pending Status Bit */
#define I2C_STAT_SLVSTATE         (0x3 << 9)	/*!< Slave State Code */
#define I2C_STAT_SLVNOTSTR        (1 << 11)		/*!< Slave not stretching Clock Bit */
#define I2C_STAT_SLVIDX           (0x3 << 12)	/*!< Slave Address Index */
#define I2C_STAT_SLVSEL           (1 << 14)		/*!< Slave Selected Bit */
#define I2C_STAT_SLVDESEL         (1 << 15)		/*!< Slave Deselect Bit */
#define I2C_STAT_MONRDY           (1 << 16)		/*!< Monitor Ready Bit */
#define I2C_STAT_MONOV            (1 << 17)		/*!< Monitor Overflow Flag */
#define I2C_STAT_MONACTIVE        (1 << 18)		/*!< Monitor Active Flag */
#define I2C_STAT_MONIDLE          (1 << 19)		/*!< Monitor Idle Flag */
#define I2C_STAT_EVENTTIMEOUT     (1 << 24)		/*!< Event Timeout Interrupt Flag */
#define I2C_STAT_SCLTIMEOUT       (1 << 25)		/*!< SCL Timeout Interrupt Flag */

#define I2C_STAT_MSTCODE_IDLE       (0)			/*!< Master Idle State Code */
#define I2C_STAT_MSTCODE_RXREADY    (1)			/*!< Master Receive Ready State Code */
#define I2C_STAT_MSTCODE_TXREADY    (2)			/*!< Master Transmit Ready State Code */
#define I2C_STAT_MSTCODE_NACKADR    (3)			/*!< Master NACK by slave on address State Code */
#define I2C_STAT_MSTCODE_NACKDAT    (4)			/*!< Master NACK by slave on data State Code */

#define I2C_STAT_SLVCODE_ADDR         (0)		/*!< Master Idle State Code */
#define I2C_STAT_SLVCODE_RX           (1)		/*!< Received data is available Code */
#define I2C_STAT_SLVCODE_TX           (2)		/*!< Data can be transmitted Code */

/*
 * @brief I2C Interrupt Enable Set register Bit definition
 */
#define I2C_INTENSET_MSTPENDING       (1 << 0)		/*!< Master Pending Interrupt Enable Bit */
#define I2C_INTENSET_MSTRARBLOSS      (1 << 4)		/*!< Master Arbitration Loss Interrupt Enable Bit */
#define I2C_INTENSET_MSTSTSTPERR      (1 << 6)		/*!< Master Start Stop Error Interrupt Enable Bit */
#define I2C_INTENSET_SLVPENDING       (1 << 8)		/*!< Slave Pending Interrupt Enable Bit */
#define I2C_INTENSET_SLVNOTSTR        (1 << 11)		/*!< Slave not stretching Clock Interrupt Enable Bit */
#define I2C_INTENSET_SLVDESEL         (1 << 15)		/*!< Slave Deselect Interrupt Enable Bit */
#define I2C_INTENSET_MONRDY           (1 << 16)		/*!< Monitor Ready Interrupt Enable Bit */
#define I2C_INTENSET_MONOV            (1 << 17)		/*!< Monitor Overflow Interrupt Enable Bit */
#define I2C_INTENSET_MONIDLE          (1 << 19)		/*!< Monitor Idle Interrupt Enable Bit */
#define I2C_INTENSET_EVENTTIMEOUT     (1 << 24)		/*!< Event Timeout Interrupt Enable Bit */
#define I2C_INTENSET_SCLTIMEOUT       (1 << 25)		/*!< SCL Timeout Interrupt Enable Bit */

/*
 * @brief I2C Interrupt Enable Clear register Bit definition
 */
#define I2C_INTENCLR_MSTPENDING       (1 << 0)		/*!< Master Pending Interrupt Clear Bit */
#define I2C_INTENCLR_MSTRARBLOSS      (1 << 4)		/*!< Master Arbitration Loss Interrupt Clear Bit */
#define I2C_INTENCLR_MSTSTSTPERR      (1 << 6)		/*!< Master Start Stop Error Interrupt Clear Bit */
#define I2C_INTENCLR_SLVPENDING       (1 << 8)		/*!< Slave Pending Interrupt Clear Bit */
#define I2C_INTENCLR_SLVNOTSTR        (1 << 11)		/*!< Slave not stretching Clock Interrupt Clear Bit */
#define I2C_INTENCLR_SLVDESEL         (1 << 15)		/*!< Slave Deselect Interrupt Clear Bit */
#define I2C_INTENCLR_MONRDY           (1 << 16)		/*!< Monitor Ready Interrupt Clear Bit */
#define I2C_INTENCLR_MONOV            (1 << 17)		/*!< Monitor Overflow Interrupt Clear Bit */
#define I2C_INTENCLR_MONIDLE          (1 << 19)		/*!< Monitor Idle Interrupt Clear Bit */
#define I2C_INTENCLR_EVENTTIMEOUT     (1 << 24)		/*!< Event Timeout Interrupt Clear Bit */
#define I2C_INTENCLR_SCLTIMEOUT       (1 << 25)		/*!< SCL Timeout Interrupt Clear Bit */

/*
 * @brief I2C TimeOut Value Macro
 */
#define I2C_TIMEOUT_VAL(n)              (((uint32_t) ((n) - 1) & 0xFFF0) | 0x000F)		/*!< Macro for Timeout value register */

/*
 * @brief I2C Interrupt Status register Bit definition
 */
#define I2C_INTSTAT_MSTPENDING      (1 << 0)		/*!< Master Pending Interrupt Status Bit */
#define I2C_INTSTAT_MSTRARBLOSS     (1 << 4)		/*!< Master Arbitration Loss Interrupt Status Bit */
#define I2C_INTSTAT_MSTSTSTPERR     (1 << 6)		/*!< Master Start Stop Error Interrupt Status Bit */
#define I2C_INTSTAT_SLVPENDING      (1 << 8)		/*!< Slave Pending Interrupt Status Bit */
#define I2C_INTSTAT_SLVNOTSTR       (1 << 11)		/*!< Slave not stretching Clock Interrupt Status Bit */
#define I2C_INTSTAT_SLVDESEL        (1 << 15)		/*!< Slave Deselect Interrupt Status Bit */
#define I2C_INTSTAT_MONRDY          (1 << 16)		/*!< Monitor Ready Interrupt Status Bit */
#define I2C_INTSTAT_MONOV           (1 << 17)		/*!< Monitor Overflow Interrupt Status Bit */
#define I2C_INTSTAT_MONIDLE         (1 << 19)		/*!< Monitor Idle Interrupt Status Bit */
#define I2C_INTSTAT_EVENTTIMEOUT    (1 << 24)		/*!< Event Timeout Interrupt Status Bit */
#define I2C_INTSTAT_SCLTIMEOUT      (1 << 25)		/*!< SCL Timeout Interrupt Status Bit */

/*
 * @brief I2C Master Control register Bit definition
 */
#define I2C_MSTCTL_MSTCONTINUE  (1 << 0)		/*!< Master Continue Bit */
#define I2C_MSTCTL_MSTSTART     (1 << 1)		/*!< Master Start Control Bit */
#define I2C_MSTCTL_MSTSTOP      (1 << 2)		/*!< Master Stop Control Bit */
#define I2C_MSTCTL_MSTDMA       (1 << 3)		/*!< Master DMA Enable Bit */

/*
 * @brief I2C Master Time Register Field definition
 */
#define I2C_MSTTIME_MSTSCLLOW   (0x07 << 0)		/*!< Master SCL Low Time field */
#define I2C_MSTTIME_MSTSCLHIGH  (0x07 << 4)		/*!< Master SCL High Time field */

/*
 * @brief I2C Master Data Mask
 */
#define I2C_MSTDAT_DATAMASK         ((uint32_t) 0x00FF << 0)	/*!< Master data mask */

/*
 * @brief I2C Slave Control register Bit definition
 */
#define I2C_SLVCTL_SLVCONTINUE    (1 << 0)		/*!< Slave Continue Bit */
#define I2C_SLVCTL_SLVNACK        (1 << 1)		/*!< Slave NACK Bit */
#define I2C_SLVCTL_SLVDMA         (1 << 3)		/*!< Slave DMA Enable Bit */

/*
 * @brief I2C Slave Data Mask
 */
#define I2C_SLVDAT_DATAMASK         ((uint32_t) 0x00FF << 0)	/*!< Slave data mask */

/*
 * @brief I2C Slave Address register Bit definition
 */
#define I2C_SLVADR_SADISABLE      (1 << 0)		/*!< Slave Address n Disable Bit */
#define I2C_SLVADR_SLVADR         (0x7F << 1)	/*!< Slave Address field */
#define I2C_SLVADR_MASK           ((uint32_t) 0x00FF)	/*!< Slave Address Mask */

/*
 * @brief I2C Slave Address Qualifier 0 Register Bit definition
 */
#define I2C_SLVQUAL_QUALMODE0     (1 << 0)		/*!< Slave Qualifier Mode Enable Bit */
#define I2C_SLVQUAL_SLVQUAL0      (0x7F << 1)	/*!< Slave Qualifier Address for Address 0 */

/*
 * @brief I2C Monitor Data Register Bit definition
 */
#define I2C_MONRXDAT_DATA         (0xFF << 0)		/*!< Monitor Function Receive Data Field */
#define I2C_MONRXDAT_MONSTART     (1 << 8)			/*!< Monitor Received Start Bit */
#define I2C_MONRXDAT_MONRESTART   (1 << 9)			/*!< Monitor Received Repeated Start Bit */
#define I2C_MONRXDAT_MONNACK      (1 << 10)			/*!< Monitor Received Nack Bit */

/*
 * @brief I2C Configuration register Bit definition
 */
#define I2C_CFG_MSTEN             (1 << 0)	/*!< Master Enable/Disable Bit */
#define I2C_CFG_SLVEN             (1 << 1)	/*!< Slave Enable/Disable Bit */
#define I2C_CFG_MONEN             (1 << 2)	/*!< Monitor Enable/Disable Bit */
#define I2C_CFG_TIMEOUTEN         (1 << 3)	/*!< Timeout Enable/Disable Bit */
#define I2C_CFG_MONCLKSTR         (1 << 4)	/*!< Monitor Clock Stretching Bit */
#define I2C_CFG_MASK              ((uint32_t) 0x1F)	/*!< Configuration Register Mask */

/*
 * @brief I2C Status register Bit definition
 */
#define I2C_STAT_MSTPENDING       (1 << 0)		/*!< Master Pending Status Bit */
#define I2C_STAT_MSTSTATE         (0x7 << 1)	/*!< Master State Code */
#define I2C_STAT_MSTRARBLOSS      (1 << 4)		/*!< Master Arbitration Loss Bit */
#define I2C_STAT_MSTSTSTPERR      (1 << 6)		/*!< Master Start Stop Error Bit */
#define I2C_STAT_SLVPENDING       (1 << 8)		/*!< Slave Pending Status Bit */
#define I2C_STAT_SLVSTATE         (0x3 << 9)	/*!< Slave State Code */
#define I2C_STAT_SLVNOTSTR        (1 << 11)		/*!< Slave not stretching Clock Bit */
#define I2C_STAT_SLVIDX           (0x3 << 12)	/*!< Slave Address Index */
#define I2C_STAT_SLVSEL           (1 << 14)		/*!< Slave Selected Bit */
#define I2C_STAT_SLVDESEL         (1 << 15)		/*!< Slave Deselect Bit */
#define I2C_STAT_MONRDY           (1 << 16)		/*!< Monitor Ready Bit */
#define I2C_STAT_MONOV            (1 << 17)		/*!< Monitor Overflow Flag */
#define I2C_STAT_MONACTIVE        (1 << 18)		/*!< Monitor Active Flag */
#define I2C_STAT_MONIDLE          (1 << 19)		/*!< Monitor Idle Flag */
#define I2C_STAT_EVENTTIMEOUT     (1 << 24)		/*!< Event Timeout Interrupt Flag */
#define I2C_STAT_SCLTIMEOUT       (1 << 25)		/*!< SCL Timeout Interrupt Flag */

#define I2C_STAT_MSTCODE_IDLE       (0)			/*!< Master Idle State Code */
#define I2C_STAT_MSTCODE_RXREADY    (1)			/*!< Master Receive Ready State Code */
#define I2C_STAT_MSTCODE_TXREADY    (2)			/*!< Master Transmit Ready State Code */
#define I2C_STAT_MSTCODE_NACKADR    (3)			/*!< Master NACK by slave on address State Code */
#define I2C_STAT_MSTCODE_NACKDAT    (4)			/*!< Master NACK by slave on data State Code */

#define I2C_STAT_SLVCODE_ADDR         (0)		/*!< Master Idle State Code */
#define I2C_STAT_SLVCODE_RX           (1)		/*!< Received data is available Code */
#define I2C_STAT_SLVCODE_TX           (2)		/*!< Data can be transmitted Code */

/*
 * @brief I2C Interrupt Enable Set register Bit definition
 */
#define I2C_INTENSET_MSTPENDING       (1 << 0)		/*!< Master Pending Interrupt Enable Bit */
#define I2C_INTENSET_MSTRARBLOSS      (1 << 4)		/*!< Master Arbitration Loss Interrupt Enable Bit */
#define I2C_INTENSET_MSTSTSTPERR      (1 << 6)		/*!< Master Start Stop Error Interrupt Enable Bit */
#define I2C_INTENSET_SLVPENDING       (1 << 8)		/*!< Slave Pending Interrupt Enable Bit */
#define I2C_INTENSET_SLVNOTSTR        (1 << 11)		/*!< Slave not stretching Clock Interrupt Enable Bit */
#define I2C_INTENSET_SLVDESEL         (1 << 15)		/*!< Slave Deselect Interrupt Enable Bit */
#define I2C_INTENSET_MONRDY           (1 << 16)		/*!< Monitor Ready Interrupt Enable Bit */
#define I2C_INTENSET_MONOV            (1 << 17)		/*!< Monitor Overflow Interrupt Enable Bit */
#define I2C_INTENSET_MONIDLE          (1 << 19)		/*!< Monitor Idle Interrupt Enable Bit */
#define I2C_INTENSET_EVENTTIMEOUT     (1 << 24)		/*!< Event Timeout Interrupt Enable Bit */
#define I2C_INTENSET_SCLTIMEOUT       (1 << 25)		/*!< SCL Timeout Interrupt Enable Bit */

/*
 * @brief I2C Interrupt Enable Clear register Bit definition
 */
#define I2C_INTENCLR_MSTPENDING       (1 << 0)		/*!< Master Pending Interrupt Clear Bit */
#define I2C_INTENCLR_MSTRARBLOSS      (1 << 4)		/*!< Master Arbitration Loss Interrupt Clear Bit */
#define I2C_INTENCLR_MSTSTSTPERR      (1 << 6)		/*!< Master Start Stop Error Interrupt Clear Bit */
#define I2C_INTENCLR_SLVPENDING       (1 << 8)		/*!< Slave Pending Interrupt Clear Bit */
#define I2C_INTENCLR_SLVNOTSTR        (1 << 11)		/*!< Slave not stretching Clock Interrupt Clear Bit */
#define I2C_INTENCLR_SLVDESEL         (1 << 15)		/*!< Slave Deselect Interrupt Clear Bit */
#define I2C_INTENCLR_MONRDY           (1 << 16)		/*!< Monitor Ready Interrupt Clear Bit */
#define I2C_INTENCLR_MONOV            (1 << 17)		/*!< Monitor Overflow Interrupt Clear Bit */
#define I2C_INTENCLR_MONIDLE          (1 << 19)		/*!< Monitor Idle Interrupt Clear Bit */
#define I2C_INTENCLR_EVENTTIMEOUT     (1 << 24)		/*!< Event Timeout Interrupt Clear Bit */
#define I2C_INTENCLR_SCLTIMEOUT       (1 << 25)		/*!< SCL Timeout Interrupt Clear Bit */

/*
 * @brief I2C TimeOut Value Macro
 */
#define I2C_TIMEOUT_VAL(n)              (((uint32_t) ((n) - 1) & 0xFFF0) | 0x000F)		/*!< Macro for Timeout value register */

/*
 * @brief I2C Interrupt Status register Bit definition
 */
#define I2C_INTSTAT_MSTPENDING      (1 << 0)		/*!< Master Pending Interrupt Status Bit */
#define I2C_INTSTAT_MSTRARBLOSS     (1 << 4)		/*!< Master Arbitration Loss Interrupt Status Bit */
#define I2C_INTSTAT_MSTSTSTPERR     (1 << 6)		/*!< Master Start Stop Error Interrupt Status Bit */
#define I2C_INTSTAT_SLVPENDING      (1 << 8)		/*!< Slave Pending Interrupt Status Bit */
#define I2C_INTSTAT_SLVNOTSTR       (1 << 11)		/*!< Slave not stretching Clock Interrupt Status Bit */
#define I2C_INTSTAT_SLVDESEL        (1 << 15)		/*!< Slave Deselect Interrupt Status Bit */
#define I2C_INTSTAT_MONRDY          (1 << 16)		/*!< Monitor Ready Interrupt Status Bit */
#define I2C_INTSTAT_MONOV           (1 << 17)		/*!< Monitor Overflow Interrupt Status Bit */
#define I2C_INTSTAT_MONIDLE         (1 << 19)		/*!< Monitor Idle Interrupt Status Bit */
#define I2C_INTSTAT_EVENTTIMEOUT    (1 << 24)		/*!< Event Timeout Interrupt Status Bit */
#define I2C_INTSTAT_SCLTIMEOUT      (1 << 25)		/*!< SCL Timeout Interrupt Status Bit */

/*
 * @brief I2C Master Control register Bit definition
 */
#define I2C_MSTCTL_MSTCONTINUE  (1 << 0)		/*!< Master Continue Bit */
#define I2C_MSTCTL_MSTSTART     (1 << 1)		/*!< Master Start Control Bit */
#define I2C_MSTCTL_MSTSTOP      (1 << 2)		/*!< Master Stop Control Bit */
#define I2C_MSTCTL_MSTDMA       (1 << 3)		/*!< Master DMA Enable Bit */

/*
 * @brief I2C Master Time Register Field definition
 */
#define I2C_MSTTIME_MSTSCLLOW   (0x07 << 0)		/*!< Master SCL Low Time field */
#define I2C_MSTTIME_MSTSCLHIGH  (0x07 << 4)		/*!< Master SCL High Time field */

/*
 * @brief I2C Master Data Mask
 */
#define I2C_MSTDAT_DATAMASK         ((uint32_t) 0x00FF << 0)	/*!< Master data mask */

/*
 * @brief I2C Slave Control register Bit definition
 */
#define I2C_SLVCTL_SLVCONTINUE    (1 << 0)		/*!< Slave Continue Bit */
#define I2C_SLVCTL_SLVNACK        (1 << 1)		/*!< Slave NACK Bit */
#define I2C_SLVCTL_SLVDMA         (1 << 3)		/*!< Slave DMA Enable Bit */

/*
 * @brief I2C Slave Data Mask
 */
#define I2C_SLVDAT_DATAMASK         ((uint32_t) 0x00FF << 0)	/*!< Slave data mask */

/*
 * @brief I2C Slave Address register Bit definition
 */
#define I2C_SLVADR_SADISABLE      (1 << 0)		/*!< Slave Address n Disable Bit */
#define I2C_SLVADR_SLVADR         (0x7F << 1)	/*!< Slave Address field */
#define I2C_SLVADR_MASK           ((uint32_t) 0x00FF)	/*!< Slave Address Mask */

/*
 * @brief I2C Slave Address Qualifier 0 Register Bit definition
 */
#define I2C_SLVQUAL_QUALMODE0     (1 << 0)		/*!< Slave Qualifier Mode Enable Bit */
#define I2C_SLVQUAL_SLVQUAL0      (0x7F << 1)	/*!< Slave Qualifier Address for Address 0 */

/*
 * @brief I2C Monitor Data Register Bit definition
 */
#define I2C_MONRXDAT_DATA         (0xFF << 0)		/*!< Monitor Function Receive Data Field */
#define I2C_MONRXDAT_MONSTART     (1 << 8)			/*!< Monitor Received Start Bit */
#define I2C_MONRXDAT_MONRESTART   (1 << 9)			/*!< Monitor Received Repeated Start Bit */
#define I2C_MONRXDAT_MONNACK      (1 << 10)			/*!< Monitor Received Nack Bit */

/**
 * @}
 */



/** @defgroup PERIPH_5410X_BASE CHIP: LPC5410X Peripheral addresses and register set declarations
 * @ingroup CHIP_5410X_DRIVERS
 * @{
 */

/* Main memory addresses */
#define LPC_FLASHMEM_BASE          0x00000000UL
#define LPC_SRAM0_BASE             0x02000000UL
#define LPC_SRAM1_BASE             0x02010000UL
#define LPC_ROM_BASE               0x03000000UL
#define LPC_SRAM2_BASE             0x03400000UL
#define LPC_GPIO_PORT_BASE         0x1C000000UL
#define LPC_DMA_BASE               0x1C004000UL
#define LPC_CRC_BASE               0x1C010000UL
#define LPC_SCT_BASE               0x1C018000UL
#define LPC_MBOX_BASE              0x1C02C000UL
#define LPC_ADC_BASE               0x1C034000UL
#define LPC_FIFO_BASE              0x1C038000UL

/* APB0 peripheral group addresses */
#define LPC_SYSCON_BASE            0x40000000UL
#define LPC_TIMER2_BASE            0x40004000UL
#define LPC_TIMER3_BASE            0x40008000UL
#define LPC_TIMER4_BASE            0x4000C000UL
#define LPC_GPIO_GROUPINT0_BASE    0x40010000UL
#define LPC_GPIO_GROUPINT1_BASE    0x40014000UL
#define LPC_PIN_INT_BASE           0x40018000UL
#define LPC_IOCON_BASE             0x4001C000UL
#define LPC_UTICK_BASE             0x40020000UL
#define LPC_FMC_BASE               0x40024000UL
#define LPC_PMU_BASE               0x4002C000UL
#define LPC_WWDT_BASE              0x40038000UL
#define LPC_RTC_BASE               0x4003C000UL

/* APB1 peripheral group addresses */
#define LPC_ASYNC_SYSCON_BASE      0x40080000UL
#define LPC_USART0_BASE            0x40084000UL
#define LPC_USART1_BASE            0x40088000UL
#define LPC_USART2_BASE            0x4008C000UL
#define LPC_USART3_BASE            0x40090000UL
#define LPC_I2C0_BASE              0x40094000UL
#define LPC_I2C1_BASE              0x40098000UL
#define LPC_I2C2_BASE              0x4009C000UL
#define LPC_SPI0_BASE              0x400A4000UL
#define LPC_SPI1_BASE              0x400A8000UL
#define LPC_TIMER0_BASE            0x400B4000UL
#define LPC_TIMER1_BASE            0x400B8000UL
#define LPC_INMUX_BASE             0x40050000UL
#define LPC_RITIMER_BASE           0x40070000UL
#define LPC_MRT_BASE               0x40074000UL

/* Main memory register access */
#define LPC_GPIO           ((LPC_GPIO_T            *) LPC_GPIO_PORT_BASE)
#define LPC_DMA            ((LPC_DMA_T             *) LPC_DMA_BASE)
#define LPC_CRC            ((LPC_CRC_T             *) LPC_CRC_BASE)
#define LPC_SCT            ((LPC_SCT_T             *) LPC_SCT_BASE)
#define LPC_MBOX           ((LPC_MBOX_T            *) LPC_MBOX_BASE)
#define LPC_ADC            ((LPC_ADC_T             *) LPC_ADC_BASE)
#define LPC_FIFO           ((LPC_FIFO_T            *) LPC_FIFO_BASE)

/* APB0 peripheral group register access */
#define LPC_SYSCON         ((LPC_SYSCON_T          *) LPC_SYSCON_BASE)
#define LPC_TIMER2         ((LPC_TIMER_T           *) LPC_TIMER2_BASE)
#define LPC_TIMER3         ((LPC_TIMER_T           *) LPC_TIMER3_BASE)
#define LPC_TIMER4         ((LPC_TIMER_T           *) LPC_TIMER4_BASE)
#define LPC_GINT           ((LPC_GPIOGROUPINT_T    *) LPC_GPIO_GROUPINT0_BASE)
#define LPC_PININT         ((LPC_PIN_INT_T         *) LPC_PIN_INT_BASE)
#define LPC_IOCON          ((LPC_IOCON_T           *) LPC_IOCON_BASE)
#define LPC_UTICK          ((LPC_UTICK_T           *) LPC_UTICK_BASE)
#define LPC_WWDT           ((LPC_WWDT_T            *) LPC_WWDT_BASE)
#define LPC_RTC            ((LPC_RTC_T             *) LPC_RTC_BASE)

/* APB1 peripheral group register access */
#define LPC_ASYNC_SYSCON   ((LPC_ASYNC_SYSCON_T    *) LPC_ASYNC_SYSCON_BASE)
#define LPC_USART0         ((LPC_USART_T           *) LPC_USART0_BASE)
#define LPC_USART1         ((LPC_USART_T           *) LPC_USART1_BASE)
#define LPC_USART2         ((LPC_USART_T           *) LPC_USART2_BASE)
#define LPC_USART3         ((LPC_USART_T           *) LPC_USART3_BASE)
#define LPC_I2C0           ((LPC_I2C_T             *) LPC_I2C0_BASE)
#define LPC_I2C1           ((LPC_I2C_T             *) LPC_I2C1_BASE)
#define LPC_I2C2           ((LPC_I2C_T             *) LPC_I2C2_BASE)
#define LPC_SCT0           LPC_SCT
#define LPC_SPI0           ((LPC_SPI_T             *) LPC_SPI0_BASE)
#define LPC_SPI1           ((LPC_SPI_T             *) LPC_SPI1_BASE)
#define LPC_TIMER0         ((LPC_TIMER_T           *) LPC_TIMER0_BASE)
#define LPC_TIMER1         ((LPC_TIMER_T           *) LPC_TIMER1_BASE)
#define LPC_INMUX          ((LPC_INMUX_T           *) LPC_INMUX_BASE)
#define LPC_RITIMER        ((LPC_RITIMER_T         *) LPC_RITIMER_BASE)
#define LPC_MRT            ((LPC_MRT_T             *) LPC_MRT_BASE)
#define LPC_PMU            ((LPC_PMU_T             *) LPC_PMU_BASE)

/**
 * @}
 */


#ifdef __cplusplus
}
#endif

#endif /* __LPC5410X_H*/
