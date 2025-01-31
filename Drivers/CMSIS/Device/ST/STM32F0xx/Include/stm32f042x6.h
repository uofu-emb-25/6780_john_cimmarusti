/**
  ******************************************************************************
  * @file    stm32f042x6.h
  * @author  MCD Application Team
  * @brief   CMSIS Cortex-M0 Device Peripheral Access Layer Header File.
  *          This file contains all the peripheral register's definitions, bits
  *          definitions and memory mapping for STM32F0xx devices.
  *
  *          This file contains:
  *           - Data structures and the address mapping for all peripherals
  *           - Peripheral's registers declarations and bits definition
  *           - Macros to access peripheral's registers hardware
  *
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup stm32f042x6
  * @{
  */

#ifndef __STM32F042x6_H
#define __STM32F042x6_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/** @addtogroup Configuration_section_for_CMSIS
  * @{
  */
/**
 * @brief Configuration of the Cortex-M0 Processor and Core Peripherals
 */
#define __CM0_REV                 0 /*!< Core Revision r0p0                            */
#define __MPU_PRESENT             0 /*!< STM32F0xx do not provide MPU                  */
#define __NVIC_PRIO_BITS          2 /*!< STM32F0xx uses 2 Bits for the Priority Levels */
#define __Vendor_SysTickConfig    0     /*!< Set to 1 if different SysTick Config is used */

/**
  * @}
  */

/** @addtogroup Peripheral_interrupt_number_definition
  * @{
  */

/**
 * @brief STM32F0xx Interrupt Number Definition, according to the selected device
 *        in @ref Library_configuration_section
 */

/*!< Interrupt Number Definition */
typedef enum
{
/******  Cortex-M0 Processor Exceptions Numbers **************************************************************/
  NonMaskableInt_IRQn         = -14,    /*!< 2 Non Maskable Interrupt                                        */
  HardFault_IRQn              = -13,    /*!< 3 Cortex-M0 Hard Fault Interrupt                                */
  SVCall_IRQn                 = -5,     /*!< 11 Cortex-M0 SV Call Interrupt                                  */
  PendSV_IRQn                 = -2,     /*!< 14 Cortex-M0 Pend SV Interrupt                                  */
  SysTick_IRQn                = -1,     /*!< 15 Cortex-M0 System Tick Interrupt                              */

/******  STM32F0 specific Interrupt Numbers ******************************************************************/
  WWDG_IRQn                   = 0,      /*!< Window WatchDog Interrupt                               */
  PVD_VDDIO2_IRQn             = 1,      /*!< PVD & VDDIO2 Interrupt through EXTI Lines 16 and 31             */
  RTC_IRQn                    = 2,      /*!< RTC Interrupt through EXTI Lines 17, 19 and 20                  */
  FLASH_IRQn                  = 3,      /*!< FLASH global Interrupt                                          */
  RCC_CRS_IRQn                = 4,      /*!< RCC & CRS global Interrupt                                      */
  EXTI0_1_IRQn                = 5,      /*!< EXTI Line 0 and 1 Interrupt                                     */
  EXTI2_3_IRQn                = 6,      /*!< EXTI Line 2 and 3 Interrupt                                     */
  EXTI4_15_IRQn               = 7,      /*!< EXTI Line 4 to 15 Interrupt                                     */
  TSC_IRQn                    = 8,      /*!< Touch Sensing Controller Interrupts                             */
  DMA1_Channel1_IRQn          = 9,      /*!< DMA1 Channel 1 Interrupt                                        */
  DMA1_Channel2_3_IRQn        = 10,     /*!< DMA1 Channel 2 and Channel 3 Interrupt                          */
  DMA1_Channel4_5_IRQn        = 11,     /*!< DMA1 Channel 4 and Channel 5 Interrupt                          */
  ADC1_IRQn                   = 12,     /*!< ADC1 Interrupt                                                  */
  TIM1_BRK_UP_TRG_COM_IRQn    = 13,     /*!< TIM1 Break, Update, Trigger and Commutation Interrupt           */
  TIM1_CC_IRQn                = 14,     /*!< TIM1 Capture Compare Interrupt                                  */
  TIM2_IRQn                   = 15,     /*!< TIM2 global Interrupt                                           */
  TIM3_IRQn                   = 16,     /*!< TIM3 global Interrupt                                           */
  TIM14_IRQn                  = 19,     /*!< TIM14 global Interrupt                                          */
  TIM16_IRQn                  = 21,     /*!< TIM16 global Interrupt                                          */
  TIM17_IRQn                  = 22,     /*!< TIM17 global Interrupt                                          */
  I2C1_IRQn                   = 23,     /*!< I2C1 Event Interrupt & EXTI Line23 Interrupt (I2C1 wakeup)      */
  SPI1_IRQn                   = 25,     /*!< SPI1 global Interrupt                                           */
  SPI2_IRQn                   = 26,     /*!< SPI2 global Interrupt                                           */
  USART1_IRQn                 = 27,     /*!< USART1 global Interrupt & EXTI Line25 Interrupt (USART1 wakeup) */
  USART2_IRQn                 = 28,     /*!< USART2 global Interrupt                                         */
  CEC_CAN_IRQn                = 30,     /*!< CEC and CAN global Interrupts & EXTI Line27 Interrupt           */
  USB_IRQn                    = 31      /*!< USB global Interrupt  & EXTI Line18 Interrupt                   */
} IRQn_Type;

/**
  * @}
  */

#include "core_cm0.h"            /* Cortex-M0 processor and core peripherals */
#include "system_stm32f0xx.h"    /* STM32F0xx System Header */
#include <stdint.h>

/** @addtogroup Peripheral_registers_structures
  * @{
  */

/**
  * @brief Analog to Digital Converter
  */

typedef struct
{
  __IO uint32_t ISR;          /*!< ADC interrupt and status register,             Address offset: 0x00 */
  __IO uint32_t IER;          /*!< ADC interrupt enable register,                 Address offset: 0x04 */
  __IO uint32_t CR;           /*!< ADC control register,                          Address offset: 0x08 */
  __IO uint32_t CFGR1;        /*!< ADC configuration register 1,                  Address offset: 0x0C */
  __IO uint32_t CFGR2;        /*!< ADC configuration register 2,                  Address offset: 0x10 */
  __IO uint32_t SMPR;         /*!< ADC sampling time register,                    Address offset: 0x14 */
       uint32_t RESERVED1;    /*!< Reserved,                                                      0x18 */
       uint32_t RESERVED2;    /*!< Reserved,                                                      0x1C */
  __IO uint32_t TR;           /*!< ADC analog watchdog 1 threshold register,      Address offset: 0x20 */
       uint32_t RESERVED3;    /*!< Reserved,                                                      0x24 */
  __IO uint32_t CHSELR;       /*!< ADC group regular sequencer register,          Address offset: 0x28 */
       uint32_t RESERVED4[5]; /*!< Reserved,                                                      0x2C */
  __IO uint32_t DR;           /*!< ADC group regular data register,               Address offset: 0x40 */
} ADC_TypeDef;

typedef struct
{
  __IO uint32_t CCR;          /*!< ADC common configuration register,             Address offset: ADC1 base address + 0x308 */
} ADC_Common_TypeDef;

/**
  * @brief Controller Area Network TxMailBox
  */
typedef struct
{
  __IO uint32_t TIR;  /*!< CAN TX mailbox identifier register */
  __IO uint32_t TDTR; /*!< CAN mailbox data length control and time stamp register */
  __IO uint32_t TDLR; /*!< CAN mailbox data low register */
  __IO uint32_t TDHR; /*!< CAN mailbox data high register */
}CAN_TxMailBox_TypeDef;

/**
  * @brief Controller Area Network FIFOMailBox
  */
typedef struct
{
  __IO uint32_t RIR;  /*!< CAN receive FIFO mailbox identifier register */
  __IO uint32_t RDTR; /*!< CAN receive FIFO mailbox data length control and time stamp register */
  __IO uint32_t RDLR; /*!< CAN receive FIFO mailbox data low register */
  __IO uint32_t RDHR; /*!< CAN receive FIFO mailbox data high register */
}CAN_FIFOMailBox_TypeDef;

/**
  * @brief Controller Area Network FilterRegister
  */
typedef struct
{
  __IO uint32_t FR1; /*!< CAN Filter bank register 1 */
  __IO uint32_t FR2; /*!< CAN Filter bank register 1 */
}CAN_FilterRegister_TypeDef;

/**
  * @brief Controller Area Network
  */
typedef struct
{
  __IO uint32_t              MCR;                 /*!< CAN master control register,         Address offset: 0x00          */
  __IO uint32_t              MSR;                 /*!< CAN master status register,          Address offset: 0x04          */
  __IO uint32_t              TSR;                 /*!< CAN transmit status register,        Address offset: 0x08          */
  __IO uint32_t              RF0R;                /*!< CAN receive FIFO 0 register,         Address offset: 0x0C          */
  __IO uint32_t              RF1R;                /*!< CAN receive FIFO 1 register,         Address offset: 0x10          */
  __IO uint32_t              IER;                 /*!< CAN interrupt enable register,       Address offset: 0x14          */
  __IO uint32_t              ESR;                 /*!< CAN error status register,           Address offset: 0x18          */
  __IO uint32_t              BTR;                 /*!< CAN bit timing register,             Address offset: 0x1C          */
  uint32_t                   RESERVED0[88];       /*!< Reserved, 0x020 - 0x17F                                            */
  CAN_TxMailBox_TypeDef      sTxMailBox[3];       /*!< CAN Tx MailBox,                      Address offset: 0x180 - 0x1AC */
  CAN_FIFOMailBox_TypeDef    sFIFOMailBox[2];     /*!< CAN FIFO MailBox,                    Address offset: 0x1B0 - 0x1CC */
  uint32_t                   RESERVED1[12];       /*!< Reserved, 0x1D0 - 0x1FF                                            */
  __IO uint32_t              FMR;                 /*!< CAN filter master register,          Address offset: 0x200         */
  __IO uint32_t              FM1R;                /*!< CAN filter mode register,            Address offset: 0x204         */
  uint32_t                   RESERVED2;           /*!< Reserved, 0x208                                                    */
  __IO uint32_t              FS1R;                /*!< CAN filter scale register,           Address offset: 0x20C         */
  uint32_t                   RESERVED3;           /*!< Reserved, 0x210                                                    */
  __IO uint32_t              FFA1R;               /*!< CAN filter FIFO assignment register, Address offset: 0x214         */
  uint32_t                   RESERVED4;           /*!< Reserved, 0x218                                                    */
  __IO uint32_t              FA1R;                /*!< CAN filter activation register,      Address offset: 0x21C         */
  uint32_t                   RESERVED5[8];        /*!< Reserved, 0x220-0x23F                                              */
  CAN_FilterRegister_TypeDef sFilterRegister[14]; /*!< CAN Filter Register,                 Address offset: 0x240-0x2AC   */
}CAN_TypeDef;

/**
  * @brief HDMI-CEC
  */

typedef struct
{
  __IO uint32_t CR;           /*!< CEC control register,                                       Address offset:0x00 */
  __IO uint32_t CFGR;         /*!< CEC configuration register,                                 Address offset:0x04 */
  __IO uint32_t TXDR;         /*!< CEC Tx data register ,                                      Address offset:0x08 */
  __IO uint32_t RXDR;         /*!< CEC Rx Data Register,                                       Address offset:0x0C */
  __IO uint32_t ISR;          /*!< CEC Interrupt and Status Register,                          Address offset:0x10 */
  __IO uint32_t IER;          /*!< CEC interrupt enable register,                              Address offset:0x14 */
}CEC_TypeDef;

/**
  * @brief CRC calculation unit
  */

typedef struct
{
  __IO uint32_t DR;          /*!< CRC Data register,                           Address offset: 0x00 */
  __IO uint8_t  IDR;         /*!< CRC Independent data register,               Address offset: 0x04 */
  uint8_t       RESERVED0;   /*!< Reserved,                                                    0x05 */
  uint16_t      RESERVED1;   /*!< Reserved,                                                    0x06 */
  __IO uint32_t CR;          /*!< CRC Control register,                        Address offset: 0x08 */
  uint32_t      RESERVED2;   /*!< Reserved,                                                    0x0C */
  __IO uint32_t INIT;        /*!< Initial CRC value register,                  Address offset: 0x10 */
  __IO uint32_t RESERVED3;   /*!< Reserved,                                                    0x14 */
} CRC_TypeDef;

/**
  * @brief Clock Recovery System
  */
typedef struct
{
__IO uint32_t CR;     /*!< CRS ccontrol register,              Address offset: 0x00 */
__IO uint32_t CFGR;   /*!< CRS configuration register,         Address offset: 0x04 */
__IO uint32_t ISR;    /*!< CRS interrupt and status register,  Address offset: 0x08 */
__IO uint32_t ICR;    /*!< CRS interrupt flag clear register,  Address offset: 0x0C */
}CRS_TypeDef;

/**
  * @brief Debug MCU
  */

typedef struct
{
  __IO uint32_t IDCODE;       /*!< MCU device ID code,                          Address offset: 0x00 */
  __IO uint32_t CR;           /*!< Debug MCU configuration register,            Address offset: 0x04 */
  __IO uint32_t APB1FZ;       /*!< Debug MCU APB1 freeze register,              Address offset: 0x08 */
  __IO uint32_t APB2FZ;       /*!< Debug MCU APB2 freeze register,              Address offset: 0x0C */
}DBGMCU_TypeDef;

/**
  * @brief DMA Controller
  */

typedef struct
{
  __IO uint32_t CCR;          /*!< DMA channel x configuration register        */
  __IO uint32_t CNDTR;        /*!< DMA channel x number of data register       */
  __IO uint32_t CPAR;         /*!< DMA channel x peripheral address register   */
  __IO uint32_t CMAR;         /*!< DMA channel x memory address register       */
} DMA_Channel_TypeDef;

typedef struct
{
  __IO uint32_t ISR;          /*!< DMA interrupt status register,               Address offset: 0x00 */
  __IO uint32_t IFCR;         /*!< DMA interrupt flag clear register,           Address offset: 0x04 */
} DMA_TypeDef;

/**
  * @brief External Interrupt/Event Controller
  */

typedef struct
{
  __IO uint32_t IMR;          /*!<EXTI Interrupt mask register,                 Address offset: 0x00 */
  __IO uint32_t EMR;          /*!<EXTI Event mask register,                     Address offset: 0x04 */
  __IO uint32_t RTSR;         /*!<EXTI Rising trigger selection register ,      Address offset: 0x08 */
  __IO uint32_t FTSR;         /*!<EXTI Falling trigger selection register,      Address offset: 0x0C */
  __IO uint32_t SWIER;        /*!<EXTI Software interrupt event register,       Address offset: 0x10 */
  __IO uint32_t PR;           /*!<EXTI Pending register,                        Address offset: 0x14 */
} EXTI_TypeDef;

/**
  * @brief FLASH Registers
  */
typedef struct
{
  __IO uint32_t ACR;          /*!<FLASH access control register,                 Address offset: 0x00 */
  __IO uint32_t KEYR;         /*!<FLASH key register,                            Address offset: 0x04 */
  __IO uint32_t OPTKEYR;      /*!<FLASH OPT key register,                        Address offset: 0x08 */
  __IO uint32_t SR;           /*!<FLASH status register,                         Address offset: 0x0C */
  __IO uint32_t CR;           /*!<FLASH control register,                        Address offset: 0x10 */
  __IO uint32_t AR;           /*!<FLASH address register,                        Address offset: 0x14 */
  __IO uint32_t RESERVED;     /*!< Reserved,                                                     0x18 */
  __IO uint32_t OBR;          /*!<FLASH option bytes register,                   Address offset: 0x1C */
  __IO uint32_t WRPR;         /*!<FLASH option bytes register,                   Address offset: 0x20 */
} FLASH_TypeDef;

/**
  * @brief Option Bytes Registers
  */
typedef struct
{
  __IO uint16_t RDP;          /*!< FLASH option byte Read protection,             Address offset: 0x00 */
  __IO uint16_t USER;         /*!< FLASH option byte user options,                Address offset: 0x02 */
  __IO uint16_t DATA0;        /*!< User data byte 0 (stored in FLASH_OBR[23:16]), Address offset: 0x04 */
  __IO uint16_t DATA1;        /*!< User data byte 1 (stored in FLASH_OBR[31:24]), Address offset: 0x06 */
  __IO uint16_t WRP0;         /*!< FLASH option byte write protection 0,          Address offset: 0x08 */
} OB_TypeDef;

/**
  * @brief General Purpose I/O
  */

typedef struct
{
  __IO uint32_t MODER;        /*!< GPIO port mode register,                     Address offset: 0x00      */
  __IO uint32_t OTYPER;       /*!< GPIO port output type register,              Address offset: 0x04      */
  __IO uint32_t OSPEEDR;      /*!< GPIO port output speed register,             Address offset: 0x08      */
  __IO uint32_t PUPDR;        /*!< GPIO port pull-up/pull-down register,        Address offset: 0x0C      */
  __IO uint32_t IDR;          /*!< GPIO port input data register,               Address offset: 0x10      */
  __IO uint32_t ODR;          /*!< GPIO port output data register,              Address offset: 0x14      */
  __IO uint32_t BSRR;         /*!< GPIO port bit set/reset register,      Address offset: 0x1A */
  __IO uint32_t LCKR;         /*!< GPIO port configuration lock register,       Address offset: 0x1C      */
  __IO uint32_t AFR[2];       /*!< GPIO alternate function low register,  Address offset: 0x20-0x24 */
  __IO uint32_t BRR;          /*!< GPIO bit reset register,                     Address offset: 0x28      */
} GPIO_TypeDef;

/**
  * @brief SysTem Configuration
  */

typedef struct
{
  __IO uint32_t CFGR1;       /*!< SYSCFG configuration register 1,                           Address offset: 0x00 */
       uint32_t RESERVED;    /*!< Reserved,                                                                  0x04 */
  __IO uint32_t EXTICR[4];   /*!< SYSCFG external interrupt configuration register,     Address offset: 0x14-0x08 */
  __IO uint32_t CFGR2;       /*!< SYSCFG configuration register 2,                           Address offset: 0x18 */
} SYSCFG_TypeDef;

/**
  * @brief Inter-integrated Circuit Interface
  */

typedef struct
{
  __IO uint32_t CR1;          /*!< I2C Control register 1,                      Address offset: 0x00 */
  __IO uint32_t CR2;          /*!< I2C Control register 2,                      Address offset: 0x04 */
  __IO uint32_t OAR1;     /*!< I2C Own address 1 register,        Address offset: 0x08 */
  __IO uint32_t OAR2;     /*!< I2C Own address 2 register,        Address offset: 0x0C */
  __IO uint32_t TIMINGR;  /*!< I2C Timing register,               Address offset: 0x10 */
  __IO uint32_t TIMEOUTR; /*!< I2C Timeout register,              Address offset: 0x14 */
  __IO uint32_t ISR;      /*!< I2C Interrupt and status register, Address offset: 0x18 */
  __IO uint32_t ICR;      /*!< I2C Interrupt clear register,      Address offset: 0x1C */
  __IO uint32_t PECR;     /*!< I2C PEC register,                  Address offset: 0x20 */
  __IO uint32_t RXDR;     /*!< I2C Receive data register,         Address offset: 0x24 */
  __IO uint32_t TXDR;     /*!< I2C Transmit data register,        Address offset: 0x28 */
} I2C_TypeDef;

/**
  * @brief Independent WATCHDOG
  */

typedef struct
{
  __IO uint32_t KR;   /*!< IWDG Key register,       Address offset: 0x00 */
  __IO uint32_t PR;   /*!< IWDG Prescaler register, Address offset: 0x04 */
  __IO uint32_t RLR;  /*!< IWDG Reload register,    Address offset: 0x08 */
  __IO uint32_t SR;   /*!< IWDG Status register,    Address offset: 0x0C */
  __IO uint32_t WINR; /*!< IWDG Window register,    Address offset: 0x10 */
} IWDG_TypeDef;

/**
  * @brief Power Control
  */

typedef struct
{
  __IO uint32_t CR;   /*!< PWR power control register,                          Address offset: 0x00 */
  __IO uint32_t CSR;  /*!< PWR power control/status register,                   Address offset: 0x04 */
} PWR_TypeDef;

/**
  * @brief Reset and Clock Control
  */

typedef struct
{
  __IO uint32_t CR;            /*!< RCC clock control register,                                   Address offset: 0x00 */
  __IO uint32_t CFGR;       /*!< RCC clock configuration register,                            Address offset: 0x04 */
  __IO uint32_t CIR;        /*!< RCC clock interrupt register,                                Address offset: 0x08 */
  __IO uint32_t APB2RSTR;   /*!< RCC APB2 peripheral reset register,                          Address offset: 0x0C */
  __IO uint32_t APB1RSTR;   /*!< RCC APB1 peripheral reset register,                          Address offset: 0x10 */
  __IO uint32_t AHBENR;     /*!< RCC AHB peripheral clock register,                           Address offset: 0x14 */
  __IO uint32_t APB2ENR;    /*!< RCC APB2 peripheral clock enable register,                   Address offset: 0x18 */
  __IO uint32_t APB1ENR;    /*!< RCC APB1 peripheral clock enable register,                   Address offset: 0x1C */
  __IO uint32_t BDCR;       /*!< RCC Backup domain control register,                          Address offset: 0x20 */
  __IO uint32_t CSR;        /*!< RCC clock control & status register,                         Address offset: 0x24 */
  __IO uint32_t AHBRSTR;    /*!< RCC AHB peripheral reset register,                           Address offset: 0x28 */
  __IO uint32_t CFGR2;      /*!< RCC clock configuration register 2,                          Address offset: 0x2C */
  __IO uint32_t CFGR3;      /*!< RCC clock configuration register 3,                          Address offset: 0x30 */
  __IO uint32_t CR2;        /*!< RCC clock control register 2,                                Address offset: 0x34 */
} RCC_TypeDef;

/**
  * @brief Real-Time Clock
  */
typedef struct
{
  __IO uint32_t TR;         /*!< RTC time register,                                         Address offset: 0x00 */
  __IO uint32_t DR;         /*!< RTC date register,                                         Address offset: 0x04 */
  __IO uint32_t CR;         /*!< RTC control register,                                      Address offset: 0x08 */
  __IO uint32_t ISR;        /*!< RTC initialization and status register,                    Address offset: 0x0C */
  __IO uint32_t PRER;       /*!< RTC prescaler register,                                    Address offset: 0x10 */
       uint32_t RESERVED1;  /*!< Reserved,                                                  Address offset: 0x14 */
       uint32_t RESERVED2;  /*!< Reserved,                                                  Address offset: 0x18 */
  __IO uint32_t ALRMAR;     /*!< RTC alarm A register,                                      Address offset: 0x1C */
       uint32_t RESERVED3;  /*!< Reserved,                                                  Address offset: 0x20 */
  __IO uint32_t WPR;        /*!< RTC write protection register,                             Address offset: 0x24 */
  __IO uint32_t SSR;        /*!< RTC sub second register,                                   Address offset: 0x28 */
  __IO uint32_t SHIFTR;     /*!< RTC shift control register,                                Address offset: 0x2C */
  __IO uint32_t TSTR;       /*!< RTC time stamp time register,                              Address offset: 0x30 */
  __IO uint32_t TSDR;       /*!< RTC time stamp date register,                              Address offset: 0x34 */
  __IO uint32_t TSSSR;      /*!< RTC time-stamp sub second register,                        Address offset: 0x38 */
  __IO uint32_t CALR;       /*!< RTC calibration register,                                  Address offset: 0x3C */
  __IO uint32_t TAFCR;      /*!< RTC tamper and alternate function configuration register,  Address offset: 0x40 */
  __IO uint32_t ALRMASSR;   /*!< RTC alarm A sub second register,                           Address offset: 0x44 */
       uint32_t RESERVED4;  /*!< Reserved,                                                  Address offset: 0x48 */
       uint32_t RESERVED5;  /*!< Reserved,                                                  Address offset: 0x4C */
  __IO uint32_t BKP0R;      /*!< RTC backup register 0,                                     Address offset: 0x50 */
  __IO uint32_t BKP1R;      /*!< RTC backup register 1,                                     Address offset: 0x54 */
  __IO uint32_t BKP2R;      /*!< RTC backup register 2,                                     Address offset: 0x58 */
  __IO uint32_t BKP3R;      /*!< RTC backup register 3,                                     Address offset: 0x5C */
  __IO uint32_t BKP4R;      /*!< RTC backup register 4,                                     Address offset: 0x60 */
} RTC_TypeDef;

/**
  * @brief Serial Peripheral Interface
  */

typedef struct
{
  __IO uint32_t CR1;        /*!< SPI Control register 1 (not used in I2S mode),      Address offset: 0x00 */
  __IO uint32_t CR2;        /*!< SPI Control register 2,                             Address offset: 0x04 */
  __IO uint32_t SR;         /*!< SPI Status register,                                Address offset: 0x08 */
  __IO uint32_t DR;         /*!< SPI data register,                                  Address offset: 0x0C */
  __IO uint32_t CRCPR;      /*!< SPI CRC polynomial register (not used in I2S mode), Address offset: 0x10 */
  __IO uint32_t RXCRCR;     /*!< SPI Rx CRC register (not used in I2S mode),         Address offset: 0x14 */
  __IO uint32_t TXCRCR;     /*!< SPI Tx CRC register (not used in I2S mode),         Address offset: 0x18 */
  __IO uint32_t I2SCFGR;    /*!< SPI_I2S configuration register,                     Address offset: 0x1C */
  __IO uint32_t I2SPR;      /*!< SPI_I2S prescaler register,                         Address offset: 0x20 */
} SPI_TypeDef;

/**
  * @brief TIM
  */
typedef struct
{
  __IO uint32_t CR1;          /*!< TIM control register 1,              Address offset: 0x00 */
  __IO uint32_t CR2;          /*!< TIM control register 2,              Address offset: 0x04 */
  __IO uint32_t SMCR;         /*!< TIM slave Mode Control register,     Address offset: 0x08 */
  __IO uint32_t DIER;         /*!< TIM DMA/interrupt enable register,   Address offset: 0x0C */
  __IO uint32_t SR;           /*!< TIM status register,                 Address offset: 0x10 */
  __IO uint32_t EGR;          /*!< TIM event generation register,       Address offset: 0x14 */
  __IO uint32_t CCMR1;        /*!< TIM capture/compare mode register 1, Address offset: 0x18 */
  __IO uint32_t CCMR2;        /*!< TIM capture/compare mode register 2, Address offset: 0x1C */
  __IO uint32_t CCER;         /*!< TIM capture/compare enable register, Address offset: 0x20 */
  __IO uint32_t CNT;          /*!< TIM counter register,                Address offset: 0x24 */
  __IO uint32_t PSC;          /*!< TIM prescaler register,              Address offset: 0x28 */
  __IO uint32_t ARR;          /*!< TIM auto-reload register,            Address offset: 0x2C */
  __IO uint32_t RCR;             /*!< TIM  repetition counter register,            Address offset: 0x30 */
  __IO uint32_t CCR1;         /*!< TIM capture/compare register 1,      Address offset: 0x34 */
  __IO uint32_t CCR2;         /*!< TIM capture/compare register 2,      Address offset: 0x38 */
  __IO uint32_t CCR3;         /*!< TIM capture/compare register 3,      Address offset: 0x3C */
  __IO uint32_t CCR4;         /*!< TIM capture/compare register 4,      Address offset: 0x40 */
  __IO uint32_t BDTR;            /*!< TIM break and dead-time register,            Address offset: 0x44 */
  __IO uint32_t DCR;          /*!< TIM DMA control register,            Address offset: 0x48 */
  __IO uint32_t DMAR;            /*!< TIM DMA address for full transfer register,  Address offset: 0x4C */
  __IO uint32_t OR;           /*!< TIM option register,                 Address offset: 0x50 */
} TIM_TypeDef;

/**
  * @brief Touch Sensing Controller (TSC)
  */
typedef struct
{
  __IO uint32_t CR;        /*!< TSC control register,                                     Address offset: 0x00 */
  __IO uint32_t IER;       /*!< TSC interrupt enable register,                            Address offset: 0x04 */
  __IO uint32_t ICR;       /*!< TSC interrupt clear register,                             Address offset: 0x08 */
  __IO uint32_t ISR;       /*!< TSC interrupt status register,                            Address offset: 0x0C */
  __IO uint32_t IOHCR;     /*!< TSC I/O hysteresis control register,                      Address offset: 0x10 */
       uint32_t RESERVED1; /*!< Reserved,                                                 Address offset: 0x14 */
  __IO uint32_t IOASCR;    /*!< TSC I/O analog switch control register,                   Address offset: 0x18 */
       uint32_t RESERVED2; /*!< Reserved,                                                 Address offset: 0x1C */
  __IO uint32_t IOSCR;     /*!< TSC I/O sampling control register,                        Address offset: 0x20 */
       uint32_t RESERVED3; /*!< Reserved,                                                 Address offset: 0x24 */
  __IO uint32_t IOCCR;     /*!< TSC I/O channel control register,                         Address offset: 0x28 */
       uint32_t RESERVED4; /*!< Reserved,                                                 Address offset: 0x2C */
  __IO uint32_t IOGCSR;    /*!< TSC I/O group control status register,                    Address offset: 0x30 */
  __IO uint32_t IOGXCR[8]; /*!< TSC I/O group x counter register,                         Address offset: 0x34-50 */
}TSC_TypeDef;

/**
  * @brief Universal Synchronous Asynchronous Receiver Transmitter
  */

typedef struct
{
  __IO uint32_t CR1;    /*!< USART Control register 1,                 Address offset: 0x00 */
  __IO uint32_t CR2;    /*!< USART Control register 2,                 Address offset: 0x04 */
  __IO uint32_t CR3;    /*!< USART Control register 3,                 Address offset: 0x08 */
  __IO uint32_t BRR;    /*!< USART Baud rate register,                 Address offset: 0x0C */
  __IO uint32_t GTPR;   /*!< USART Guard time and prescaler register,  Address offset: 0x10 */
  __IO uint32_t RTOR;   /*!< USART Receiver Time Out register,         Address offset: 0x14 */
  __IO uint32_t RQR;    /*!< USART Request register,                   Address offset: 0x18 */
  __IO uint32_t ISR;    /*!< USART Interrupt and status register,      Address offset: 0x1C */
  __IO uint32_t ICR;    /*!< USART Interrupt flag Clear register,      Address offset: 0x20 */
  __IO uint16_t RDR;    /*!< USART Receive Data register,              Address offset: 0x24 */
  uint16_t  RESERVED1;  /*!< Reserved, 0x26                                                 */
  __IO uint16_t TDR;    /*!< USART Transmit Data register,             Address offset: 0x28 */
  uint16_t  RESERVED2;  /*!< Reserved, 0x2A                                                 */
} USART_TypeDef;

/**
  * @brief Universal Serial Bus Full Speed Device
  */

typedef struct
{
  __IO uint16_t EP0R;            /*!< USB Endpoint 0 register,                Address offset: 0x00 */
  __IO uint16_t RESERVED0;       /*!< Reserved */
  __IO uint16_t EP1R;            /*!< USB Endpoint 1 register,                Address offset: 0x04 */
  __IO uint16_t RESERVED1;       /*!< Reserved */
  __IO uint16_t EP2R;            /*!< USB Endpoint 2 register,                Address offset: 0x08 */
  __IO uint16_t RESERVED2;       /*!< Reserved */
  __IO uint16_t EP3R;            /*!< USB Endpoint 3 register,                Address offset: 0x0C */
  __IO uint16_t RESERVED3;       /*!< Reserved */
  __IO uint16_t EP4R;            /*!< USB Endpoint 4 register,                Address offset: 0x10 */
  __IO uint16_t RESERVED4;       /*!< Reserved */
  __IO uint16_t EP5R;            /*!< USB Endpoint 5 register,                Address offset: 0x14 */
  __IO uint16_t RESERVED5;       /*!< Reserved */
  __IO uint16_t EP6R;            /*!< USB Endpoint 6 register,                Address offset: 0x18 */
  __IO uint16_t RESERVED6;       /*!< Reserved */
  __IO uint16_t EP7R;            /*!< USB Endpoint 7 register,                Address offset: 0x1C */
  __IO uint16_t RESERVED7[17];   /*!< Reserved */
  __IO uint16_t CNTR;            /*!< Control register,                       Address offset: 0x40 */
  __IO uint16_t RESERVED8;       /*!< Reserved */
  __IO uint16_t ISTR;            /*!< Interrupt status register,              Address offset: 0x44 */
  __IO uint16_t RESERVED9;       /*!< Reserved */
  __IO uint16_t FNR;             /*!< Frame number register,                  Address offset: 0x48 */
  __IO uint16_t RESERVEDA;       /*!< Reserved */
  __IO uint16_t DADDR;           /*!< Device address register,                Address offset: 0x4C */
  __IO uint16_t RESERVEDB;       /*!< Reserved */
  __IO uint16_t BTABLE;          /*!< Buffer Table address register,          Address offset: 0x50 */
  __IO uint16_t RESERVEDC;       /*!< Reserved */
  __IO uint16_t LPMCSR;          /*!< LPM Control and Status register,        Address offset: 0x54 */
  __IO uint16_t RESERVEDD;       /*!< Reserved */
  __IO uint16_t BCDR;            /*!< Battery Charging detector register,     Address offset: 0x58 */
  __IO uint16_t RESERVEDE;       /*!< Reserved */
} USB_TypeDef;

/**
  * @brief Window WATCHDOG
  */
typedef struct
{
  __IO uint32_t CR;   /*!< WWDG Control register,       Address offset: 0x00 */
  __IO uint32_t CFR;  /*!< WWDG Configuration register, Address offset: 0x04 */
  __IO uint32_t SR;   /*!< WWDG Status register,        Address offset: 0x08 */
} WWDG_TypeDef;

/**
  * @}
  */

/** @addtogroup Peripheral_memory_map
  * @{
  */

#define FLASH_BASE            0x08000000UL              /*!< FLASH base address in the alias region */
#define FLASH_BANK1_END       0x08007FFFUL /*!< FLASH END address of bank1 */
#define SRAM_BASE             0x20000000UL              /*!< SRAM base address in the alias region */
#define PERIPH_BASE           0x40000000UL              /*!< Peripheral base address in the alias region */

/*!< Peripheral memory map */
#define APBPERIPH_BASE        PERIPH_BASE
#define AHBPERIPH_BASE        (PERIPH_BASE + 0x00020000UL)
#define AHB2PERIPH_BASE       (PERIPH_BASE + 0x08000000UL)

/*!< APB peripherals */
#define TIM2_BASE             (APBPERIPH_BASE + 0x00000000UL)
#define TIM3_BASE             (APBPERIPH_BASE + 0x00000400UL)
#define TIM14_BASE            (APBPERIPH_BASE + 0x00002000UL)
#define RTC_BASE              (APBPERIPH_BASE + 0x00002800UL)
#define WWDG_BASE             (APBPERIPH_BASE + 0x00002C00UL)
#define IWDG_BASE             (APBPERIPH_BASE + 0x00003000UL)
#define SPI2_BASE             (APBPERIPH_BASE + 0x00003800UL)
#define USART2_BASE           (APBPERIPH_BASE + 0x00004400UL)
#define I2C1_BASE             (APBPERIPH_BASE + 0x00005400UL)
#define USB_BASE              (APBPERIPH_BASE + 0x00005C00UL) /*!< USB_IP Peripheral Registers base address */
#define USB_PMAADDR           (APBPERIPH_BASE + 0x00006000UL) /*!< USB_IP Packet Memory Area base address */
#define CAN_BASE              (APBPERIPH_BASE + 0x00006400UL)
#define CRS_BASE              (APBPERIPH_BASE + 0x00006C00UL)
#define PWR_BASE              (APBPERIPH_BASE + 0x00007000UL)
#define CEC_BASE              (APBPERIPH_BASE + 0x00007800UL)

#define SYSCFG_BASE           (APBPERIPH_BASE + 0x00010000UL)
#define EXTI_BASE             (APBPERIPH_BASE + 0x00010400UL)
#define ADC1_BASE             (APBPERIPH_BASE + 0x00012400UL)
#define ADC_BASE              (APBPERIPH_BASE + 0x00012708UL)
#define TIM1_BASE             (APBPERIPH_BASE + 0x00012C00UL)
#define SPI1_BASE             (APBPERIPH_BASE + 0x00013000UL)
#define USART1_BASE           (APBPERIPH_BASE + 0x00013800UL)
#define TIM16_BASE            (APBPERIPH_BASE + 0x00014400UL)
#define TIM17_BASE            (APBPERIPH_BASE + 0x00014800UL)
#define DBGMCU_BASE           (APBPERIPH_BASE + 0x00015800UL)

/*!< AHB peripherals */
#define DMA1_BASE             (AHBPERIPH_BASE + 0x00000000UL)
#define DMA1_Channel1_BASE    (DMA1_BASE + 0x00000008UL)
#define DMA1_Channel2_BASE    (DMA1_BASE + 0x0000001CUL)
#define DMA1_Channel3_BASE    (DMA1_BASE + 0x00000030UL)
#define DMA1_Channel4_BASE    (DMA1_BASE + 0x00000044UL)
#define DMA1_Channel5_BASE    (DMA1_BASE + 0x00000058UL)
#define DMA1_Channel6_BASE    (DMA1_BASE + 0x0000006CUL)
#define DMA1_Channel7_BASE    (DMA1_BASE + 0x00000080UL)

#define RCC_BASE              (AHBPERIPH_BASE + 0x00001000UL)
#define FLASH_R_BASE          (AHBPERIPH_BASE + 0x00002000UL) /*!< FLASH registers base address */
#define OB_BASE               0x1FFFF800UL       /*!< FLASH Option Bytes base address */
#define FLASHSIZE_BASE        0x1FFFF7CCUL       /*!< FLASH Size register base address */
#define UID_BASE              0x1FFFF7ACUL       /*!< Unique device ID register base address */
#define CRC_BASE              (AHBPERIPH_BASE + 0x00003000UL)
#define TSC_BASE              (AHBPERIPH_BASE + 0x00004000UL)

/*!< AHB2 peripherals */
#define GPIOA_BASE            (AHB2PERIPH_BASE + 0x00000000UL)
#define GPIOB_BASE            (AHB2PERIPH_BASE + 0x00000400UL)
#define GPIOC_BASE            (AHB2PERIPH_BASE + 0x00000800UL)
#define GPIOF_BASE            (AHB2PERIPH_BASE + 0x00001400UL)

/**
  * @}
  */

/** @addtogroup Peripheral_declaration
  * @{
  */

#define TIM2                ((TIM_TypeDef *) TIM2_BASE)
#define TIM3                ((TIM_TypeDef *) TIM3_BASE)
#define TIM14               ((TIM_TypeDef *) TIM14_BASE)
#define RTC                 ((RTC_TypeDef *) RTC_BASE)
#define WWDG                ((WWDG_TypeDef *) WWDG_BASE)
#define IWDG                ((IWDG_TypeDef *) IWDG_BASE)
#define USART2              ((USART_TypeDef *) USART2_BASE)
#define I2C1                ((I2C_TypeDef *) I2C1_BASE)
#define CAN                 ((CAN_TypeDef *) CAN_BASE)
#define CRS                 ((CRS_TypeDef *) CRS_BASE)
#define PWR                 ((PWR_TypeDef *) PWR_BASE)
#define CEC                 ((CEC_TypeDef *) CEC_BASE)
#define SYSCFG              ((SYSCFG_TypeDef *) SYSCFG_BASE)
#define EXTI                ((EXTI_TypeDef *) EXTI_BASE)
#define ADC1                ((ADC_TypeDef *) ADC1_BASE)
#define ADC1_COMMON         ((ADC_Common_TypeDef *) ADC_BASE)
#define ADC                 ((ADC_Common_TypeDef *) ADC_BASE) /* Kept for legacy purpose */
#define TIM1                ((TIM_TypeDef *) TIM1_BASE)
#define SPI1                ((SPI_TypeDef *) SPI1_BASE)
#define SPI2                ((SPI_TypeDef *) SPI2_BASE)
#define USART1              ((USART_TypeDef *) USART1_BASE)
#define TIM16               ((TIM_TypeDef *) TIM16_BASE)
#define TIM17               ((TIM_TypeDef *) TIM17_BASE)
#define DBGMCU              ((DBGMCU_TypeDef *) DBGMCU_BASE)
#define DMA1                ((DMA_TypeDef *) DMA1_BASE)
#define DMA1_Channel1       ((DMA_Channel_TypeDef *) DMA1_Channel1_BASE)
#define DMA1_Channel2       ((DMA_Channel_TypeDef *) DMA1_Channel2_BASE)
#define DMA1_Channel3       ((DMA_Channel_TypeDef *) DMA1_Channel3_BASE)
#define DMA1_Channel4       ((DMA_Channel_TypeDef *) DMA1_Channel4_BASE)
#define DMA1_Channel5       ((DMA_Channel_TypeDef *) DMA1_Channel5_BASE)
#define DMA1_Channel6       ((DMA_Channel_TypeDef *) DMA1_Channel6_BASE)
#define DMA1_Channel7       ((DMA_Channel_TypeDef *) DMA1_Channel7_BASE)
#define FLASH               ((FLASH_TypeDef *) FLASH_R_BASE)
#define OB                  ((OB_TypeDef *) OB_BASE)
#define RCC                 ((RCC_TypeDef *) RCC_BASE)
#define CRC                 ((CRC_TypeDef *) CRC_BASE)
#define TSC                 ((TSC_TypeDef *) TSC_BASE)
#define GPIOA               ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC               ((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOF               ((GPIO_TypeDef *) GPIOF_BASE)
#define USB                 ((USB_TypeDef *) USB_BASE)
/**
  * @}
  */

/** @addtogroup Exported_constants
  * @{
  */

/** @addtogroup Hardware_Constant_Definition
  * @{
  */
#define LSI_STARTUP_TIME 85U /*!< LSI Maximum startup time in us */

/**
  * @}
  */

/** @addtogroup Peripheral_Registers_Bits_Definition
  * @{
  */

/******************************************************************************/
/*                         Peripheral Registers Bits Definition               */
/******************************************************************************/

/******************************************************************************/
/*                                                                            */
/*                      Analog to Digital Converter (ADC)                     */
/*                                                                            */
/******************************************************************************/

/*
 * @brief Specific device feature definitions (not present on all devices in the STM32F0 series)
 */
#define ADC_CHANNEL_VBAT_SUPPORT                       /*!< ADC feature available only on specific devices: ADC internal channel Vbat */

/********************  Bits definition for ADC_ISR register  ******************/
#define ADC_ISR_ADRDY_Pos         (0U)
#define ADC_ISR_ADRDY_Msk         (0x1UL << ADC_ISR_ADRDY_Pos)                  /*!< 0x00000001 */
#define ADC_ISR_ADRDY             ADC_ISR_ADRDY_Msk                            /*!< ADC ready flag */
#define ADC_ISR_EOSMP_Pos         (1U)
#define ADC_ISR_EOSMP_Msk         (0x1UL << ADC_ISR_EOSMP_Pos)                  /*!< 0x00000002 */
#define ADC_ISR_EOSMP             ADC_ISR_EOSMP_Msk                            /*!< ADC group regular end of sampling flag */
#define ADC_ISR_EOC_Pos           (2U)
#define ADC_ISR_EOC_Msk           (0x1UL << ADC_ISR_EOC_Pos)                    /*!< 0x00000004 */
#define ADC_ISR_EOC               ADC_ISR_EOC_Msk                              /*!< ADC group regular end of unitary conversion flag */
#define ADC_ISR_EOS_Pos           (3U)
#define ADC_ISR_EOS_Msk           (0x1UL << ADC_ISR_EOS_Pos)                    /*!< 0x00000008 */
#define ADC_ISR_EOS               ADC_ISR_EOS_Msk                              /*!< ADC group regular end of sequence conversions flag */
#define ADC_ISR_OVR_Pos           (4U)
#define ADC_ISR_OVR_Msk           (0x1UL << ADC_ISR_OVR_Pos)                    /*!< 0x00000010 */
#define ADC_ISR_OVR               ADC_ISR_OVR_Msk                              /*!< ADC group regular overrun flag */
#define ADC_ISR_AWD1_Pos          (7U)
#define ADC_ISR_AWD1_Msk          (0x1UL << ADC_ISR_AWD1_Pos)                   /*!< 0x00000080 */
#define ADC_ISR_AWD1              ADC_ISR_AWD1_Msk                             /*!< ADC analog watchdog 1 flag */

/* Legacy defines */
#define ADC_ISR_AWD             (ADC_ISR_AWD1)
#define ADC_ISR_EOSEQ           (ADC_ISR_EOS)

/********************  Bits definition for ADC_IER register  ******************/
#define ADC_IER_ADRDYIE_Pos       (0U)
#define ADC_IER_ADRDYIE_Msk       (0x1UL << ADC_IER_ADRDYIE_Pos)                /*!< 0x00000001 */
#define ADC_IER_ADRDYIE           ADC_IER_ADRDYIE_Msk                          /*!< ADC ready interrupt */
#define ADC_IER_EOSMPIE_Pos       (1U)
#define ADC_IER_EOSMPIE_Msk       (0x1UL << ADC_IER_EOSMPIE_Pos)                /*!< 0x00000002 */
#define ADC_IER_EOSMPIE           ADC_IER_EOSMPIE_Msk                          /*!< ADC group regular end of sampling interrupt */
#define ADC_IER_EOCIE_Pos         (2U)
#define ADC_IER_EOCIE_Msk         (0x1UL << ADC_IER_EOCIE_Pos)                  /*!< 0x00000004 */
#define ADC_IER_EOCIE             ADC_IER_EOCIE_Msk                            /*!< ADC group regular end of unitary conversion interrupt */
#define ADC_IER_EOSIE_Pos         (3U)
#define ADC_IER_EOSIE_Msk         (0x1UL << ADC_IER_EOSIE_Pos)                  /*!< 0x00000008 */
#define ADC_IER_EOSIE             ADC_IER_EOSIE_Msk                            /*!< ADC group regular end of sequence conversions interrupt */
#define ADC_IER_OVRIE_Pos         (4U)
#define ADC_IER_OVRIE_Msk         (0x1UL << ADC_IER_OVRIE_Pos)                  /*!< 0x00000010 */
#define ADC_IER_OVRIE             ADC_IER_OVRIE_Msk                            /*!< ADC group regular overrun interrupt */
#define ADC_IER_AWD1IE_Pos        (7U)
#define ADC_IER_AWD1IE_Msk        (0x1UL << ADC_IER_AWD1IE_Pos)                 /*!< 0x00000080 */
#define ADC_IER_AWD1IE            ADC_IER_AWD1IE_Msk                           /*!< ADC analog watchdog 1 interrupt */

/* Legacy defines */
#define ADC_IER_AWDIE           (ADC_IER_AWD1IE)
#define ADC_IER_EOSEQIE         (ADC_IER_EOSIE)

/********************  Bits definition for ADC_CR register  *******************/
#define ADC_CR_ADEN_Pos           (0U)
#define ADC_CR_ADEN_Msk           (0x1UL << ADC_CR_ADEN_Pos)                    /*!< 0x00000001 */
#define ADC_CR_ADEN               ADC_CR_ADEN_Msk                              /*!< ADC enable */
#define ADC_CR_ADDIS_Pos          (1U)
#define ADC_CR_ADDIS_Msk          (0x1UL << ADC_CR_ADDIS_Pos)                   /*!< 0x00000002 */
#define ADC_CR_ADDIS              ADC_CR_ADDIS_Msk                             /*!< ADC disable */
#define ADC_CR_ADSTART_Pos        (2U)
#define ADC_CR_ADSTART_Msk        (0x1UL << ADC_CR_ADSTART_Pos)                 /*!< 0x00000004 */
#define ADC_CR_ADSTART            ADC_CR_ADSTART_Msk                           /*!< ADC group regular conversion start */
#define ADC_CR_ADSTP_Pos          (4U)
#define ADC_CR_ADSTP_Msk          (0x1UL << ADC_CR_ADSTP_Pos)                   /*!< 0x00000010 */
#define ADC_CR_ADSTP              ADC_CR_ADSTP_Msk                             /*!< ADC group regular conversion stop */
#define ADC_CR_ADCAL_Pos          (31U)
#define ADC_CR_ADCAL_Msk          (0x1UL << ADC_CR_ADCAL_Pos)                   /*!< 0x80000000 */
#define ADC_CR_ADCAL              ADC_CR_ADCAL_Msk                             /*!< ADC calibration */

/*******************  Bits definition for ADC_CFGR1 register  *****************/
#define ADC_CFGR1_DMAEN_Pos       (0U)
#define ADC_CFGR1_DMAEN_Msk       (0x1UL << ADC_CFGR1_DMAEN_Pos)                /*!< 0x00000001 */
#define ADC_CFGR1_DMAEN           ADC_CFGR1_DMAEN_Msk                          /*!< ADC DMA transfer enable */
#define ADC_CFGR1_DMACFG_Pos      (1U)
#define ADC_CFGR1_DMACFG_Msk      (0x1UL << ADC_CFGR1_DMACFG_Pos)               /*!< 0x00000002 */
#define ADC_CFGR1_DMACFG          ADC_CFGR1_DMACFG_Msk                         /*!< ADC DMA transfer configuration */
#define ADC_CFGR1_SCANDIR_Pos     (2U)
#define ADC_CFGR1_SCANDIR_Msk     (0x1UL << ADC_CFGR1_SCANDIR_Pos)              /*!< 0x00000004 */
#define ADC_CFGR1_SCANDIR         ADC_CFGR1_SCANDIR_Msk                        /*!< ADC group regular sequencer scan direction */

#define ADC_CFGR1_RES_Pos         (3U)
#define ADC_CFGR1_RES_Msk         (0x3UL << ADC_CFGR1_RES_Pos)                  /*!< 0x00000018 */
#define ADC_CFGR1_RES             ADC_CFGR1_RES_Msk                            /*!< ADC data resolution */
#define ADC_CFGR1_RES_0           (0x1UL << ADC_CFGR1_RES_Pos)                  /*!< 0x00000008 */
#define ADC_CFGR1_RES_1           (0x2UL << ADC_CFGR1_RES_Pos)                  /*!< 0x00000010 */

#define ADC_CFGR1_ALIGN_Pos       (5U)
#define ADC_CFGR1_ALIGN_Msk       (0x1UL << ADC_CFGR1_ALIGN_Pos)                /*!< 0x00000020 */
#define ADC_CFGR1_ALIGN           ADC_CFGR1_ALIGN_Msk                          /*!< ADC data alignment */

#define ADC_CFGR1_EXTSEL_Pos      (6U)
#define ADC_CFGR1_EXTSEL_Msk      (0x7UL << ADC_CFGR1_EXTSEL_Pos)               /*!< 0x000001C0 */
#define ADC_CFGR1_EXTSEL          ADC_CFGR1_EXTSEL_Msk                         /*!< ADC group regular external trigger source */
#define ADC_CFGR1_EXTSEL_0        (0x1UL << ADC_CFGR1_EXTSEL_Pos)               /*!< 0x00000040 */
#define ADC_CFGR1_EXTSEL_1        (0x2UL << ADC_CFGR1_EXTSEL_Pos)               /*!< 0x00000080 */
#define ADC_CFGR1_EXTSEL_2        (0x4UL << ADC_CFGR1_EXTSEL_Pos)               /*!< 0x00000100 */

#define ADC_CFGR1_EXTEN_Pos       (10U)
#define ADC_CFGR1_EXTEN_Msk       (0x3UL << ADC_CFGR1_EXTEN_Pos)                /*!< 0x00000C00 */
#define ADC_CFGR1_EXTEN           ADC_CFGR1_EXTEN_Msk                          /*!< ADC group regular external trigger polarity */
#define ADC_CFGR1_EXTEN_0         (0x1UL << ADC_CFGR1_EXTEN_Pos)                /*!< 0x00000400 */
#define ADC_CFGR1_EXTEN_1         (0x2UL << ADC_CFGR1_EXTEN_Pos)                /*!< 0x00000800 */

#define ADC_CFGR1_OVRMOD_Pos      (12U)
#define ADC_CFGR1_OVRMOD_Msk      (0x1UL << ADC_CFGR1_OVRMOD_Pos)               /*!< 0x00001000 */
#define ADC_CFGR1_OVRMOD          ADC_CFGR1_OVRMOD_Msk                         /*!< ADC group regular overrun configuration */
#define ADC_CFGR1_CONT_Pos        (13U)
#define ADC_CFGR1_CONT_Msk        (0x1UL << ADC_CFGR1_CONT_Pos)                 /*!< 0x00002000 */
#define ADC_CFGR1_CONT            ADC_CFGR1_CONT_Msk                           /*!< ADC group regular continuous conversion mode */
#define ADC_CFGR1_WAIT_Pos        (14U)
#define ADC_CFGR1_WAIT_Msk        (0x1UL << ADC_CFGR1_WAIT_Pos)                 /*!< 0x00004000 */
#define ADC_CFGR1_WAIT            ADC_CFGR1_WAIT_Msk                           /*!< ADC low power auto wait */
#define ADC_CFGR1_AUTOFF_Pos      (15U)
#define ADC_CFGR1_AUTOFF_Msk      (0x1UL << ADC_CFGR1_AUTOFF_Pos)               /*!< 0x00008000 */
#define ADC_CFGR1_AUTOFF          ADC_CFGR1_AUTOFF_Msk                         /*!< ADC low power auto power off */
#define ADC_CFGR1_DISCEN_Pos      (16U)
#define ADC_CFGR1_DISCEN_Msk      (0x1UL << ADC_CFGR1_DISCEN_Pos)               /*!< 0x00010000 */
#define ADC_CFGR1_DISCEN          ADC_CFGR1_DISCEN_Msk                         /*!< ADC group regular sequencer discontinuous mode */

#define ADC_CFGR1_AWD1SGL_Pos     (22U)
#define ADC_CFGR1_AWD1SGL_Msk     (0x1UL << ADC_CFGR1_AWD1SGL_Pos)              /*!< 0x00400000 */
#define ADC_CFGR1_AWD1SGL         ADC_CFGR1_AWD1SGL_Msk                        /*!< ADC analog watchdog 1 monitoring a single channel or all channels */
#define ADC_CFGR1_AWD1EN_Pos      (23U)
#define ADC_CFGR1_AWD1EN_Msk      (0x1UL << ADC_CFGR1_AWD1EN_Pos)               /*!< 0x00800000 */
#define ADC_CFGR1_AWD1EN          ADC_CFGR1_AWD1EN_Msk                         /*!< ADC analog watchdog 1 enable on scope ADC group regular */

#define ADC_CFGR1_AWD1CH_Pos      (26U)
#define ADC_CFGR1_AWD1CH_Msk      (0x1FUL << ADC_CFGR1_AWD1CH_Pos)              /*!< 0x7C000000 */
#define ADC_CFGR1_AWD1CH          ADC_CFGR1_AWD1CH_Msk                         /*!< ADC analog watchdog 1 monitored channel selection */
#define ADC_CFGR1_AWD1CH_0        (0x01UL << ADC_CFGR1_AWD1CH_Pos)              /*!< 0x04000000 */
#define ADC_CFGR1_AWD1CH_1        (0x02UL << ADC_CFGR1_AWD1CH_Pos)              /*!< 0x08000000 */
#define ADC_CFGR1_AWD1CH_2        (0x04UL << ADC_CFGR1_AWD1CH_Pos)              /*!< 0x10000000 */
#define ADC_CFGR1_AWD1CH_3        (0x08UL << ADC_CFGR1_AWD1CH_Pos)              /*!< 0x20000000 */
#define ADC_CFGR1_AWD1CH_4        (0x10UL << ADC_CFGR1_AWD1CH_Pos)              /*!< 0x40000000 */

/* Legacy defines */
#define ADC_CFGR1_AUTDLY        (ADC_CFGR1_WAIT)
#define ADC_CFGR1_AWDSGL        (ADC_CFGR1_AWD1SGL)
#define ADC_CFGR1_AWDEN         (ADC_CFGR1_AWD1EN)
#define ADC_CFGR1_AWDCH         (ADC_CFGR1_AWD1CH)
#define ADC_CFGR1_AWDCH_0       (ADC_CFGR1_AWD1CH_0)
#define ADC_CFGR1_AWDCH_1       (ADC_CFGR1_AWD1CH_1)
#define ADC_CFGR1_AWDCH_2       (ADC_CFGR1_AWD1CH_2)
#define ADC_CFGR1_AWDCH_3       (ADC_CFGR1_AWD1CH_3)
#define ADC_CFGR1_AWDCH_4       (ADC_CFGR1_AWD1CH_4)

/*******************  Bits definition for ADC_CFGR2 register  *****************/
#define ADC_CFGR2_CKMODE_Pos      (30U)
#define ADC_CFGR2_CKMODE_Msk      (0x3UL << ADC_CFGR2_CKMODE_Pos)               /*!< 0xC0000000 */
#define ADC_CFGR2_CKMODE          ADC_CFGR2_CKMODE_Msk                         /*!< ADC clock source and prescaler (prescaler only for clock source synchronous) */
#define ADC_CFGR2_CKMODE_1        (0x2UL << ADC_CFGR2_CKMODE_Pos)               /*!< 0x80000000 */
#define ADC_CFGR2_CKMODE_0        (0x1UL << ADC_CFGR2_CKMODE_Pos)               /*!< 0x40000000 */

/* Legacy defines */
#define  ADC_CFGR2_JITOFFDIV4   (ADC_CFGR2_CKMODE_1)   /*!< ADC clocked by PCLK div4 */
#define  ADC_CFGR2_JITOFFDIV2   (ADC_CFGR2_CKMODE_0)   /*!< ADC clocked by PCLK div2 */

/******************  Bit definition for ADC_SMPR register  ********************/
#define ADC_SMPR_SMP_Pos          (0U)
#define ADC_SMPR_SMP_Msk          (0x7UL << ADC_SMPR_SMP_Pos)                   /*!< 0x00000007 */
#define ADC_SMPR_SMP              ADC_SMPR_SMP_Msk                             /*!< ADC group of channels sampling time 2 */
#define ADC_SMPR_SMP_0            (0x1UL << ADC_SMPR_SMP_Pos)                   /*!< 0x00000001 */
#define ADC_SMPR_SMP_1            (0x2UL << ADC_SMPR_SMP_Pos)                   /*!< 0x00000002 */
#define ADC_SMPR_SMP_2            (0x4UL << ADC_SMPR_SMP_Pos)                   /*!< 0x00000004 */

/* Legacy defines */
#define  ADC_SMPR1_SMPR         (ADC_SMPR_SMP)         /*!< SMP[2:0] bits (Sampling time selection) */
#define  ADC_SMPR1_SMPR_0       (ADC_SMPR_SMP_0)       /*!< bit 0 */
#define  ADC_SMPR1_SMPR_1       (ADC_SMPR_SMP_1)       /*!< bit 1 */
#define  ADC_SMPR1_SMPR_2       (ADC_SMPR_SMP_2)       /*!< bit 2 */

/*******************  Bit definition for ADC_TR register  ********************/
#define ADC_TR1_LT1_Pos           (0U)
#define ADC_TR1_LT1_Msk           (0xFFFUL << ADC_TR1_LT1_Pos)                  /*!< 0x00000FFF */
#define ADC_TR1_LT1               ADC_TR1_LT1_Msk                              /*!< ADC analog watchdog 1 threshold low */
#define ADC_TR1_LT1_0             (0x001UL << ADC_TR1_LT1_Pos)                  /*!< 0x00000001 */
#define ADC_TR1_LT1_1             (0x002UL << ADC_TR1_LT1_Pos)                  /*!< 0x00000002 */
#define ADC_TR1_LT1_2             (0x004UL << ADC_TR1_LT1_Pos)                  /*!< 0x00000004 */
#define ADC_TR1_LT1_3             (0x008UL << ADC_TR1_LT1_Pos)                  /*!< 0x00000008 */
#define ADC_TR1_LT1_4             (0x010UL << ADC_TR1_LT1_Pos)                  /*!< 0x00000010 */
#define ADC_TR1_LT1_5             (0x020UL << ADC_TR1_LT1_Pos)                  /*!< 0x00000020 */
#define ADC_TR1_LT1_6             (0x040UL << ADC_TR1_LT1_Pos)                  /*!< 0x00000040 */
#define ADC_TR1_LT1_7             (0x080UL << ADC_TR1_LT1_Pos)                  /*!< 0x00000080 */
#define ADC_TR1_LT1_8             (0x100UL << ADC_TR1_LT1_Pos)                  /*!< 0x00000100 */
#define ADC_TR1_LT1_9             (0x200UL << ADC_TR1_LT1_Pos)                  /*!< 0x00000200 */
#define ADC_TR1_LT1_10            (0x400UL << ADC_TR1_LT1_Pos)                  /*!< 0x00000400 */
#define ADC_TR1_LT1_11            (0x800UL << ADC_TR1_LT1_Pos)                  /*!< 0x00000800 */

#define ADC_TR1_HT1_Pos           (16U)
#define ADC_TR1_HT1_Msk           (0xFFFUL << ADC_TR1_HT1_Pos)                  /*!< 0x0FFF0000 */
#define ADC_TR1_HT1               ADC_TR1_HT1_Msk                              /*!< ADC Analog watchdog 1 threshold high */
#define ADC_TR1_HT1_0             (0x001UL << ADC_TR1_HT1_Pos)                  /*!< 0x00010000 */
#define ADC_TR1_HT1_1             (0x002UL << ADC_TR1_HT1_Pos)                  /*!< 0x00020000 */
#define ADC_TR1_HT1_2             (0x004UL << ADC_TR1_HT1_Pos)                  /*!< 0x00040000 */
#define ADC_TR1_HT1_3             (0x008UL << ADC_TR1_HT1_Pos)                  /*!< 0x00080000 */
#define ADC_TR1_HT1_4             (0x010UL << ADC_TR1_HT1_Pos)                  /*!< 0x00100000 */
#define ADC_TR1_HT1_5             (0x020UL << ADC_TR1_HT1_Pos)                  /*!< 0x00200000 */
#define ADC_TR1_HT1_6             (0x040UL << ADC_TR1_HT1_Pos)                  /*!< 0x00400000 */
#define ADC_TR1_HT1_7             (0x080UL << ADC_TR1_HT1_Pos)                  /*!< 0x00800000 */
#define ADC_TR1_HT1_8             (0x100UL << ADC_TR1_HT1_Pos)                  /*!< 0x01000000 */
#define ADC_TR1_HT1_9             (0x200UL << ADC_TR1_HT1_Pos)                  /*!< 0x02000000 */
#define ADC_TR1_HT1_10            (0x400UL << ADC_TR1_HT1_Pos)                  /*!< 0x04000000 */
#define ADC_TR1_HT1_11            (0x800UL << ADC_TR1_HT1_Pos)                  /*!< 0x08000000 */

/* Legacy defines */
#define  ADC_TR_HT              (ADC_TR1_HT1)
#define  ADC_TR_LT              (ADC_TR1_LT1)
#define  ADC_HTR_HT             (ADC_TR1_HT1)
#define  ADC_LTR_LT             (ADC_TR1_LT1)

/******************  Bit definition for ADC_CHSELR register  ******************/
#define ADC_CHSELR_CHSEL_Pos      (0U)
#define ADC_CHSELR_CHSEL_Msk      (0x7FFFFUL << ADC_CHSELR_CHSEL_Pos)           /*!< 0x0007FFFF */
#define ADC_CHSELR_CHSEL          ADC_CHSELR_CHSEL_Msk                         /*!< ADC group regular sequencer channels, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL18_Pos    (18U)
#define ADC_CHSELR_CHSEL18_Msk    (0x1UL << ADC_CHSELR_CHSEL18_Pos)             /*!< 0x00040000 */
#define ADC_CHSELR_CHSEL18        ADC_CHSELR_CHSEL18_Msk                       /*!< ADC group regular sequencer channel 18, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL17_Pos    (17U)
#define ADC_CHSELR_CHSEL17_Msk    (0x1UL << ADC_CHSELR_CHSEL17_Pos)             /*!< 0x00020000 */
#define ADC_CHSELR_CHSEL17        ADC_CHSELR_CHSEL17_Msk                       /*!< ADC group regular sequencer channel 17, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL16_Pos    (16U)
#define ADC_CHSELR_CHSEL16_Msk    (0x1UL << ADC_CHSELR_CHSEL16_Pos)             /*!< 0x00010000 */
#define ADC_CHSELR_CHSEL16        ADC_CHSELR_CHSEL16_Msk                       /*!< ADC group regular sequencer channel 16, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL15_Pos    (15U)
#define ADC_CHSELR_CHSEL15_Msk    (0x1UL << ADC_CHSELR_CHSEL15_Pos)             /*!< 0x00008000 */
#define ADC_CHSELR_CHSEL15        ADC_CHSELR_CHSEL15_Msk                       /*!< ADC group regular sequencer channel 15, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL14_Pos    (14U)
#define ADC_CHSELR_CHSEL14_Msk    (0x1UL << ADC_CHSELR_CHSEL14_Pos)             /*!< 0x00004000 */
#define ADC_CHSELR_CHSEL14        ADC_CHSELR_CHSEL14_Msk                       /*!< ADC group regular sequencer channel 14, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL13_Pos    (13U)
#define ADC_CHSELR_CHSEL13_Msk    (0x1UL << ADC_CHSELR_CHSEL13_Pos)             /*!< 0x00002000 */
#define ADC_CHSELR_CHSEL13        ADC_CHSELR_CHSEL13_Msk                       /*!< ADC group regular sequencer channel 13, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL12_Pos    (12U)
#define ADC_CHSELR_CHSEL12_Msk    (0x1UL << ADC_CHSELR_CHSEL12_Pos)             /*!< 0x00001000 */
#define ADC_CHSELR_CHSEL12        ADC_CHSELR_CHSEL12_Msk                       /*!< ADC group regular sequencer channel 12, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL11_Pos    (11U)
#define ADC_CHSELR_CHSEL11_Msk    (0x1UL << ADC_CHSELR_CHSEL11_Pos)             /*!< 0x00000800 */
#define ADC_CHSELR_CHSEL11        ADC_CHSELR_CHSEL11_Msk                       /*!< ADC group regular sequencer channel 11, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL10_Pos    (10U)
#define ADC_CHSELR_CHSEL10_Msk    (0x1UL << ADC_CHSELR_CHSEL10_Pos)             /*!< 0x00000400 */
#define ADC_CHSELR_CHSEL10        ADC_CHSELR_CHSEL10_Msk                       /*!< ADC group regular sequencer channel 10, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL9_Pos     (9U)
#define ADC_CHSELR_CHSEL9_Msk     (0x1UL << ADC_CHSELR_CHSEL9_Pos)              /*!< 0x00000200 */
#define ADC_CHSELR_CHSEL9         ADC_CHSELR_CHSEL9_Msk                        /*!< ADC group regular sequencer channel 9, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL8_Pos     (8U)
#define ADC_CHSELR_CHSEL8_Msk     (0x1UL << ADC_CHSELR_CHSEL8_Pos)              /*!< 0x00000100 */
#define ADC_CHSELR_CHSEL8         ADC_CHSELR_CHSEL8_Msk                        /*!< ADC group regular sequencer channel 8, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL7_Pos     (7U)
#define ADC_CHSELR_CHSEL7_Msk     (0x1UL << ADC_CHSELR_CHSEL7_Pos)              /*!< 0x00000080 */
#define ADC_CHSELR_CHSEL7         ADC_CHSELR_CHSEL7_Msk                        /*!< ADC group regular sequencer channel 7, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL6_Pos     (6U)
#define ADC_CHSELR_CHSEL6_Msk     (0x1UL << ADC_CHSELR_CHSEL6_Pos)              /*!< 0x00000040 */
#define ADC_CHSELR_CHSEL6         ADC_CHSELR_CHSEL6_Msk                        /*!< ADC group regular sequencer channel 6, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL5_Pos     (5U)
#define ADC_CHSELR_CHSEL5_Msk     (0x1UL << ADC_CHSELR_CHSEL5_Pos)              /*!< 0x00000020 */
#define ADC_CHSELR_CHSEL5         ADC_CHSELR_CHSEL5_Msk                        /*!< ADC group regular sequencer channel 5, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL4_Pos     (4U)
#define ADC_CHSELR_CHSEL4_Msk     (0x1UL << ADC_CHSELR_CHSEL4_Pos)              /*!< 0x00000010 */
#define ADC_CHSELR_CHSEL4         ADC_CHSELR_CHSEL4_Msk                        /*!< ADC group regular sequencer channel 4, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL3_Pos     (3U)
#define ADC_CHSELR_CHSEL3_Msk     (0x1UL << ADC_CHSELR_CHSEL3_Pos)              /*!< 0x00000008 */
#define ADC_CHSELR_CHSEL3         ADC_CHSELR_CHSEL3_Msk                        /*!< ADC group regular sequencer channel 3, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL2_Pos     (2U)
#define ADC_CHSELR_CHSEL2_Msk     (0x1UL << ADC_CHSELR_CHSEL2_Pos)              /*!< 0x00000004 */
#define ADC_CHSELR_CHSEL2         ADC_CHSELR_CHSEL2_Msk                        /*!< ADC group regular sequencer channel 2, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL1_Pos     (1U)
#define ADC_CHSELR_CHSEL1_Msk     (0x1UL << ADC_CHSELR_CHSEL1_Pos)              /*!< 0x00000002 */
#define ADC_CHSELR_CHSEL1         ADC_CHSELR_CHSEL1_Msk                        /*!< ADC group regular sequencer channel 1, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL0_Pos     (0U)
#define ADC_CHSELR_CHSEL0_Msk     (0x1UL << ADC_CHSELR_CHSEL0_Pos)              /*!< 0x00000001 */
#define ADC_CHSELR_CHSEL0         ADC_CHSELR_CHSEL0_Msk                        /*!< ADC group regular sequencer channel 0, available when ADC_CFGR1_CHSELRMOD is reset */

/********************  Bit definition for ADC_DR register  ********************/
#define ADC_DR_DATA_Pos           (0U)
#define ADC_DR_DATA_Msk           (0xFFFFUL << ADC_DR_DATA_Pos)                 /*!< 0x0000FFFF */
#define ADC_DR_DATA               ADC_DR_DATA_Msk                              /*!< ADC group regular conversion data */
#define ADC_DR_DATA_0             (0x0001UL << ADC_DR_DATA_Pos)                 /*!< 0x00000001 */
#define ADC_DR_DATA_1             (0x0002UL << ADC_DR_DATA_Pos)                 /*!< 0x00000002 */
#define ADC_DR_DATA_2             (0x0004UL << ADC_DR_DATA_Pos)                 /*!< 0x00000004 */
#define ADC_DR_DATA_3             (0x0008UL << ADC_DR_DATA_Pos)                 /*!< 0x00000008 */
#define ADC_DR_DATA_4             (0x0010UL << ADC_DR_DATA_Pos)                 /*!< 0x00000010 */
#define ADC_DR_DATA_5             (0x0020UL << ADC_DR_DATA_Pos)                 /*!< 0x00000020 */
#define ADC_DR_DATA_6             (0x0040UL << ADC_DR_DATA_Pos)                 /*!< 0x00000040 */
#define ADC_DR_DATA_7             (0x0080UL << ADC_DR_DATA_Pos)                 /*!< 0x00000080 */
#define ADC_DR_DATA_8             (0x0100UL << ADC_DR_DATA_Pos)                 /*!< 0x00000100 */
#define ADC_DR_DATA_9             (0x0200UL << ADC_DR_DATA_Pos)                 /*!< 0x00000200 */
#define ADC_DR_DATA_10            (0x0400UL << ADC_DR_DATA_Pos)                 /*!< 0x00000400 */
#define ADC_DR_DATA_11            (0x0800UL << ADC_DR_DATA_Pos)                 /*!< 0x00000800 */
#define ADC_DR_DATA_12            (0x1000UL << ADC_DR_DATA_Pos)                 /*!< 0x00001000 */
#define ADC_DR_DATA_13            (0x2000UL << ADC_DR_DATA_Pos)                 /*!< 0x00002000 */
#define ADC_DR_DATA_14            (0x4000UL << ADC_DR_DATA_Pos)                 /*!< 0x00004000 */
#define ADC_DR_DATA_15            (0x8000UL << ADC_DR_DATA_Pos)                 /*!< 0x00008000 */

/*************************  ADC Common registers  *****************************/
/*******************  Bit definition for ADC_CCR register  ********************/
#define ADC_CCR_VREFEN_Pos        (22U)
#define ADC_CCR_VREFEN_Msk        (0x1UL << ADC_CCR_VREFEN_Pos)                 /*!< 0x00400000 */
#define ADC_CCR_VREFEN            ADC_CCR_VREFEN_Msk                           /*!< ADC internal path to VrefInt enable */
#define ADC_CCR_TSEN_Pos          (23U)
#define ADC_CCR_TSEN_Msk          (0x1UL << ADC_CCR_TSEN_Pos)                   /*!< 0x00800000 */
#define ADC_CCR_TSEN              ADC_CCR_TSEN_Msk                             /*!< ADC internal path to temperature sensor enable */

#define ADC_CCR_VBATEN_Pos        (24U)
#define ADC_CCR_VBATEN_Msk        (0x1UL << ADC_CCR_VBATEN_Pos)                 /*!< 0x01000000 */
#define ADC_CCR_VBATEN            ADC_CCR_VBATEN_Msk                           /*!< ADC internal path to battery voltage enable */

/******************************************************************************/
/*                                                                            */
/*                   Controller Area Network (CAN )                           */
/*                                                                            */
/******************************************************************************/
/*!<CAN control and status registers */
/*******************  Bit definition for CAN_MCR register  ********************/
#define CAN_MCR_INRQ_Pos       (0U)
#define CAN_MCR_INRQ_Msk       (0x1UL << CAN_MCR_INRQ_Pos)                      /*!< 0x00000001 */
#define CAN_MCR_INRQ           CAN_MCR_INRQ_Msk                                /*!<Initialization Request */
#define CAN_MCR_SLEEP_Pos      (1U)
#define CAN_MCR_SLEEP_Msk      (0x1UL << CAN_MCR_SLEEP_Pos)                     /*!< 0x00000002 */
#define CAN_MCR_SLEEP          CAN_MCR_SLEEP_Msk                               /*!<Sleep Mode Request */
#define CAN_MCR_TXFP_Pos       (2U)
#define CAN_MCR_TXFP_Msk       (0x1UL << CAN_MCR_TXFP_Pos)                      /*!< 0x00000004 */
#define CAN_MCR_TXFP           CAN_MCR_TXFP_Msk                                /*!<Transmit FIFO Priority */
#define CAN_MCR_RFLM_Pos       (3U)
#define CAN_MCR_RFLM_Msk       (0x1UL << CAN_MCR_RFLM_Pos)                      /*!< 0x00000008 */
#define CAN_MCR_RFLM           CAN_MCR_RFLM_Msk                                /*!<Receive FIFO Locked Mode */
#define CAN_MCR_NART_Pos       (4U)
#define CAN_MCR_NART_Msk       (0x1UL << CAN_MCR_NART_Pos)                      /*!< 0x00000010 */
#define CAN_MCR_NART           CAN_MCR_NART_Msk                                /*!<No Automatic Retransmission */
#define CAN_MCR_AWUM_Pos       (5U)
#define CAN_MCR_AWUM_Msk       (0x1UL << CAN_MCR_AWUM_Pos)                      /*!< 0x00000020 */
#define CAN_MCR_AWUM           CAN_MCR_AWUM_Msk                                /*!<Automatic Wakeup Mode */
#define CAN_MCR_ABOM_Pos       (6U)
#define CAN_MCR_ABOM_Msk       (0x1UL << CAN_MCR_ABOM_Pos)                      /*!< 0x00000040 */
#define CAN_MCR_ABOM           CAN_MCR_ABOM_Msk                                /*!<Automatic Bus-Off Management */
#define CAN_MCR_TTCM_Pos       (7U)
#define CAN_MCR_TTCM_Msk       (0x1UL << CAN_MCR_TTCM_Pos)                      /*!< 0x00000080 */
#define CAN_MCR_TTCM           CAN_MCR_TTCM_Msk                                /*!<Time Triggered Communication Mode */
#define CAN_MCR_RESET_Pos      (15U)
#define CAN_MCR_RESET_Msk      (0x1UL << CAN_MCR_RESET_Pos)                     /*!< 0x00008000 */
#define CAN_MCR_RESET          CAN_MCR_RESET_Msk                               /*!<bxCAN software master reset */

/*******************  Bit definition for CAN_MSR register  ********************/
#define CAN_MSR_INAK_Pos       (0U)
#define CAN_MSR_INAK_Msk       (0x1UL << CAN_MSR_INAK_Pos)                      /*!< 0x00000001 */
#define CAN_MSR_INAK           CAN_MSR_INAK_Msk                                /*!<Initialization Acknowledge */
#define CAN_MSR_SLAK_Pos       (1U)
#define CAN_MSR_SLAK_Msk       (0x1UL << CAN_MSR_SLAK_Pos)                      /*!< 0x00000002 */
#define CAN_MSR_SLAK           CAN_MSR_SLAK_Msk                                /*!<Sleep Acknowledge */
#define CAN_MSR_ERRI_Pos       (2U)
#define CAN_MSR_ERRI_Msk       (0x1UL << CAN_MSR_ERRI_Pos)                      /*!< 0x00000004 */
#define CAN_MSR_ERRI           CAN_MSR_ERRI_Msk                                /*!<Error Interrupt */
#define CAN_MSR_WKUI_Pos       (3U)
#define CAN_MSR_WKUI_Msk       (0x1UL << CAN_MSR_WKUI_Pos)                      /*!< 0x00000008 */
#define CAN_MSR_WKUI           CAN_MSR_WKUI_Msk                                /*!<Wakeup Interrupt */
#define CAN_MSR_SLAKI_Pos      (4U)
#define CAN_MSR_SLAKI_Msk      (0x1UL << CAN_MSR_SLAKI_Pos)                     /*!< 0x00000010 */
#define CAN_MSR_SLAKI          CAN_MSR_SLAKI_Msk                               /*!<Sleep Acknowledge Interrupt */
#define CAN_MSR_TXM_Pos        (8U)
#define CAN_MSR_TXM_Msk        (0x1UL << CAN_MSR_TXM_Pos)                       /*!< 0x00000100 */
#define CAN_MSR_TXM            CAN_MSR_TXM_Msk                                 /*!<Transmit Mode */
#define CAN_MSR_RXM_Pos        (9U)
#define CAN_MSR_RXM_Msk        (0x1UL << CAN_MSR_RXM_Pos)                       /*!< 0x00000200 */
#define CAN_MSR_RXM            CAN_MSR_RXM_Msk                                 /*!<Receive Mode */
#define CAN_MSR_SAMP_Pos       (10U)
#define CAN_MSR_SAMP_Msk       (0x1UL << CAN_MSR_SAMP_Pos)                      /*!< 0x00000400 */
#define CAN_MSR_SAMP           CAN_MSR_SAMP_Msk                                /*!<Last Sample Point */
#define CAN_MSR_RX_Pos         (11U)
#define CAN_MSR_RX_Msk         (0x1UL << CAN_MSR_RX_Pos)                        /*!< 0x00000800 */
#define CAN_MSR_RX             CAN_MSR_RX_Msk                                  /*!<CAN Rx Signal */

/*******************  Bit definition for CAN_TSR register  ********************/
#define CAN_TSR_RQCP0_Pos      (0U)
#define CAN_TSR_RQCP0_Msk      (0x1UL << CAN_TSR_RQCP0_Pos)                     /*!< 0x00000001 */
#define CAN_TSR_RQCP0          CAN_TSR_RQCP0_Msk                               /*!<Request Completed Mailbox0 */
#define CAN_TSR_TXOK0_Pos      (1U)
#define CAN_TSR_TXOK0_Msk      (0x1UL << CAN_TSR_TXOK0_Pos)                     /*!< 0x00000002 */
#define CAN_TSR_TXOK0          CAN_TSR_TXOK0_Msk                               /*!<Transmission OK of Mailbox0 */
#define CAN_TSR_ALST0_Pos      (2U)
#define CAN_TSR_ALST0_Msk      (0x1UL << CAN_TSR_ALST0_Pos)                     /*!< 0x00000004 */
#define CAN_TSR_ALST0          CAN_TSR_ALST0_Msk                               /*!<Arbitration Lost for Mailbox0 */
#define CAN_TSR_TERR0_Pos      (3U)
#define CAN_TSR_TERR0_Msk      (0x1UL << CAN_TSR_TERR0_Pos)                     /*!< 0x00000008 */
#define CAN_TSR_TERR0          CAN_TSR_TERR0_Msk                               /*!<Transmission Error of Mailbox0 */
#define CAN_TSR_ABRQ0_Pos      (7U)
#define CAN_TSR_ABRQ0_Msk      (0x1UL << CAN_TSR_ABRQ0_Pos)                     /*!< 0x00000080 */
#define CAN_TSR_ABRQ0          CAN_TSR_ABRQ0_Msk                               /*!<Abort Request for Mailbox0 */
#define CAN_TSR_RQCP1_Pos      (8U)
#define CAN_TSR_RQCP1_Msk      (0x1UL << CAN_TSR_RQCP1_Pos)                     /*!< 0x00000100 */
#define CAN_TSR_RQCP1          CAN_TSR_RQCP1_Msk                               /*!<Request Completed Mailbox1 */
#define CAN_TSR_TXOK1_Pos      (9U)
#define CAN_TSR_TXOK1_Msk      (0x1UL << CAN_TSR_TXOK1_Pos)                     /*!< 0x00000200 */
#define CAN_TSR_TXOK1          CAN_TSR_TXOK1_Msk                               /*!<Transmission OK of Mailbox1 */
#define CAN_TSR_ALST1_Pos      (10U)
#define CAN_TSR_ALST1_Msk      (0x1UL << CAN_TSR_ALST1_Pos)                     /*!< 0x00000400 */
#define CAN_TSR_ALST1          CAN_TSR_ALST1_Msk                               /*!<Arbitration Lost for Mailbox1 */
#define CAN_TSR_TERR1_Pos      (11U)
#define CAN_TSR_TERR1_Msk      (0x1UL << CAN_TSR_TERR1_Pos)                     /*!< 0x00000800 */
#define CAN_TSR_TERR1          CAN_TSR_TERR1_Msk                               /*!<Transmission Error of Mailbox1 */
#define CAN_TSR_ABRQ1_Pos      (15U)
#define CAN_TSR_ABRQ1_Msk      (0x1UL << CAN_TSR_ABRQ1_Pos)                     /*!< 0x00008000 */
#define CAN_TSR_ABRQ1          CAN_TSR_ABRQ1_Msk                               /*!<Abort Request for Mailbox 1 */
#define CAN_TSR_RQCP2_Pos      (16U)
#define CAN_TSR_RQCP2_Msk      (0x1UL << CAN_TSR_RQCP2_Pos)                     /*!< 0x00010000 */
#define CAN_TSR_RQCP2          CAN_TSR_RQCP2_Msk                               /*!<Request Completed Mailbox2 */
#define CAN_TSR_TXOK2_Pos      (17U)
#define CAN_TSR_TXOK2_Msk      (0x1UL << CAN_TSR_TXOK2_Pos)                     /*!< 0x00020000 */
#define CAN_TSR_TXOK2          CAN_TSR_TXOK2_Msk                               /*!<Transmission OK of Mailbox 2 */
#define CAN_TSR_ALST2_Pos      (18U)
#define CAN_TSR_ALST2_Msk      (0x1UL << CAN_TSR_ALST2_Pos)                     /*!< 0x00040000 */
#define CAN_TSR_ALST2          CAN_TSR_ALST2_Msk                               /*!<Arbitration Lost for mailbox 2 */
#define CAN_TSR_TERR2_Pos      (19U)
#define CAN_TSR_TERR2_Msk      (0x1UL << CAN_TSR_TERR2_Pos)                     /*!< 0x00080000 */
#define CAN_TSR_TERR2          CAN_TSR_TERR2_Msk                               /*!<Transmission Error of Mailbox 2 */
#define CAN_TSR_ABRQ2_Pos      (23U)
#define CAN_TSR_ABRQ2_Msk      (0x1UL << CAN_TSR_ABRQ2_Pos)                     /*!< 0x00800000 */
#define CAN_TSR_ABRQ2          CAN_TSR_ABRQ2_Msk                               /*!<Abort Request for Mailbox 2 */
#define CAN_TSR_CODE_Pos       (24U)
#define CAN_TSR_CODE_Msk       (0x3UL << CAN_TSR_CODE_Pos)                      /*!< 0x03000000 */
#define CAN_TSR_CODE           CAN_TSR_CODE_Msk                                /*!<Mailbox Code */

#define CAN_TSR_TME_Pos        (26U)
#define CAN_TSR_TME_Msk        (0x7UL << CAN_TSR_TME_Pos)                       /*!< 0x1C000000 */
#define CAN_TSR_TME            CAN_TSR_TME_Msk                                 /*!<TME[2:0] bits */
#define CAN_TSR_TME0_Pos       (26U)
#define CAN_TSR_TME0_Msk       (0x1UL << CAN_TSR_TME0_Pos)                      /*!< 0x04000000 */
#define CAN_TSR_TME0           CAN_TSR_TME0_Msk                                /*!<Transmit Mailbox 0 Empty */
#define CAN_TSR_TME1_Pos       (27U)
#define CAN_TSR_TME1_Msk       (0x1UL << CAN_TSR_TME1_Pos)                      /*!< 0x08000000 */
#define CAN_TSR_TME1           CAN_TSR_TME1_Msk                                /*!<Transmit Mailbox 1 Empty */
#define CAN_TSR_TME2_Pos       (28U)
#define CAN_TSR_TME2_Msk       (0x1UL << CAN_TSR_TME2_Pos)                      /*!< 0x10000000 */
#define CAN_TSR_TME2           CAN_TSR_TME2_Msk                                /*!<Transmit Mailbox 2 Empty */

#define CAN_TSR_LOW_Pos        (29U)
#define CAN_TSR_LOW_Msk        (0x7UL << CAN_TSR_LOW_Pos)                       /*!< 0xE0000000 */
#define CAN_TSR_LOW            CAN_TSR_LOW_Msk                                 /*!<LOW[2:0] bits */
#define CAN_TSR_LOW0_Pos       (29U)
#define CAN_TSR_LOW0_Msk       (0x1UL << CAN_TSR_LOW0_Pos)                      /*!< 0x20000000 */
#define CAN_TSR_LOW0           CAN_TSR_LOW0_Msk                                /*!<Lowest Priority Flag for Mailbox 0 */
#define CAN_TSR_LOW1_Pos       (30U)
#define CAN_TSR_LOW1_Msk       (0x1UL << CAN_TSR_LOW1_Pos)                      /*!< 0x40000000 */
#define CAN_TSR_LOW1           CAN_TSR_LOW1_Msk                                /*!<Lowest Priority Flag for Mailbox 1 */
#define CAN_TSR_LOW2_Pos       (31U)
#define CAN_TSR_LOW2_Msk       (0x1UL << CAN_TSR_LOW2_Pos)                      /*!< 0x80000000 */
#define CAN_TSR_LOW2           CAN_TSR_LOW2_Msk                                /*!<Lowest Priority Flag for Mailbox 2 */

/*******************  Bit definition for CAN_RF0R register  *******************/
#define CAN_RF0R_FMP0_Pos      (0U)
#define CAN_RF0R_FMP0_Msk      (0x3UL << CAN_RF0R_FMP0_Pos)                     /*!< 0x00000003 */
#define CAN_RF0R_FMP0          CAN_RF0R_FMP0_Msk                               /*!<FIFO 0 Message Pending */
#define CAN_RF0R_FULL0_Pos     (3U)
#define CAN_RF0R_FULL0_Msk     (0x1UL << CAN_RF0R_FULL0_Pos)                    /*!< 0x00000008 */
#define CAN_RF0R_FULL0         CAN_RF0R_FULL0_Msk                              /*!<FIFO 0 Full */
#define CAN_RF0R_FOVR0_Pos     (4U)
#define CAN_RF0R_FOVR0_Msk     (0x1UL << CAN_RF0R_FOVR0_Pos)                    /*!< 0x00000010 */
#define CAN_RF0R_FOVR0         CAN_RF0R_FOVR0_Msk                              /*!<FIFO 0 Overrun */
#define CAN_RF0R_RFOM0_Pos     (5U)
#define CAN_RF0R_RFOM0_Msk     (0x1UL << CAN_RF0R_RFOM0_Pos)                    /*!< 0x00000020 */
#define CAN_RF0R_RFOM0         CAN_RF0R_RFOM0_Msk                              /*!<Release FIFO 0 Output Mailbox */

/*******************  Bit definition for CAN_RF1R register  *******************/
#define CAN_RF1R_FMP1_Pos      (0U)
#define CAN_RF1R_FMP1_Msk      (0x3UL << CAN_RF1R_FMP1_Pos)                     /*!< 0x00000003 */
#define CAN_RF1R_FMP1          CAN_RF1R_FMP1_Msk                               /*!<FIFO 1 Message Pending */
#define CAN_RF1R_FULL1_Pos     (3U)
#define CAN_RF1R_FULL1_Msk     (0x1UL << CAN_RF1R_FULL1_Pos)                    /*!< 0x00000008 */
#define CAN_RF1R_FULL1         CAN_RF1R_FULL1_Msk                              /*!<FIFO 1 Full */
#define CAN_RF1R_FOVR1_Pos     (4U)
#define CAN_RF1R_FOVR1_Msk     (0x1UL << CAN_RF1R_FOVR1_Pos)                    /*!< 0x00000010 */
#define CAN_RF1R_FOVR1         CAN_RF1R_FOVR1_Msk                              /*!<FIFO 1 Overrun */
#define CAN_RF1R_RFOM1_Pos     (5U)
#define CAN_RF1R_RFOM1_Msk     (0x1UL << CAN_RF1R_RFOM1_Pos)                    /*!< 0x00000020 */
#define CAN_RF1R_RFOM1         CAN_RF1R_RFOM1_Msk                              /*!<Release FIFO 1 Output Mailbox */

/********************  Bit definition for CAN_IER register  *******************/
#define CAN_IER_TMEIE_Pos      (0U)
#define CAN_IER_TMEIE_Msk      (0x1UL << CAN_IER_TMEIE_Pos)                     /*!< 0x00000001 */
#define CAN_IER_TMEIE          CAN_IER_TMEIE_Msk                               /*!<Transmit Mailbox Empty Interrupt Enable */
#define CAN_IER_FMPIE0_Pos     (1U)
#define CAN_IER_FMPIE0_Msk     (0x1UL << CAN_IER_FMPIE0_Pos)                    /*!< 0x00000002 */
#define CAN_IER_FMPIE0         CAN_IER_FMPIE0_Msk                              /*!<FIFO Message Pending Interrupt Enable */
#define CAN_IER_FFIE0_Pos      (2U)
#define CAN_IER_FFIE0_Msk      (0x1UL << CAN_IER_FFIE0_Pos)                     /*!< 0x00000004 */
#define CAN_IER_FFIE0          CAN_IER_FFIE0_Msk                               /*!<FIFO Full Interrupt Enable */
#define CAN_IER_FOVIE0_Pos     (3U)
#define CAN_IER_FOVIE0_Msk     (0x1UL << CAN_IER_FOVIE0_Pos)                    /*!< 0x00000008 */
#define CAN_IER_FOVIE0         CAN_IER_FOVIE0_Msk                              /*!<FIFO Overrun Interrupt Enable */
#define CAN_IER_FMPIE1_Pos     (4U)
#define CAN_IER_FMPIE1_Msk     (0x1UL << CAN_IER_FMPIE1_Pos)                    /*!< 0x00000010 */
#define CAN_IER_FMPIE1         CAN_IER_FMPIE1_Msk                              /*!<FIFO Message Pending Interrupt Enable */
#define CAN_IER_FFIE1_Pos      (5U)
#define CAN_IER_FFIE1_Msk      (0x1UL << CAN_IER_FFIE1_Pos)                     /*!< 0x00000020 */
#define CAN_IER_FFIE1          CAN_IER_FFIE1_Msk                               /*!<FIFO Full Interrupt Enable */
#define CAN_IER_FOVIE1_Pos     (6U)
#define CAN_IER_FOVIE1_Msk     (0x1UL << CAN_IER_FOVIE1_Pos)                    /*!< 0x00000040 */
#define CAN_IER_FOVIE1         CAN_IER_FOVIE1_Msk                              /*!<FIFO Overrun Interrupt Enable */
#define CAN_IER_EWGIE_Pos      (8U)
#define CAN_IER_EWGIE_Msk      (0x1UL << CAN_IER_EWGIE_Pos)                     /*!< 0x00000100 */
#define CAN_IER_EWGIE          CAN_IER_EWGIE_Msk                               /*!<Error Warning Interrupt Enable */
#define CAN_IER_EPVIE_Pos      (9U)
#define CAN_IER_EPVIE_Msk      (0x1UL << CAN_IER_EPVIE_Pos)                     /*!< 0x00000200 */
#define CAN_IER_EPVIE          CAN_IER_EPVIE_Msk                               /*!<Error Passive Interrupt Enable */
#define CAN_IER_BOFIE_Pos      (10U)
#define CAN_IER_BOFIE_Msk      (0x1UL << CAN_IER_BOFIE_Pos)                     /*!< 0x00000400 */
#define CAN_IER_BOFIE          CAN_IER_BOFIE_Msk                               /*!<Bus-Off Interrupt Enable */
#define CAN_IER_LECIE_Pos      (11U)
#define CAN_IER_LECIE_Msk      (0x1UL << CAN_IER_LECIE_Pos)                     /*!< 0x00000800 */
#define CAN_IER_LECIE          CAN_IER_LECIE_Msk                               /*!<Last Error Code Interrupt Enable */
#define CAN_IER_ERRIE_Pos      (15U)
#define CAN_IER_ERRIE_Msk      (0x1UL << CAN_IER_ERRIE_Pos)                     /*!< 0x00008000 */
#define CAN_IER_ERRIE          CAN_IER_ERRIE_Msk                               /*!<Error Interrupt Enable */
#define CAN_IER_WKUIE_Pos      (16U)
#define CAN_IER_WKUIE_Msk      (0x1UL << CAN_IER_WKUIE_Pos)                     /*!< 0x00010000 */
#define CAN_IER_WKUIE          CAN_IER_WKUIE_Msk                               /*!<Wakeup Interrupt Enable */
#define CAN_IER_SLKIE_Pos      (17U)
#define CAN_IER_SLKIE_Msk      (0x1UL << CAN_IER_SLKIE_Pos)                     /*!< 0x00020000 */
#define CAN_IER_SLKIE          CAN_IER_SLKIE_Msk                               /*!<Sleep Interrupt Enable */

/********************  Bit definition for CAN_ESR register  *******************/
#define CAN_ESR_EWGF_Pos       (0U)
#define CAN_ESR_EWGF_Msk       (0x1UL << CAN_ESR_EWGF_Pos)                      /*!< 0x00000001 */
#define CAN_ESR_EWGF           CAN_ESR_EWGF_Msk                                /*!<Error Warning Flag */
#define CAN_ESR_EPVF_Pos       (1U)
#define CAN_ESR_EPVF_Msk       (0x1UL << CAN_ESR_EPVF_Pos)                      /*!< 0x00000002 */
#define CAN_ESR_EPVF           CAN_ESR_EPVF_Msk                                /*!<Error Passive Flag */
#define CAN_ESR_BOFF_Pos       (2U)
#define CAN_ESR_BOFF_Msk       (0x1UL << CAN_ESR_BOFF_Pos)                      /*!< 0x00000004 */
#define CAN_ESR_BOFF           CAN_ESR_BOFF_Msk                                /*!<Bus-Off Flag */

#define CAN_ESR_LEC_Pos        (4U)
#define CAN_ESR_LEC_Msk        (0x7UL << CAN_ESR_LEC_Pos)                       /*!< 0x00000070 */
#define CAN_ESR_LEC            CAN_ESR_LEC_Msk                                 /*!<LEC[2:0] bits (Last Error Code) */
#define CAN_ESR_LEC_0          (0x1UL << CAN_ESR_LEC_Pos)                       /*!< 0x00000010 */
#define CAN_ESR_LEC_1          (0x2UL << CAN_ESR_LEC_Pos)                       /*!< 0x00000020 */
#define CAN_ESR_LEC_2          (0x4UL << CAN_ESR_LEC_Pos)                       /*!< 0x00000040 */

#define CAN_ESR_TEC_Pos        (16U)
#define CAN_ESR_TEC_Msk        (0xFFUL << CAN_ESR_TEC_Pos)                      /*!< 0x00FF0000 */
#define CAN_ESR_TEC            CAN_ESR_TEC_Msk                                 /*!<Least significant byte of the 9-bit Transmit Error Counter */
#define CAN_ESR_REC_Pos        (24U)
#define CAN_ESR_REC_Msk        (0xFFUL << CAN_ESR_REC_Pos)                      /*!< 0xFF000000 */
#define CAN_ESR_REC            CAN_ESR_REC_Msk                                 /*!<Receive Error Counter */

/*******************  Bit definition for CAN_BTR register  ********************/
#define CAN_BTR_BRP_Pos        (0U)
#define CAN_BTR_BRP_Msk        (0x3FFUL << CAN_BTR_BRP_Pos)                     /*!< 0x000003FF */
#define CAN_BTR_BRP            CAN_BTR_BRP_Msk                                 /*!<Baud Rate Prescaler */
#define CAN_BTR_TS1_Pos        (16U)
#define CAN_BTR_TS1_Msk        (0xFUL << CAN_BTR_TS1_Pos)                       /*!< 0x000F0000 */
#define CAN_BTR_TS1            CAN_BTR_TS1_Msk                                 /*!<Time Segment 1 */
#define CAN_BTR_TS1_0          (0x1UL << CAN_BTR_TS1_Pos)                       /*!< 0x00010000 */
#define CAN_BTR_TS1_1          (0x2UL << CAN_BTR_TS1_Pos)                       /*!< 0x00020000 */
#define CAN_BTR_TS1_2          (0x4UL << CAN_BTR_TS1_Pos)                       /*!< 0x00040000 */
#define CAN_BTR_TS1_3          (0x8UL << CAN_BTR_TS1_Pos)                       /*!< 0x00080000 */
#define CAN_BTR_TS2_Pos        (20U)
#define CAN_BTR_TS2_Msk        (0x7UL << CAN_BTR_TS2_Pos)                       /*!< 0x00700000 */
#define CAN_BTR_TS2            CAN_BTR_TS2_Msk                                 /*!<Time Segment 2 */
#define CAN_BTR_TS2_0          (0x1UL << CAN_BTR_TS2_Pos)                       /*!< 0x00100000 */
#define CAN_BTR_TS2_1          (0x2UL << CAN_BTR_TS2_Pos)                       /*!< 0x00200000 */
#define CAN_BTR_TS2_2          (0x4UL << CAN_BTR_TS2_Pos)                       /*!< 0x00400000 */
#define CAN_BTR_SJW_Pos        (24U)
#define CAN_BTR_SJW_Msk        (0x3UL << CAN_BTR_SJW_Pos)                       /*!< 0x03000000 */
#define CAN_BTR_SJW            CAN_BTR_SJW_Msk                                 /*!<Resynchronization Jump Width */
#define CAN_BTR_SJW_0          (0x1UL << CAN_BTR_SJW_Pos)                       /*!< 0x01000000 */
#define CAN_BTR_SJW_1          (0x2UL << CAN_BTR_SJW_Pos)                       /*!< 0x02000000 */
#define CAN_BTR_LBKM_Pos       (30U)
#define CAN_BTR_LBKM_Msk       (0x1UL << CAN_BTR_LBKM_Pos)                      /*!< 0x40000000 */
#define CAN_BTR_LBKM           CAN_BTR_LBKM_Msk                                /*!<Loop Back Mode (Debug) */
#define CAN_BTR_SILM_Pos       (31U)
#define CAN_BTR_SILM_Msk       (0x1UL << CAN_BTR_SILM_Pos)                      /*!< 0x80000000 */
#define CAN_BTR_SILM           CAN_BTR_SILM_Msk                                /*!<Silent Mode */

/*!<Mailbox registers */
/******************  Bit definition for CAN_TI0R register  ********************/
#define CAN_TI0R_TXRQ_Pos      (0U)
#define CAN_TI0R_TXRQ_Msk      (0x1UL << CAN_TI0R_TXRQ_Pos)                     /*!< 0x00000001 */
#define CAN_TI0R_TXRQ          CAN_TI0R_TXRQ_Msk                               /*!<Transmit Mailbox Request */
#define CAN_TI0R_RTR_Pos       (1U)
#define CAN_TI0R_RTR_Msk       (0x1UL << CAN_TI0R_RTR_Pos)                      /*!< 0x00000002 */
#define CAN_TI0R_RTR           CAN_TI0R_RTR_Msk                                /*!<Remote Transmission Request */
#define CAN_TI0R_IDE_Pos       (2U)
#define CAN_TI0R_IDE_Msk       (0x1UL << CAN_TI0R_IDE_Pos)                      /*!< 0x00000004 */
#define CAN_TI0R_IDE           CAN_TI0R_IDE_Msk                                /*!<Identifier Extension */
#define CAN_TI0R_EXID_Pos      (3U)
#define CAN_TI0R_EXID_Msk      (0x3FFFFUL << CAN_TI0R_EXID_Pos)                 /*!< 0x001FFFF8 */
#define CAN_TI0R_EXID          CAN_TI0R_EXID_Msk                               /*!<Extended Identifier */
#define CAN_TI0R_STID_Pos      (21U)
#define CAN_TI0R_STID_Msk      (0x7FFUL << CAN_TI0R_STID_Pos)                   /*!< 0xFFE00000 */
#define CAN_TI0R_STID          CAN_TI0R_STID_Msk                               /*!<Standard Identifier or Extended Identifier */

/******************  Bit definition for CAN_TDT0R register  *******************/
#define CAN_TDT0R_DLC_Pos      (0U)
#define CAN_TDT0R_DLC_Msk      (0xFUL << CAN_TDT0R_DLC_Pos)                     /*!< 0x0000000F */
#define CAN_TDT0R_DLC          CAN_TDT0R_DLC_Msk                               /*!<Data Length Code */
#define CAN_TDT0R_TGT_Pos      (8U)
#define CAN_TDT0R_TGT_Msk      (0x1UL << CAN_TDT0R_TGT_Pos)                     /*!< 0x00000100 */
#define CAN_TDT0R_TGT          CAN_TDT0R_TGT_Msk                               /*!<Transmit Global Time */
#define CAN_TDT0R_TIME_Pos     (16U)
#define CAN_TDT0R_TIME_Msk     (0xFFFFUL << CAN_TDT0R_TIME_Pos)                 /*!< 0xFFFF0000 */
#define CAN_TDT0R_TIME         CAN_TDT0R_TIME_Msk                              /*!<Message Time Stamp */

/******************  Bit definition for CAN_TDL0R register  *******************/
#define CAN_TDL0R_DATA0_Pos    (0U)
#define CAN_TDL0R_DATA0_Msk    (0xFFUL << CAN_TDL0R_DATA0_Pos)                  /*!< 0x000000FF */
#define CAN_TDL0R_DATA0        CAN_TDL0R_DATA0_Msk                             /*!<Data byte 0 */
#define CAN_TDL0R_DATA1_Pos    (8U)
#define CAN_TDL0R_DATA1_Msk    (0xFFUL << CAN_TDL0R_DATA1_Pos)                  /*!< 0x0000FF00 */
#define CAN_TDL0R_DATA1        CAN_TDL0R_DATA1_Msk                             /*!<Data byte 1 */
#define CAN_TDL0R_DATA2_Pos    (16U)
#define CAN_TDL0R_DATA2_Msk    (0xFFUL << CAN_TDL0R_DATA2_Pos)                  /*!< 0x00FF0000 */
#define CAN_TDL0R_DATA2        CAN_TDL0R_DATA2_Msk                             /*!<Data byte 2 */
#define CAN_TDL0R_DATA3_Pos    (24U)
#define CAN_TDL0R_DATA3_Msk    (0xFFUL << CAN_TDL0R_DATA3_Pos)                  /*!< 0xFF000000 */
#define CAN_TDL0R_DATA3        CAN_TDL0R_DATA3_Msk                             /*!<Data byte 3 */

/******************  Bit definition for CAN_TDH0R register  *******************/
#define CAN_TDH0R_DATA4_Pos    (0U)
#define CAN_TDH0R_DATA4_Msk    (0xFFUL << CAN_TDH0R_DATA4_Pos)                  /*!< 0x000000FF */
#define CAN_TDH0R_DATA4        CAN_TDH0R_DATA4_Msk                             /*!<Data byte 4 */
#define CAN_TDH0R_DATA5_Pos    (8U)
#define CAN_TDH0R_DATA5_Msk    (0xFFUL << CAN_TDH0R_DATA5_Pos)                  /*!< 0x0000FF00 */
#define CAN_TDH0R_DATA5        CAN_TDH0R_DATA5_Msk                             /*!<Data byte 5 */
#define CAN_TDH0R_DATA6_Pos    (16U)
#define CAN_TDH0R_DATA6_Msk    (0xFFUL << CAN_TDH0R_DATA6_Pos)                  /*!< 0x00FF0000 */
#define CAN_TDH0R_DATA6        CAN_TDH0R_DATA6_Msk                             /*!<Data byte 6 */
#define CAN_TDH0R_DATA7_Pos    (24U)
#define CAN_TDH0R_DATA7_Msk    (0xFFUL << CAN_TDH0R_DATA7_Pos)                  /*!< 0xFF000000 */
#define CAN_TDH0R_DATA7        CAN_TDH0R_DATA7_Msk                             /*!<Data byte 7 */

/*******************  Bit definition for CAN_TI1R register  *******************/
#define CAN_TI1R_TXRQ_Pos      (0U)
#define CAN_TI1R_TXRQ_Msk      (0x1UL << CAN_TI1R_TXRQ_Pos)                     /*!< 0x00000001 */
#define CAN_TI1R_TXRQ          CAN_TI1R_TXRQ_Msk                               /*!<Transmit Mailbox Request */
#define CAN_TI1R_RTR_Pos       (1U)
#define CAN_TI1R_RTR_Msk       (0x1UL << CAN_TI1R_RTR_Pos)                      /*!< 0x00000002 */
#define CAN_TI1R_RTR           CAN_TI1R_RTR_Msk                                /*!<Remote Transmission Request */
#define CAN_TI1R_IDE_Pos       (2U)
#define CAN_TI1R_IDE_Msk       (0x1UL << CAN_TI1R_IDE_Pos)                      /*!< 0x00000004 */
#define CAN_TI1R_IDE           CAN_TI1R_IDE_Msk                                /*!<Identifier Extension */
#define CAN_TI1R_EXID_Pos      (3U)
#define CAN_TI1R_EXID_Msk      (0x3FFFFUL << CAN_TI1R_EXID_Pos)                 /*!< 0x001FFFF8 */
#define CAN_TI1R_EXID          CAN_TI1R_EXID_Msk                               /*!<Extended Identifier */
#define CAN_TI1R_STID_Pos      (21U)
#define CAN_TI1R_STID_Msk      (0x7FFUL << CAN_TI1R_STID_Pos)                   /*!< 0xFFE00000 */
#define CAN_TI1R_STID          CAN_TI1R_STID_Msk                               /*!<Standard Identifier or Extended Identifier */

/*******************  Bit definition for CAN_TDT1R register  ******************/
#define CAN_TDT1R_DLC_Pos      (0U)
#define CAN_TDT1R_DLC_Msk      (0xFUL << CAN_TDT1R_DLC_Pos)                     /*!< 0x0000000F */
#define CAN_TDT1R_DLC          CAN_TDT1R_DLC_Msk                               /*!<Data Length Code */
#define CAN_TDT1R_TGT_Pos      (8U)
#define CAN_TDT1R_TGT_Msk      (0x1UL << CAN_TDT1R_TGT_Pos)                     /*!< 0x00000100 */
#define CAN_TDT1R_TGT          CAN_TDT1R_TGT_Msk                               /*!<Transmit Global Time */
#define CAN_TDT1R_TIME_Pos     (16U)
#define CAN_TDT1R_TIME_Msk     (0xFFFFUL << CAN_TDT1R_TIME_Pos)                 /*!< 0xFFFF0000 */
#define CAN_TDT1R_TIME         CAN_TDT1R_TIME_Msk                              /*!<Message Time Stamp */

/*******************  Bit definition for CAN_TDL1R register  ******************/
#define CAN_TDL1R_DATA0_Pos    (0U)
#define CAN_TDL1R_DATA0_Msk    (0xFFUL << CAN_TDL1R_DATA0_Pos)                  /*!< 0x000000FF */
#define CAN_TDL1R_DATA0        CAN_TDL1R_DATA0_Msk                             /*!<Data byte 0 */
#define CAN_TDL1R_DATA1_Pos    (8U)
#define CAN_TDL1R_DATA1_Msk    (0xFFUL << CAN_TDL1R_DATA1_Pos)                  /*!< 0x0000FF00 */
#define CAN_TDL1R_DATA1        CAN_TDL1R_DATA1_Msk                             /*!<Data byte 1 */
#define CAN_TDL1R_DATA2_Pos    (16U)
#define CAN_TDL1R_DATA2_Msk    (0xFFUL << CAN_TDL1R_DATA2_Pos)                  /*!< 0x00FF0000 */
#define CAN_TDL1R_DATA2        CAN_TDL1R_DATA2_Msk                             /*!<Data byte 2 */
#define CAN_TDL1R_DATA3_Pos    (24U)
#define CAN_TDL1R_DATA3_Msk    (0xFFUL << CAN_TDL1R_DATA3_Pos)                  /*!< 0xFF000000 */
#define CAN_TDL1R_DATA3        CAN_TDL1R_DATA3_Msk                             /*!<Data byte 3 */

/*******************  Bit definition for CAN_TDH1R register  ******************/
#define CAN_TDH1R_DATA4_Pos    (0U)
#define CAN_TDH1R_DATA4_Msk    (0xFFUL << CAN_TDH1R_DATA4_Pos)                  /*!< 0x000000FF */
#define CAN_TDH1R_DATA4        CAN_TDH1R_DATA4_Msk                             /*!<Data byte 4 */
#define CAN_TDH1R_DATA5_Pos    (8U)
#define CAN_TDH1R_DATA5_Msk    (0xFFUL << CAN_TDH1R_DATA5_Pos)                  /*!< 0x0000FF00 */
#define CAN_TDH1R_DATA5        CAN_TDH1R_DATA5_Msk                             /*!<Data byte 5 */
#define CAN_TDH1R_DATA6_Pos    (16U)
#define CAN_TDH1R_DATA6_Msk    (0xFFUL << CAN_TDH1R_DATA6_Pos)                  /*!< 0x00FF0000 */
#define CAN_TDH1R_DATA6        CAN_TDH1R_DATA6_Msk                             /*!<Data byte 6 */
#define CAN_TDH1R_DATA7_Pos    (24U)
#define CAN_TDH1R_DATA7_Msk    (0xFFUL << CAN_TDH1R_DATA7_Pos)                  /*!< 0xFF000000 */
#define CAN_TDH1R_DATA7        CAN_TDH1R_DATA7_Msk                             /*!<Data byte 7 */

/*******************  Bit definition for CAN_TI2R register  *******************/
#define CAN_TI2R_TXRQ_Pos      (0U)
#define CAN_TI2R_TXRQ_Msk      (0x1UL << CAN_TI2R_TXRQ_Pos)                     /*!< 0x00000001 */
#define CAN_TI2R_TXRQ          CAN_TI2R_TXRQ_Msk                               /*!<Transmit Mailbox Request */
#define CAN_TI2R_RTR_Pos       (1U)
#define CAN_TI2R_RTR_Msk       (0x1UL << CAN_TI2R_RTR_Pos)                      /*!< 0x00000002 */
#define CAN_TI2R_RTR           CAN_TI2R_RTR_Msk                                /*!<Remote Transmission Request */
#define CAN_TI2R_IDE_Pos       (2U)
#define CAN_TI2R_IDE_Msk       (0x1UL << CAN_TI2R_IDE_Pos)                      /*!< 0x00000004 */
#define CAN_TI2R_IDE           CAN_TI2R_IDE_Msk                                /*!<Identifier Extension */
#define CAN_TI2R_EXID_Pos      (3U)
#define CAN_TI2R_EXID_Msk      (0x3FFFFUL << CAN_TI2R_EXID_Pos)                 /*!< 0x001FFFF8 */
#define CAN_TI2R_EXID          CAN_TI2R_EXID_Msk                               /*!<Extended identifier */
#define CAN_TI2R_STID_Pos      (21U)
#define CAN_TI2R_STID_Msk      (0x7FFUL << CAN_TI2R_STID_Pos)                   /*!< 0xFFE00000 */
#define CAN_TI2R_STID          CAN_TI2R_STID_Msk                               /*!<Standard Identifier or Extended Identifier */

/*******************  Bit definition for CAN_TDT2R register  ******************/
#define CAN_TDT2R_DLC_Pos      (0U)
#define CAN_TDT2R_DLC_Msk      (0xFUL << CAN_TDT2R_DLC_Pos)                     /*!< 0x0000000F */
#define CAN_TDT2R_DLC          CAN_TDT2R_DLC_Msk                               /*!<Data Length Code */
#define CAN_TDT2R_TGT_Pos      (8U)
#define CAN_TDT2R_TGT_Msk      (0x1UL << CAN_TDT2R_TGT_Pos)                     /*!< 0x00000100 */
#define CAN_TDT2R_TGT          CAN_TDT2R_TGT_Msk                               /*!<Transmit Global Time */
#define CAN_TDT2R_TIME_Pos     (16U)
#define CAN_TDT2R_TIME_Msk     (0xFFFFUL << CAN_TDT2R_TIME_Pos)                 /*!< 0xFFFF0000 */
#define CAN_TDT2R_TIME         CAN_TDT2R_TIME_Msk                              /*!<Message Time Stamp */

/*******************  Bit definition for CAN_TDL2R register  ******************/
#define CAN_TDL2R_DATA0_Pos    (0U)
#define CAN_TDL2R_DATA0_Msk    (0xFFUL << CAN_TDL2R_DATA0_Pos)                  /*!< 0x000000FF */
#define CAN_TDL2R_DATA0        CAN_TDL2R_DATA0_Msk                             /*!<Data byte 0 */
#define CAN_TDL2R_DATA1_Pos    (8U)
#define CAN_TDL2R_DATA1_Msk    (0xFFUL << CAN_TDL2R_DATA1_Pos)                  /*!< 0x0000FF00 */
#define CAN_TDL2R_DATA1        CAN_TDL2R_DATA1_Msk                             /*!<Data byte 1 */
#define CAN_TDL2R_DATA2_Pos    (16U)
#define CAN_TDL2R_DATA2_Msk    (0xFFUL << CAN_TDL2R_DATA2_Pos)                  /*!< 0x00FF0000 */
#define CAN_TDL2R_DATA2        CAN_TDL2R_DATA2_Msk                             /*!<Data byte 2 */
#define CAN_TDL2R_DATA3_Pos    (24U)
#define CAN_TDL2R_DATA3_Msk    (0xFFUL << CAN_TDL2R_DATA3_Pos)                  /*!< 0xFF000000 */
#define CAN_TDL2R_DATA3        CAN_TDL2R_DATA3_Msk                             /*!<Data byte 3 */

/*******************  Bit definition for CAN_TDH2R register  ******************/
#define CAN_TDH2R_DATA4_Pos    (0U)
#define CAN_TDH2R_DATA4_Msk    (0xFFUL << CAN_TDH2R_DATA4_Pos)                  /*!< 0x000000FF */
#define CAN_TDH2R_DATA4        CAN_TDH2R_DATA4_Msk                             /*!<Data byte 4 */
#define CAN_TDH2R_DATA5_Pos    (8U)
#define CAN_TDH2R_DATA5_Msk    (0xFFUL << CAN_TDH2R_DATA5_Pos)                  /*!< 0x0000FF00 */
#define CAN_TDH2R_DATA5        CAN_TDH2R_DATA5_Msk                             /*!<Data byte 5 */
#define CAN_TDH2R_DATA6_Pos    (16U)
#define CAN_TDH2R_DATA6_Msk    (0xFFUL << CAN_TDH2R_DATA6_Pos)                  /*!< 0x00FF0000 */
#define CAN_TDH2R_DATA6        CAN_TDH2R_DATA6_Msk                             /*!<Data byte 6 */
#define CAN_TDH2R_DATA7_Pos    (24U)
#define CAN_TDH2R_DATA7_Msk    (0xFFUL << CAN_TDH2R_DATA7_Pos)                  /*!< 0xFF000000 */
#define CAN_TDH2R_DATA7        CAN_TDH2R_DATA7_Msk                             /*!<Data byte 7 */

/*******************  Bit definition for CAN_RI0R register  *******************/
#define CAN_RI0R_RTR_Pos       (1U)
#define CAN_RI0R_RTR_Msk       (0x1UL << CAN_RI0R_RTR_Pos)                      /*!< 0x00000002 */
#define CAN_RI0R_RTR           CAN_RI0R_RTR_Msk                                /*!<Remote Transmission Request */
#define CAN_RI0R_IDE_Pos       (2U)
#define CAN_RI0R_IDE_Msk       (0x1UL << CAN_RI0R_IDE_Pos)                      /*!< 0x00000004 */
#define CAN_RI0R_IDE           CAN_RI0R_IDE_Msk                                /*!<Identifier Extension */
#define CAN_RI0R_EXID_Pos      (3U)
#define CAN_RI0R_EXID_Msk      (0x3FFFFUL << CAN_RI0R_EXID_Pos)                 /*!< 0x001FFFF8 */
#define CAN_RI0R_EXID          CAN_RI0R_EXID_Msk                               /*!<Extended Identifier */
#define CAN_RI0R_STID_Pos      (21U)
#define CAN_RI0R_STID_Msk      (0x7FFUL << CAN_RI0R_STID_Pos)                   /*!< 0xFFE00000 */
#define CAN_RI0R_STID          CAN_RI0R_STID_Msk                               /*!<Standard Identifier or Extended Identifier */

/*******************  Bit definition for CAN_RDT0R register  ******************/
#define CAN_RDT0R_DLC_Pos      (0U)
#define CAN_RDT0R_DLC_Msk      (0xFUL << CAN_RDT0R_DLC_Pos)                     /*!< 0x0000000F */
#define CAN_RDT0R_DLC          CAN_RDT0R_DLC_Msk                               /*!<Data Length Code */
#define CAN_RDT0R_FMI_Pos      (8U)
#define CAN_RDT0R_FMI_Msk      (0xFFUL << CAN_RDT0R_FMI_Pos)                    /*!< 0x0000FF00 */
#define CAN_RDT0R_FMI          CAN_RDT0R_FMI_Msk                               /*!<Filter Match Index */
#define CAN_RDT0R_TIME_Pos     (16U)
#define CAN_RDT0R_TIME_Msk     (0xFFFFUL << CAN_RDT0R_TIME_Pos)                 /*!< 0xFFFF0000 */
#define CAN_RDT0R_TIME         CAN_RDT0R_TIME_Msk                              /*!<Message Time Stamp */

/*******************  Bit definition for CAN_RDL0R register  ******************/
#define CAN_RDL0R_DATA0_Pos    (0U)
#define CAN_RDL0R_DATA0_Msk    (0xFFUL << CAN_RDL0R_DATA0_Pos)                  /*!< 0x000000FF */
#define CAN_RDL0R_DATA0        CAN_RDL0R_DATA0_Msk                             /*!<Data byte 0 */
#define CAN_RDL0R_DATA1_Pos    (8U)
#define CAN_RDL0R_DATA1_Msk    (0xFFUL << CAN_RDL0R_DATA1_Pos)                  /*!< 0x0000FF00 */
#define CAN_RDL0R_DATA1        CAN_RDL0R_DATA1_Msk                             /*!<Data byte 1 */
#define CAN_RDL0R_DATA2_Pos    (16U)
#define CAN_RDL0R_DATA2_Msk    (0xFFUL << CAN_RDL0R_DATA2_Pos)                  /*!< 0x00FF0000 */
#define CAN_RDL0R_DATA2        CAN_RDL0R_DATA2_Msk                             /*!<Data byte 2 */
#define CAN_RDL0R_DATA3_Pos    (24U)
#define CAN_RDL0R_DATA3_Msk    (0xFFUL << CAN_RDL0R_DATA3_Pos)                  /*!< 0xFF000000 */
#define CAN_RDL0R_DATA3        CAN_RDL0R_DATA3_Msk                             /*!<Data byte 3 */

/*******************  Bit definition for CAN_RDH0R register  ******************/
#define CAN_RDH0R_DATA4_Pos    (0U)
#define CAN_RDH0R_DATA4_Msk    (0xFFUL << CAN_RDH0R_DATA4_Pos)                  /*!< 0x000000FF */
#define CAN_RDH0R_DATA4        CAN_RDH0R_DATA4_Msk                             /*!<Data byte 4 */
#define CAN_RDH0R_DATA5_Pos    (8U)
#define CAN_RDH0R_DATA5_Msk    (0xFFUL << CAN_RDH0R_DATA5_Pos)                  /*!< 0x0000FF00 */
#define CAN_RDH0R_DATA5        CAN_RDH0R_DATA5_Msk                             /*!<Data byte 5 */
#define CAN_RDH0R_DATA6_Pos    (16U)
#define CAN_RDH0R_DATA6_Msk    (0xFFUL << CAN_RDH0R_DATA6_Pos)                  /*!< 0x00FF0000 */
#define CAN_RDH0R_DATA6        CAN_RDH0R_DATA6_Msk                             /*!<Data byte 6 */
#define CAN_RDH0R_DATA7_Pos    (24U)
#define CAN_RDH0R_DATA7_Msk    (0xFFUL << CAN_RDH0R_DATA7_Pos)                  /*!< 0xFF000000 */
#define CAN_RDH0R_DATA7        CAN_RDH0R_DATA7_Msk                             /*!<Data byte 7 */

/*******************  Bit definition for CAN_RI1R register  *******************/
#define CAN_RI1R_RTR_Pos       (1U)
#define CAN_RI1R_RTR_Msk       (0x1UL << CAN_RI1R_RTR_Pos)                      /*!< 0x00000002 */
#define CAN_RI1R_RTR           CAN_RI1R_RTR_Msk                                /*!<Remote Transmission Request */
#define CAN_RI1R_IDE_Pos       (2U)
#define CAN_RI1R_IDE_Msk       (0x1UL << CAN_RI1R_IDE_Pos)                      /*!< 0x00000004 */
#define CAN_RI1R_IDE           CAN_RI1R_IDE_Msk                                /*!<Identifier Extension */
#define CAN_RI1R_EXID_Pos      (3U)
#define CAN_RI1R_EXID_Msk      (0x3FFFFUL << CAN_RI1R_EXID_Pos)                 /*!< 0x001FFFF8 */
#define CAN_RI1R_EXID          CAN_RI1R_EXID_Msk                               /*!<Extended identifier */
#define CAN_RI1R_STID_Pos      (21U)
#define CAN_RI1R_STID_Msk      (0x7FFUL << CAN_RI1R_STID_Pos)                   /*!< 0xFFE00000 */
#define CAN_RI1R_STID          CAN_RI1R_STID_Msk                               /*!<Standard Identifier or Extended Identifier */

/*******************  Bit definition for CAN_RDT1R register  ******************/
#define CAN_RDT1R_DLC_Pos      (0U)
#define CAN_RDT1R_DLC_Msk      (0xFUL << CAN_RDT1R_DLC_Pos)                     /*!< 0x0000000F */
#define CAN_RDT1R_DLC          CAN_RDT1R_DLC_Msk                               /*!<Data Length Code */
#define CAN_RDT1R_FMI_Pos      (8U)
#define CAN_RDT1R_FMI_Msk      (0xFFUL << CAN_RDT1R_FMI_Pos)                    /*!< 0x0000FF00 */
#define CAN_RDT1R_FMI          CAN_RDT1R_FMI_Msk                               /*!<Filter Match Index */
#define CAN_RDT1R_TIME_Pos     (16U)
#define CAN_RDT1R_TIME_Msk     (0xFFFFUL << CAN_RDT1R_TIME_Pos)                 /*!< 0xFFFF0000 */
#define CAN_RDT1R_TIME         CAN_RDT1R_TIME_Msk                              /*!<Message Time Stamp */

/*******************  Bit definition for CAN_RDL1R register  ******************/
#define CAN_RDL1R_DATA0_Pos    (0U)
#define CAN_RDL1R_DATA0_Msk    (0xFFUL << CAN_RDL1R_DATA0_Pos)                  /*!< 0x000000FF */
#define CAN_RDL1R_DATA0        CAN_RDL1R_DATA0_Msk                             /*!<Data byte 0 */
#define CAN_RDL1R_DATA1_Pos    (8U)
#define CAN_RDL1R_DATA1_Msk    (0xFFUL << CAN_RDL1R_DATA1_Pos)                  /*!< 0x0000FF00 */
#define CAN_RDL1R_DATA1        CAN_RDL1R_DATA1_Msk                             /*!<Data byte 1 */
#define CAN_RDL1R_DATA2_Pos    (16U)
#define CAN_RDL1R_DATA2_Msk    (0xFFUL << CAN_RDL1R_DATA2_Pos)                  /*!< 0x00FF0000 */
#define CAN_RDL1R_DATA2        CAN_RDL1R_DATA2_Msk                             /*!<Data byte 2 */
#define CAN_RDL1R_DATA3_Pos    (24U)
#define CAN_RDL1R_DATA3_Msk    (0xFFUL << CAN_RDL1R_DATA3_Pos)                  /*!< 0xFF000000 */
#define CAN_RDL1R_DATA3        CAN_RDL1R_DATA3_Msk                             /*!<Data byte 3 */

/*******************  Bit definition for CAN_RDH1R register  ******************/
#define CAN_RDH1R_DATA4_Pos    (0U)
#define CAN_RDH1R_DATA4_Msk    (0xFFUL << CAN_RDH1R_DATA4_Pos)                  /*!< 0x000000FF */
#define CAN_RDH1R_DATA4        CAN_RDH1R_DATA4_Msk                             /*!<Data byte 4 */
#define CAN_RDH1R_DATA5_Pos    (8U)
#define CAN_RDH1R_DATA5_Msk    (0xFFUL << CAN_RDH1R_DATA5_Pos)                  /*!< 0x0000FF00 */
#define CAN_RDH1R_DATA5        CAN_RDH1R_DATA5_Msk                             /*!<Data byte 5 */
#define CAN_RDH1R_DATA6_Pos    (16U)
#define CAN_RDH1R_DATA6_Msk    (0xFFUL << CAN_RDH1R_DATA6_Pos)                  /*!< 0x00FF0000 */
#define CAN_RDH1R_DATA6        CAN_RDH1R_DATA6_Msk                             /*!<Data byte 6 */
#define CAN_RDH1R_DATA7_Pos    (24U)
#define CAN_RDH1R_DATA7_Msk    (0xFFUL << CAN_RDH1R_DATA7_Pos)                  /*!< 0xFF000000 */
#define CAN_RDH1R_DATA7        CAN_RDH1R_DATA7_Msk                             /*!<Data byte 7 */

/*!<CAN filter registers */
/*******************  Bit definition for CAN_FMR register  ********************/
#define CAN_FMR_FINIT_Pos      (0U)
#define CAN_FMR_FINIT_Msk      (0x1UL << CAN_FMR_FINIT_Pos)                     /*!< 0x00000001 */
#define CAN_FMR_FINIT          CAN_FMR_FINIT_Msk                               /*!<Filter Init Mode */
#define CAN_FMR_CAN2SB_Pos     (8U)
#define CAN_FMR_CAN2SB_Msk     (0x3FUL << CAN_FMR_CAN2SB_Pos)                   /*!< 0x00003F00 */
#define CAN_FMR_CAN2SB         CAN_FMR_CAN2SB_Msk                              /*!<CAN2 start bank */

/*******************  Bit definition for CAN_FM1R register  *******************/
#define CAN_FM1R_FBM_Pos       (0U)
#define CAN_FM1R_FBM_Msk       (0xFFFFFFFUL << CAN_FM1R_FBM_Pos)                /*!< 0x0FFFFFFF */
#define CAN_FM1R_FBM           CAN_FM1R_FBM_Msk                                /*!<Filter Mode */
#define CAN_FM1R_FBM0_Pos      (0U)
#define CAN_FM1R_FBM0_Msk      (0x1UL << CAN_FM1R_FBM0_Pos)                     /*!< 0x00000001 */
#define CAN_FM1R_FBM0          CAN_FM1R_FBM0_Msk                               /*!<Filter Init Mode bit 0 */
#define CAN_FM1R_FBM1_Pos      (1U)
#define CAN_FM1R_FBM1_Msk      (0x1UL << CAN_FM1R_FBM1_Pos)                     /*!< 0x00000002 */
#define CAN_FM1R_FBM1          CAN_FM1R_FBM1_Msk                               /*!<Filter Init Mode bit 1 */
#define CAN_FM1R_FBM2_Pos      (2U)
#define CAN_FM1R_FBM2_Msk      (0x1UL << CAN_FM1R_FBM2_Pos)                     /*!< 0x00000004 */
#define CAN_FM1R_FBM2          CAN_FM1R_FBM2_Msk                               /*!<Filter Init Mode bit 2 */
#define CAN_FM1R_FBM3_Pos      (3U)
#define CAN_FM1R_FBM3_Msk      (0x1UL << CAN_FM1R_FBM3_Pos)                     /*!< 0x00000008 */
#define CAN_FM1R_FBM3          CAN_FM1R_FBM3_Msk                               /*!<Filter Init Mode bit 3 */
#define CAN_FM1R_FBM4_Pos      (4U)
#define CAN_FM1R_FBM4_Msk      (0x1UL << CAN_FM1R_FBM4_Pos)                     /*!< 0x00000010 */
#define CAN_FM1R_FBM4          CAN_FM1R_FBM4_Msk                               /*!<Filter Init Mode bit 4 */
#define CAN_FM1R_FBM5_Pos      (5U)
#define CAN_FM1R_FBM5_Msk      (0x1UL << CAN_FM1R_FBM5_Pos)                     /*!< 0x00000020 */
#define CAN_FM1R_FBM5          CAN_FM1R_FBM5_Msk                               /*!<Filter Init Mode bit 5 */
#define CAN_FM1R_FBM6_Pos      (6U)
#define CAN_FM1R_FBM6_Msk      (0x1UL << CAN_FM1R_FBM6_Pos)                     /*!< 0x00000040 */
#define CAN_FM1R_FBM6          CAN_FM1R_FBM6_Msk                               /*!<Filter Init Mode bit 6 */
#define CAN_FM1R_FBM7_Pos      (7U)
#define CAN_FM1R_FBM7_Msk      (0x1UL << CAN_FM1R_FBM7_Pos)                     /*!< 0x00000080 */
#define CAN_FM1R_FBM7          CAN_FM1R_FBM7_Msk                               /*!<Filter Init Mode bit 7 */
#define CAN_FM1R_FBM8_Pos      (8U)
#define CAN_FM1R_FBM8_Msk      (0x1UL << CAN_FM1R_FBM8_Pos)                     /*!< 0x00000100 */
#define CAN_FM1R_FBM8          CAN_FM1R_FBM8_Msk                               /*!<Filter Init Mode bit 8 */
#define CAN_FM1R_FBM9_Pos      (9U)
#define CAN_FM1R_FBM9_Msk      (0x1UL << CAN_FM1R_FBM9_Pos)                     /*!< 0x00000200 */
#define CAN_FM1R_FBM9          CAN_FM1R_FBM9_Msk                               /*!<Filter Init Mode bit 9 */
#define CAN_FM1R_FBM10_Pos     (10U)
#define CAN_FM1R_FBM10_Msk     (0x1UL << CAN_FM1R_FBM10_Pos)                    /*!< 0x00000400 */
#define CAN_FM1R_FBM10         CAN_FM1R_FBM10_Msk                              /*!<Filter Init Mode bit 10 */
#define CAN_FM1R_FBM11_Pos     (11U)
#define CAN_FM1R_FBM11_Msk     (0x1UL << CAN_FM1R_FBM11_Pos)                    /*!< 0x00000800 */
#define CAN_FM1R_FBM11         CAN_FM1R_FBM11_Msk                              /*!<Filter Init Mode bit 11 */
#define CAN_FM1R_FBM12_Pos     (12U)
#define CAN_FM1R_FBM12_Msk     (0x1UL << CAN_FM1R_FBM12_Pos)                    /*!< 0x00001000 */
#define CAN_FM1R_FBM12         CAN_FM1R_FBM12_Msk                              /*!<Filter Init Mode bit 12 */
#define CAN_FM1R_FBM13_Pos     (13U)
#define CAN_FM1R_FBM13_Msk     (0x1UL << CAN_FM1R_FBM13_Pos)                    /*!< 0x00002000 */
#define CAN_FM1R_FBM13         CAN_FM1R_FBM13_Msk                              /*!<Filter Init Mode bit 13 */

/*******************  Bit definition for CAN_FS1R register  *******************/
#define CAN_FS1R_FSC_Pos       (0U)
#define CAN_FS1R_FSC_Msk       (0xFFFFFFFUL << CAN_FS1R_FSC_Pos)                /*!< 0x0FFFFFFF */
#define CAN_FS1R_FSC           CAN_FS1R_FSC_Msk                                /*!<Filter Scale Configuration */
#define CAN_FS1R_FSC0_Pos      (0U)
#define CAN_FS1R_FSC0_Msk      (0x1UL << CAN_FS1R_FSC0_Pos)                     /*!< 0x00000001 */
#define CAN_FS1R_FSC0          CAN_FS1R_FSC0_Msk                               /*!<Filter Scale Configuration bit 0 */
#define CAN_FS1R_FSC1_Pos      (1U)
#define CAN_FS1R_FSC1_Msk      (0x1UL << CAN_FS1R_FSC1_Pos)                     /*!< 0x00000002 */
#define CAN_FS1R_FSC1          CAN_FS1R_FSC1_Msk                               /*!<Filter Scale Configuration bit 1 */
#define CAN_FS1R_FSC2_Pos      (2U)
#define CAN_FS1R_FSC2_Msk      (0x1UL << CAN_FS1R_FSC2_Pos)                     /*!< 0x00000004 */
#define CAN_FS1R_FSC2          CAN_FS1R_FSC2_Msk                               /*!<Filter Scale Configuration bit 2 */
#define CAN_FS1R_FSC3_Pos      (3U)
#define CAN_FS1R_FSC3_Msk      (0x1UL << CAN_FS1R_FSC3_Pos)                     /*!< 0x00000008 */
#define CAN_FS1R_FSC3          CAN_FS1R_FSC3_Msk                               /*!<Filter Scale Configuration bit 3 */
#define CAN_FS1R_FSC4_Pos      (4U)
#define CAN_FS1R_FSC4_Msk      (0x1UL << CAN_FS1R_FSC4_Pos)                     /*!< 0x00000010 */
#define CAN_FS1R_FSC4          CAN_FS1R_FSC4_Msk                               /*!<Filter Scale Configuration bit 4 */
#define CAN_FS1R_FSC5_Pos      (5U)
#define CAN_FS1R_FSC5_Msk      (0x1UL << CAN_FS1R_FSC5_Pos)                     /*!< 0x00000020 */
#define CAN_FS1R_FSC5          CAN_FS1R_FSC5_Msk                               /*!<Filter Scale Configuration bit 5 */
#define CAN_FS1R_FSC6_Pos      (6U)
#define CAN_FS1R_FSC6_Msk      (0x1UL << CAN_FS1R_FSC6_Pos)                     /*!< 0x00000040 */
#define CAN_FS1R_FSC6          CAN_FS1R_FSC6_Msk                               /*!<Filter Scale Configuration bit 6 */
#define CAN_FS1R_FSC7_Pos      (7U)
#define CAN_FS1R_FSC7_Msk      (0x1UL << CAN_FS1R_FSC7_Pos)                     /*!< 0x00000080 */
#define CAN_FS1R_FSC7          CAN_FS1R_FSC7_Msk                               /*!<Filter Scale Configuration bit 7 */
#define CAN_FS1R_FSC8_Pos      (8U)
#define CAN_FS1R_FSC8_Msk      (0x1UL << CAN_FS1R_FSC8_Pos)                     /*!< 0x00000100 */
#define CAN_FS1R_FSC8          CAN_FS1R_FSC8_Msk                               /*!<Filter Scale Configuration bit 8 */
#define CAN_FS1R_FSC9_Pos      (9U)
#define CAN_FS1R_FSC9_Msk      (0x1UL << CAN_FS1R_FSC9_Pos)                     /*!< 0x00000200 */
#define CAN_FS1R_FSC9          CAN_FS1R_FSC9_Msk                               /*!<Filter Scale Configuration bit 9 */
#define CAN_FS1R_FSC10_Pos     (10U)
#define CAN_FS1R_FSC10_Msk     (0x1UL << CAN_FS1R_FSC10_Pos)                    /*!< 0x00000400 */
#define CAN_FS1R_FSC10         CAN_FS1R_FSC10_Msk                              /*!<Filter Scale Configuration bit 10 */
#define CAN_FS1R_FSC11_Pos     (11U)
#define CAN_FS1R_FSC11_Msk     (0x1UL << CAN_FS1R_FSC11_Pos)                    /*!< 0x00000800 */
#define CAN_FS1R_FSC11         CAN_FS1R_FSC11_Msk                              /*!<Filter Scale Configuration bit 11 */
#define CAN_FS1R_FSC12_Pos     (12U)
#define CAN_FS1R_FSC12_Msk     (0x1UL << CAN_FS1R_FSC12_Pos)                    /*!< 0x00001000 */
#define CAN_FS1R_FSC12         CAN_FS1R_FSC12_Msk                              /*!<Filter Scale Configuration bit 12 */
#define CAN_FS1R_FSC13_Pos     (13U)
#define CAN_FS1R_FSC13_Msk     (0x1UL << CAN_FS1R_FSC13_Pos)                    /*!< 0x00002000 */
#define CAN_FS1R_FSC13         CAN_FS1R_FSC13_Msk                              /*!<Filter Scale Configuration bit 13 */

/******************  Bit definition for CAN_FFA1R register  *******************/
#define CAN_FFA1R_FFA_Pos      (0U)
#define CAN_FFA1R_FFA_Msk      (0xFFFFFFFUL << CAN_FFA1R_FFA_Pos)               /*!< 0x0FFFFFFF */
#define CAN_FFA1R_FFA          CAN_FFA1R_FFA_Msk                               /*!<Filter FIFO Assignment */
#define CAN_FFA1R_FFA0_Pos     (0U)
#define CAN_FFA1R_FFA0_Msk     (0x1UL << CAN_FFA1R_FFA0_Pos)                    /*!< 0x00000001 */
#define CAN_FFA1R_FFA0         CAN_FFA1R_FFA0_Msk                              /*!<Filter FIFO Assignment bit 0 */
#define CAN_FFA1R_FFA1_Pos     (1U)
#define CAN_FFA1R_FFA1_Msk     (0x1UL << CAN_FFA1R_FFA1_Pos)                    /*!< 0x00000002 */
#define CAN_FFA1R_FFA1         CAN_FFA1R_FFA1_Msk                              /*!<Filter FIFO Assignment bit 1 */
#define CAN_FFA1R_FFA2_Pos     (2U)
#define CAN_FFA1R_FFA2_Msk     (0x1UL << CAN_FFA1R_FFA2_Pos)                    /*!< 0x00000004 */
#define CAN_FFA1R_FFA2         CAN_FFA1R_FFA2_Msk                              /*!<Filter FIFO Assignment bit 2 */
#define CAN_FFA1R_FFA3_Pos     (3U)
#define CAN_FFA1R_FFA3_Msk     (0x1UL << CAN_FFA1R_FFA3_Pos)                    /*!< 0x00000008 */
#define CAN_FFA1R_FFA3         CAN_FFA1R_FFA3_Msk                              /*!<Filter FIFO Assignment bit 3 */
#define CAN_FFA1R_FFA4_Pos     (4U)
#define CAN_FFA1R_FFA4_Msk     (0x1UL << CAN_FFA1R_FFA4_Pos)                    /*!< 0x00000010 */
#define CAN_FFA1R_FFA4         CAN_FFA1R_FFA4_Msk                              /*!<Filter FIFO Assignment bit 4 */
#define CAN_FFA1R_FFA5_Pos     (5U)
#define CAN_FFA1R_FFA5_Msk     (0x1UL << CAN_FFA1R_FFA5_Pos)                    /*!< 0x00000020 */
#define CAN_FFA1R_FFA5         CAN_FFA1R_FFA5_Msk                              /*!<Filter FIFO Assignment bit 5 */
#define CAN_FFA1R_FFA6_Pos     (6U)
#define CAN_FFA1R_FFA6_Msk     (0x1UL << CAN_FFA1R_FFA6_Pos)                    /*!< 0x00000040 */
#define CAN_FFA1R_FFA6         CAN_FFA1R_FFA6_Msk                              /*!<Filter FIFO Assignment bit 6 */
#define CAN_FFA1R_FFA7_Pos     (7U)
#define CAN_FFA1R_FFA7_Msk     (0x1UL << CAN_FFA1R_FFA7_Pos)                    /*!< 0x00000080 */
#define CAN_FFA1R_FFA7         CAN_FFA1R_FFA7_Msk                              /*!<Filter FIFO Assignment bit 7 */
#define CAN_FFA1R_FFA8_Pos     (8U)
#define CAN_FFA1R_FFA8_Msk     (0x1UL << CAN_FFA1R_FFA8_Pos)                    /*!< 0x00000100 */
#define CAN_FFA1R_FFA8         CAN_FFA1R_FFA8_Msk                              /*!<Filter FIFO Assignment bit 8 */
#define CAN_FFA1R_FFA9_Pos     (9U)
#define CAN_FFA1R_FFA9_Msk     (0x1UL << CAN_FFA1R_FFA9_Pos)                    /*!< 0x00000200 */
#define CAN_FFA1R_FFA9         CAN_FFA1R_FFA9_Msk                              /*!<Filter FIFO Assignment bit 9 */
#define CAN_FFA1R_FFA10_Pos    (10U)
#define CAN_FFA1R_FFA10_Msk    (0x1UL << CAN_FFA1R_FFA10_Pos)                   /*!< 0x00000400 */
#define CAN_FFA1R_FFA10        CAN_FFA1R_FFA10_Msk                             /*!<Filter FIFO Assignment bit 10 */
#define CAN_FFA1R_FFA11_Pos    (11U)
#define CAN_FFA1R_FFA11_Msk    (0x1UL << CAN_FFA1R_FFA11_Pos)                   /*!< 0x00000800 */
#define CAN_FFA1R_FFA11        CAN_FFA1R_FFA11_Msk                             /*!<Filter FIFO Assignment bit 11 */
#define CAN_FFA1R_FFA12_Pos    (12U)
#define CAN_FFA1R_FFA12_Msk    (0x1UL << CAN_FFA1R_FFA12_Pos)                   /*!< 0x00001000 */
#define CAN_FFA1R_FFA12        CAN_FFA1R_FFA12_Msk                             /*!<Filter FIFO Assignment bit 12 */
#define CAN_FFA1R_FFA13_Pos    (13U)
#define CAN_FFA1R_FFA13_Msk    (0x1UL << CAN_FFA1R_FFA13_Pos)                   /*!< 0x00002000 */
#define CAN_FFA1R_FFA13        CAN_FFA1R_FFA13_Msk                             /*!<Filter FIFO Assignment bit 13 */

/*******************  Bit definition for CAN_FA1R register  *******************/
#define CAN_FA1R_FACT_Pos      (0U)
#define CAN_FA1R_FACT_Msk      (0xFFFFFFFUL << CAN_FA1R_FACT_Pos)               /*!< 0x0FFFFFFF */
#define CAN_FA1R_FACT          CAN_FA1R_FACT_Msk                               /*!<Filter Active */
#define CAN_FA1R_FACT0_Pos     (0U)
#define CAN_FA1R_FACT0_Msk     (0x1UL << CAN_FA1R_FACT0_Pos)                    /*!< 0x00000001 */
#define CAN_FA1R_FACT0         CAN_FA1R_FACT0_Msk                              /*!<Filter Active bit 0 */
#define CAN_FA1R_FACT1_Pos     (1U)
#define CAN_FA1R_FACT1_Msk     (0x1UL << CAN_FA1R_FACT1_Pos)                    /*!< 0x00000002 */
#define CAN_FA1R_FACT1         CAN_FA1R_FACT1_Msk                              /*!<Filter Active bit 1 */
#define CAN_FA1R_FACT2_Pos     (2U)
#define CAN_FA1R_FACT2_Msk     (0x1UL << CAN_FA1R_FACT2_Pos)                    /*!< 0x00000004 */
#define CAN_FA1R_FACT2         CAN_FA1R_FACT2_Msk                              /*!<Filter Active bit 2 */
#define CAN_FA1R_FACT3_Pos     (3U)
#define CAN_FA1R_FACT3_Msk     (0x1UL << CAN_FA1R_FACT3_Pos)                    /*!< 0x00000008 */
#define CAN_FA1R_FACT3         CAN_FA1R_FACT3_Msk                              /*!<Filter Active bit 3 */
#define CAN_FA1R_FACT4_Pos     (4U)
#define CAN_FA1R_FACT4_Msk     (0x1UL << CAN_FA1R_FACT4_Pos)                    /*!< 0x00000010 */
#define CAN_FA1R_FACT4         CAN_FA1R_FACT4_Msk                              /*!<Filter Active bit 4 */
#define CAN_FA1R_FACT5_Pos     (5U)
#define CAN_FA1R_FACT5_Msk     (0x1UL << CAN_FA1R_FACT5_Pos)                    /*!< 0x00000020 */
#define CAN_FA1R_FACT5         CAN_FA1R_FACT5_Msk                              /*!<Filter Active bit 5 */
#define CAN_FA1R_FACT6_Pos     (6U)
#define CAN_FA1R_FACT6_Msk     (0x1UL << CAN_FA1R_FACT6_Pos)                    /*!< 0x00000040 */
#define CAN_FA1R_FACT6         CAN_FA1R_FACT6_Msk                              /*!<Filter Active bit 6 */
#define CAN_FA1R_FACT7_Pos     (7U)
#define CAN_FA1R_FACT7_Msk     (0x1UL << CAN_FA1R_FACT7_Pos)                    /*!< 0x00000080 */
#define CAN_FA1R_FACT7         CAN_FA1R_FACT7_Msk                              /*!<Filter Active bit 7 */
#define CAN_FA1R_FACT8_Pos     (8U)
#define CAN_FA1R_FACT8_Msk     (0x1UL << CAN_FA1R_FACT8_Pos)                    /*!< 0x00000100 */
#define CAN_FA1R_FACT8         CAN_FA1R_FACT8_Msk                              /*!<Filter Active bit 8 */
#define CAN_FA1R_FACT9_Pos     (9U)
#define CAN_FA1R_FACT9_Msk     (0x1UL << CAN_FA1R_FACT9_Pos)                    /*!< 0x00000200 */
#define CAN_FA1R_FACT9         CAN_FA1R_FACT9_Msk                              /*!<Filter Active bit 9 */
#define CAN_FA1R_FACT10_Pos    (10U)
#define CAN_FA1R_FACT10_Msk    (0x1UL << CAN_FA1R_FACT10_Pos)                   /*!< 0x00000400 */
#define CAN_FA1R_FACT10        CAN_FA1R_FACT10_Msk                             /*!<Filter Active bit 10 */
#define CAN_FA1R_FACT11_Pos    (11U)
#define CAN_FA1R_FACT11_Msk    (0x1UL << CAN_FA1R_FACT11_Pos)                   /*!< 0x00000800 */
#define CAN_FA1R_FACT11        CAN_FA1R_FACT11_Msk                             /*!<Filter Active bit 11 */
#define CAN_FA1R_FACT12_Pos    (12U)
#define CAN_FA1R_FACT12_Msk    (0x1UL << CAN_FA1R_FACT12_Pos)                   /*!< 0x00001000 */
#define CAN_FA1R_FACT12        CAN_FA1R_FACT12_Msk                             /*!<Filter Active bit 12 */
#define CAN_FA1R_FACT13_Pos    (13U)
#define CAN_FA1R_FACT13_Msk    (0x1UL << CAN_FA1R_FACT13_Pos)                   /*!< 0x00002000 */
#define CAN_FA1R_FACT13        CAN_FA1R_FACT13_Msk                             /*!<Filter Active bit 13 */

/*******************  Bit definition for CAN_F0R1 register  *******************/
#define CAN_F0R1_FB0_Pos       (0U)
#define CAN_F0R1_FB0_Msk       (0x1UL << CAN_F0R1_FB0_Pos)                      /*!< 0x00000001 */
#define CAN_F0R1_FB0           CAN_F0R1_FB0_Msk                                /*!<Filter bit 0 */
#define CAN_F0R1_FB1_Pos       (1U)
#define CAN_F0R1_FB1_Msk       (0x1UL << CAN_F0R1_FB1_Pos)                      /*!< 0x00000002 */
#define CAN_F0R1_FB1           CAN_F0R1_FB1_Msk                                /*!<Filter bit 1 */
#define CAN_F0R1_FB2_Pos       (2U)
#define CAN_F0R1_FB2_Msk       (0x1UL << CAN_F0R1_FB2_Pos)                      /*!< 0x00000004 */
#define CAN_F0R1_FB2           CAN_F0R1_FB2_Msk                                /*!<Filter bit 2 */
#define CAN_F0R1_FB3_Pos       (3U)
#define CAN_F0R1_FB3_Msk       (0x1UL << CAN_F0R1_FB3_Pos)                      /*!< 0x00000008 */
#define CAN_F0R1_FB3           CAN_F0R1_FB3_Msk                                /*!<Filter bit 3 */
#define CAN_F0R1_FB4_Pos       (4U)
#define CAN_F0R1_FB4_Msk       (0x1UL << CAN_F0R1_FB4_Pos)                      /*!< 0x00000010 */
#define CAN_F0R1_FB4           CAN_F0R1_FB4_Msk                                /*!<Filter bit 4 */
#define CAN_F0R1_FB5_Pos       (5U)
#define CAN_F0R1_FB5_Msk       (0x1UL << CAN_F0R1_FB5_Pos)                      /*!< 0x00000020 */
#define CAN_F0R1_FB5           CAN_F0R1_FB5_Msk                                /*!<Filter bit 5 */
#define CAN_F0R1_FB6_Pos       (6U)
#define CAN_F0R1_FB6_Msk       (0x1UL << CAN_F0R1_FB6_Pos)                      /*!< 0x00000040 */
#define CAN_F0R1_FB6           CAN_F0R1_FB6_Msk                                /*!<Filter bit 6 */
#define CAN_F0R1_FB7_Pos       (7U)
#define CAN_F0R1_FB7_Msk       (0x1UL << CAN_F0R1_FB7_Pos)                      /*!< 0x00000080 */
#define CAN_F0R1_FB7           CAN_F0R1_FB7_Msk                                /*!<Filter bit 7 */
#define CAN_F0R1_FB8_Pos       (8U)
#define CAN_F0R1_FB8_Msk       (0x1UL << CAN_F0R1_FB8_Pos)                      /*!< 0x00000100 */
#define CAN_F0R1_FB8           CAN_F0R1_FB8_Msk                                /*!<Filter bit 8 */
#define CAN_F0R1_FB9_Pos       (9U)
#define CAN_F0R1_FB9_Msk       (0x1UL << CAN_F0R1_FB9_Pos)                      /*!< 0x00000200 */
#define CAN_F0R1_FB9           CAN_F0R1_FB9_Msk                                /*!<Filter bit 9 */
#define CAN_F0R1_FB10_Pos      (10U)
#define CAN_F0R1_FB10_Msk      (0x1UL << CAN_F0R1_FB10_Pos)                     /*!< 0x00000400 */
#define CAN_F0R1_FB10          CAN_F0R1_FB10_Msk                               /*!<Filter bit 10 */
#define CAN_F0R1_FB11_Pos      (11U)
#define CAN_F0R1_FB11_Msk      (0x1UL << CAN_F0R1_FB11_Pos)                     /*!< 0x00000800 */
#define CAN_F0R1_FB11          CAN_F0R1_FB11_Msk                               /*!<Filter bit 11 */
#define CAN_F0R1_FB12_Pos      (12U)
#define CAN_F0R1_FB12_Msk      (0x1UL << CAN_F0R1_FB12_Pos)                     /*!< 0x00001000 */
#define CAN_F0R1_FB12          CAN_F0R1_FB12_Msk                               /*!<Filter bit 12 */
#define CAN_F0R1_FB13_Pos      (13U)
#define CAN_F0R1_FB13_Msk      (0x1UL << CAN_F0R1_FB13_Pos)                     /*!< 0x00002000 */
#define CAN_F0R1_FB13          CAN_F0R1_FB13_Msk                               /*!<Filter bit 13 */
#define CAN_F0R1_FB14_Pos      (14U)
#define CAN_F0R1_FB14_Msk      (0x1UL << CAN_F0R1_FB14_Pos)                     /*!< 0x00004000 */
#define CAN_F0R1_FB14          CAN_F0R1_FB14_Msk                               /*!<Filter bit 14 */
#define CAN_F0R1_FB15_Pos      (15U)
#define CAN_F0R1_FB15_Msk      (0x1UL << CAN_F0R1_FB15_Pos)                     /*!< 0x00008000 */
#define CAN_F0R1_FB15          CAN_F0R1_FB15_Msk                               /*!<Filter bit 15 */
#define CAN_F0R1_FB16_Pos      (16U)
#define CAN_F0R1_FB16_Msk      (0x1UL << CAN_F0R1_FB16_Pos)                     /*!< 0x00010000 */
#define CAN_F0R1_FB16          CAN_F0R1_FB16_Msk                               /*!<Filter bit 16 */
#define CAN_F0R1_FB17_Pos      (17U)
#define CAN_F0R1_FB17_Msk      (0x1UL << CAN_F0R1_FB17_Pos)                     /*!< 0x00020000 */
#define CAN_F0R1_FB17          CAN_F0R1_FB17_Msk                               /*!<Filter bit 17 */
#define CAN_F0R1_FB18_Pos      (18U)
#define CAN_F0R1_FB18_Msk      (0x1UL << CAN_F0R1_FB18_Pos)                     /*!< 0x00040000 */
#define CAN_F0R1_FB18          CAN_F0R1_FB18_Msk                               /*!<Filter bit 18 */
#define CAN_F0R1_FB19_Pos      (19U)
#define CAN_F0R1_FB19_Msk      (0x1UL << CAN_F0R1_FB19_Pos)                     /*!< 0x00080000 */
#define CAN_F0R1_FB19          CAN_F0R1_FB19_Msk                               /*!<Filter bit 19 */
#define CAN_F0R1_FB20_Pos      (20U)
#define CAN_F0R1_FB20_Msk      (0x1UL << CAN_F0R1_FB20_Pos)                     /*!< 0x00100000 */
#define CAN_F0R1_FB20          CAN_F0R1_FB20_Msk                               /*!<Filter bit 20 */
#define CAN_F0R1_FB21_Pos      (21U)
#define CAN_F0R1_FB21_Msk      (0x1UL << CAN_F0R1_FB21_Pos)                     /*!< 0x00200000 */
#define CAN_F0R1_FB21          CAN_F0R1_FB21_Msk                               /*!<Filter bit 21 */
#define CAN_F0R1_FB22_Pos      (22U)
#define CAN_F0R1_FB22_Msk      (0x1UL << CAN_F0R1_FB22_Pos)                     /*!< 0x00400000 */
#define CAN_F0R1_FB22          CAN_F0R1_FB22_Msk                               /*!<Filter bit 22 */
#define CAN_F0R1_FB23_Pos      (23U)
#define CAN_F0R1_FB23_Msk      (0x1UL << CAN_F0R1_FB23_Pos)                     /*!< 0x00800000 */
#define CAN_F0R1_FB23          CAN_F0R1_FB23_Msk                               /*!<Filter bit 23 */
#define CAN_F0R1_FB24_Pos      (24U)
#define CAN_F0R1_FB24_Msk      (0x1UL << CAN_F0R1_FB24_Pos)                     /*!< 0x01000000 */
#define CAN_F0R1_FB24          CAN_F0R1_FB24_Msk                               /*!<Filter bit 24 */
#define CAN_F0R1_FB25_Pos      (25U)
#define CAN_F0R1_FB25_Msk      (0x1UL << CAN_F0R1_FB25_Pos)                     /*!< 0x02000000 */
#define CAN_F0R1_FB25          CAN_F0R1_FB25_Msk                               /*!<Filter bit 25 */
#define CAN_F0R1_FB26_Pos      (26U)
#define CAN_F0R1_FB26_Msk      (0x1UL << CAN_F0R1_FB26_Pos)                     /*!< 0x04000000 */
#define CAN_F0R1_FB26          CAN_F0R1_FB26_Msk                               /*!<Filter bit 26 */
#define CAN_F0R1_FB27_Pos      (27U)
#define CAN_F0R1_FB27_Msk      (0x1UL << CAN_F0R1_FB27_Pos)                     /*!< 0x08000000 */
#define CAN_F0R1_FB27          CAN_F0R1_FB27_Msk                               /*!<Filter bit 27 */
#define CAN_F0R1_FB28_Pos      (28U)
#define CAN_F0R1_FB28_Msk      (0x1UL << CAN_F0R1_FB28_Pos)                     /*!< 0x10000000 */
#define CAN_F0R1_FB28          CAN_F0R1_FB28_Msk                               /*!<Filter bit 28 */
#define CAN_F0R1_FB29_Pos      (29U)
#define CAN_F0R1_FB29_Msk      (0x1UL << CAN_F0R1_FB29_Pos)                     /*!< 0x20000000 */
#define CAN_F0R1_FB29          CAN_F0R1_FB29_Msk                               /*!<Filter bit 29 */
#define CAN_F0R1_FB30_Pos      (30U)
#define CAN_F0R1_FB30_Msk      (0x1UL << CAN_F0R1_FB30_Pos)                     /*!< 0x40000000 */
#define CAN_F0R1_FB30          CAN_F0R1_FB30_Msk                               /*!<Filter bit 30 */
#define CAN_F0R1_FB31_Pos      (31U)
#define CAN_F0R1_FB31_Msk      (0x1UL << CAN_F0R1_FB31_Pos)                     /*!< 0x80000000 */
#define CAN_F0R1_FB31          CAN_F0R1_FB31_Msk                               /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F1R1 register  *******************/
#define CAN_F1R1_FB0_Pos       (0U)
#define CAN_F1R1_FB0_Msk       (0x1UL << CAN_F1R1_FB0_Pos)                      /*!< 0x00000001 */
#define CAN_F1R1_FB0           CAN_F1R1_FB0_Msk                                /*!<Filter bit 0 */
#define CAN_F1R1_FB1_Pos       (1U)
#define CAN_F1R1_FB1_Msk       (0x1UL << CAN_F1R1_FB1_Pos)                      /*!< 0x00000002 */
#define CAN_F1R1_FB1           CAN_F1R1_FB1_Msk                                /*!<Filter bit 1 */
#define CAN_F1R1_FB2_Pos       (2U)
#define CAN_F1R1_FB2_Msk       (0x1UL << CAN_F1R1_FB2_Pos)                      /*!< 0x00000004 */
#define CAN_F1R1_FB2           CAN_F1R1_FB2_Msk                                /*!<Filter bit 2 */
#define CAN_F1R1_FB3_Pos       (3U)
#define CAN_F1R1_FB3_Msk       (0x1UL << CAN_F1R1_FB3_Pos)                      /*!< 0x00000008 */
#define CAN_F1R1_FB3           CAN_F1R1_FB3_Msk                                /*!<Filter bit 3 */
#define CAN_F1R1_FB4_Pos       (4U)
#define CAN_F1R1_FB4_Msk       (0x1UL << CAN_F1R1_FB4_Pos)                      /*!< 0x00000010 */
#define CAN_F1R1_FB4           CAN_F1R1_FB4_Msk                                /*!<Filter bit 4 */
#define CAN_F1R1_FB5_Pos       (5U)
#define CAN_F1R1_FB5_Msk       (0x1UL << CAN_F1R1_FB5_Pos)                      /*!< 0x00000020 */
#define CAN_F1R1_FB5           CAN_F1R1_FB5_Msk                                /*!<Filter bit 5 */
#define CAN_F1R1_FB6_Pos       (6U)
#define CAN_F1R1_FB6_Msk       (0x1UL << CAN_F1R1_FB6_Pos)                      /*!< 0x00000040 */
#define CAN_F1R1_FB6           CAN_F1R1_FB6_Msk                                /*!<Filter bit 6 */
#define CAN_F1R1_FB7_Pos       (7U)
#define CAN_F1R1_FB7_Msk       (0x1UL << CAN_F1R1_FB7_Pos)                      /*!< 0x00000080 */
#define CAN_F1R1_FB7           CAN_F1R1_FB7_Msk                                /*!<Filter bit 7 */
#define CAN_F1R1_FB8_Pos       (8U)
#define CAN_F1R1_FB8_Msk       (0x1UL << CAN_F1R1_FB8_Pos)                      /*!< 0x00000100 */
#define CAN_F1R1_FB8           CAN_F1R1_FB8_Msk                                /*!<Filter bit 8 */
#define CAN_F1R1_FB9_Pos       (9U)
#define CAN_F1R1_FB9_Msk       (0x1UL << CAN_F1R1_FB9_Pos)                      /*!< 0x00000200 */
#define CAN_F1R1_FB9           CAN_F1R1_FB9_Msk                                /*!<Filter bit 9 */
#define CAN_F1R1_FB10_Pos      (10U)
#define CAN_F1R1_FB10_Msk      (0x1UL << CAN_F1R1_FB10_Pos)                     /*!< 0x00000400 */
#define CAN_F1R1_FB10          CAN_F1R1_FB10_Msk                               /*!<Filter bit 10 */
#define CAN_F1R1_FB11_Pos      (11U)
#define CAN_F1R1_FB11_Msk      (0x1UL << CAN_F1R1_FB11_Pos)                     /*!< 0x00000800 */
#define CAN_F1R1_FB11          CAN_F1R1_FB11_Msk                               /*!<Filter bit 11 */
#define CAN_F1R1_FB12_Pos      (12U)
#define CAN_F1R1_FB12_Msk      (0x1UL << CAN_F1R1_FB12_Pos)                     /*!< 0x00001000 */
#define CAN_F1R1_FB12          CAN_F1R1_FB12_Msk                               /*!<Filter bit 12 */
#define CAN_F1R1_FB13_Pos      (13U)
#define CAN_F1R1_FB13_Msk      (0x1UL << CAN_F1R1_FB13_Pos)                     /*!< 0x00002000 */
#define CAN_F1R1_FB13          CAN_F1R1_FB13_Msk                               /*!<Filter bit 13 */
#define CAN_F1R1_FB14_Pos      (14U)
#define CAN_F1R1_FB14_Msk      (0x1UL << CAN_F1R1_FB14_Pos)                     /*!< 0x00004000 */
#define CAN_F1R1_FB14          CAN_F1R1_FB14_Msk                               /*!<Filter bit 14 */
#define CAN_F1R1_FB15_Pos      (15U)
#define CAN_F1R1_FB15_Msk      (0x1UL << CAN_F1R1_FB15_Pos)                     /*!< 0x00008000 */
#define CAN_F1R1_FB15          CAN_F1R1_FB15_Msk                               /*!<Filter bit 15 */
#define CAN_F1R1_FB16_Pos      (16U)
#define CAN_F1R1_FB16_Msk      (0x1UL << CAN_F1R1_FB16_Pos)                     /*!< 0x00010000 */
#define CAN_F1R1_FB16          CAN_F1R1_FB16_Msk                               /*!<Filter bit 16 */
#define CAN_F1R1_FB17_Pos      (17U)
#define CAN_F1R1_FB17_Msk      (0x1UL << CAN_F1R1_FB17_Pos)                     /*!< 0x00020000 */
#define CAN_F1R1_FB17          CAN_F1R1_FB17_Msk                               /*!<Filter bit 17 */
#define CAN_F1R1_FB18_Pos      (18U)
#define CAN_F1R1_FB18_Msk      (0x1UL << CAN_F1R1_FB18_Pos)                     /*!< 0x00040000 */
#define CAN_F1R1_FB18          CAN_F1R1_FB18_Msk                               /*!<Filter bit 18 */
#define CAN_F1R1_FB19_Pos      (19U)
#define CAN_F1R1_FB19_Msk      (0x1UL << CAN_F1R1_FB19_Pos)                     /*!< 0x00080000 */
#define CAN_F1R1_FB19          CAN_F1R1_FB19_Msk                               /*!<Filter bit 19 */
#define CAN_F1R1_FB20_Pos      (20U)
#define CAN_F1R1_FB20_Msk      (0x1UL << CAN_F1R1_FB20_Pos)                     /*!< 0x00100000 */
#define CAN_F1R1_FB20          CAN_F1R1_FB20_Msk                               /*!<Filter bit 20 */
#define CAN_F1R1_FB21_Pos      (21U)
#define CAN_F1R1_FB21_Msk      (0x1UL << CAN_F1R1_FB21_Pos)                     /*!< 0x00200000 */
#define CAN_F1R1_FB21          CAN_F1R1_FB21_Msk                               /*!<Filter bit 21 */
#define CAN_F1R1_FB22_Pos      (22U)
#define CAN_F1R1_FB22_Msk      (0x1UL << CAN_F1R1_FB22_Pos)                     /*!< 0x00400000 */
#define CAN_F1R1_FB22          CAN_F1R1_FB22_Msk                               /*!<Filter bit 22 */
#define CAN_F1R1_FB23_Pos      (23U)
#define CAN_F1R1_FB23_Msk      (0x1UL << CAN_F1R1_FB23_Pos)                     /*!< 0x00800000 */
#define CAN_F1R1_FB23          CAN_F1R1_FB23_Msk                               /*!<Filter bit 23 */
#define CAN_F1R1_FB24_Pos      (24U)
#define CAN_F1R1_FB24_Msk      (0x1UL << CAN_F1R1_FB24_Pos)                     /*!< 0x01000000 */
#define CAN_F1R1_FB24          CAN_F1R1_FB24_Msk                               /*!<Filter bit 24 */
#define CAN_F1R1_FB25_Pos      (25U)
#define CAN_F1R1_FB25_Msk      (0x1UL << CAN_F1R1_FB25_Pos)                     /*!< 0x02000000 */
#define CAN_F1R1_FB25          CAN_F1R1_FB25_Msk                               /*!<Filter bit 25 */
#define CAN_F1R1_FB26_Pos      (26U)
#define CAN_F1R1_FB26_Msk      (0x1UL << CAN_F1R1_FB26_Pos)                     /*!< 0x04000000 */
#define CAN_F1R1_FB26          CAN_F1R1_FB26_Msk                               /*!<Filter bit 26 */
#define CAN_F1R1_FB27_Pos      (27U)
#define CAN_F1R1_FB27_Msk      (0x1UL << CAN_F1R1_FB27_Pos)                     /*!< 0x08000000 */
#define CAN_F1R1_FB27          CAN_F1R1_FB27_Msk                               /*!<Filter bit 27 */
#define CAN_F1R1_FB28_Pos      (28U)
#define CAN_F1R1_FB28_Msk      (0x1UL << CAN_F1R1_FB28_Pos)                     /*!< 0x10000000 */
#define CAN_F1R1_FB28          CAN_F1R1_FB28_Msk                               /*!<Filter bit 28 */
#define CAN_F1R1_FB29_Pos      (29U)
#define CAN_F1R1_FB29_Msk      (0x1UL << CAN_F1R1_FB29_Pos)                     /*!< 0x20000000 */
#define CAN_F1R1_FB29          CAN_F1R1_FB29_Msk                               /*!<Filter bit 29 */
#define CAN_F1R1_FB30_Pos      (30U)
#define CAN_F1R1_FB30_Msk      (0x1UL << CAN_F1R1_FB30_Pos)                     /*!< 0x40000000 */
#define CAN_F1R1_FB30          CAN_F1R1_FB30_Msk                               /*!<Filter bit 30 */
#define CAN_F1R1_FB31_Pos      (31U)
#define CAN_F1R1_FB31_Msk      (0x1UL << CAN_F1R1_FB31_Pos)                     /*!< 0x80000000 */
#define CAN_F1R1_FB31          CAN_F1R1_FB31_Msk                               /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F2R1 register  *******************/
#define CAN_F2R1_FB0_Pos       (0U)
#define CAN_F2R1_FB0_Msk       (0x1UL << CAN_F2R1_FB0_Pos)                      /*!< 0x00000001 */
#define CAN_F2R1_FB0           CAN_F2R1_FB0_Msk                                /*!<Filter bit 0 */
#define CAN_F2R1_FB1_Pos       (1U)
#define CAN_F2R1_FB1_Msk       (0x1UL << CAN_F2R1_FB1_Pos)                      /*!< 0x00000002 */
#define CAN_F2R1_FB1           CAN_F2R1_FB1_Msk                                /*!<Filter bit 1 */
#define CAN_F2R1_FB2_Pos       (2U)
#define CAN_F2R1_FB2_Msk       (0x1UL << CAN_F2R1_FB2_Pos)                      /*!< 0x00000004 */
#define CAN_F2R1_FB2           CAN_F2R1_FB2_Msk                                /*!<Filter bit 2 */
#define CAN_F2R1_FB3_Pos       (3U)
#define CAN_F2R1_FB3_Msk       (0x1UL << CAN_F2R1_FB3_Pos)                      /*!< 0x00000008 */
#define CAN_F2R1_FB3           CAN_F2R1_FB3_Msk                                /*!<Filter bit 3 */
#define CAN_F2R1_FB4_Pos       (4U)
#define CAN_F2R1_FB4_Msk       (0x1UL << CAN_F2R1_FB4_Pos)                      /*!< 0x00000010 */
#define CAN_F2R1_FB4           CAN_F2R1_FB4_Msk                                /*!<Filter bit 4 */
#define CAN_F2R1_FB5_Pos       (5U)
#define CAN_F2R1_FB5_Msk       (0x1UL << CAN_F2R1_FB5_Pos)                      /*!< 0x00000020 */
#define CAN_F2R1_FB5           CAN_F2R1_FB5_Msk                                /*!<Filter bit 5 */
#define CAN_F2R1_FB6_Pos       (6U)
#define CAN_F2R1_FB6_Msk       (0x1UL << CAN_F2R1_FB6_Pos)                      /*!< 0x00000040 */
#define CAN_F2R1_FB6           CAN_F2R1_FB6_Msk                                /*!<Filter bit 6 */
#define CAN_F2R1_FB7_Pos       (7U)
#define CAN_F2R1_FB7_Msk       (0x1UL << CAN_F2R1_FB7_Pos)                      /*!< 0x00000080 */
#define CAN_F2R1_FB7           CAN_F2R1_FB7_Msk                                /*!<Filter bit 7 */
#define CAN_F2R1_FB8_Pos       (8U)
#define CAN_F2R1_FB8_Msk       (0x1UL << CAN_F2R1_FB8_Pos)                      /*!< 0x00000100 */
#define CAN_F2R1_FB8           CAN_F2R1_FB8_Msk                                /*!<Filter bit 8 */
#define CAN_F2R1_FB9_Pos       (9U)
#define CAN_F2R1_FB9_Msk       (0x1UL << CAN_F2R1_FB9_Pos)                      /*!< 0x00000200 */
#define CAN_F2R1_FB9           CAN_F2R1_FB9_Msk                                /*!<Filter bit 9 */
#define CAN_F2R1_FB10_Pos      (10U)
#define CAN_F2R1_FB10_Msk      (0x1UL << CAN_F2R1_FB10_Pos)                     /*!< 0x00000400 */
#define CAN_F2R1_FB10          CAN_F2R1_FB10_Msk                               /*!<Filter bit 10 */
#define CAN_F2R1_FB11_Pos      (11U)
#define CAN_F2R1_FB11_Msk      (0x1UL << CAN_F2R1_FB11_Pos)                     /*!< 0x00000800 */
#define CAN_F2R1_FB11          CAN_F2R1_FB11_Msk                               /*!<Filter bit 11 */
#define CAN_F2R1_FB12_Pos      (12U)
#define CAN_F2R1_FB12_Msk      (0x1UL << CAN_F2R1_FB12_Pos)                     /*!< 0x00001000 */
#define CAN_F2R1_FB12          CAN_F2R1_FB12_Msk                               /*!<Filter bit 12 */
#define CAN_F2R1_FB13_Pos      (13U)
#define CAN_F2R1_FB13_Msk      (0x1UL << CAN_F2R1_FB13_Pos)                     /*!< 0x00002000 */
#define CAN_F2R1_FB13          CAN_F2R1_FB13_Msk                               /*!<Filter bit 13 */
#define CAN_F2R1_FB14_Pos      (14U)
#define CAN_F2R1_FB14_Msk      (0x1UL << CAN_F2R1_FB14_Pos)                     /*!< 0x00004000 */
#define CAN_F2R1_FB14          CAN_F2R1_FB14_Msk                               /*!<Filter bit 14 */
#define CAN_F2R1_FB15_Pos      (15U)
#define CAN_F2R1_FB15_Msk      (0x1UL << CAN_F2R1_FB15_Pos)                     /*!< 0x00008000 */
#define CAN_F2R1_FB15          CAN_F2R1_FB15_Msk                               /*!<Filter bit 15 */
#define CAN_F2R1_FB16_Pos      (16U)
#define CAN_F2R1_FB16_Msk      (0x1UL << CAN_F2R1_FB16_Pos)                     /*!< 0x00010000 */
#define CAN_F2R1_FB16          CAN_F2R1_FB16_Msk                               /*!<Filter bit 16 */
#define CAN_F2R1_FB17_Pos      (17U)
#define CAN_F2R1_FB17_Msk      (0x1UL << CAN_F2R1_FB17_Pos)                     /*!< 0x00020000 */
#define CAN_F2R1_FB17          CAN_F2R1_FB17_Msk                               /*!<Filter bit 17 */
#define CAN_F2R1_FB18_Pos      (18U)
#define CAN_F2R1_FB18_Msk      (0x1UL << CAN_F2R1_FB18_Pos)                     /*!< 0x00040000 */
#define CAN_F2R1_FB18          CAN_F2R1_FB18_Msk                               /*!<Filter bit 18 */
#define CAN_F2R1_FB19_Pos      (19U)
#define CAN_F2R1_FB19_Msk      (0x1UL << CAN_F2R1_FB19_Pos)                     /*!< 0x00080000 */
#define CAN_F2R1_FB19          CAN_F2R1_FB19_Msk                               /*!<Filter bit 19 */
#define CAN_F2R1_FB20_Pos      (20U)
#define CAN_F2R1_FB20_Msk      (0x1UL << CAN_F2R1_FB20_Pos)                     /*!< 0x00100000 */
#define CAN_F2R1_FB20          CAN_F2R1_FB20_Msk                               /*!<Filter bit 20 */
#define CAN_F2R1_FB21_Pos      (21U)
#define CAN_F2R1_FB21_Msk      (0x1UL << CAN_F2R1_FB21_Pos)                     /*!< 0x00200000 */
#define CAN_F2R1_FB21          CAN_F2R1_FB21_Msk                               /*!<Filter bit 21 */
#define CAN_F2R1_FB22_Pos      (22U)
#define CAN_F2R1_FB22_Msk      (0x1UL << CAN_F2R1_FB22_Pos)                     /*!< 0x00400000 */
#define CAN_F2R1_FB22          CAN_F2R1_FB22_Msk                               /*!<Filter bit 22 */
#define CAN_F2R1_FB23_Pos      (23U)
#define CAN_F2R1_FB23_Msk      (0x1UL << CAN_F2R1_FB23_Pos)                     /*!< 0x00800000 */
#define CAN_F2R1_FB23          CAN_F2R1_FB23_Msk                               /*!<Filter bit 23 */
#define CAN_F2R1_FB24_Pos      (24U)
#define CAN_F2R1_FB24_Msk      (0x1UL << CAN_F2R1_FB24_Pos)                     /*!< 0x01000000 */
#define CAN_F2R1_FB24          CAN_F2R1_FB24_Msk                               /*!<Filter bit 24 */
#define CAN_F2R1_FB25_Pos      (25U)
#define CAN_F2R1_FB25_Msk      (0x1UL << CAN_F2R1_FB25_Pos)                     /*!< 0x02000000 */
#define CAN_F2R1_FB25          CAN_F2R1_FB25_Msk                               /*!<Filter bit 25 */
#define CAN_F2R1_FB26_Pos      (26U)
#define CAN_F2R1_FB26_Msk      (0x1UL << CAN_F2R1_FB26_Pos)                     /*!< 0x04000000 */
#define CAN_F2R1_FB26          CAN_F2R1_FB26_Msk                               /*!<Filter bit 26 */
#define CAN_F2R1_FB27_Pos      (27U)
#define CAN_F2R1_FB27_Msk      (0x1UL << CAN_F2R1_FB27_Pos)                     /*!< 0x08000000 */
#define CAN_F2R1_FB27          CAN_F2R1_FB27_Msk                               /*!<Filter bit 27 */
#define CAN_F2R1_FB28_Pos      (28U)
#define CAN_F2R1_FB28_Msk      (0x1UL << CAN_F2R1_FB28_Pos)                     /*!< 0x10000000 */
#define CAN_F2R1_FB28          CAN_F2R1_FB28_Msk                               /*!<Filter bit 28 */
#define CAN_F2R1_FB29_Pos      (29U)
#define CAN_F2R1_FB29_Msk      (0x1UL << CAN_F2R1_FB29_Pos)                     /*!< 0x20000000 */
#define CAN_F2R1_FB29          CAN_F2R1_FB29_Msk                               /*!<Filter bit 29 */
#define CAN_F2R1_FB30_Pos      (30U)
#define CAN_F2R1_FB30_Msk      (0x1UL << CAN_F2R1_FB30_Pos)                     /*!< 0x40000000 */
#define CAN_F2R1_FB30          CAN_F2R1_FB30_Msk                               /*!<Filter bit 30 */
#define CAN_F2R1_FB31_Pos      (31U)
#define CAN_F2R1_FB31_Msk      (0x1UL << CAN_F2R1_FB31_Pos)                     /*!< 0x80000000 */
#define CAN_F2R1_FB31          CAN_F2R1_FB31_Msk                               /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F3R1 register  *******************/
#define CAN_F3R1_FB0_Pos       (0U)
#define CAN_F3R1_FB0_Msk       (0x1UL << CAN_F3R1_FB0_Pos)                      /*!< 0x00000001 */
#define CAN_F3R1_FB0           CAN_F3R1_FB0_Msk                                /*!<Filter bit 0 */
#define CAN_F3R1_FB1_Pos       (1U)
#define CAN_F3R1_FB1_Msk       (0x1UL << CAN_F3R1_FB1_Pos)                      /*!< 0x00000002 */
#define CAN_F3R1_FB1           CAN_F3R1_FB1_Msk                                /*!<Filter bit 1 */
#define CAN_F3R1_FB2_Pos       (2U)
#define CAN_F3R1_FB2_Msk       (0x1UL << CAN_F3R1_FB2_Pos)                      /*!< 0x00000004 */
#define CAN_F3R1_FB2           CAN_F3R1_FB2_Msk                                /*!<Filter bit 2 */
#define CAN_F3R1_FB3_Pos       (3U)
#define CAN_F3R1_FB3_Msk       (0x1UL << CAN_F3R1_FB3_Pos)                      /*!< 0x00000008 */
#define CAN_F3R1_FB3           CAN_F3R1_FB3_Msk                                /*!<Filter bit 3 */
#define CAN_F3R1_FB4_Pos       (4U)
#define CAN_F3R1_FB4_Msk       (0x1UL << CAN_F3R1_FB4_Pos)                      /*!< 0x00000010 */
#define CAN_F3R1_FB4           CAN_F3R1_FB4_Msk                                /*!<Filter bit 4 */
#define CAN_F3R1_FB5_Pos       (5U)
#define CAN_F3R1_FB5_Msk       (0x1UL << CAN_F3R1_FB5_Pos)                      /*!< 0x00000020 */
#define CAN_F3R1_FB5           CAN_F3R1_FB5_Msk                                /*!<Filter bit 5 */
#define CAN_F3R1_FB6_Pos       (6U)
#define CAN_F3R1_FB6_Msk       (0x1UL << CAN_F3R1_FB6_Pos)                      /*!< 0x00000040 */
#define CAN_F3R1_FB6           CAN_F3R1_FB6_Msk                                /*!<Filter bit 6 */
#define CAN_F3R1_FB7_Pos       (7U)
#define CAN_F3R1_FB7_Msk       (0x1UL << CAN_F3R1_FB7_Pos)                      /*!< 0x00000080 */
#define CAN_F3R1_FB7           CAN_F3R1_FB7_Msk                                /*!<Filter bit 7 */
#define CAN_F3R1_FB8_Pos       (8U)
#define CAN_F3R1_FB8_Msk       (0x1UL << CAN_F3R1_FB8_Pos)                      /*!< 0x00000100 */
#define CAN_F3R1_FB8           CAN_F3R1_FB8_Msk                                /*!<Filter bit 8 */
#define CAN_F3R1_FB9_Pos       (9U)
#define CAN_F3R1_FB9_Msk       (0x1UL << CAN_F3R1_FB9_Pos)                      /*!< 0x00000200 */
#define CAN_F3R1_FB9           CAN_F3R1_FB9_Msk                                /*!<Filter bit 9 */
#define CAN_F3R1_FB10_Pos      (10U)
#define CAN_F3R1_FB10_Msk      (0x1UL << CAN_F3R1_FB10_Pos)                     /*!< 0x00000400 */
#define CAN_F3R1_FB10          CAN_F3R1_FB10_Msk                               /*!<Filter bit 10 */
#define CAN_F3R1_FB11_Pos      (11U)
#define CAN_F3R1_FB11_Msk      (0x1UL << CAN_F3R1_FB11_Pos)                     /*!< 0x00000800 */
#define CAN_F3R1_FB11          CAN_F3R1_FB11_Msk                               /*!<Filter bit 11 */
#define CAN_F3R1_FB12_Pos      (12U)
#define CAN_F3R1_FB12_Msk      (0x1UL << CAN_F3R1_FB12_Pos)                     /*!< 0x00001000 */
#define CAN_F3R1_FB12          CAN_F3R1_FB12_Msk                               /*!<Filter bit 12 */
#define CAN_F3R1_FB13_Pos      (13U)
#define CAN_F3R1_FB13_Msk      (0x1UL << CAN_F3R1_FB13_Pos)                     /*!< 0x00002000 */
#define CAN_F3R1_FB13          CAN_F3R1_FB13_Msk                               /*!<Filter bit 13 */
#define CAN_F3R1_FB14_Pos      (14U)
#define CAN_F3R1_FB14_Msk      (0x1UL << CAN_F3R1_FB14_Pos)                     /*!< 0x00004000 */
#define CAN_F3R1_FB14          CAN_F3R1_FB14_Msk                               /*!<Filter bit 14 */
#define CAN_F3R1_FB15_Pos      (15U)
#define CAN_F3R1_FB15_Msk      (0x1UL << CAN_F3R1_FB15_Pos)                     /*!< 0x00008000 */
#define CAN_F3R1_FB15          CAN_F3R1_FB15_Msk                               /*!<Filter bit 15 */
#define CAN_F3R1_FB16_Pos      (16U)
#define CAN_F3R1_FB16_Msk      (0x1UL << CAN_F3R1_FB16_Pos)                     /*!< 0x00010000 */
#define CAN_F3R1_FB16          CAN_F3R1_FB16_Msk                               /*!<Filter bit 16 */
#define CAN_F3R1_FB17_Pos      (17U)
#define CAN_F3R1_FB17_Msk      (0x1UL << CAN_F3R1_FB17_Pos)                     /*!< 0x00020000 */
#define CAN_F3R1_FB17          CAN_F3R1_FB17_Msk                               /*!<Filter bit 17 */
#define CAN_F3R1_FB18_Pos      (18U)
#define CAN_F3R1_FB18_Msk      (0x1UL << CAN_F3R1_FB18_Pos)                     /*!< 0x00040000 */
#define CAN_F3R1_FB18          CAN_F3R1_FB18_Msk                               /*!<Filter bit 18 */
#define CAN_F3R1_FB19_Pos      (19U)
#define CAN_F3R1_FB19_Msk      (0x1UL << CAN_F3R1_FB19_Pos)                     /*!< 0x00080000 */
#define CAN_F3R1_FB19          CAN_F3R1_FB19_Msk                               /*!<Filter bit 19 */
#define CAN_F3R1_FB20_Pos      (20U)
#define CAN_F3R1_FB20_Msk      (0x1UL << CAN_F3R1_FB20_Pos)                     /*!< 0x00100000 */
#define CAN_F3R1_FB20          CAN_F3R1_FB20_Msk                               /*!<Filter bit 20 */
#define CAN_F3R1_FB21_Pos      (21U)
#define CAN_F3R1_FB21_Msk      (0x1UL << CAN_F3R1_FB21_Pos)                     /*!< 0x00200000 */
#define CAN_F3R1_FB21          CAN_F3R1_FB21_Msk                               /*!<Filter bit 21 */
#define CAN_F3R1_FB22_Pos      (22U)
#define CAN_F3R1_FB22_Msk      (0x1UL << CAN_F3R1_FB22_Pos)                     /*!< 0x00400000 */
#define CAN_F3R1_FB22          CAN_F3R1_FB22_Msk                               /*!<Filter bit 22 */
#define CAN_F3R1_FB23_Pos      (23U)
#define CAN_F3R1_FB23_Msk      (0x1UL << CAN_F3R1_FB23_Pos)                     /*!< 0x00800000 */
#define CAN_F3R1_FB23          CAN_F3R1_FB23_Msk                               /*!<Filter bit 23 */
#define CAN_F3R1_FB24_Pos      (24U)
#define CAN_F3R1_FB24_Msk      (0x1UL << CAN_F3R1_FB24_Pos)                     /*!< 0x01000000 */
#define CAN_F3R1_FB24          CAN_F3R1_FB24_Msk                               /*!<Filter bit 24 */
#define CAN_F3R1_FB25_Pos      (25U)
#define CAN_F3R1_FB25_Msk      (0x1UL << CAN_F3R1_FB25_Pos)                     /*!< 0x02000000 */
#define CAN_F3R1_FB25          CAN_F3R1_FB25_Msk                               /*!<Filter bit 25 */
#define CAN_F3R1_FB26_Pos      (26U)
#define CAN_F3R1_FB26_Msk      (0x1UL << CAN_F3R1_FB26_Pos)                     /*!< 0x04000000 */
#define CAN_F3R1_FB26          CAN_F3R1_FB26_Msk                               /*!<Filter bit 26 */
#define CAN_F3R1_FB27_Pos      (27U)
#define CAN_F3R1_FB27_Msk      (0x1UL << CAN_F3R1_FB27_Pos)                     /*!< 0x08000000 */
#define CAN_F3R1_FB27          CAN_F3R1_FB27_Msk                               /*!<Filter bit 27 */
#define CAN_F3R1_FB28_Pos      (28U)
#define CAN_F3R1_FB28_Msk      (0x1UL << CAN_F3R1_FB28_Pos)                     /*!< 0x10000000 */
#define CAN_F3R1_FB28          CAN_F3R1_FB28_Msk                               /*!<Filter bit 28 */
#define CAN_F3R1_FB29_Pos      (29U)
#define CAN_F3R1_FB29_Msk      (0x1UL << CAN_F3R1_FB29_Pos)                     /*!< 0x20000000 */
#define CAN_F3R1_FB29          CAN_F3R1_FB29_Msk                               /*!<Filter bit 29 */
#define CAN_F3R1_FB30_Pos      (30U)
#define CAN_F3R1_FB30_Msk      (0x1UL << CAN_F3R1_FB30_Pos)                     /*!< 0x40000000 */
#define CAN_F3R1_FB30          CAN_F3R1_FB30_Msk                               /*!<Filter bit 30 */
#define CAN_F3R1_FB31_Pos      (31U)
#define CAN_F3R1_FB31_Msk      (0x1UL << CAN_F3R1_FB31_Pos)                     /*!< 0x80000000 */
#define CAN_F3R1_FB31          CAN_F3R1_FB31_Msk                               /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F4R1 register  *******************/
#define CAN_F4R1_FB0_Pos       (0U)
#define CAN_F4R1_FB0_Msk       (0x1UL << CAN_F4R1_FB0_Pos)                      /*!< 0x00000001 */
#define CAN_F4R1_FB0           CAN_F4R1_FB0_Msk                                /*!<Filter bit 0 */
#define CAN_F4R1_FB1_Pos       (1U)
#define CAN_F4R1_FB1_Msk       (0x1UL << CAN_F4R1_FB1_Pos)                      /*!< 0x00000002 */
#define CAN_F4R1_FB1           CAN_F4R1_FB1_Msk                                /*!<Filter bit 1 */
#define CAN_F4R1_FB2_Pos       (2U)
#define CAN_F4R1_FB2_Msk       (0x1UL << CAN_F4R1_FB2_Pos)                      /*!< 0x00000004 */
#define CAN_F4R1_FB2           CAN_F4R1_FB2_Msk                                /*!<Filter bit 2 */
#define CAN_F4R1_FB3_Pos       (3U)
#define CAN_F4R1_FB3_Msk       (0x1UL << CAN_F4R1_FB3_Pos)                      /*!< 0x00000008 */
#define CAN_F4R1_FB3           CAN_F4R1_FB3_Msk                                /*!<Filter bit 3 */
#define CAN_F4R1_FB4_Pos       (4U)
#define CAN_F4R1_FB4_Msk       (0x1UL << CAN_F4R1_FB4_Pos)                      /*!< 0x00000010 */
#define CAN_F4R1_FB4           CAN_F4R1_FB4_Msk                                /*!<Filter bit 4 */
#define CAN_F4R1_FB5_Pos       (5U)
#define CAN_F4R1_FB5_Msk       (0x1UL << CAN_F4R1_FB5_Pos)                      /*!< 0x00000020 */
#define CAN_F4R1_FB5           CAN_F4R1_FB5_Msk                                /*!<Filter bit 5 */
#define CAN_F4R1_FB6_Pos       (6U)
#define CAN_F4R1_FB6_Msk       (0x1UL << CAN_F4R1_FB6_Pos)                      /*!< 0x00000040 */
#define CAN_F4R1_FB6           CAN_F4R1_FB6_Msk                                /*!<Filter bit 6 */
#define CAN_F4R1_FB7_Pos       (7U)
#define CAN_F4R1_FB7_Msk       (0x1UL << CAN_F4R1_FB7_Pos)                      /*!< 0x00000080 */
#define CAN_F4R1_FB7           CAN_F4R1_FB7_Msk                                /*!<Filter bit 7 */
#define CAN_F4R1_FB8_Pos       (8U)
#define CAN_F4R1_FB8_Msk       (0x1UL << CAN_F4R1_FB8_Pos)                      /*!< 0x00000100 */
#define CAN_F4R1_FB8           CAN_F4R1_FB8_Msk                                /*!<Filter bit 8 */
#define CAN_F4R1_FB9_Pos       (9U)
#define CAN_F4R1_FB9_Msk       (0x1UL << CAN_F4R1_FB9_Pos)                      /*!< 0x00000200 */
#define CAN_F4R1_FB9           CAN_F4R1_FB9_Msk                                /*!<Filter bit 9 */
#define CAN_F4R1_FB10_Pos      (10U)
#define CAN_F4R1_FB10_Msk      (0x1UL << CAN_F4R1_FB10_Pos)                     /*!< 0x00000400 */
#define CAN_F4R1_FB10          CAN_F4R1_FB10_Msk                               /*!<Filter bit 10 */
#define CAN_F4R1_FB11_Pos      (11U)
#define CAN_F4R1_FB11_Msk      (0x1UL << CAN_F4R1_FB11_Pos)                     /*!< 0x00000800 */
#define CAN_F4R1_FB11          CAN_F4R1_FB11_Msk                               /*!<Filter bit 11 */
#define CAN_F4R1_FB12_Pos      (12U)
#define CAN_F4R1_FB12_Msk      (0x1UL << CAN_F4R1_FB12_Pos)                     /*!< 0x00001000 */
#define CAN_F4R1_FB12          CAN_F4R1_FB12_Msk                               /*!<Filter bit 12 */
#define CAN_F4R1_FB13_Pos      (13U)
#define CAN_F4R1_FB13_Msk      (0x1UL << CAN_F4R1_FB13_Pos)                     /*!< 0x00002000 */
#define CAN_F4R1_FB13          CAN_F4R1_FB13_Msk                               /*!<Filter bit 13 */
#define CAN_F4R1_FB14_Pos      (14U)
#define CAN_F4R1_FB14_Msk      (0x1UL << CAN_F4R1_FB14_Pos)                     /*!< 0x00004000 */
#define CAN_F4R1_FB14          CAN_F4R1_FB14_Msk                               /*!<Filter bit 14 */
#define CAN_F4R1_FB15_Pos      (15U)
#define CAN_F4R1_FB15_Msk      (0x1UL << CAN_F4R1_FB15_Pos)                     /*!< 0x00008000 */
#define CAN_F4R1_FB15          CAN_F4R1_FB15_Msk                               /*!<Filter bit 15 */
#define CAN_F4R1_FB16_Pos      (16U)
#define CAN_F4R1_FB16_Msk      (0x1UL << CAN_F4R1_FB16_Pos)                     /*!< 0x00010000 */
#define CAN_F4R1_FB16          CAN_F4R1_FB16_Msk                               /*!<Filter bit 16 */
#define CAN_F4R1_FB17_Pos      (17U)
#define CAN_F4R1_FB17_Msk      (0x1UL << CAN_F4R1_FB17_Pos)                     /*!< 0x00020000 */
#define CAN_F4R1_FB17          CAN_F4R1_FB17_Msk                               /*!<Filter bit 17 */
#define CAN_F4R1_FB18_Pos      (18U)
#define CAN_F4R1_FB18_Msk      (0x1UL << CAN_F4R1_FB18_Pos)                     /*!< 0x00040000 */
#define CAN_F4R1_FB18          CAN_F4R1_FB18_Msk                               /*!<Filter bit 18 */
#define CAN_F4R1_FB19_Pos      (19U)
#define CAN_F4R1_FB19_Msk      (0x1UL << CAN_F4R1_FB19_Pos)                     /*!< 0x00080000 */
#define CAN_F4R1_FB19          CAN_F4R1_FB19_Msk                               /*!<Filter bit 19 */
#define CAN_F4R1_FB20_Pos      (20U)
#define CAN_F4R1_FB20_Msk      (0x1UL << CAN_F4R1_FB20_Pos)                     /*!< 0x00100000 */
#define CAN_F4R1_FB20          CAN_F4R1_FB20_Msk                               /*!<Filter bit 20 */
#define CAN_F4R1_FB21_Pos      (21U)
#define CAN_F4R1_FB21_Msk      (0x1UL << CAN_F4R1_FB21_Pos)                     /*!< 0x00200000 */
#define CAN_F4R1_FB21          CAN_F4R1_FB21_Msk                               /*!<Filter bit 21 */
#define CAN_F4R1_FB22_Pos      (22U)
#define CAN_F4R1_FB22_Msk      (0x1UL << CAN_F4R1_FB22_Pos)                     /*!< 0x00400000 */
#define CAN_F4R1_FB22          CAN_F4R1_FB22_Msk                               /*!<Filter bit 22 */
#define CAN_F4R1_FB23_Pos      (23U)
#define CAN_F4R1_FB23_Msk      (0x1UL << CAN_F4R1_FB23_Pos)                     /*!< 0x00800000 */
#define CAN_F4R1_FB23          CAN_F4R1_FB23_Msk                               /*!<Filter bit 23 */
#define CAN_F4R1_FB24_Pos      (24U)
#define CAN_F4R1_FB24_Msk      (0x1UL << CAN_F4R1_FB24_Pos)                     /*!< 0x01000000 */
#define CAN_F4R1_FB24          CAN_F4R1_FB24_Msk                               /*!<Filter bit 24 */
#define CAN_F4R1_FB25_Pos      (25U)
#define CAN_F4R1_FB25_Msk      (0x1UL << CAN_F4R1_FB25_Pos)                     /*!< 0x02000000 */
#define CAN_F4R1_FB25          CAN_F4R1_FB25_Msk                               /*!<Filter bit 25 */
#define CAN_F4R1_FB26_Pos      (26U)
#define CAN_F4R1_FB26_Msk      (0x1UL << CAN_F4R1_FB26_Pos)                     /*!< 0x04000000 */
#define CAN_F4R1_FB26          CAN_F4R1_FB26_Msk                               /*!<Filter bit 26 */
#define CAN_F4R1_FB27_Pos      (27U)
#define CAN_F4R1_FB27_Msk      (0x1UL << CAN_F4R1_FB27_Pos)                     /*!< 0x08000000 */
#define CAN_F4R1_FB27          CAN_F4R1_FB27_Msk                               /*!<Filter bit 27 */
#define CAN_F4R1_FB28_Pos      (28U)
#define CAN_F4R1_FB28_Msk      (0x1UL << CAN_F4R1_FB28_Pos)                     /*!< 0x10000000 */
#define CAN_F4R1_FB28          CAN_F4R1_FB28_Msk                               /*!<Filter bit 28 */
#define CAN_F4R1_FB29_Pos      (29U)
#define CAN_F4R1_FB29_Msk      (0x1UL << CAN_F4R1_FB29_Pos)                     /*!< 0x20000000 */
#define CAN_F4R1_FB29          CAN_F4R1_FB29_Msk                               /*!<Filter bit 29 */
#define CAN_F4R1_FB30_Pos      (30U)
#define CAN_F4R1_FB30_Msk      (0x1UL << CAN_F4R1_FB30_Pos)                     /*!< 0x40000000 */
#define CAN_F4R1_FB30          CAN_F4R1_FB30_Msk                               /*!<Filter bit 30 */
#define CAN_F4R1_FB31_Pos      (31U)
#define CAN_F4R1_FB31_Msk      (0x1UL << CAN_F4R1_FB31_Pos)                     /*!< 0x80000000 */
#define CAN_F4R1_FB31          CAN_F4R1_FB31_Msk                               /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F5R1 register  *******************/
#define CAN_F5R1_FB0_Pos       (0U)
#define CAN_F5R1_FB0_Msk       (0x1UL << CAN_F5R1_FB0_Pos)                      /*!< 0x00000001 */
#define CAN_F5R1_FB0           CAN_F5R1_FB0_Msk                                /*!<Filter bit 0 */
#define CAN_F5R1_FB1_Pos       (1U)
#define CAN_F5R1_FB1_Msk       (0x1UL << CAN_F5R1_FB1_Pos)                      /*!< 0x00000002 */
#define CAN_F5R1_FB1           CAN_F5R1_FB1_Msk                                /*!<Filter bit 1 */
#define CAN_F5R1_FB2_Pos       (2U)
#define CAN_F5R1_FB2_Msk       (0x1UL << CAN_F5R1_FB2_Pos)                      /*!< 0x00000004 */
#define CAN_F5R1_FB2           CAN_F5R1_FB2_Msk                                /*!<Filter bit 2 */
#define CAN_F5R1_FB3_Pos       (3U)
#define CAN_F5R1_FB3_Msk       (0x1UL << CAN_F5R1_FB3_Pos)                      /*!< 0x00000008 */
#define CAN_F5R1_FB3           CAN_F5R1_FB3_Msk                                /*!<Filter bit 3 */
#define CAN_F5R1_FB4_Pos       (4U)
#define CAN_F5R1_FB4_Msk       (0x1UL << CAN_F5R1_FB4_Pos)                      /*!< 0x00000010 */
#define CAN_F5R1_FB4           CAN_F5R1_FB4_Msk                                /*!<Filter bit 4 */
#define CAN_F5R1_FB5_Pos       (5U)
#define CAN_F5R1_FB5_Msk       (0x1UL << CAN_F5R1_FB5_Pos)                      /*!< 0x00000020 */
#define CAN_F5R1_FB5           CAN_F5R1_FB5_Msk                                /*!<Filter bit 5 */
#define CAN_F5R1_FB6_Pos       (6U)
#define CAN_F5R1_FB6_Msk       (0x1UL << CAN_F5R1_FB6_Pos)                      /*!< 0x00000040 */
#define CAN_F5R1_FB6           CAN_F5R1_FB6_Msk                                /*!<Filter bit 6 */
#define CAN_F5R1_FB7_Pos       (7U)
#define CAN_F5R1_FB7_Msk       (0x1UL << CAN_F5R1_FB7_Pos)                      /*!< 0x00000080 */
#define CAN_F5R1_FB7           CAN_F5R1_FB7_Msk                                /*!<Filter bit 7 */
#define CAN_F5R1_FB8_Pos       (8U)
#define CAN_F5R1_FB8_Msk       (0x1UL << CAN_F5R1_FB8_Pos)                      /*!< 0x00000100 */
#define CAN_F5R1_FB8           CAN_F5R1_FB8_Msk                                /*!<Filter bit 8 */
#define CAN_F5R1_FB9_Pos       (9U)
#define CAN_F5R1_FB9_Msk       (0x1UL << CAN_F5R1_FB9_Pos)                      /*!< 0x00000200 */
#define CAN_F5R1_FB9           CAN_F5R1_FB9_Msk                                /*!<Filter bit 9 */
#define CAN_F5R1_FB10_Pos      (10U)
#define CAN_F5R1_FB10_Msk      (0x1UL << CAN_F5R1_FB10_Pos)                     /*!< 0x00000400 */
#define CAN_F5R1_FB10          CAN_F5R1_FB10_Msk                               /*!<Filter bit 10 */
#define CAN_F5R1_FB11_Pos      (11U)
#define CAN_F5R1_FB11_Msk      (0x1UL << CAN_F5R1_FB11_Pos)                     /*!< 0x00000800 */
#define CAN_F5R1_FB11          CAN_F5R1_FB11_Msk                               /*!<Filter bit 11 */
#define CAN_F5R1_FB12_Pos      (12U)
#define CAN_F5R1_FB12_Msk      (0x1UL << CAN_F5R1_FB12_Pos)                     /*!< 0x00001000 */
#define CAN_F5R1_FB12          CAN_F5R1_FB12_Msk                               /*!<Filter bit 12 */
#define CAN_F5R1_FB13_Pos      (13U)
#define CAN_F5R1_FB13_Msk      (0x1UL << CAN_F5R1_FB13_Pos)                     /*!< 0x00002000 */
#define CAN_F5R1_FB13          CAN_F5R1_FB13_Msk                               /*!<Filter bit 13 */
#define CAN_F5R1_FB14_Pos      (14U)
#define CAN_F5R1_FB14_Msk      (0x1UL << CAN_F5R1_FB14_Pos)                     /*!< 0x00004000 */
#define CAN_F5R1_FB14          CAN_F5R1_FB14_Msk                               /*!<Filter bit 14 */
#define CAN_F5R1_FB15_Pos      (15U)
#define CAN_F5R1_FB15_Msk      (0x1UL << CAN_F5R1_FB15_Pos)                     /*!< 0x00008000 */
#define CAN_F5R1_FB15          CAN_F5R1_FB15_Msk                               /*!<Filter bit 15 */
#define CAN_F5R1_FB16_Pos      (16U)
#define CAN_F5R1_FB16_Msk      (0x1UL << CAN_F5R1_FB16_Pos)                     /*!< 0x00010000 */
#define CAN_F5R1_FB16          CAN_F5R1_FB16_Msk                               /*!<Filter bit 16 */
#define CAN_F5R1_FB17_Pos      (17U)
#define CAN_F5R1_FB17_Msk      (0x1UL << CAN_F5R1_FB17_Pos)                     /*!< 0x00020000 */
#define CAN_F5R1_FB17          CAN_F5R1_FB17_Msk                               /*!<Filter bit 17 */
#define CAN_F5R1_FB18_Pos      (18U)
#define CAN_F5R1_FB18_Msk      (0x1UL << CAN_F5R1_FB18_Pos)                     /*!< 0x00040000 */
#define CAN_F5R1_FB18          CAN_F5R1_FB18_Msk                               /*!<Filter bit 18 */
#define CAN_F5R1_FB19_Pos      (19U)
#define CAN_F5R1_FB19_Msk      (0x1UL << CAN_F5R1_FB19_Pos)                     /*!< 0x00080000 */
#define CAN_F5R1_FB19          CAN_F5R1_FB19_Msk                               /*!<Filter bit 19 */
#define CAN_F5R1_FB20_Pos      (20U)
#define CAN_F5R1_FB20_Msk      (0x1UL << CAN_F5R1_FB20_Pos)                     /*!< 0x00100000 */
#define CAN_F5R1_FB20          CAN_F5R1_FB20_Msk                               /*!<Filter bit 20 */
#define CAN_F5R1_FB21_Pos      (21U)
#define CAN_F5R1_FB21_Msk      (0x1UL << CAN_F5R1_FB21_Pos)                     /*!< 0x00200000 */
#define CAN_F5R1_FB21          CAN_F5R1_FB21_Msk                               /*!<Filter bit 21 */
#define CAN_F5R1_FB22_Pos      (22U)
#define CAN_F5R1_FB22_Msk      (0x1UL << CAN_F5R1_FB22_Pos)                     /*!< 0x00400000 */
#define CAN_F5R1_FB22          CAN_F5R1_FB22_Msk                               /*!<Filter bit 22 */
#define CAN_F5R1_FB23_Pos      (23U)
#define CAN_F5R1_FB23_Msk      (0x1UL << CAN_F5R1_FB23_Pos)                     /*!< 0x00800000 */
#define CAN_F5R1_FB23          CAN_F5R1_FB23_Msk                               /*!<Filter bit 23 */
#define CAN_F5R1_FB24_Pos      (24U)
#define CAN_F5R1_FB24_Msk      (0x1UL << CAN_F5R1_FB24_Pos)                     /*!< 0x01000000 */
#define CAN_F5R1_FB24          CAN_F5R1_FB24_Msk                               /*!<Filter bit 24 */
#define CAN_F5R1_FB25_Pos      (25U)
#define CAN_F5R1_FB25_Msk      (0x1UL << CAN_F5R1_FB25_Pos)                     /*!< 0x02000000 */
#define CAN_F5R1_FB25          CAN_F5R1_FB25_Msk                               /*!<Filter bit 25 */
#define CAN_F5R1_FB26_Pos      (26U)
#define CAN_F5R1_FB26_Msk      (0x1UL << CAN_F5R1_FB26_Pos)                     /*!< 0x04000000 */
#define CAN_F5R1_FB26          CAN_F5R1_FB26_Msk                               /*!<Filter bit 26 */
#define CAN_F5R1_FB27_Pos      (27U)
#define CAN_F5R1_FB27_Msk      (0x1UL << CAN_F5R1_FB27_Pos)                     /*!< 0x08000000 */
#define CAN_F5R1_FB27          CAN_F5R1_FB27_Msk                               /*!<Filter bit 27 */
#define CAN_F5R1_FB28_Pos      (28U)
#define CAN_F5R1_FB28_Msk      (0x1UL << CAN_F5R1_FB28_Pos)                     /*!< 0x10000000 */
#define CAN_F5R1_FB28          CAN_F5R1_FB28_Msk                               /*!<Filter bit 28 */
#define CAN_F5R1_FB29_Pos      (29U)
#define CAN_F5R1_FB29_Msk      (0x1UL << CAN_F5R1_FB29_Pos)                     /*!< 0x20000000 */
#define CAN_F5R1_FB29          CAN_F5R1_FB29_Msk                               /*!<Filter bit 29 */
#define CAN_F5R1_FB30_Pos      (30U)
#define CAN_F5R1_FB30_Msk      (0x1UL << CAN_F5R1_FB30_Pos)                     /*!< 0x40000000 */
#define CAN_F5R1_FB30          CAN_F5R1_FB30_Msk                               /*!<Filter bit 30 */
#define CAN_F5R1_FB31_Pos      (31U)
#define CAN_F5R1_FB31_Msk      (0x1UL << CAN_F5R1_FB31_Pos)                     /*!< 0x80000000 */
#define CAN_F5R1_FB31          CAN_F5R1_FB31_Msk                               /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F6R1 register  *******************/
#define CAN_F6R1_FB0_Pos       (0U)
#define CAN_F6R1_FB0_Msk       (0x1UL << CAN_F6R1_FB0_Pos)                      /*!< 0x00000001 */
#define CAN_F6R1_FB0           CAN_F6R1_FB0_Msk                                /*!<Filter bit 0 */
#define CAN_F6R1_FB1_Pos       (1U)
#define CAN_F6R1_FB1_Msk       (0x1UL << CAN_F6R1_FB1_Pos)                      /*!< 0x00000002 */
#define CAN_F6R1_FB1           CAN_F6R1_FB1_Msk                                /*!<Filter bit 1 */
#define CAN_F6R1_FB2_Pos       (2U)
#define CAN_F6R1_FB2_Msk       (0x1UL << CAN_F6R1_FB2_Pos)                      /*!< 0x00000004 */
#define CAN_F6R1_FB2           CAN_F6R1_FB2_Msk                                /*!<Filter bit 2 */
#define CAN_F6R1_FB3_Pos       (3U)
#define CAN_F6R1_FB3_Msk       (0x1UL << CAN_F6R1_FB3_Pos)                      /*!< 0x00000008 */
#define CAN_F6R1_FB3           CAN_F6R1_FB3_Msk                                /*!<Filter bit 3 */
#define CAN_F6R1_FB4_Pos       (4U)
#define CAN_F6R1_FB4_Msk       (0x1UL << CAN_F6R1_FB4_Pos)                      /*!< 0x00000010 */
#define CAN_F6R1_FB4           CAN_F6R1_FB4_Msk                                /*!<Filter bit 4 */
#define CAN_F6R1_FB5_Pos       (5U)
#define CAN_F6R1_FB5_Msk       (0x1UL << CAN_F6R1_FB5_Pos)                      /*!< 0x00000020 */
#define CAN_F6R1_FB5           CAN_F6R1_FB5_Msk                                /*!<Filter bit 5 */
#define CAN_F6R1_FB6_Pos       (6U)
#define CAN_F6R1_FB6_Msk       (0x1UL << CAN_F6R1_FB6_Pos)                      /*!< 0x00000040 */
#define CAN_F6R1_FB6           CAN_F6R1_FB6_Msk                                /*!<Filter bit 6 */
#define CAN_F6R1_FB7_Pos       (7U)
#define CAN_F6R1_FB7_Msk       (0x1UL << CAN_F6R1_FB7_Pos)                      /*!< 0x00000080 */
#define CAN_F6R1_FB7           CAN_F6R1_FB7_Msk                                /*!<Filter bit 7 */
#define CAN_F6R1_FB8_Pos       (8U)
#define CAN_F6R1_FB8_Msk       (0x1UL << CAN_F6R1_FB8_Pos)                      /*!< 0x00000100 */
#define CAN_F6R1_FB8           CAN_F6R1_FB8_Msk                                /*!<Filter bit 8 */
#define CAN_F6R1_FB9_Pos       (9U)
#define CAN_F6R1_FB9_Msk       (0x1UL << CAN_F6R1_FB9_Pos)                      /*!< 0x00000200 */
#define CAN_F6R1_FB9           CAN_F6R1_FB9_Msk                                /*!<Filter bit 9 */
#define CAN_F6R1_FB10_Pos      (10U)
#define CAN_F6R1_FB10_Msk      (0x1UL << CAN_F6R1_FB10_Pos)                     /*!< 0x00000400 */
#define CAN_F6R1_FB10          CAN_F6R1_FB10_Msk                               /*!<Filter bit 10 */
#define CAN_F6R1_FB11_Pos      (11U)
#define CAN_F6R1_FB11_Msk      (0x1UL << CAN_F6R1_FB11_Pos)                     /*!< 0x00000800 */
#define CAN_F6R1_FB11          CAN_F6R1_FB11_Msk                               /*!<Filter bit 11 */
#define CAN_F6R1_FB12_Pos      (12U)
#define CAN_F6R1_FB12_Msk      (0x1UL << CAN_F6R1_FB12_Pos)                     /*!< 0x00001000 */
#define CAN_F6R1_FB12          CAN_F6R1_FB12_Msk                               /*!<Filter bit 12 */
#define CAN_F6R1_FB13_Pos      (13U)
#define CAN_F6R1_FB13_Msk      (0x1UL << CAN_F6R1_FB13_Pos)                     /*!< 0x00002000 */
#define CAN_F6R1_FB13          CAN_F6R1_FB13_Msk                               /*!<Filter bit 13 */
#define CAN_F6R1_FB14_Pos      (14U)
#define CAN_F6R1_FB14_Msk      (0x1UL << CAN_F6R1_FB14_Pos)                     /*!< 0x00004000 */
#define CAN_F6R1_FB14          CAN_F6R1_FB14_Msk                               /*!<Filter bit 14 */
#define CAN_F6R1_FB15_Pos      (15U)
#define CAN_F6R1_FB15_Msk      (0x1UL << CAN_F6R1_FB15_Pos)                     /*!< 0x00008000 */
#define CAN_F6R1_FB15          CAN_F6R1_FB15_Msk                               /*!<Filter bit 15 */
#define CAN_F6R1_FB16_Pos      (16U)
#define CAN_F6R1_FB16_Msk      (0x1UL << CAN_F6R1_FB16_Pos)                     /*!< 0x00010000 */
#define CAN_F6R1_FB16          CAN_F6R1_FB16_Msk                               /*!<Filter bit 16 */
#define CAN_F6R1_FB17_Pos      (17U)
#define CAN_F6R1_FB17_Msk      (0x1UL << CAN_F6R1_FB17_Pos)                     /*!< 0x00020000 */
#define CAN_F6R1_FB17          CAN_F6R1_FB17_Msk                               /*!<Filter bit 17 */
#define CAN_F6R1_FB18_Pos      (18U)
#define CAN_F6R1_FB18_Msk      (0x1UL << CAN_F6R1_FB18_Pos)                     /*!< 0x00040000 */
#define CAN_F6R1_FB18          CAN_F6R1_FB18_Msk                               /*!<Filter bit 18 */
#define CAN_F6R1_FB19_Pos      (19U)
#define CAN_F6R1_FB19_Msk      (0x1UL << CAN_F6R1_FB19_Pos)                     /*!< 0x00080000 */
#define CAN_F6R1_FB19          CAN_F6R1_FB19_Msk                               /*!<Filter bit 19 */
#define CAN_F6R1_FB20_Pos      (20U)
#define CAN_F6R1_FB20_Msk      (0x1UL << CAN_F6R1_FB20_Pos)                     /*!< 0x00100000 */
#define CAN_F6R1_FB20          CAN_F6R1_FB20_Msk                               /*!<Filter bit 20 */
#define CAN_F6R1_FB21_Pos      (21U)
#define CAN_F6R1_FB21_Msk      (0x1UL << CAN_F6R1_FB21_Pos)                     /*!< 0x00200000 */
#define CAN_F6R1_FB21          CAN_F6R1_FB21_Msk                               /*!<Filter bit 21 */
#define CAN_F6R1_FB22_Pos      (22U)
#define CAN_F6R1_FB22_Msk      (0x1UL << CAN_F6R1_FB22_Pos)                     /*!< 0x00400000 */
#define CAN_F6R1_FB22          CAN_F6R1_FB22_Msk                               /*!<Filter bit 22 */
#define CAN_F6R1_FB23_Pos      (23U)
#define CAN_F6R1_FB23_Msk      (0x1UL << CAN_F6R1_FB23_Pos)                     /*!< 0x00800000 */
#define CAN_F6R1_FB23          CAN_F6R1_FB23_Msk                               /*!<Filter bit 23 */
#define CAN_F6R1_FB24_Pos      (24U)
#define CAN_F6R1_FB24_Msk      (0x1UL << CAN_F6R1_FB24_Pos)                     /*!< 0x01000000 */
#define CAN_F6R1_FB24          CAN_F6R1_FB24_Msk                               /*!<Filter bit 24 */
#define CAN_F6R1_FB25_Pos      (25U)
#define CAN_F6R1_FB25_Msk      (0x1UL << CAN_F6R1_FB25_Pos)                     /*!< 0x02000000 */
#define CAN_F6R1_FB25          CAN_F6R1_FB25_Msk                               /*!<Filter bit 25 */
#define CAN_F6R1_FB26_Pos      (26U)
#define CAN_F6R1_FB26_Msk      (0x1UL << CAN_F6R1_FB26_Pos)                     /*!< 0x04000000 */
#define CAN_F6R1_FB26          CAN_F6R1_FB26_Msk                               /*!<Filter bit 26 */
#define CAN_F6R1_FB27_Pos      (27U)
#define CAN_F6R1_FB27_Msk      (0x1UL << CAN_F6R1_FB27_Pos)                     /*!< 0x08000000 */
#define CAN_F6R1_FB27          CAN_F6R1_FB27_Msk                               /*!<Filter bit 27 */
#define CAN_F6R1_FB28_Pos      (28U)
#define CAN_F6R1_FB28_Msk      (0x1UL << CAN_F6R1_FB28_Pos)                     /*!< 0x10000000 */
#define CAN_F6R1_FB28          CAN_F6R1_FB28_Msk                               /*!<Filter bit 28 */
#define CAN_F6R1_FB29_Pos      (29U)
#define CAN_F6R1_FB29_Msk      (0x1UL << CAN_F6R1_FB29_Pos)                     /*!< 0x20000000 */
#define CAN_F6R1_FB29          CAN_F6R1_FB29_Msk                               /*!<Filter bit 29 */
#define CAN_F6R1_FB30_Pos      (30U)
#define CAN_F6R1_FB30_Msk      (0x1UL << CAN_F6R1_FB30_Pos)                     /*!< 0x40000000 */
#define CAN_F6R1_FB30          CAN_F6R1_FB30_Msk                               /*!<Filter bit 30 */
#define CAN_F6R1_FB31_Pos      (31U)
#define CAN_F6R1_FB31_Msk      (0x1UL << CAN_F6R1_FB31_Pos)                     /*!< 0x80000000 */
#define CAN_F6R1_FB31          CAN_F6R1_FB31_Msk                               /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F7R1 register  *******************/
#define CAN_F7R1_FB0_Pos       (0U)
#define CAN_F7R1_FB0_Msk       (0x1UL << CAN_F7R1_FB0_Pos)                      /*!< 0x00000001 */
#define CAN_F7R1_FB0           CAN_F7R1_FB0_Msk                                /*!<Filter bit 0 */
#define CAN_F7R1_FB1_Pos       (1U)
#define CAN_F7R1_FB1_Msk       (0x1UL << CAN_F7R1_FB1_Pos)                      /*!< 0x00000002 */
#define CAN_F7R1_FB1           CAN_F7R1_FB1_Msk                                /*!<Filter bit 1 */
#define CAN_F7R1_FB2_Pos       (2U)
#define CAN_F7R1_FB2_Msk       (0x1UL << CAN_F7R1_FB2_Pos)                      /*!< 0x00000004 */
#define CAN_F7R1_FB2           CAN_F7R1_FB2_Msk                                /*!<Filter bit 2 */
#define CAN_F7R1_FB3_Pos       (3U)
#define CAN_F7R1_FB3_Msk       (0x1UL << CAN_F7R1_FB3_Pos)                      /*!< 0x00000008 */
#define CAN_F7R1_FB3           CAN_F7R1_FB3_Msk                                /*!<Filter bit 3 */
#define CAN_F7R1_FB4_Pos       (4U)
#define CAN_F7R1_FB4_Msk       (0x1UL << CAN_F7R1_FB4_Pos)                      /*!< 0x00000010 */
#define CAN_F7R1_FB4           CAN_F7R1_FB4_Msk                                /*!<Filter bit 4 */
#define CAN_F7R1_FB5_Pos       (5U)
#define CAN_F7R1_FB5_Msk       (0x1UL << CAN_F7R1_FB5_Pos)                      /*!< 0x00000020 */
#define CAN_F7R1_FB5           CAN_F7R1_FB5_Msk                                /*!<Filter bit 5 */
#define CAN_F7R1_FB6_Pos       (6U)
#define CAN_F7R1_FB6_Msk       (0x1UL << CAN_F7R1_FB6_Pos)                      /*!< 0x00000040 */
#define CAN_F7R1_FB6           CAN_F7R1_FB6_Msk                                /*!<Filter bit 6 */
#define CAN_F7R1_FB7_Pos       (7U)
#define CAN_F7R1_FB7_Msk       (0x1UL << CAN_F7R1_FB7_Pos)                      /*!< 0x00000080 */
#define CAN_F7R1_FB7           CAN_F7R1_FB7_Msk                                /*!<Filter bit 7 */
#define CAN_F7R1_FB8_Pos       (8U)
#define CAN_F7R1_FB8_Msk       (0x1UL << CAN_F7R1_FB8_Pos)                      /*!< 0x00000100 */
#define CAN_F7R1_FB8           CAN_F7R1_FB8_Msk                                /*!<Filter bit 8 */
#define CAN_F7R1_FB9_Pos       (9U)
#define CAN_F7R1_FB9_Msk       (0x1UL << CAN_F7R1_FB9_Pos)                      /*!< 0x00000200 */
#define CAN_F7R1_FB9           CAN_F7R1_FB9_Msk                                /*!<Filter bit 9 */
#define CAN_F7R1_FB10_Pos      (10U)
#define CAN_F7R1_FB10_Msk      (0x1UL << CAN_F7R1_FB10_Pos)                     /*!< 0x00000400 */
#define CAN_F7R1_FB10          CAN_F7R1_FB10_Msk                               /*!<Filter bit 10 */
#define CAN_F7R1_FB11_Pos      (11U)
#define CAN_F7R1_FB11_Msk      (0x1UL << CAN_F7R1_FB11_Pos)                     /*!< 0x00000800 */
#define CAN_F7R1_FB11          CAN_F7R1_FB11_Msk                               /*!<Filter bit 11 */
#define CAN_F7R1_FB12_Pos      (12U)
#define CAN_F7R1_FB12_Msk      (0x1UL << CAN_F7R1_FB12_Pos)                     /*!< 0x00001000 */
#define CAN_F7R1_FB12          CAN_F7R1_FB12_Msk                               /*!<Filter bit 12 */
#define CAN_F7R1_FB13_Pos      (13U)
#define CAN_F7R1_FB13_Msk      (0x1UL << CAN_F7R1_FB13_Pos)                     /*!< 0x00002000 */
#define CAN_F7R1_FB13          CAN_F7R1_FB13_Msk                               /*!<Filter bit 13 */
#define CAN_F7R1_FB14_Pos      (14U)
#define CAN_F7R1_FB14_Msk      (0x1UL << CAN_F7R1_FB14_Pos)                     /*!< 0x00004000 */
#define CAN_F7R1_FB14          CAN_F7R1_FB14_Msk                               /*!<Filter bit 14 */
#define CAN_F7R1_FB15_Pos      (15U)
#define CAN_F7R1_FB15_Msk      (0x1UL << CAN_F7R1_FB15_Pos)                     /*!< 0x00008000 */
#define CAN_F7R1_FB15          CAN_F7R1_FB15_Msk                               /*!<Filter bit 15 */
#define CAN_F7R1_FB16_Pos      (16U)
#define CAN_F7R1_FB16_Msk      (0x1UL << CAN_F7R1_FB16_Pos)                     /*!< 0x00010000 */
#define CAN_F7R1_FB16          CAN_F7R1_FB16_Msk                               /*!<Filter bit 16 */
#define CAN_F7R1_FB17_Pos      (17U)
#define CAN_F7R1_FB17_Msk      (0x1UL << CAN_F7R1_FB17_Pos)                     /*!< 0x00020000 */
#define CAN_F7R1_FB17          CAN_F7R1_FB17_Msk                               /*!<Filter bit 17 */
#define CAN_F7R1_FB18_Pos      (18U)
#define CAN_F7R1_FB18_Msk      (0x1UL << CAN_F7R1_FB18_Pos)                     /*!< 0x00040000 */
#define CAN_F7R1_FB18          CAN_F7R1_FB18_Msk                               /*!<Filter bit 18 */
#define CAN_F7R1_FB19_Pos      (19U)
#define CAN_F7R1_FB19_Msk      (0x1UL << CAN_F7R1_FB19_Pos)                     /*!< 0x00080000 */
#define CAN_F7R1_FB19          CAN_F7R1_FB19_Msk                               /*!<Filter bit 19 */
#define CAN_F7R1_FB20_Pos      (20U)
#define CAN_F7R1_FB20_Msk      (0x1UL << CAN_F7R1_FB20_Pos)                     /*!< 0x00100000 */
#define CAN_F7R1_FB20          CAN_F7R1_FB20_Msk                               /*!<Filter bit 20 */
#define CAN_F7R1_FB21_Pos      (21U)
#define CAN_F7R1_FB21_Msk      (0x1UL << CAN_F7R1_FB21_Pos)                     /*!< 0x00200000 */
#define CAN_F7R1_FB21          CAN_F7R1_FB21_Msk                               /*!<Filter bit 21 */
#define CAN_F7R1_FB22_Pos      (22U)
#define CAN_F7R1_FB22_Msk      (0x1UL << CAN_F7R1_FB22_Pos)                     /*!< 0x00400000 */
#define CAN_F7R1_FB22          CAN_F7R1_FB22_Msk                               /*!<Filter bit 22 */
#define CAN_F7R1_FB23_Pos      (23U)
#define CAN_F7R1_FB23_Msk      (0x1UL << CAN_F7R1_FB23_Pos)                     /*!< 0x00800000 */
#define CAN_F7R1_FB23          CAN_F7R1_FB23_Msk                               /*!<Filter bit 23 */
#define CAN_F7R1_FB24_Pos      (24U)
#define CAN_F7R1_FB24_Msk      (0x1UL << CAN_F7R1_FB24_Pos)                     /*!< 0x01000000 */
#define CAN_F7R1_FB24          CAN_F7R1_FB24_Msk                               /*!<Filter bit 24 */
#define CAN_F7R1_FB25_Pos      (25U)
#define CAN_F7R1_FB25_Msk      (0x1UL << CAN_F7R1_FB25_Pos)                     /*!< 0x02000000 */
#define CAN_F7R1_FB25          CAN_F7R1_FB25_Msk                               /*!<Filter bit 25 */
#define CAN_F7R1_FB26_Pos      (26U)
#define CAN_F7R1_FB26_Msk      (0x1UL << CAN_F7R1_FB26_Pos)                     /*!< 0x04000000 */
#define CAN_F7R1_FB26          CAN_F7R1_FB26_Msk                               /*!<Filter bit 26 */
#define CAN_F7R1_FB27_Pos      (27U)
#define CAN_F7R1_FB27_Msk      (0x1UL << CAN_F7R1_FB27_Pos)                     /*!< 0x08000000 */
#define CAN_F7R1_FB27          CAN_F7R1_FB27_Msk                               /*!<Filter bit 27 */
#define CAN_F7R1_FB28_Pos      (28U)
#define CAN_F7R1_FB28_Msk      (0x1UL << CAN_F7R1_FB28_Pos)                     /*!< 0x10000000 */
#define CAN_F7R1_FB28          CAN_F7R1_FB28_Msk                               /*!<Filter bit 28 */
#define CAN_F7R1_FB29_Pos      (29U)
#define CAN_F7R1_FB29_Msk      (0x1UL << CAN_F7R1_FB29_Pos)                     /*!< 0x20000000 */
#define CAN_F7R1_FB29          CAN_F7R1_FB29_Msk                               /*!<Filter bit 29 */
#define CAN_F7R1_FB30_Pos      (30U)
#define CAN_F7R1_FB30_Msk      (0x1UL << CAN_F7R1_FB30_Pos)                     /*!< 0x40000000 */
#define CAN_F7R1_FB30          CAN_F7R1_FB30_Msk                               /*!<Filter bit 30 */
#define CAN_F7R1_FB31_Pos      (31U)
#define CAN_F7R1_FB31_Msk      (0x1UL << CAN_F7R1_FB31_Pos)                     /*!< 0x80000000 */
#define CAN_F7R1_FB31          CAN_F7R1_FB31_Msk                               /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F8R1 register  *******************/
#define CAN_F8R1_FB0_Pos       (0U)
#define CAN_F8R1_FB0_Msk       (0x1UL << CAN_F8R1_FB0_Pos)                      /*!< 0x00000001 */
#define CAN_F8R1_FB0           CAN_F8R1_FB0_Msk                                /*!<Filter bit 0 */
#define CAN_F8R1_FB1_Pos       (1U)
#define CAN_F8R1_FB1_Msk       (0x1UL << CAN_F8R1_FB1_Pos)                      /*!< 0x00000002 */
#define CAN_F8R1_FB1           CAN_F8R1_FB1_Msk                                /*!<Filter bit 1 */
#define CAN_F8R1_FB2_Pos       (2U)
#define CAN_F8R1_FB2_Msk       (0x1UL << CAN_F8R1_FB2_Pos)                      /*!< 0x00000004 */
#define CAN_F8R1_FB2           CAN_F8R1_FB2_Msk                                /*!<Filter bit 2 */
#define CAN_F8R1_FB3_Pos       (3U)
#define CAN_F8R1_FB3_Msk       (0x1UL << CAN_F8R1_FB3_Pos)                      /*!< 0x00000008 */
#define CAN_F8R1_FB3           CAN_F8R1_FB3_Msk                                /*!<Filter bit 3 */
#define CAN_F8R1_FB4_Pos       (4U)
#define CAN_F8R1_FB4_Msk       (0x1UL << CAN_F8R1_FB4_Pos)                      /*!< 0x00000010 */
#define CAN_F8R1_FB4           CAN_F8R1_FB4_Msk                                /*!<Filter bit 4 */
#define CAN_F8R1_FB5_Pos       (5U)
#define CAN_F8R1_FB5_Msk       (0x1UL << CAN_F8R1_FB5_Pos)                      /*!< 0x00000020 */
#define CAN_F8R1_FB5           CAN_F8R1_FB5_Msk                                /*!<Filter bit 5 */
#define CAN_F8R1_FB6_Pos       (6U)
#define CAN_F8R1_FB6_Msk       (0x1UL << CAN_F8R1_FB6_Pos)                      /*!< 0x00000040 */
#define CAN_F8R1_FB6           CAN_F8R1_FB6_Msk                                /*!<Filter bit 6 */
#define CAN_F8R1_FB7_Pos       (7U)
#define CAN_F8R1_FB7_Msk       (0x1UL << CAN_F8R1_FB7_Pos)                      /*!< 0x00000080 */
#define CAN_F8R1_FB7           CAN_F8R1_FB7_Msk                                /*!<Filter bit 7 */
#define CAN_F8R1_FB8_Pos       (8U)
#define CAN_F8R1_FB8_Msk       (0x1UL << CAN_F8R1_FB8_Pos)                      /*!< 0x00000100 */
#define CAN_F8R1_FB8           CAN_F8R1_FB8_Msk                                /*!<Filter bit 8 */
#define CAN_F8R1_FB9_Pos       (9U)
#define CAN_F8R1_FB9_Msk       (0x1UL << CAN_F8R1_FB9_Pos)                      /*!< 0x00000200 */
#define CAN_F8R1_FB9           CAN_F8R1_FB9_Msk                                /*!<Filter bit 9 */
#define CAN_F8R1_FB10_Pos      (10U)
#define CAN_F8R1_FB10_Msk      (0x1UL << CAN_F8R1_FB10_Pos)                     /*!< 0x00000400 */
#define CAN_F8R1_FB10          CAN_F8R1_FB10_Msk                               /*!<Filter bit 10 */
#define CAN_F8R1_FB11_Pos      (11U)
#define CAN_F8R1_FB11_Msk      (0x1UL << CAN_F8R1_FB11_Pos)                     /*!< 0x00000800 */
#define CAN_F8R1_FB11          CAN_F8R1_FB11_Msk                               /*!<Filter bit 11 */
#define CAN_F8R1_FB12_Pos      (12U)
#define CAN_F8R1_FB12_Msk      (0x1UL << CAN_F8R1_FB12_Pos)                     /*!< 0x00001000 */
#define CAN_F8R1_FB12          CAN_F8R1_FB12_Msk                               /*!<Filter bit 12 */
#define CAN_F8R1_FB13_Pos      (13U)
#define CAN_F8R1_FB13_Msk      (0x1UL << CAN_F8R1_FB13_Pos)                     /*!< 0x00002000 */
#define CAN_F8R1_FB13          CAN_F8R1_FB13_Msk                               /*!<Filter bit 13 */
#define CAN_F8R1_FB14_Pos      (14U)
#define CAN_F8R1_FB14_Msk      (0x1UL << CAN_F8R1_FB14_Pos)                     /*!< 0x00004000 */
#define CAN_F8R1_FB14          CAN_F8R1_FB14_Msk                               /*!<Filter bit 14 */
#define CAN_F8R1_FB15_Pos      (15U)
#define CAN_F8R1_FB15_Msk      (0x1UL << CAN_F8R1_FB15_Pos)                     /*!< 0x00008000 */
#define CAN_F8R1_FB15          CAN_F8R1_FB15_Msk                               /*!<Filter bit 15 */
#define CAN_F8R1_FB16_Pos      (16U)
#define CAN_F8R1_FB16_Msk      (0x1UL << CAN_F8R1_FB16_Pos)                     /*!< 0x00010000 */
#define CAN_F8R1_FB16          CAN_F8R1_FB16_Msk                               /*!<Filter bit 16 */
#define CAN_F8R1_FB17_Pos      (17U)
#define CAN_F8R1_FB17_Msk      (0x1UL << CAN_F8R1_FB17_Pos)                     /*!< 0x00020000 */
#define CAN_F8R1_FB17          CAN_F8R1_FB17_Msk                               /*!<Filter bit 17 */
#define CAN_F8R1_FB18_Pos      (18U)
#define CAN_F8R1_FB18_Msk      (0x1UL << CAN_F8R1_FB18_Pos)                     /*!< 0x00040000 */
#define CAN_F8R1_FB18          CAN_F8R1_FB18_Msk                               /*!<Filter bit 18 */
#define CAN_F8R1_FB19_Pos      (19U)
#define CAN_F8R1_FB19_Msk      (0x1UL << CAN_F8R1_FB19_Pos)                     /*!< 0x00080000 */
#define CAN_F8R1_FB19          CAN_F8R1_FB19_Msk                               /*!<Filter bit 19 */
#define CAN_F8R1_FB20_Pos      (20U)
#define CAN_F8R1_FB20_Msk      (0x1UL << CAN_F8R1_FB20_Pos)                     /*!< 0x00100000 */
#define CAN_F8R1_FB20          CAN_F8R1_FB20_Msk                               /*!<Filter bit 20 */
#define CAN_F8R1_FB21_Pos      (21U)
#define CAN_F8R1_FB21_Msk      (0x1UL << CAN_F8R1_FB21_Pos)                     /*!< 0x00200000 */
#define CAN_F8R1_FB21          CAN_F8R1_FB21_Msk                               /*!<Filter bit 21 */
#define CAN_F8R1_FB22_Pos      (22U)
#define CAN_F8R1_FB22_Msk      (0x1UL << CAN_F8R1_FB22_Pos)                     /*!< 0x00400000 */
#define CAN_F8R1_FB22          CAN_F8R1_FB22_Msk                               /*!<Filter bit 22 */
#define CAN_F8R1_FB23_Pos      (23U)
#define CAN_F8R1_FB23_Msk      (0x1UL << CAN_F8R1_FB23_Pos)                     /*!< 0x00800000 */
#define CAN_F8R1_FB23          CAN_F8R1_FB23_Msk                               /*!<Filter bit 23 */
#define CAN_F8R1_FB24_Pos      (24U)
#define CAN_F8R1_FB24_Msk      (0x1UL << CAN_F8R1_FB24_Pos)                     /*!< 0x01000000 */
#define CAN_F8R1_FB24          CAN_F8R1_FB24_Msk                               /*!<Filter bit 24 */
#define CAN_F8R1_FB25_Pos      (25U)
#define CAN_F8R1_FB25_Msk      (0x1UL << CAN_F8R1_FB25_Pos)                     /*!< 0x02000000 */
#define CAN_F8R1_FB25          CAN_F8R1_FB25_Msk                               /*!<Filter bit 25 */
#define CAN_F8R1_FB26_Pos      (26U)
#define CAN_F8R1_FB26_Msk      (0x1UL << CAN_F8R1_FB26_Pos)                     /*!< 0x04000000 */
#define CAN_F8R1_FB26          CAN_F8R1_FB26_Msk                               /*!<Filter bit 26 */
#define CAN_F8R1_FB27_Pos      (27U)
#define CAN_F8R1_FB27_Msk      (0x1UL << CAN_F8R1_FB27_Pos)                     /*!< 0x08000000 */
#define CAN_F8R1_FB27          CAN_F8R1_FB27_Msk                               /*!<Filter bit 27 */
#define CAN_F8R1_FB28_Pos      (28U)
#define CAN_F8R1_FB28_Msk      (0x1UL << CAN_F8R1_FB28_Pos)                     /*!< 0x10000000 */
#define CAN_F8R1_FB28          CAN_F8R1_FB28_Msk                               /*!<Filter bit 28 */
#define CAN_F8R1_FB29_Pos      (29U)
#define CAN_F8R1_FB29_Msk      (0x1UL << CAN_F8R1_FB29_Pos)                     /*!< 0x20000000 */
#define CAN_F8R1_FB29          CAN_F8R1_FB29_Msk                               /*!<Filter bit 29 */
#define CAN_F8R1_FB30_Pos      (30U)
#define CAN_F8R1_FB30_Msk      (0x1UL << CAN_F8R1_FB30_Pos)                     /*!< 0x40000000 */
#define CAN_F8R1_FB30          CAN_F8R1_FB30_Msk                               /*!<Filter bit 30 */
#define CAN_F8R1_FB31_Pos      (31U)
#define CAN_F8R1_FB31_Msk      (0x1UL << CAN_F8R1_FB31_Pos)                     /*!< 0x80000000 */
#define CAN_F8R1_FB31          CAN_F8R1_FB31_Msk                               /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F9R1 register  *******************/
#define CAN_F9R1_FB0_Pos       (0U)
#define CAN_F9R1_FB0_Msk       (0x1UL << CAN_F9R1_FB0_Pos)                      /*!< 0x00000001 */
#define CAN_F9R1_FB0           CAN_F9R1_FB0_Msk                                /*!<Filter bit 0 */
#define CAN_F9R1_FB1_Pos       (1U)
#define CAN_F9R1_FB1_Msk       (0x1UL << CAN_F9R1_FB1_Pos)                      /*!< 0x00000002 */
#define CAN_F9R1_FB1           CAN_F9R1_FB1_Msk                                /*!<Filter bit 1 */
#define CAN_F9R1_FB2_Pos       (2U)
#define CAN_F9R1_FB2_Msk       (0x1UL << CAN_F9R1_FB2_Pos)                      /*!< 0x00000004 */
#define CAN_F9R1_FB2           CAN_F9R1_FB2_Msk                                /*!<Filter bit 2 */
#define CAN_F9R1_FB3_Pos       (3U)
#define CAN_F9R1_FB3_Msk       (0x1UL << CAN_F9R1_FB3_Pos)                      /*!< 0x00000008 */
#define CAN_F9R1_FB3           CAN_F9R1_FB3_Msk                                /*!<Filter bit 3 */
#define CAN_F9R1_FB4_Pos       (4U)
#define CAN_F9R1_FB4_Msk       (0x1UL << CAN_F9R1_FB4_Pos)                      /*!< 0x00000010 */
#define CAN_F9R1_FB4           CAN_F9R1_FB4_Msk                                /*!<Filter bit 4 */
#define CAN_F9R1_FB5_Pos       (5U)
#define CAN_F9R1_FB5_Msk       (0x1UL << CAN_F9R1_FB5_Pos)                      /*!< 0x00000020 */
#define CAN_F9R1_FB5           CAN_F9R1_FB5_Msk                                /*!<Filter bit 5 */
#define CAN_F9R1_FB6_Pos       (6U)
#define CAN_F9R1_FB6_Msk       (0x1UL << CAN_F9R1_FB6_Pos)                      /*!< 0x00000040 */
#define CAN_F9R1_FB6           CAN_F9R1_FB6_Msk                                /*!<Filter bit 6 */
#define CAN_F9R1_FB7_Pos       (7U)
#define CAN_F9R1_FB7_Msk       (0x1UL << CAN_F9R1_FB7_Pos)                      /*!< 0x00000080 */
#define CAN_F9R1_FB7           CAN_F9R1_FB7_Msk                                /*!<Filter bit 7 */
#define CAN_F9R1_FB8_Pos       (8U)
#define CAN_F9R1_FB8_Msk       (0x1UL << CAN_F9R1_FB8_Pos)                      /*!< 0x00000100 */
#define CAN_F9R1_FB8           CAN_F9R1_FB8_Msk                                /*!<Filter bit 8 */
#define CAN_F9R1_FB9_Pos       (9U)
#define CAN_F9R1_FB9_Msk       (0x1UL << CAN_F9R1_FB9_Pos)                      /*!< 0x00000200 */
#define CAN_F9R1_FB9           CAN_F9R1_FB9_Msk                                /*!<Filter bit 9 */
#define CAN_F9R1_FB10_Pos      (10U)
#define CAN_F9R1_FB10_Msk      (0x1UL << CAN_F9R1_FB10_Pos)                     /*!< 0x00000400 */
#define CAN_F9R1_FB10          CAN_F9R1_FB10_Msk                               /*!<Filter bit 10 */
#define CAN_F9R1_FB11_Pos      (11U)
#define CAN_F9R1_FB11_Msk      (0x1UL << CAN_F9R1_FB11_Pos)                     /*!< 0x00000800 */
#define CAN_F9R1_FB11          CAN_F9R1_FB11_Msk                               /*!<Filter bit 11 */
#define CAN_F9R1_FB12_Pos      (12U)
#define CAN_F9R1_FB12_Msk      (0x1UL << CAN_F9R1_FB12_Pos)                     /*!< 0x00001000 */
#define CAN_F9R1_FB12          CAN_F9R1_FB12_Msk                               /*!<Filter bit 12 */
#define CAN_F9R1_FB13_Pos      (13U)
#define CAN_F9R1_FB13_Msk      (0x1UL << CAN_F9R1_FB13_Pos)                     /*!< 0x00002000 */
#define CAN_F9R1_FB13          CAN_F9R1_FB13_Msk                               /*!<Filter bit 13 */
#define CAN_F9R1_FB14_Pos      (14U)
#define CAN_F9R1_FB14_Msk      (0x1UL << CAN_F9R1_FB14_Pos)                     /*!< 0x00004000 */
#define CAN_F9R1_FB14          CAN_F9R1_FB14_Msk                               /*!<Filter bit 14 */
#define CAN_F9R1_FB15_Pos      (15U)
#define CAN_F9R1_FB15_Msk      (0x1UL << CAN_F9R1_FB15_Pos)                     /*!< 0x00008000 */
#define CAN_F9R1_FB15          CAN_F9R1_FB15_Msk                               /*!<Filter bit 15 */
#define CAN_F9R1_FB16_Pos      (16U)
#define CAN_F9R1_FB16_Msk      (0x1UL << CAN_F9R1_FB16_Pos)                     /*!< 0x00010000 */
#define CAN_F9R1_FB16          CAN_F9R1_FB16_Msk                               /*!<Filter bit 16 */
#define CAN_F9R1_FB17_Pos      (17U)
#define CAN_F9R1_FB17_Msk      (0x1UL << CAN_F9R1_FB17_Pos)                     /*!< 0x00020000 */
#define CAN_F9R1_FB17          CAN_F9R1_FB17_Msk                               /*!<Filter bit 17 */
#define CAN_F9R1_FB18_Pos      (18U)
#define CAN_F9R1_FB18_Msk      (0x1UL << CAN_F9R1_FB18_Pos)                     /*!< 0x00040000 */
#define CAN_F9R1_FB18          CAN_F9R1_FB18_Msk                               /*!<Filter bit 18 */
#define CAN_F9R1_FB19_Pos      (19U)
#define CAN_F9R1_FB19_Msk      (0x1UL << CAN_F9R1_FB19_Pos)                     /*!< 0x00080000 */
#define CAN_F9R1_FB19          CAN_F9R1_FB19_Msk                               /*!<Filter bit 19 */
#define CAN_F9R1_FB20_Pos      (20U)
#define CAN_F9R1_FB20_Msk      (0x1UL << CAN_F9R1_FB20_Pos)                     /*!< 0x00100000 */
#define CAN_F9R1_FB20          CAN_F9R1_FB20_Msk                               /*!<Filter bit 20 */
#define CAN_F9R1_FB21_Pos      (21U)
#define CAN_F9R1_FB21_Msk      (0x1UL << CAN_F9R1_FB21_Pos)                     /*!< 0x00200000 */
#define CAN_F9R1_FB21          CAN_F9R1_FB21_Msk                               /*!<Filter bit 21 */
#define CAN_F9R1_FB22_Pos      (22U)
#define CAN_F9R1_FB22_Msk      (0x1UL << CAN_F9R1_FB22_Pos)                     /*!< 0x00400000 */
#define CAN_F9R1_FB22          CAN_F9R1_FB22_Msk                               /*!<Filter bit 22 */
#define CAN_F9R1_FB23_Pos      (23U)
#define CAN_F9R1_FB23_Msk      (0x1UL << CAN_F9R1_FB23_Pos)                     /*!< 0x00800000 */
#define CAN_F9R1_FB23          CAN_F9R1_FB23_Msk                               /*!<Filter bit 23 */
#define CAN_F9R1_FB24_Pos      (24U)
#define CAN_F9R1_FB24_Msk      (0x1UL << CAN_F9R1_FB24_Pos)                     /*!< 0x01000000 */
#define CAN_F9R1_FB24          CAN_F9R1_FB24_Msk                               /*!<Filter bit 24 */
#define CAN_F9R1_FB25_Pos      (25U)
#define CAN_F9R1_FB25_Msk      (0x1UL << CAN_F9R1_FB25_Pos)                     /*!< 0x02000000 */
#define CAN_F9R1_FB25          CAN_F9R1_FB25_Msk                               /*!<Filter bit 25 */
#define CAN_F9R1_FB26_Pos      (26U)
#define CAN_F9R1_FB26_Msk      (0x1UL << CAN_F9R1_FB26_Pos)                     /*!< 0x04000000 */
#define CAN_F9R1_FB26          CAN_F9R1_FB26_Msk                               /*!<Filter bit 26 */
#define CAN_F9R1_FB27_Pos      (27U)
#define CAN_F9R1_FB27_Msk      (0x1UL << CAN_F9R1_FB27_Pos)                     /*!< 0x08000000 */
#define CAN_F9R1_FB27          CAN_F9R1_FB27_Msk                               /*!<Filter bit 27 */
#define CAN_F9R1_FB28_Pos      (28U)
#define CAN_F9R1_FB28_Msk      (0x1UL << CAN_F9R1_FB28_Pos)                     /*!< 0x10000000 */
#define CAN_F9R1_FB28          CAN_F9R1_FB28_Msk                               /*!<Filter bit 28 */
#define CAN_F9R1_FB29_Pos      (29U)
#define CAN_F9R1_FB29_Msk      (0x1UL << CAN_F9R1_FB29_Pos)                     /*!< 0x20000000 */
#define CAN_F9R1_FB29          CAN_F9R1_FB29_Msk                               /*!<Filter bit 29 */
#define CAN_F9R1_FB30_Pos      (30U)
#define CAN_F9R1_FB30_Msk      (0x1UL << CAN_F9R1_FB30_Pos)                     /*!< 0x40000000 */
#define CAN_F9R1_FB30          CAN_F9R1_FB30_Msk                               /*!<Filter bit 30 */
#define CAN_F9R1_FB31_Pos      (31U)
#define CAN_F9R1_FB31_Msk      (0x1UL << CAN_F9R1_FB31_Pos)                     /*!< 0x80000000 */
#define CAN_F9R1_FB31          CAN_F9R1_FB31_Msk                               /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F10R1 register  ******************/
#define CAN_F10R1_FB0_Pos      (0U)
#define CAN_F10R1_FB0_Msk      (0x1UL << CAN_F10R1_FB0_Pos)                     /*!< 0x00000001 */
#define CAN_F10R1_FB0          CAN_F10R1_FB0_Msk                               /*!<Filter bit 0 */
#define CAN_F10R1_FB1_Pos      (1U)
#define CAN_F10R1_FB1_Msk      (0x1UL << CAN_F10R1_FB1_Pos)                     /*!< 0x00000002 */
#define CAN_F10R1_FB1          CAN_F10R1_FB1_Msk                               /*!<Filter bit 1 */
#define CAN_F10R1_FB2_Pos      (2U)
#define CAN_F10R1_FB2_Msk      (0x1UL << CAN_F10R1_FB2_Pos)                     /*!< 0x00000004 */
#define CAN_F10R1_FB2          CAN_F10R1_FB2_Msk                               /*!<Filter bit 2 */
#define CAN_F10R1_FB3_Pos      (3U)
#define CAN_F10R1_FB3_Msk      (0x1UL << CAN_F10R1_FB3_Pos)                     /*!< 0x00000008 */
#define CAN_F10R1_FB3          CAN_F10R1_FB3_Msk                               /*!<Filter bit 3 */
#define CAN_F10R1_FB4_Pos      (4U)
#define CAN_F10R1_FB4_Msk      (0x1UL << CAN_F10R1_FB4_Pos)                     /*!< 0x00000010 */
#define CAN_F10R1_FB4          CAN_F10R1_FB4_Msk                               /*!<Filter bit 4 */
#define CAN_F10R1_FB5_Pos      (5U)
#define CAN_F10R1_FB5_Msk      (0x1UL << CAN_F10R1_FB5_Pos)                     /*!< 0x00000020 */
#define CAN_F10R1_FB5          CAN_F10R1_FB5_Msk                               /*!<Filter bit 5 */
#define CAN_F10R1_FB6_Pos      (6U)
#define CAN_F10R1_FB6_Msk      (0x1UL << CAN_F10R1_FB6_Pos)                     /*!< 0x00000040 */
#define CAN_F10R1_FB6          CAN_F10R1_FB6_Msk                               /*!<Filter bit 6 */
#define CAN_F10R1_FB7_Pos      (7U)
#define CAN_F10R1_FB7_Msk      (0x1UL << CAN_F10R1_FB7_Pos)                     /*!< 0x00000080 */
#define CAN_F10R1_FB7          CAN_F10R1_FB7_Msk                               /*!<Filter bit 7 */
#define CAN_F10R1_FB8_Pos      (8U)
#define CAN_F10R1_FB8_Msk      (0x1UL << CAN_F10R1_FB8_Pos)                     /*!< 0x00000100 */
#define CAN_F10R1_FB8          CAN_F10R1_FB8_Msk                               /*!<Filter bit 8 */
#define CAN_F10R1_FB9_Pos      (9U)
#define CAN_F10R1_FB9_Msk      (0x1UL << CAN_F10R1_FB9_Pos)                     /*!< 0x00000200 */
#define CAN_F10R1_FB9          CAN_F10R1_FB9_Msk                               /*!<Filter bit 9 */
#define CAN_F10R1_FB10_Pos     (10U)
#define CAN_F10R1_FB10_Msk     (0x1UL << CAN_F10R1_FB10_Pos)                    /*!< 0x00000400 */
#define CAN_F10R1_FB10         CAN_F10R1_FB10_Msk                              /*!<Filter bit 10 */
#define CAN_F10R1_FB11_Pos     (11U)
#define CAN_F10R1_FB11_Msk     (0x1UL << CAN_F10R1_FB11_Pos)                    /*!< 0x00000800 */
#define CAN_F10R1_FB11         CAN_F10R1_FB11_Msk                              /*!<Filter bit 11 */
#define CAN_F10R1_FB12_Pos     (12U)
#define CAN_F10R1_FB12_Msk     (0x1UL << CAN_F10R1_FB12_Pos)                    /*!< 0x00001000 */
#define CAN_F10R1_FB12         CAN_F10R1_FB12_Msk                              /*!<Filter bit 12 */
#define CAN_F10R1_FB13_Pos     (13U)
#define CAN_F10R1_FB13_Msk     (0x1UL << CAN_F10R1_FB13_Pos)                    /*!< 0x00002000 */
#define CAN_F10R1_FB13         CAN_F10R1_FB13_Msk                              /*!<Filter bit 13 */
#define CAN_F10R1_FB14_Pos     (14U)
#define CAN_F10R1_FB14_Msk     (0x1UL << CAN_F10R1_FB14_Pos)                    /*!< 0x00004000 */
#define CAN_F10R1_FB14         CAN_F10R1_FB14_Msk                              /*!<Filter bit 14 */
#define CAN_F10R1_FB15_Pos     (15U)
#define CAN_F10R1_FB15_Msk     (0x1UL << CAN_F10R1_FB15_Pos)                    /*!< 0x00008000 */
#define CAN_F10R1_FB15         CAN_F10R1_FB15_Msk                              /*!<Filter bit 15 */
#define CAN_F10R1_FB16_Pos     (16U)
#define CAN_F10R1_FB16_Msk     (0x1UL << CAN_F10R1_FB16_Pos)                    /*!< 0x00010000 */
#define CAN_F10R1_FB16         CAN_F10R1_FB16_Msk                              /*!<Filter bit 16 */
#define CAN_F10R1_FB17_Pos     (17U)
#define CAN_F10R1_FB17_Msk     (0x1UL << CAN_F10R1_FB17_Pos)                    /*!< 0x00020000 */
#define CAN_F10R1_FB17         CAN_F10R1_FB17_Msk                              /*!<Filter bit 17 */
#define CAN_F10R1_FB18_Pos     (18U)
#define CAN_F10R1_FB18_Msk     (0x1UL << CAN_F10R1_FB18_Pos)                    /*!< 0x00040000 */
#define CAN_F10R1_FB18         CAN_F10R1_FB18_Msk                              /*!<Filter bit 18 */
#define CAN_F10R1_FB19_Pos     (19U)
#define CAN_F10R1_FB19_Msk     (0x1UL << CAN_F10R1_FB19_Pos)                    /*!< 0x00080000 */
#define CAN_F10R1_FB19         CAN_F10R1_FB19_Msk                              /*!<Filter bit 19 */
#define CAN_F10R1_FB20_Pos     (20U)
#define CAN_F10R1_FB20_Msk     (0x1UL << CAN_F10R1_FB20_Pos)                    /*!< 0x00100000 */
#define CAN_F10R1_FB20         CAN_F10R1_FB20_Msk                              /*!<Filter bit 20 */
#define CAN_F10R1_FB21_Pos     (21U)
#define CAN_F10R1_FB21_Msk     (0x1UL << CAN_F10R1_FB21_Pos)                    /*!< 0x00200000 */
#define CAN_F10R1_FB21         CAN_F10R1_FB21_Msk                              /*!<Filter bit 21 */
#define CAN_F10R1_FB22_Pos     (22U)
#define CAN_F10R1_FB22_Msk     (0x1UL << CAN_F10R1_FB22_Pos)                    /*!< 0x00400000 */
#define CAN_F10R1_FB22         CAN_F10R1_FB22_Msk                              /*!<Filter bit 22 */
#define CAN_F10R1_FB23_Pos     (23U)
#define CAN_F10R1_FB23_Msk     (0x1UL << CAN_F10R1_FB23_Pos)                    /*!< 0x00800000 */
#define CAN_F10R1_FB23         CAN_F10R1_FB23_Msk                              /*!<Filter bit 23 */
#define CAN_F10R1_FB24_Pos     (24U)
#define CAN_F10R1_FB24_Msk     (0x1UL << CAN_F10R1_FB24_Pos)                    /*!< 0x01000000 */
#define CAN_F10R1_FB24         CAN_F10R1_FB24_Msk                              /*!<Filter bit 24 */
#define CAN_F10R1_FB25_Pos     (25U)
#define CAN_F10R1_FB25_Msk     (0x1UL << CAN_F10R1_FB25_Pos)                    /*!< 0x02000000 */
#define CAN_F10R1_FB25         CAN_F10R1_FB25_Msk                              /*!<Filter bit 25 */
#define CAN_F10R1_FB26_Pos     (26U)
#define CAN_F10R1_FB26_Msk     (0x1UL << CAN_F10R1_FB26_Pos)                    /*!< 0x04000000 */
#define CAN_F10R1_FB26         CAN_F10R1_FB26_Msk                              /*!<Filter bit 26 */
#define CAN_F10R1_FB27_Pos     (27U)
#define CAN_F10R1_FB27_Msk     (0x1UL << CAN_F10R1_FB27_Pos)                    /*!< 0x08000000 */
#define CAN_F10R1_FB27         CAN_F10R1_FB27_Msk                              /*!<Filter bit 27 */
#define CAN_F10R1_FB28_Pos     (28U)
#define CAN_F10R1_FB28_Msk     (0x1UL << CAN_F10R1_FB28_Pos)                    /*!< 0x10000000 */
#define CAN_F10R1_FB28         CAN_F10R1_FB28_Msk                              /*!<Filter bit 28 */
#define CAN_F10R1_FB29_Pos     (29U)
#define CAN_F10R1_FB29_Msk     (0x1UL << CAN_F10R1_FB29_Pos)                    /*!< 0x20000000 */
#define CAN_F10R1_FB29         CAN_F10R1_FB29_Msk                              /*!<Filter bit 29 */
#define CAN_F10R1_FB30_Pos     (30U)
#define CAN_F10R1_FB30_Msk     (0x1UL << CAN_F10R1_FB30_Pos)                    /*!< 0x40000000 */
#define CAN_F10R1_FB30         CAN_F10R1_FB30_Msk                              /*!<Filter bit 30 */
#define CAN_F10R1_FB31_Pos     (31U)
#define CAN_F10R1_FB31_Msk     (0x1UL << CAN_F10R1_FB31_Pos)                    /*!< 0x80000000 */
#define CAN_F10R1_FB31         CAN_F10R1_FB31_Msk                              /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F11R1 register  ******************/
#define CAN_F11R1_FB0_Pos      (0U)
#define CAN_F11R1_FB0_Msk      (0x1UL << CAN_F11R1_FB0_Pos)                     /*!< 0x00000001 */
#define CAN_F11R1_FB0          CAN_F11R1_FB0_Msk                               /*!<Filter bit 0 */
#define CAN_F11R1_FB1_Pos      (1U)
#define CAN_F11R1_FB1_Msk      (0x1UL << CAN_F11R1_FB1_Pos)                     /*!< 0x00000002 */
#define CAN_F11R1_FB1          CAN_F11R1_FB1_Msk                               /*!<Filter bit 1 */
#define CAN_F11R1_FB2_Pos      (2U)
#define CAN_F11R1_FB2_Msk      (0x1UL << CAN_F11R1_FB2_Pos)                     /*!< 0x00000004 */
#define CAN_F11R1_FB2          CAN_F11R1_FB2_Msk                               /*!<Filter bit 2 */
#define CAN_F11R1_FB3_Pos      (3U)
#define CAN_F11R1_FB3_Msk      (0x1UL << CAN_F11R1_FB3_Pos)                     /*!< 0x00000008 */
#define CAN_F11R1_FB3          CAN_F11R1_FB3_Msk                               /*!<Filter bit 3 */
#define CAN_F11R1_FB4_Pos      (4U)
#define CAN_F11R1_FB4_Msk      (0x1UL << CAN_F11R1_FB4_Pos)                     /*!< 0x00000010 */
#define CAN_F11R1_FB4          CAN_F11R1_FB4_Msk                               /*!<Filter bit 4 */
#define CAN_F11R1_FB5_Pos      (5U)
#define CAN_F11R1_FB5_Msk      (0x1UL << CAN_F11R1_FB5_Pos)                     /*!< 0x00000020 */
#define CAN_F11R1_FB5          CAN_F11R1_FB5_Msk                               /*!<Filter bit 5 */
#define CAN_F11R1_FB6_Pos      (6U)
#define CAN_F11R1_FB6_Msk      (0x1UL << CAN_F11R1_FB6_Pos)                     /*!< 0x00000040 */
#define CAN_F11R1_FB6          CAN_F11R1_FB6_Msk                               /*!<Filter bit 6 */
#define CAN_F11R1_FB7_Pos      (7U)
#define CAN_F11R1_FB7_Msk      (0x1UL << CAN_F11R1_FB7_Pos)                     /*!< 0x00000080 */
#define CAN_F11R1_FB7          CAN_F11R1_FB7_Msk                               /*!<Filter bit 7 */
#define CAN_F11R1_FB8_Pos      (8U)
#define CAN_F11R1_FB8_Msk      (0x1UL << CAN_F11R1_FB8_Pos)                     /*!< 0x00000100 */
#define CAN_F11R1_FB8          CAN_F11R1_FB8_Msk                               /*!<Filter bit 8 */
#define CAN_F11R1_FB9_Pos      (9U)
#define CAN_F11R1_FB9_Msk      (0x1UL << CAN_F11R1_FB9_Pos)                     /*!< 0x00000200 */
#define CAN_F11R1_FB9          CAN_F11R1_FB9_Msk                               /*!<Filter bit 9 */
#define CAN_F11R1_FB10_Pos     (10U)
#define CAN_F11R1_FB10_Msk     (0x1UL << CAN_F11R1_FB10_Pos)                    /*!< 0x00000400 */
#define CAN_F11R1_FB10         CAN_F11R1_FB10_Msk                              /*!<Filter bit 10 */
#define CAN_F11R1_FB11_Pos     (11U)
#define CAN_F11R1_FB11_Msk     (0x1UL << CAN_F11R1_FB11_Pos)                    /*!< 0x00000800 */
#define CAN_F11R1_FB11         CAN_F11R1_FB11_Msk                              /*!<Filter bit 11 */
#define CAN_F11R1_FB12_Pos     (12U)
#define CAN_F11R1_FB12_Msk     (0x1UL << CAN_F11R1_FB12_Pos)                    /*!< 0x00001000 */
#define CAN_F11R1_FB12         CAN_F11R1_FB12_Msk                              /*!<Filter bit 12 */
#define CAN_F11R1_FB13_Pos     (13U)
#define CAN_F11R1_FB13_Msk     (0x1UL << CAN_F11R1_FB13_Pos)                    /*!< 0x00002000 */
#define CAN_F11R1_FB13         CAN_F11R1_FB13_Msk                              /*!<Filter bit 13 */
#define CAN_F11R1_FB14_Pos     (14U)
#define CAN_F11R1_FB14_Msk     (0x1UL << CAN_F11R1_FB14_Pos)                    /*!< 0x00004000 */
#define CAN_F11R1_FB14         CAN_F11R1_FB14_Msk                              /*!<Filter bit 14 */
#define CAN_F11R1_FB15_Pos     (15U)
#define CAN_F11R1_FB15_Msk     (0x1UL << CAN_F11R1_FB15_Pos)                    /*!< 0x00008000 */
#define CAN_F11R1_FB15         CAN_F11R1_FB15_Msk                              /*!<Filter bit 15 */
#define CAN_F11R1_FB16_Pos     (16U)
#define CAN_F11R1_FB16_Msk     (0x1UL << CAN_F11R1_FB16_Pos)                    /*!< 0x00010000 */
#define CAN_F11R1_FB16         CAN_F11R1_FB16_Msk                              /*!<Filter bit 16 */
#define CAN_F11R1_FB17_Pos     (17U)
#define CAN_F11R1_FB17_Msk     (0x1UL << CAN_F11R1_FB17_Pos)                    /*!< 0x00020000 */
#define CAN_F11R1_FB17         CAN_F11R1_FB17_Msk                              /*!<Filter bit 17 */
#define CAN_F11R1_FB18_Pos     (18U)
#define CAN_F11R1_FB18_Msk     (0x1UL << CAN_F11R1_FB18_Pos)                    /*!< 0x00040000 */
#define CAN_F11R1_FB18         CAN_F11R1_FB18_Msk                              /*!<Filter bit 18 */
#define CAN_F11R1_FB19_Pos     (19U)
#define CAN_F11R1_FB19_Msk     (0x1UL << CAN_F11R1_FB19_Pos)                    /*!< 0x00080000 */
#define CAN_F11R1_FB19         CAN_F11R1_FB19_Msk                              /*!<Filter bit 19 */
#define CAN_F11R1_FB20_Pos     (20U)
#define CAN_F11R1_FB20_Msk     (0x1UL << CAN_F11R1_FB20_Pos)                    /*!< 0x00100000 */
#define CAN_F11R1_FB20         CAN_F11R1_FB20_Msk                              /*!<Filter bit 20 */
#define CAN_F11R1_FB21_Pos     (21U)
#define CAN_F11R1_FB21_Msk     (0x1UL << CAN_F11R1_FB21_Pos)                    /*!< 0x00200000 */
#define CAN_F11R1_FB21         CAN_F11R1_FB21_Msk                              /*!<Filter bit 21 */
#define CAN_F11R1_FB22_Pos     (22U)
#define CAN_F11R1_FB22_Msk     (0x1UL << CAN_F11R1_FB22_Pos)                    /*!< 0x00400000 */
#define CAN_F11R1_FB22         CAN_F11R1_FB22_Msk                              /*!<Filter bit 22 */
#define CAN_F11R1_FB23_Pos     (23U)
#define CAN_F11R1_FB23_Msk     (0x1UL << CAN_F11R1_FB23_Pos)                    /*!< 0x00800000 */
#define CAN_F11R1_FB23         CAN_F11R1_FB23_Msk                              /*!<Filter bit 23 */
#define CAN_F11R1_FB24_Pos     (24U)
#define CAN_F11R1_FB24_Msk     (0x1UL << CAN_F11R1_FB24_Pos)                    /*!< 0x01000000 */
#define CAN_F11R1_FB24         CAN_F11R1_FB24_Msk                              /*!<Filter bit 24 */
#define CAN_F11R1_FB25_Pos     (25U)
#define CAN_F11R1_FB25_Msk     (0x1UL << CAN_F11R1_FB25_Pos)                    /*!< 0x02000000 */
#define CAN_F11R1_FB25         CAN_F11R1_FB25_Msk                              /*!<Filter bit 25 */
#define CAN_F11R1_FB26_Pos     (26U)
#define CAN_F11R1_FB26_Msk     (0x1UL << CAN_F11R1_FB26_Pos)                    /*!< 0x04000000 */
#define CAN_F11R1_FB26         CAN_F11R1_FB26_Msk                              /*!<Filter bit 26 */
#define CAN_F11R1_FB27_Pos     (27U)
#define CAN_F11R1_FB27_Msk     (0x1UL << CAN_F11R1_FB27_Pos)                    /*!< 0x08000000 */
#define CAN_F11R1_FB27         CAN_F11R1_FB27_Msk                              /*!<Filter bit 27 */
#define CAN_F11R1_FB28_Pos     (28U)
#define CAN_F11R1_FB28_Msk     (0x1UL << CAN_F11R1_FB28_Pos)                    /*!< 0x10000000 */
#define CAN_F11R1_FB28         CAN_F11R1_FB28_Msk                              /*!<Filter bit 28 */
#define CAN_F11R1_FB29_Pos     (29U)
#define CAN_F11R1_FB29_Msk     (0x1UL << CAN_F11R1_FB29_Pos)                    /*!< 0x20000000 */
#define CAN_F11R1_FB29         CAN_F11R1_FB29_Msk                              /*!<Filter bit 29 */
#define CAN_F11R1_FB30_Pos     (30U)
#define CAN_F11R1_FB30_Msk     (0x1UL << CAN_F11R1_FB30_Pos)                    /*!< 0x40000000 */
#define CAN_F11R1_FB30         CAN_F11R1_FB30_Msk                              /*!<Filter bit 30 */
#define CAN_F11R1_FB31_Pos     (31U)
#define CAN_F11R1_FB31_Msk     (0x1UL << CAN_F11R1_FB31_Pos)                    /*!< 0x80000000 */
#define CAN_F11R1_FB31         CAN_F11R1_FB31_Msk                              /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F12R1 register  ******************/
#define CAN_F12R1_FB0_Pos      (0U)
#define CAN_F12R1_FB0_Msk      (0x1UL << CAN_F12R1_FB0_Pos)                     /*!< 0x00000001 */
#define CAN_F12R1_FB0          CAN_F12R1_FB0_Msk                               /*!<Filter bit 0 */
#define CAN_F12R1_FB1_Pos      (1U)
#define CAN_F12R1_FB1_Msk      (0x1UL << CAN_F12R1_FB1_Pos)                     /*!< 0x00000002 */
#define CAN_F12R1_FB1          CAN_F12R1_FB1_Msk                               /*!<Filter bit 1 */
#define CAN_F12R1_FB2_Pos      (2U)
#define CAN_F12R1_FB2_Msk      (0x1UL << CAN_F12R1_FB2_Pos)                     /*!< 0x00000004 */
#define CAN_F12R1_FB2          CAN_F12R1_FB2_Msk                               /*!<Filter bit 2 */
#define CAN_F12R1_FB3_Pos      (3U)
#define CAN_F12R1_FB3_Msk      (0x1UL << CAN_F12R1_FB3_Pos)                     /*!< 0x00000008 */
#define CAN_F12R1_FB3          CAN_F12R1_FB3_Msk                               /*!<Filter bit 3 */
#define CAN_F12R1_FB4_Pos      (4U)
#define CAN_F12R1_FB4_Msk      (0x1UL << CAN_F12R1_FB4_Pos)                     /*!< 0x00000010 */
#define CAN_F12R1_FB4          CAN_F12R1_FB4_Msk                               /*!<Filter bit 4 */
#define CAN_F12R1_FB5_Pos      (5U)
#define CAN_F12R1_FB5_Msk      (0x1UL << CAN_F12R1_FB5_Pos)                     /*!< 0x00000020 */
#define CAN_F12R1_FB5          CAN_F12R1_FB5_Msk                               /*!<Filter bit 5 */
#define CAN_F12R1_FB6_Pos      (6U)
#define CAN_F12R1_FB6_Msk      (0x1UL << CAN_F12R1_FB6_Pos)                     /*!< 0x00000040 */
#define CAN_F12R1_FB6          CAN_F12R1_FB6_Msk                               /*!<Filter bit 6 */
#define CAN_F12R1_FB7_Pos      (7U)
#define CAN_F12R1_FB7_Msk      (0x1UL << CAN_F12R1_FB7_Pos)                     /*!< 0x00000080 */
#define CAN_F12R1_FB7          CAN_F12R1_FB7_Msk                               /*!<Filter bit 7 */
#define CAN_F12R1_FB8_Pos      (8U)
#define CAN_F12R1_FB8_Msk      (0x1UL << CAN_F12R1_FB8_Pos)                     /*!< 0x00000100 */
#define CAN_F12R1_FB8          CAN_F12R1_FB8_Msk                               /*!<Filter bit 8 */
#define CAN_F12R1_FB9_Pos      (9U)
#define CAN_F12R1_FB9_Msk      (0x1UL << CAN_F12R1_FB9_Pos)                     /*!< 0x00000200 */
#define CAN_F12R1_FB9          CAN_F12R1_FB9_Msk                               /*!<Filter bit 9 */
#define CAN_F12R1_FB10_Pos     (10U)
#define CAN_F12R1_FB10_Msk     (0x1UL << CAN_F12R1_FB10_Pos)                    /*!< 0x00000400 */
#define CAN_F12R1_FB10         CAN_F12R1_FB10_Msk                              /*!<Filter bit 10 */
#define CAN_F12R1_FB11_Pos     (11U)
#define CAN_F12R1_FB11_Msk     (0x1UL << CAN_F12R1_FB11_Pos)                    /*!< 0x00000800 */
#define CAN_F12R1_FB11         CAN_F12R1_FB11_Msk                              /*!<Filter bit 11 */
#define CAN_F12R1_FB12_Pos     (12U)
#define CAN_F12R1_FB12_Msk     (0x1UL << CAN_F12R1_FB12_Pos)                    /*!< 0x00001000 */
#define CAN_F12R1_FB12         CAN_F12R1_FB12_Msk                              /*!<Filter bit 12 */
#define CAN_F12R1_FB13_Pos     (13U)
#define CAN_F12R1_FB13_Msk     (0x1UL << CAN_F12R1_FB13_Pos)                    /*!< 0x00002000 */
#define CAN_F12R1_FB13         CAN_F12R1_FB13_Msk                              /*!<Filter bit 13 */
#define CAN_F12R1_FB14_Pos     (14U)
#define CAN_F12R1_FB14_Msk     (0x1UL << CAN_F12R1_FB14_Pos)                    /*!< 0x00004000 */
#define CAN_F12R1_FB14         CAN_F12R1_FB14_Msk                              /*!<Filter bit 14 */
#define CAN_F12R1_FB15_Pos     (15U)
#define CAN_F12R1_FB15_Msk     (0x1UL << CAN_F12R1_FB15_Pos)                    /*!< 0x00008000 */
#define CAN_F12R1_FB15         CAN_F12R1_FB15_Msk                              /*!<Filter bit 15 */
#define CAN_F12R1_FB16_Pos     (16U)
#define CAN_F12R1_FB16_Msk     (0x1UL << CAN_F12R1_FB16_Pos)                    /*!< 0x00010000 */
#define CAN_F12R1_FB16         CAN_F12R1_FB16_Msk                              /*!<Filter bit 16 */
#define CAN_F12R1_FB17_Pos     (17U)
#define CAN_F12R1_FB17_Msk     (0x1UL << CAN_F12R1_FB17_Pos)                    /*!< 0x00020000 */
#define CAN_F12R1_FB17         CAN_F12R1_FB17_Msk                              /*!<Filter bit 17 */
#define CAN_F12R1_FB18_Pos     (18U)
#define CAN_F12R1_FB18_Msk     (0x1UL << CAN_F12R1_FB18_Pos)                    /*!< 0x00040000 */
#define CAN_F12R1_FB18         CAN_F12R1_FB18_Msk                              /*!<Filter bit 18 */
#define CAN_F12R1_FB19_Pos     (19U)
#define CAN_F12R1_FB19_Msk     (0x1UL << CAN_F12R1_FB19_Pos)                    /*!< 0x00080000 */
#define CAN_F12R1_FB19         CAN_F12R1_FB19_Msk                              /*!<Filter bit 19 */
#define CAN_F12R1_FB20_Pos     (20U)
#define CAN_F12R1_FB20_Msk     (0x1UL << CAN_F12R1_FB20_Pos)                    /*!< 0x00100000 */
#define CAN_F12R1_FB20         CAN_F12R1_FB20_Msk                              /*!<Filter bit 20 */
#define CAN_F12R1_FB21_Pos     (21U)
#define CAN_F12R1_FB21_Msk     (0x1UL << CAN_F12R1_FB21_Pos)                    /*!< 0x00200000 */
#define CAN_F12R1_FB21         CAN_F12R1_FB21_Msk                              /*!<Filter bit 21 */
#define CAN_F12R1_FB22_Pos     (22U)
#define CAN_F12R1_FB22_Msk     (0x1UL << CAN_F12R1_FB22_Pos)                    /*!< 0x00400000 */
#define CAN_F12R1_FB22         CAN_F12R1_FB22_Msk                              /*!<Filter bit 22 */
#define CAN_F12R1_FB23_Pos     (23U)
#define CAN_F12R1_FB23_Msk     (0x1UL << CAN_F12R1_FB23_Pos)                    /*!< 0x00800000 */
#define CAN_F12R1_FB23         CAN_F12R1_FB23_Msk                              /*!<Filter bit 23 */
#define CAN_F12R1_FB24_Pos     (24U)
#define CAN_F12R1_FB24_Msk     (0x1UL << CAN_F12R1_FB24_Pos)                    /*!< 0x01000000 */
#define CAN_F12R1_FB24         CAN_F12R1_FB24_Msk                              /*!<Filter bit 24 */
#define CAN_F12R1_FB25_Pos     (25U)
#define CAN_F12R1_FB25_Msk     (0x1UL << CAN_F12R1_FB25_Pos)                    /*!< 0x02000000 */
#define CAN_F12R1_FB25         CAN_F12R1_FB25_Msk                              /*!<Filter bit 25 */
#define CAN_F12R1_FB26_Pos     (26U)
#define CAN_F12R1_FB26_Msk     (0x1UL << CAN_F12R1_FB26_Pos)                    /*!< 0x04000000 */
#define CAN_F12R1_FB26         CAN_F12R1_FB26_Msk                              /*!<Filter bit 26 */
#define CAN_F12R1_FB27_Pos     (27U)
#define CAN_F12R1_FB27_Msk     (0x1UL << CAN_F12R1_FB27_Pos)                    /*!< 0x08000000 */
#define CAN_F12R1_FB27         CAN_F12R1_FB27_Msk                              /*!<Filter bit 27 */
#define CAN_F12R1_FB28_Pos     (28U)
#define CAN_F12R1_FB28_Msk     (0x1UL << CAN_F12R1_FB28_Pos)                    /*!< 0x10000000 */
#define CAN_F12R1_FB28         CAN_F12R1_FB28_Msk                              /*!<Filter bit 28 */
#define CAN_F12R1_FB29_Pos     (29U)
#define CAN_F12R1_FB29_Msk     (0x1UL << CAN_F12R1_FB29_Pos)                    /*!< 0x20000000 */
#define CAN_F12R1_FB29         CAN_F12R1_FB29_Msk                              /*!<Filter bit 29 */
#define CAN_F12R1_FB30_Pos     (30U)
#define CAN_F12R1_FB30_Msk     (0x1UL << CAN_F12R1_FB30_Pos)                    /*!< 0x40000000 */
#define CAN_F12R1_FB30         CAN_F12R1_FB30_Msk                              /*!<Filter bit 30 */
#define CAN_F12R1_FB31_Pos     (31U)
#define CAN_F12R1_FB31_Msk     (0x1UL << CAN_F12R1_FB31_Pos)                    /*!< 0x80000000 */
#define CAN_F12R1_FB31         CAN_F12R1_FB31_Msk                              /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F13R1 register  ******************/
#define CAN_F13R1_FB0_Pos      (0U)
#define CAN_F13R1_FB0_Msk      (0x1UL << CAN_F13R1_FB0_Pos)                     /*!< 0x00000001 */
#define CAN_F13R1_FB0          CAN_F13R1_FB0_Msk                               /*!<Filter bit 0 */
#define CAN_F13R1_FB1_Pos      (1U)
#define CAN_F13R1_FB1_Msk      (0x1UL << CAN_F13R1_FB1_Pos)                     /*!< 0x00000002 */
#define CAN_F13R1_FB1          CAN_F13R1_FB1_Msk                               /*!<Filter bit 1 */
#define CAN_F13R1_FB2_Pos      (2U)
#define CAN_F13R1_FB2_Msk      (0x1UL << CAN_F13R1_FB2_Pos)                     /*!< 0x00000004 */
#define CAN_F13R1_FB2          CAN_F13R1_FB2_Msk                               /*!<Filter bit 2 */
#define CAN_F13R1_FB3_Pos      (3U)
#define CAN_F13R1_FB3_Msk      (0x1UL << CAN_F13R1_FB3_Pos)                     /*!< 0x00000008 */
#define CAN_F13R1_FB3          CAN_F13R1_FB3_Msk                               /*!<Filter bit 3 */
#define CAN_F13R1_FB4_Pos      (4U)
#define CAN_F13R1_FB4_Msk      (0x1UL << CAN_F13R1_FB4_Pos)                     /*!< 0x00000010 */
#define CAN_F13R1_FB4          CAN_F13R1_FB4_Msk                               /*!<Filter bit 4 */
#define CAN_F13R1_FB5_Pos      (5U)
#define CAN_F13R1_FB5_Msk      (0x1UL << CAN_F13R1_FB5_Pos)                     /*!< 0x00000020 */
#define CAN_F13R1_FB5          CAN_F13R1_FB5_Msk                               /*!<Filter bit 5 */
#define CAN_F13R1_FB6_Pos      (6U)
#define CAN_F13R1_FB6_Msk      (0x1UL << CAN_F13R1_FB6_Pos)                     /*!< 0x00000040 */
#define CAN_F13R1_FB6          CAN_F13R1_FB6_Msk                               /*!<Filter bit 6 */
#define CAN_F13R1_FB7_Pos      (7U)
#define CAN_F13R1_FB7_Msk      (0x1UL << CAN_F13R1_FB7_Pos)                     /*!< 0x00000080 */
#define CAN_F13R1_FB7          CAN_F13R1_FB7_Msk                               /*!<Filter bit 7 */
#define CAN_F13R1_FB8_Pos      (8U)
#define CAN_F13R1_FB8_Msk      (0x1UL << CAN_F13R1_FB8_Pos)                     /*!< 0x00000100 */
#define CAN_F13R1_FB8          CAN_F13R1_FB8_Msk                               /*!<Filter bit 8 */
#define CAN_F13R1_FB9_Pos      (9U)
#define CAN_F13R1_FB9_Msk      (0x1UL << CAN_F13R1_FB9_Pos)                     /*!< 0x00000200 */
#define CAN_F13R1_FB9          CAN_F13R1_FB9_Msk                               /*!<Filter bit 9 */
#define CAN_F13R1_FB10_Pos     (10U)
#define CAN_F13R1_FB10_Msk     (0x1UL << CAN_F13R1_FB10_Pos)                    /*!< 0x00000400 */
#define CAN_F13R1_FB10         CAN_F13R1_FB10_Msk                              /*!<Filter bit 10 */
#define CAN_F13R1_FB11_Pos     (11U)
#define CAN_F13R1_FB11_Msk     (0x1UL << CAN_F13R1_FB11_Pos)                    /*!< 0x00000800 */
#define CAN_F13R1_FB11         CAN_F13R1_FB11_Msk                              /*!<Filter bit 11 */
#define CAN_F13R1_FB12_Pos     (12U)
#define CAN_F13R1_FB12_Msk     (0x1UL << CAN_F13R1_FB12_Pos)                    /*!< 0x00001000 */
#define CAN_F13R1_FB12         CAN_F13R1_FB12_Msk                              /*!<Filter bit 12 */
#define CAN_F13R1_FB13_Pos     (13U)
#define CAN_F13R1_FB13_Msk     (0x1UL << CAN_F13R1_FB13_Pos)                    /*!< 0x00002000 */
#define CAN_F13R1_FB13         CAN_F13R1_FB13_Msk                              /*!<Filter bit 13 */
#define CAN_F13R1_FB14_Pos     (14U)
#define CAN_F13R1_FB14_Msk     (0x1UL << CAN_F13R1_FB14_Pos)                    /*!< 0x00004000 */
#define CAN_F13R1_FB14         CAN_F13R1_FB14_Msk                              /*!<Filter bit 14 */
#define CAN_F13R1_FB15_Pos     (15U)
#define CAN_F13R1_FB15_Msk     (0x1UL << CAN_F13R1_FB15_Pos)                    /*!< 0x00008000 */
#define CAN_F13R1_FB15         CAN_F13R1_FB15_Msk                              /*!<Filter bit 15 */
#define CAN_F13R1_FB16_Pos     (16U)
#define CAN_F13R1_FB16_Msk     (0x1UL << CAN_F13R1_FB16_Pos)                    /*!< 0x00010000 */
#define CAN_F13R1_FB16         CAN_F13R1_FB16_Msk                              /*!<Filter bit 16 */
#define CAN_F13R1_FB17_Pos     (17U)
#define CAN_F13R1_FB17_Msk     (0x1UL << CAN_F13R1_FB17_Pos)                    /*!< 0x00020000 */
#define CAN_F13R1_FB17         CAN_F13R1_FB17_Msk                              /*!<Filter bit 17 */
#define CAN_F13R1_FB18_Pos     (18U)
#define CAN_F13R1_FB18_Msk     (0x1UL << CAN_F13R1_FB18_Pos)                    /*!< 0x00040000 */
#define CAN_F13R1_FB18         CAN_F13R1_FB18_Msk                              /*!<Filter bit 18 */
#define CAN_F13R1_FB19_Pos     (19U)
#define CAN_F13R1_FB19_Msk     (0x1UL << CAN_F13R1_FB19_Pos)                    /*!< 0x00080000 */
#define CAN_F13R1_FB19         CAN_F13R1_FB19_Msk                              /*!<Filter bit 19 */
#define CAN_F13R1_FB20_Pos     (20U)
#define CAN_F13R1_FB20_Msk     (0x1UL << CAN_F13R1_FB20_Pos)                    /*!< 0x00100000 */
#define CAN_F13R1_FB20         CAN_F13R1_FB20_Msk                              /*!<Filter bit 20 */
#define CAN_F13R1_FB21_Pos     (21U)
#define CAN_F13R1_FB21_Msk     (0x1UL << CAN_F13R1_FB21_Pos)                    /*!< 0x00200000 */
#define CAN_F13R1_FB21         CAN_F13R1_FB21_Msk                              /*!<Filter bit 21 */
#define CAN_F13R1_FB22_Pos     (22U)
#define CAN_F13R1_FB22_Msk     (0x1UL << CAN_F13R1_FB22_Pos)                    /*!< 0x00400000 */
#define CAN_F13R1_FB22         CAN_F13R1_FB22_Msk                              /*!<Filter bit 22 */
#define CAN_F13R1_FB23_Pos     (23U)
#define CAN_F13R1_FB23_Msk     (0x1UL << CAN_F13R1_FB23_Pos)                    /*!< 0x00800000 */
#define CAN_F13R1_FB23         CAN_F13R1_FB23_Msk                              /*!<Filter bit 23 */
#define CAN_F13R1_FB24_Pos     (24U)
#define CAN_F13R1_FB24_Msk     (0x1UL << CAN_F13R1_FB24_Pos)                    /*!< 0x01000000 */
#define CAN_F13R1_FB24         CAN_F13R1_FB24_Msk                              /*!<Filter bit 24 */
#define CAN_F13R1_FB25_Pos     (25U)
#define CAN_F13R1_FB25_Msk     (0x1UL << CAN_F13R1_FB25_Pos)                    /*!< 0x02000000 */
#define CAN_F13R1_FB25         CAN_F13R1_FB25_Msk                              /*!<Filter bit 25 */
#define CAN_F13R1_FB26_Pos     (26U)
#define CAN_F13R1_FB26_Msk     (0x1UL << CAN_F13R1_FB26_Pos)                    /*!< 0x04000000 */
#define CAN_F13R1_FB26         CAN_F13R1_FB26_Msk                              /*!<Filter bit 26 */
#define CAN_F13R1_FB27_Pos     (27U)
#define CAN_F13R1_FB27_Msk     (0x1UL << CAN_F13R1_FB27_Pos)                    /*!< 0x08000000 */
#define CAN_F13R1_FB27         CAN_F13R1_FB27_Msk                              /*!<Filter bit 27 */
#define CAN_F13R1_FB28_Pos     (28U)
#define CAN_F13R1_FB28_Msk     (0x1UL << CAN_F13R1_FB28_Pos)                    /*!< 0x10000000 */
#define CAN_F13R1_FB28         CAN_F13R1_FB28_Msk                              /*!<Filter bit 28 */
#define CAN_F13R1_FB29_Pos     (29U)
#define CAN_F13R1_FB29_Msk     (0x1UL << CAN_F13R1_FB29_Pos)                    /*!< 0x20000000 */
#define CAN_F13R1_FB29         CAN_F13R1_FB29_Msk                              /*!<Filter bit 29 */
#define CAN_F13R1_FB30_Pos     (30U)
#define CAN_F13R1_FB30_Msk     (0x1UL << CAN_F13R1_FB30_Pos)                    /*!< 0x40000000 */
#define CAN_F13R1_FB30         CAN_F13R1_FB30_Msk                              /*!<Filter bit 30 */
#define CAN_F13R1_FB31_Pos     (31U)
#define CAN_F13R1_FB31_Msk     (0x1UL << CAN_F13R1_FB31_Pos)                    /*!< 0x80000000 */
#define CAN_F13R1_FB31         CAN_F13R1_FB31_Msk                              /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F0R2 register  *******************/
#define CAN_F0R2_FB0_Pos       (0U)
#define CAN_F0R2_FB0_Msk       (0x1UL << CAN_F0R2_FB0_Pos)                      /*!< 0x00000001 */
#define CAN_F0R2_FB0           CAN_F0R2_FB0_Msk                                /*!<Filter bit 0 */
#define CAN_F0R2_FB1_Pos       (1U)
#define CAN_F0R2_FB1_Msk       (0x1UL << CAN_F0R2_FB1_Pos)                      /*!< 0x00000002 */
#define CAN_F0R2_FB1           CAN_F0R2_FB1_Msk                                /*!<Filter bit 1 */
#define CAN_F0R2_FB2_Pos       (2U)
#define CAN_F0R2_FB2_Msk       (0x1UL << CAN_F0R2_FB2_Pos)                      /*!< 0x00000004 */
#define CAN_F0R2_FB2           CAN_F0R2_FB2_Msk                                /*!<Filter bit 2 */
#define CAN_F0R2_FB3_Pos       (3U)
#define CAN_F0R2_FB3_Msk       (0x1UL << CAN_F0R2_FB3_Pos)                      /*!< 0x00000008 */
#define CAN_F0R2_FB3           CAN_F0R2_FB3_Msk                                /*!<Filter bit 3 */
#define CAN_F0R2_FB4_Pos       (4U)
#define CAN_F0R2_FB4_Msk       (0x1UL << CAN_F0R2_FB4_Pos)                      /*!< 0x00000010 */
#define CAN_F0R2_FB4           CAN_F0R2_FB4_Msk                                /*!<Filter bit 4 */
#define CAN_F0R2_FB5_Pos       (5U)
#define CAN_F0R2_FB5_Msk       (0x1UL << CAN_F0R2_FB5_Pos)                      /*!< 0x00000020 */
#define CAN_F0R2_FB5           CAN_F0R2_FB5_Msk                                /*!<Filter bit 5 */
#define CAN_F0R2_FB6_Pos       (6U)
#define CAN_F0R2_FB6_Msk       (0x1UL << CAN_F0R2_FB6_Pos)                      /*!< 0x00000040 */
#define CAN_F0R2_FB6           CAN_F0R2_FB6_Msk                                /*!<Filter bit 6 */
#define CAN_F0R2_FB7_Pos       (7U)
#define CAN_F0R2_FB7_Msk       (0x1UL << CAN_F0R2_FB7_Pos)                      /*!< 0x00000080 */
#define CAN_F0R2_FB7           CAN_F0R2_FB7_Msk                                /*!<Filter bit 7 */
#define CAN_F0R2_FB8_Pos       (8U)
#define CAN_F0R2_FB8_Msk       (0x1UL << CAN_F0R2_FB8_Pos)                      /*!< 0x00000100 */
#define CAN_F0R2_FB8           CAN_F0R2_FB8_Msk                                /*!<Filter bit 8 */
#define CAN_F0R2_FB9_Pos       (9U)
#define CAN_F0R2_FB9_Msk       (0x1UL << CAN_F0R2_FB9_Pos)                      /*!< 0x00000200 */
#define CAN_F0R2_FB9           CAN_F0R2_FB9_Msk                                /*!<Filter bit 9 */
#define CAN_F0R2_FB10_Pos      (10U)
#define CAN_F0R2_FB10_Msk      (0x1UL << CAN_F0R2_FB10_Pos)                     /*!< 0x00000400 */
#define CAN_F0R2_FB10          CAN_F0R2_FB10_Msk                               /*!<Filter bit 10 */
#define CAN_F0R2_FB11_Pos      (11U)
#define CAN_F0R2_FB11_Msk      (0x1UL << CAN_F0R2_FB11_Pos)                     /*!< 0x00000800 */
#define CAN_F0R2_FB11          CAN_F0R2_FB11_Msk                               /*!<Filter bit 11 */
#define CAN_F0R2_FB12_Pos      (12U)
#define CAN_F0R2_FB12_Msk      (0x1UL << CAN_F0R2_FB12_Pos)                     /*!< 0x00001000 */
#define CAN_F0R2_FB12          CAN_F0R2_FB12_Msk                               /*!<Filter bit 12 */
#define CAN_F0R2_FB13_Pos      (13U)
#define CAN_F0R2_FB13_Msk      (0x1UL << CAN_F0R2_FB13_Pos)                     /*!< 0x00002000 */
#define CAN_F0R2_FB13          CAN_F0R2_FB13_Msk                               /*!<Filter bit 13 */
#define CAN_F0R2_FB14_Pos      (14U)
#define CAN_F0R2_FB14_Msk      (0x1UL << CAN_F0R2_FB14_Pos)                     /*!< 0x00004000 */
#define CAN_F0R2_FB14          CAN_F0R2_FB14_Msk                               /*!<Filter bit 14 */
#define CAN_F0R2_FB15_Pos      (15U)
#define CAN_F0R2_FB15_Msk      (0x1UL << CAN_F0R2_FB15_Pos)                     /*!< 0x00008000 */
#define CAN_F0R2_FB15          CAN_F0R2_FB15_Msk                               /*!<Filter bit 15 */
#define CAN_F0R2_FB16_Pos      (16U)
#define CAN_F0R2_FB16_Msk      (0x1UL << CAN_F0R2_FB16_Pos)                     /*!< 0x00010000 */
#define CAN_F0R2_FB16          CAN_F0R2_FB16_Msk                               /*!<Filter bit 16 */
#define CAN_F0R2_FB17_Pos      (17U)
#define CAN_F0R2_FB17_Msk      (0x1UL << CAN_F0R2_FB17_Pos)                     /*!< 0x00020000 */
#define CAN_F0R2_FB17          CAN_F0R2_FB17_Msk                               /*!<Filter bit 17 */
#define CAN_F0R2_FB18_Pos      (18U)
#define CAN_F0R2_FB18_Msk      (0x1UL << CAN_F0R2_FB18_Pos)                     /*!< 0x00040000 */
#define CAN_F0R2_FB18          CAN_F0R2_FB18_Msk                               /*!<Filter bit 18 */
#define CAN_F0R2_FB19_Pos      (19U)
#define CAN_F0R2_FB19_Msk      (0x1UL << CAN_F0R2_FB19_Pos)                     /*!< 0x00080000 */
#define CAN_F0R2_FB19          CAN_F0R2_FB19_Msk                               /*!<Filter bit 19 */
#define CAN_F0R2_FB20_Pos      (20U)
#define CAN_F0R2_FB20_Msk      (0x1UL << CAN_F0R2_FB20_Pos)                     /*!< 0x00100000 */
#define CAN_F0R2_FB20          CAN_F0R2_FB20_Msk                               /*!<Filter bit 20 */
#define CAN_F0R2_FB21_Pos      (21U)
#define CAN_F0R2_FB21_Msk      (0x1UL << CAN_F0R2_FB21_Pos)                     /*!< 0x00200000 */
#define CAN_F0R2_FB21          CAN_F0R2_FB21_Msk                               /*!<Filter bit 21 */
#define CAN_F0R2_FB22_Pos      (22U)
#define CAN_F0R2_FB22_Msk      (0x1UL << CAN_F0R2_FB22_Pos)                     /*!< 0x00400000 */
#define CAN_F0R2_FB22          CAN_F0R2_FB22_Msk                               /*!<Filter bit 22 */
#define CAN_F0R2_FB23_Pos      (23U)
#define CAN_F0R2_FB23_Msk      (0x1UL << CAN_F0R2_FB23_Pos)                     /*!< 0x00800000 */
#define CAN_F0R2_FB23          CAN_F0R2_FB23_Msk                               /*!<Filter bit 23 */
#define CAN_F0R2_FB24_Pos      (24U)
#define CAN_F0R2_FB24_Msk      (0x1UL << CAN_F0R2_FB24_Pos)                     /*!< 0x01000000 */
#define CAN_F0R2_FB24          CAN_F0R2_FB24_Msk                               /*!<Filter bit 24 */
#define CAN_F0R2_FB25_Pos      (25U)
#define CAN_F0R2_FB25_Msk      (0x1UL << CAN_F0R2_FB25_Pos)                     /*!< 0x02000000 */
#define CAN_F0R2_FB25          CAN_F0R2_FB25_Msk                               /*!<Filter bit 25 */
#define CAN_F0R2_FB26_Pos      (26U)
#define CAN_F0R2_FB26_Msk      (0x1UL << CAN_F0R2_FB26_Pos)                     /*!< 0x04000000 */
#define CAN_F0R2_FB26          CAN_F0R2_FB26_Msk                               /*!<Filter bit 26 */
#define CAN_F0R2_FB27_Pos      (27U)
#define CAN_F0R2_FB27_Msk      (0x1UL << CAN_F0R2_FB27_Pos)                     /*!< 0x08000000 */
#define CAN_F0R2_FB27          CAN_F0R2_FB27_Msk                               /*!<Filter bit 27 */
#define CAN_F0R2_FB28_Pos      (28U)
#define CAN_F0R2_FB28_Msk      (0x1UL << CAN_F0R2_FB28_Pos)                     /*!< 0x10000000 */
#define CAN_F0R2_FB28          CAN_F0R2_FB28_Msk                               /*!<Filter bit 28 */
#define CAN_F0R2_FB29_Pos      (29U)
#define CAN_F0R2_FB29_Msk      (0x1UL << CAN_F0R2_FB29_Pos)                     /*!< 0x20000000 */
#define CAN_F0R2_FB29          CAN_F0R2_FB29_Msk                               /*!<Filter bit 29 */
#define CAN_F0R2_FB30_Pos      (30U)
#define CAN_F0R2_FB30_Msk      (0x1UL << CAN_F0R2_FB30_Pos)                     /*!< 0x40000000 */
#define CAN_F0R2_FB30          CAN_F0R2_FB30_Msk                               /*!<Filter bit 30 */
#define CAN_F0R2_FB31_Pos      (31U)
#define CAN_F0R2_FB31_Msk      (0x1UL << CAN_F0R2_FB31_Pos)                     /*!< 0x80000000 */
#define CAN_F0R2_FB31          CAN_F0R2_FB31_Msk                               /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F1R2 register  *******************/
#define CAN_F1R2_FB0_Pos       (0U)
#define CAN_F1R2_FB0_Msk       (0x1UL << CAN_F1R2_FB0_Pos)                      /*!< 0x00000001 */
#define CAN_F1R2_FB0           CAN_F1R2_FB0_Msk                                /*!<Filter bit 0 */
#define CAN_F1R2_FB1_Pos       (1U)
#define CAN_F1R2_FB1_Msk       (0x1UL << CAN_F1R2_FB1_Pos)                      /*!< 0x00000002 */
#define CAN_F1R2_FB1           CAN_F1R2_FB1_Msk                                /*!<Filter bit 1 */
#define CAN_F1R2_FB2_Pos       (2U)
#define CAN_F1R2_FB2_Msk       (0x1UL << CAN_F1R2_FB2_Pos)                      /*!< 0x00000004 */
#define CAN_F1R2_FB2           CAN_F1R2_FB2_Msk                                /*!<Filter bit 2 */
#define CAN_F1R2_FB3_Pos       (3U)
#define CAN_F1R2_FB3_Msk       (0x1UL << CAN_F1R2_FB3_Pos)                      /*!< 0x00000008 */
#define CAN_F1R2_FB3           CAN_F1R2_FB3_Msk                                /*!<Filter bit 3 */
#define CAN_F1R2_FB4_Pos       (4U)
#define CAN_F1R2_FB4_Msk       (0x1UL << CAN_F1R2_FB4_Pos)                      /*!< 0x00000010 */
#define CAN_F1R2_FB4           CAN_F1R2_FB4_Msk                                /*!<Filter bit 4 */
#define CAN_F1R2_FB5_Pos       (5U)
#define CAN_F1R2_FB5_Msk       (0x1UL << CAN_F1R2_FB5_Pos)                      /*!< 0x00000020 */
#define CAN_F1R2_FB5           CAN_F1R2_FB5_Msk                                /*!<Filter bit 5 */
#define CAN_F1R2_FB6_Pos       (6U)
#define CAN_F1R2_FB6_Msk       (0x1UL << CAN_F1R2_FB6_Pos)                      /*!< 0x00000040 */
#define CAN_F1R2_FB6           CAN_F1R2_FB6_Msk                                /*!<Filter bit 6 */
#define CAN_F1R2_FB7_Pos       (7U)
#define CAN_F1R2_FB7_Msk       (0x1UL << CAN_F1R2_FB7_Pos)                      /*!< 0x00000080 */
#define CAN_F1R2_FB7           CAN_F1R2_FB7_Msk                                /*!<Filter bit 7 */
#define CAN_F1R2_FB8_Pos       (8U)
#define CAN_F1R2_FB8_Msk       (0x1UL << CAN_F1R2_FB8_Pos)                      /*!< 0x00000100 */
#define CAN_F1R2_FB8           CAN_F1R2_FB8_Msk                                /*!<Filter bit 8 */
#define CAN_F1R2_FB9_Pos       (9U)
#define CAN_F1R2_FB9_Msk       (0x1UL << CAN_F1R2_FB9_Pos)                      /*!< 0x00000200 */
#define CAN_F1R2_FB9           CAN_F1R2_FB9_Msk                                /*!<Filter bit 9 */
#define CAN_F1R2_FB10_Pos      (10U)
#define CAN_F1R2_FB10_Msk      (0x1UL << CAN_F1R2_FB10_Pos)                     /*!< 0x00000400 */
#define CAN_F1R2_FB10          CAN_F1R2_FB10_Msk                               /*!<Filter bit 10 */
#define CAN_F1R2_FB11_Pos      (11U)
#define CAN_F1R2_FB11_Msk      (0x1UL << CAN_F1R2_FB11_Pos)                     /*!< 0x00000800 */
#define CAN_F1R2_FB11          CAN_F1R2_FB11_Msk                               /*!<Filter bit 11 */
#define CAN_F1R2_FB12_Pos      (12U)
#define CAN_F1R2_FB12_Msk      (0x1UL << CAN_F1R2_FB12_Pos)                     /*!< 0x00001000 */
#define CAN_F1R2_FB12          CAN_F1R2_FB12_Msk                               /*!<Filter bit 12 */
#define CAN_F1R2_FB13_Pos      (13U)
#define CAN_F1R2_FB13_Msk      (0x1UL << CAN_F1R2_FB13_Pos)                     /*!< 0x00002000 */
#define CAN_F1R2_FB13          CAN_F1R2_FB13_Msk                               /*!<Filter bit 13 */
#define CAN_F1R2_FB14_Pos      (14U)
#define CAN_F1R2_FB14_Msk      (0x1UL << CAN_F1R2_FB14_Pos)                     /*!< 0x00004000 */
#define CAN_F1R2_FB14          CAN_F1R2_FB14_Msk                               /*!<Filter bit 14 */
#define CAN_F1R2_FB15_Pos      (15U)
#define CAN_F1R2_FB15_Msk      (0x1UL << CAN_F1R2_FB15_Pos)                     /*!< 0x00008000 */
#define CAN_F1R2_FB15          CAN_F1R2_FB15_Msk                               /*!<Filter bit 15 */
#define CAN_F1R2_FB16_Pos      (16U)
#define CAN_F1R2_FB16_Msk      (0x1UL << CAN_F1R2_FB16_Pos)                     /*!< 0x00010000 */
#define CAN_F1R2_FB16          CAN_F1R2_FB16_Msk                               /*!<Filter bit 16 */
#define CAN_F1R2_FB17_Pos      (17U)
#define CAN_F1R2_FB17_Msk      (0x1UL << CAN_F1R2_FB17_Pos)                     /*!< 0x00020000 */
#define CAN_F1R2_FB17          CAN_F1R2_FB17_Msk                               /*!<Filter bit 17 */
#define CAN_F1R2_FB18_Pos      (18U)
#define CAN_F1R2_FB18_Msk      (0x1UL << CAN_F1R2_FB18_Pos)                     /*!< 0x00040000 */
#define CAN_F1R2_FB18          CAN_F1R2_FB18_Msk                               /*!<Filter bit 18 */
#define CAN_F1R2_FB19_Pos      (19U)
#define CAN_F1R2_FB19_Msk      (0x1UL << CAN_F1R2_FB19_Pos)                     /*!< 0x00080000 */
#define CAN_F1R2_FB19          CAN_F1R2_FB19_Msk                               /*!<Filter bit 19 */
#define CAN_F1R2_FB20_Pos      (20U)
#define CAN_F1R2_FB20_Msk      (0x1UL << CAN_F1R2_FB20_Pos)                     /*!< 0x00100000 */
#define CAN_F1R2_FB20          CAN_F1R2_FB20_Msk                               /*!<Filter bit 20 */
#define CAN_F1R2_FB21_Pos      (21U)
#define CAN_F1R2_FB21_Msk      (0x1UL << CAN_F1R2_FB21_Pos)                     /*!< 0x00200000 */
#define CAN_F1R2_FB21          CAN_F1R2_FB21_Msk                               /*!<Filter bit 21 */
#define CAN_F1R2_FB22_Pos      (22U)
#define CAN_F1R2_FB22_Msk      (0x1UL << CAN_F1R2_FB22_Pos)                     /*!< 0x00400000 */
#define CAN_F1R2_FB22          CAN_F1R2_FB22_Msk                               /*!<Filter bit 22 */
#define CAN_F1R2_FB23_Pos      (23U)
#define CAN_F1R2_FB23_Msk      (0x1UL << CAN_F1R2_FB23_Pos)                     /*!< 0x00800000 */
#define CAN_F1R2_FB23          CAN_F1R2_FB23_Msk                               /*!<Filter bit 23 */
#define CAN_F1R2_FB24_Pos      (24U)
#define CAN_F1R2_FB24_Msk      (0x1UL << CAN_F1R2_FB24_Pos)                     /*!< 0x01000000 */
#define CAN_F1R2_FB24          CAN_F1R2_FB24_Msk                               /*!<Filter bit 24 */
#define CAN_F1R2_FB25_Pos      (25U)
#define CAN_F1R2_FB25_Msk      (0x1UL << CAN_F1R2_FB25_Pos)                     /*!< 0x02000000 */
#define CAN_F1R2_FB25          CAN_F1R2_FB25_Msk                               /*!<Filter bit 25 */
#define CAN_F1R2_FB26_Pos      (26U)
#define CAN_F1R2_FB26_Msk      (0x1UL << CAN_F1R2_FB26_Pos)                     /*!< 0x04000000 */
#define CAN_F1R2_FB26          CAN_F1R2_FB26_Msk                               /*!<Filter bit 26 */
#define CAN_F1R2_FB27_Pos      (27U)
#define CAN_F1R2_FB27_Msk      (0x1UL << CAN_F1R2_FB27_Pos)                     /*!< 0x08000000 */
#define CAN_F1R2_FB27          CAN_F1R2_FB27_Msk                               /*!<Filter bit 27 */
#define CAN_F1R2_FB28_Pos      (28U)
#define CAN_F1R2_FB28_Msk      (0x1UL << CAN_F1R2_FB28_Pos)                     /*!< 0x10000000 */
#define CAN_F1R2_FB28          CAN_F1R2_FB28_Msk                               /*!<Filter bit 28 */
#define CAN_F1R2_FB29_Pos      (29U)
#define CAN_F1R2_FB29_Msk      (0x1UL << CAN_F1R2_FB29_Pos)                     /*!< 0x20000000 */
#define CAN_F1R2_FB29          CAN_F1R2_FB29_Msk                               /*!<Filter bit 29 */
#define CAN_F1R2_FB30_Pos      (30U)
#define CAN_F1R2_FB30_Msk      (0x1UL << CAN_F1R2_FB30_Pos)                     /*!< 0x40000000 */
#define CAN_F1R2_FB30          CAN_F1R2_FB30_Msk                               /*!<Filter bit 30 */
#define CAN_F1R2_FB31_Pos      (31U)
#define CAN_F1R2_FB31_Msk      (0x1UL << CAN_F1R2_FB31_Pos)                     /*!< 0x80000000 */
#define CAN_F1R2_FB31          CAN_F1R2_FB31_Msk                               /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F2R2 register  *******************/
#define CAN_F2R2_FB0_Pos       (0U)
#define CAN_F2R2_FB0_Msk       (0x1UL << CAN_F2R2_FB0_Pos)                      /*!< 0x00000001 */
#define CAN_F2R2_FB0           CAN_F2R2_FB0_Msk                                /*!<Filter bit 0 */
#define CAN_F2R2_FB1_Pos       (1U)
#define CAN_F2R2_FB1_Msk       (0x1UL << CAN_F2R2_FB1_Pos)                      /*!< 0x00000002 */
#define CAN_F2R2_FB1           CAN_F2R2_FB1_Msk                                /*!<Filter bit 1 */
#define CAN_F2R2_FB2_Pos       (2U)
#define CAN_F2R2_FB2_Msk       (0x1UL << CAN_F2R2_FB2_Pos)                      /*!< 0x00000004 */
#define CAN_F2R2_FB2           CAN_F2R2_FB2_Msk                                /*!<Filter bit 2 */
#define CAN_F2R2_FB3_Pos       (3U)
#define CAN_F2R2_FB3_Msk       (0x1UL << CAN_F2R2_FB3_Pos)                      /*!< 0x00000008 */
#define CAN_F2R2_FB3           CAN_F2R2_FB3_Msk                                /*!<Filter bit 3 */
#define CAN_F2R2_FB4_Pos       (4U)
#define CAN_F2R2_FB4_Msk       (0x1UL << CAN_F2R2_FB4_Pos)                      /*!< 0x00000010 */
#define CAN_F2R2_FB4           CAN_F2R2_FB4_Msk                                /*!<Filter bit 4 */
#define CAN_F2R2_FB5_Pos       (5U)
#define CAN_F2R2_FB5_Msk       (0x1UL << CAN_F2R2_FB5_Pos)                      /*!< 0x00000020 */
#define CAN_F2R2_FB5           CAN_F2R2_FB5_Msk                                /*!<Filter bit 5 */
#define CAN_F2R2_FB6_Pos       (6U)
#define CAN_F2R2_FB6_Msk       (0x1UL << CAN_F2R2_FB6_Pos)                      /*!< 0x00000040 */
#define CAN_F2R2_FB6           CAN_F2R2_FB6_Msk                                /*!<Filter bit 6 */
#define CAN_F2R2_FB7_Pos       (7U)
#define CAN_F2R2_FB7_Msk       (0x1UL << CAN_F2R2_FB7_Pos)                      /*!< 0x00000080 */
#define CAN_F2R2_FB7           CAN_F2R2_FB7_Msk                                /*!<Filter bit 7 */
#define CAN_F2R2_FB8_Pos       (8U)
#define CAN_F2R2_FB8_Msk       (0x1UL << CAN_F2R2_FB8_Pos)                      /*!< 0x00000100 */
#define CAN_F2R2_FB8           CAN_F2R2_FB8_Msk                                /*!<Filter bit 8 */
#define CAN_F2R2_FB9_Pos       (9U)
#define CAN_F2R2_FB9_Msk       (0x1UL << CAN_F2R2_FB9_Pos)                      /*!< 0x00000200 */
#define CAN_F2R2_FB9           CAN_F2R2_FB9_Msk                                /*!<Filter bit 9 */
#define CAN_F2R2_FB10_Pos      (10U)
#define CAN_F2R2_FB10_Msk      (0x1UL << CAN_F2R2_FB10_Pos)                     /*!< 0x00000400 */
#define CAN_F2R2_FB10          CAN_F2R2_FB10_Msk                               /*!<Filter bit 10 */
#define CAN_F2R2_FB11_Pos      (11U)
#define CAN_F2R2_FB11_Msk      (0x1UL << CAN_F2R2_FB11_Pos)                     /*!< 0x00000800 */
#define CAN_F2R2_FB11          CAN_F2R2_FB11_Msk                               /*!<Filter bit 11 */
#define CAN_F2R2_FB12_Pos      (12U)
#define CAN_F2R2_FB12_Msk      (0x1UL << CAN_F2R2_FB12_Pos)                     /*!< 0x00001000 */
#define CAN_F2R2_FB12          CAN_F2R2_FB12_Msk                               /*!<Filter bit 12 */
#define CAN_F2R2_FB13_Pos      (13U)
#define CAN_F2R2_FB13_Msk      (0x1UL << CAN_F2R2_FB13_Pos)                     /*!< 0x00002000 */
#define CAN_F2R2_FB13          CAN_F2R2_FB13_Msk                               /*!<Filter bit 13 */
#define CAN_F2R2_FB14_Pos      (14U)
#define CAN_F2R2_FB14_Msk      (0x1UL << CAN_F2R2_FB14_Pos)                     /*!< 0x00004000 */
#define CAN_F2R2_FB14          CAN_F2R2_FB14_Msk                               /*!<Filter bit 14 */
#define CAN_F2R2_FB15_Pos      (15U)
#define CAN_F2R2_FB15_Msk      (0x1UL << CAN_F2R2_FB15_Pos)                     /*!< 0x00008000 */
#define CAN_F2R2_FB15          CAN_F2R2_FB15_Msk                               /*!<Filter bit 15 */
#define CAN_F2R2_FB16_Pos      (16U)
#define CAN_F2R2_FB16_Msk      (0x1UL << CAN_F2R2_FB16_Pos)                     /*!< 0x00010000 */
#define CAN_F2R2_FB16          CAN_F2R2_FB16_Msk                               /*!<Filter bit 16 */
#define CAN_F2R2_FB17_Pos      (17U)
#define CAN_F2R2_FB17_Msk      (0x1UL << CAN_F2R2_FB17_Pos)                     /*!< 0x00020000 */
#define CAN_F2R2_FB17          CAN_F2R2_FB17_Msk                               /*!<Filter bit 17 */
#define CAN_F2R2_FB18_Pos      (18U)
#define CAN_F2R2_FB18_Msk      (0x1UL << CAN_F2R2_FB18_Pos)                     /*!< 0x00040000 */
#define CAN_F2R2_FB18          CAN_F2R2_FB18_Msk                               /*!<Filter bit 18 */
#define CAN_F2R2_FB19_Pos      (19U)
#define CAN_F2R2_FB19_Msk      (0x1UL << CAN_F2R2_FB19_Pos)                     /*!< 0x00080000 */
#define CAN_F2R2_FB19          CAN_F2R2_FB19_Msk                               /*!<Filter bit 19 */
#define CAN_F2R2_FB20_Pos      (20U)
#define CAN_F2R2_FB20_Msk      (0x1UL << CAN_F2R2_FB20_Pos)                     /*!< 0x00100000 */
#define CAN_F2R2_FB20          CAN_F2R2_FB20_Msk                               /*!<Filter bit 20 */
#define CAN_F2R2_FB21_Pos      (21U)
#define CAN_F2R2_FB21_Msk      (0x1UL << CAN_F2R2_FB21_Pos)                     /*!< 0x00200000 */
#define CAN_F2R2_FB21          CAN_F2R2_FB21_Msk                               /*!<Filter bit 21 */
#define CAN_F2R2_FB22_Pos      (22U)
#define CAN_F2R2_FB22_Msk      (0x1UL << CAN_F2R2_FB22_Pos)                     /*!< 0x00400000 */
#define CAN_F2R2_FB22          CAN_F2R2_FB22_Msk                               /*!<Filter bit 22 */
#define CAN_F2R2_FB23_Pos      (23U)
#define CAN_F2R2_FB23_Msk      (0x1UL << CAN_F2R2_FB23_Pos)                     /*!< 0x00800000 */
#define CAN_F2R2_FB23          CAN_F2R2_FB23_Msk                               /*!<Filter bit 23 */
#define CAN_F2R2_FB24_Pos      (24U)
#define CAN_F2R2_FB24_Msk      (0x1UL << CAN_F2R2_FB24_Pos)                     /*!< 0x01000000 */
#define CAN_F2R2_FB24          CAN_F2R2_FB24_Msk                               /*!<Filter bit 24 */
#define CAN_F2R2_FB25_Pos      (25U)
#define CAN_F2R2_FB25_Msk      (0x1UL << CAN_F2R2_FB25_Pos)                     /*!< 0x02000000 */
#define CAN_F2R2_FB25          CAN_F2R2_FB25_Msk                               /*!<Filter bit 25 */
#define CAN_F2R2_FB26_Pos      (26U)
#define CAN_F2R2_FB26_Msk      (0x1UL << CAN_F2R2_FB26_Pos)                     /*!< 0x04000000 */
#define CAN_F2R2_FB26          CAN_F2R2_FB26_Msk                               /*!<Filter bit 26 */
#define CAN_F2R2_FB27_Pos      (27U)
#define CAN_F2R2_FB27_Msk      (0x1UL << CAN_F2R2_FB27_Pos)                     /*!< 0x08000000 */
#define CAN_F2R2_FB27          CAN_F2R2_FB27_Msk                               /*!<Filter bit 27 */
#define CAN_F2R2_FB28_Pos      (28U)
#define CAN_F2R2_FB28_Msk      (0x1UL << CAN_F2R2_FB28_Pos)                     /*!< 0x10000000 */
#define CAN_F2R2_FB28          CAN_F2R2_FB28_Msk                               /*!<Filter bit 28 */
#define CAN_F2R2_FB29_Pos      (29U)
#define CAN_F2R2_FB29_Msk      (0x1UL << CAN_F2R2_FB29_Pos)                     /*!< 0x20000000 */
#define CAN_F2R2_FB29          CAN_F2R2_FB29_Msk                               /*!<Filter bit 29 */
#define CAN_F2R2_FB30_Pos      (30U)
#define CAN_F2R2_FB30_Msk      (0x1UL << CAN_F2R2_FB30_Pos)                     /*!< 0x40000000 */
#define CAN_F2R2_FB30          CAN_F2R2_FB30_Msk                               /*!<Filter bit 30 */
#define CAN_F2R2_FB31_Pos      (31U)
#define CAN_F2R2_FB31_Msk      (0x1UL << CAN_F2R2_FB31_Pos)                     /*!< 0x80000000 */
#define CAN_F2R2_FB31          CAN_F2R2_FB31_Msk                               /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F3R2 register  *******************/
#define CAN_F3R2_FB0_Pos       (0U)
#define CAN_F3R2_FB0_Msk       (0x1UL << CAN_F3R2_FB0_Pos)                      /*!< 0x00000001 */
#define CAN_F3R2_FB0           CAN_F3R2_FB0_Msk                                /*!<Filter bit 0 */
#define CAN_F3R2_FB1_Pos       (1U)
#define CAN_F3R2_FB1_Msk       (0x1UL << CAN_F3R2_FB1_Pos)                      /*!< 0x00000002 */
#define CAN_F3R2_FB1           CAN_F3R2_FB1_Msk                                /*!<Filter bit 1 */
#define CAN_F3R2_FB2_Pos       (2U)
#define CAN_F3R2_FB2_Msk       (0x1UL << CAN_F3R2_FB2_Pos)                      /*!< 0x00000004 */
#define CAN_F3R2_FB2           CAN_F3R2_FB2_Msk                                /*!<Filter bit 2 */
#define CAN_F3R2_FB3_Pos       (3U)
#define CAN_F3R2_FB3_Msk       (0x1UL << CAN_F3R2_FB3_Pos)                      /*!< 0x00000008 */
#define CAN_F3R2_FB3           CAN_F3R2_FB3_Msk                                /*!<Filter bit 3 */
#define CAN_F3R2_FB4_Pos       (4U)
#define CAN_F3R2_FB4_Msk       (0x1UL << CAN_F3R2_FB4_Pos)                      /*!< 0x00000010 */
#define CAN_F3R2_FB4           CAN_F3R2_FB4_Msk                                /*!<Filter bit 4 */
#define CAN_F3R2_FB5_Pos       (5U)
#define CAN_F3R2_FB5_Msk       (0x1UL << CAN_F3R2_FB5_Pos)                      /*!< 0x00000020 */
#define CAN_F3R2_FB5           CAN_F3R2_FB5_Msk                                /*!<Filter bit 5 */
#define CAN_F3R2_FB6_Pos       (6U)
#define CAN_F3R2_FB6_Msk       (0x1UL << CAN_F3R2_FB6_Pos)                      /*!< 0x00000040 */
#define CAN_F3R2_FB6           CAN_F3R2_FB6_Msk                                /*!<Filter bit 6 */
#define CAN_F3R2_FB7_Pos       (7U)
#define CAN_F3R2_FB7_Msk       (0x1UL << CAN_F3R2_FB7_Pos)                      /*!< 0x00000080 */
#define CAN_F3R2_FB7           CAN_F3R2_FB7_Msk                                /*!<Filter bit 7 */
#define CAN_F3R2_FB8_Pos       (8U)
#define CAN_F3R2_FB8_Msk       (0x1UL << CAN_F3R2_FB8_Pos)                      /*!< 0x00000100 */
#define CAN_F3R2_FB8           CAN_F3R2_FB8_Msk                                /*!<Filter bit 8 */
#define CAN_F3R2_FB9_Pos       (9U)
#define CAN_F3R2_FB9_Msk       (0x1UL << CAN_F3R2_FB9_Pos)                      /*!< 0x00000200 */
#define CAN_F3R2_FB9           CAN_F3R2_FB9_Msk                                /*!<Filter bit 9 */
#define CAN_F3R2_FB10_Pos      (10U)
#define CAN_F3R2_FB10_Msk      (0x1UL << CAN_F3R2_FB10_Pos)                     /*!< 0x00000400 */
#define CAN_F3R2_FB10          CAN_F3R2_FB10_Msk                               /*!<Filter bit 10 */
#define CAN_F3R2_FB11_Pos      (11U)
#define CAN_F3R2_FB11_Msk      (0x1UL << CAN_F3R2_FB11_Pos)                     /*!< 0x00000800 */
#define CAN_F3R2_FB11          CAN_F3R2_FB11_Msk                               /*!<Filter bit 11 */
#define CAN_F3R2_FB12_Pos      (12U)
#define CAN_F3R2_FB12_Msk      (0x1UL << CAN_F3R2_FB12_Pos)                     /*!< 0x00001000 */
#define CAN_F3R2_FB12          CAN_F3R2_FB12_Msk                               /*!<Filter bit 12 */
#define CAN_F3R2_FB13_Pos      (13U)
#define CAN_F3R2_FB13_Msk      (0x1UL << CAN_F3R2_FB13_Pos)                     /*!< 0x00002000 */
#define CAN_F3R2_FB13          CAN_F3R2_FB13_Msk                               /*!<Filter bit 13 */
#define CAN_F3R2_FB14_Pos      (14U)
#define CAN_F3R2_FB14_Msk      (0x1UL << CAN_F3R2_FB14_Pos)                     /*!< 0x00004000 */
#define CAN_F3R2_FB14          CAN_F3R2_FB14_Msk                               /*!<Filter bit 14 */
#define CAN_F3R2_FB15_Pos      (15U)
#define CAN_F3R2_FB15_Msk      (0x1UL << CAN_F3R2_FB15_Pos)                     /*!< 0x00008000 */
#define CAN_F3R2_FB15          CAN_F3R2_FB15_Msk                               /*!<Filter bit 15 */
#define CAN_F3R2_FB16_Pos      (16U)
#define CAN_F3R2_FB16_Msk      (0x1UL << CAN_F3R2_FB16_Pos)                     /*!< 0x00010000 */
#define CAN_F3R2_FB16          CAN_F3R2_FB16_Msk                               /*!<Filter bit 16 */
#define CAN_F3R2_FB17_Pos      (17U)
#define CAN_F3R2_FB17_Msk      (0x1UL << CAN_F3R2_FB17_Pos)                     /*!< 0x00020000 */
#define CAN_F3R2_FB17          CAN_F3R2_FB17_Msk                               /*!<Filter bit 17 */
#define CAN_F3R2_FB18_Pos      (18U)
#define CAN_F3R2_FB18_Msk      (0x1UL << CAN_F3R2_FB18_Pos)                     /*!< 0x00040000 */
#define CAN_F3R2_FB18          CAN_F3R2_FB18_Msk                               /*!<Filter bit 18 */
#define CAN_F3R2_FB19_Pos      (19U)
#define CAN_F3R2_FB19_Msk      (0x1UL << CAN_F3R2_FB19_Pos)                     /*!< 0x00080000 */
#define CAN_F3R2_FB19          CAN_F3R2_FB19_Msk                               /*!<Filter bit 19 */
#define CAN_F3R2_FB20_Pos      (20U)
#define CAN_F3R2_FB20_Msk      (0x1UL << CAN_F3R2_FB20_Pos)                     /*!< 0x00100000 */
#define CAN_F3R2_FB20          CAN_F3R2_FB20_Msk                               /*!<Filter bit 20 */
#define CAN_F3R2_FB21_Pos      (21U)
#define CAN_F3R2_FB21_Msk      (0x1UL << CAN_F3R2_FB21_Pos)                     /*!< 0x00200000 */
#define CAN_F3R2_FB21          CAN_F3R2_FB21_Msk                               /*!<Filter bit 21 */
#define CAN_F3R2_FB22_Pos      (22U)
#define CAN_F3R2_FB22_Msk      (0x1UL << CAN_F3R2_FB22_Pos)                     /*!< 0x00400000 */
#define CAN_F3R2_FB22          CAN_F3R2_FB22_Msk                               /*!<Filter bit 22 */
#define CAN_F3R2_FB23_Pos      (23U)
#define CAN_F3R2_FB23_Msk      (0x1UL << CAN_F3R2_FB23_Pos)                     /*!< 0x00800000 */
#define CAN_F3R2_FB23          CAN_F3R2_FB23_Msk                               /*!<Filter bit 23 */
#define CAN_F3R2_FB24_Pos      (24U)
#define CAN_F3R2_FB24_Msk      (0x1UL << CAN_F3R2_FB24_Pos)                     /*!< 0x01000000 */
#define CAN_F3R2_FB24          CAN_F3R2_FB24_Msk                               /*!<Filter bit 24 */
#define CAN_F3R2_FB25_Pos      (25U)
#define CAN_F3R2_FB25_Msk      (0x1UL << CAN_F3R2_FB25_Pos)                     /*!< 0x02000000 */
#define CAN_F3R2_FB25          CAN_F3R2_FB25_Msk                               /*!<Filter bit 25 */
#define CAN_F3R2_FB26_Pos      (26U)
#define CAN_F3R2_FB26_Msk      (0x1UL << CAN_F3R2_FB26_Pos)                     /*!< 0x04000000 */
#define CAN_F3R2_FB26          CAN_F3R2_FB26_Msk                               /*!<Filter bit 26 */
#define CAN_F3R2_FB27_Pos      (27U)
#define CAN_F3R2_FB27_Msk      (0x1UL << CAN_F3R2_FB27_Pos)                     /*!< 0x08000000 */
#define CAN_F3R2_FB27          CAN_F3R2_FB27_Msk                               /*!<Filter bit 27 */
#define CAN_F3R2_FB28_Pos      (28U)
#define CAN_F3R2_FB28_Msk      (0x1UL << CAN_F3R2_FB28_Pos)                     /*!< 0x10000000 */
#define CAN_F3R2_FB28          CAN_F3R2_FB28_Msk                               /*!<Filter bit 28 */
#define CAN_F3R2_FB29_Pos      (29U)
#define CAN_F3R2_FB29_Msk      (0x1UL << CAN_F3R2_FB29_Pos)                     /*!< 0x20000000 */
#define CAN_F3R2_FB29          CAN_F3R2_FB29_Msk                               /*!<Filter bit 29 */
#define CAN_F3R2_FB30_Pos      (30U)
#define CAN_F3R2_FB30_Msk      (0x1UL << CAN_F3R2_FB30_Pos)                     /*!< 0x40000000 */
#define CAN_F3R2_FB30          CAN_F3R2_FB30_Msk                               /*!<Filter bit 30 */
#define CAN_F3R2_FB31_Pos      (31U)
#define CAN_F3R2_FB31_Msk      (0x1UL << CAN_F3R2_FB31_Pos)                     /*!< 0x80000000 */
#define CAN_F3R2_FB31          CAN_F3R2_FB31_Msk                               /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F4R2 register  *******************/
#define CAN_F4R2_FB0_Pos       (0U)
#define CAN_F4R2_FB0_Msk       (0x1UL << CAN_F4R2_FB0_Pos)                      /*!< 0x00000001 */
#define CAN_F4R2_FB0           CAN_F4R2_FB0_Msk                                /*!<Filter bit 0 */
#define CAN_F4R2_FB1_Pos       (1U)
#define CAN_F4R2_FB1_Msk       (0x1UL << CAN_F4R2_FB1_Pos)                      /*!< 0x00000002 */
#define CAN_F4R2_FB1           CAN_F4R2_FB1_Msk                                /*!<Filter bit 1 */
#define CAN_F4R2_FB2_Pos       (2U)
#define CAN_F4R2_FB2_Msk       (0x1UL << CAN_F4R2_FB2_Pos)                      /*!< 0x00000004 */
#define CAN_F4R2_FB2           CAN_F4R2_FB2_Msk                                /*!<Filter bit 2 */
#define CAN_F4R2_FB3_Pos       (3U)
#define CAN_F4R2_FB3_Msk       (0x1UL << CAN_F4R2_FB3_Pos)                      /*!< 0x00000008 */
#define CAN_F4R2_FB3           CAN_F4R2_FB3_Msk                                /*!<Filter bit 3 */
#define CAN_F4R2_FB4_Pos       (4U)
#define CAN_F4R2_FB4_Msk       (0x1UL << CAN_F4R2_FB4_Pos)                      /*!< 0x00000010 */
#define CAN_F4R2_FB4           CAN_F4R2_FB4_Msk                                /*!<Filter bit 4 */
#define CAN_F4R2_FB5_Pos       (5U)
#define CAN_F4R2_FB5_Msk       (0x1UL << CAN_F4R2_FB5_Pos)                      /*!< 0x00000020 */
#define CAN_F4R2_FB5           CAN_F4R2_FB5_Msk                                /*!<Filter bit 5 */
#define CAN_F4R2_FB6_Pos       (6U)
#define CAN_F4R2_FB6_Msk       (0x1UL << CAN_F4R2_FB6_Pos)                      /*!< 0x00000040 */
#define CAN_F4R2_FB6           CAN_F4R2_FB6_Msk                                /*!<Filter bit 6 */
#define CAN_F4R2_FB7_Pos       (7U)
#define CAN_F4R2_FB7_Msk       (0x1UL << CAN_F4R2_FB7_Pos)                      /*!< 0x00000080 */
#define CAN_F4R2_FB7           CAN_F4R2_FB7_Msk                                /*!<Filter bit 7 */
#define CAN_F4R2_FB8_Pos       (8U)
#define CAN_F4R2_FB8_Msk       (0x1UL << CAN_F4R2_FB8_Pos)                      /*!< 0x00000100 */
#define CAN_F4R2_FB8           CAN_F4R2_FB8_Msk                                /*!<Filter bit 8 */
#define CAN_F4R2_FB9_Pos       (9U)
#define CAN_F4R2_FB9_Msk       (0x1UL << CAN_F4R2_FB9_Pos)                      /*!< 0x00000200 */
#define CAN_F4R2_FB9           CAN_F4R2_FB9_Msk                                /*!<Filter bit 9 */
#define CAN_F4R2_FB10_Pos      (10U)
#define CAN_F4R2_FB10_Msk      (0x1UL << CAN_F4R2_FB10_Pos)                     /*!< 0x00000400 */
#define CAN_F4R2_FB10          CAN_F4R2_FB10_Msk                               /*!<Filter bit 10 */
#define CAN_F4R2_FB11_Pos      (11U)
#define CAN_F4R2_FB11_Msk      (0x1UL << CAN_F4R2_FB11_Pos)                     /*!< 0x00000800 */
#define CAN_F4R2_FB11          CAN_F4R2_FB11_Msk                               /*!<Filter bit 11 */
#define CAN_F4R2_FB12_Pos      (12U)
#define CAN_F4R2_FB12_Msk      (0x1UL << CAN_F4R2_FB12_Pos)                     /*!< 0x00001000 */
#define CAN_F4R2_FB12          CAN_F4R2_FB12_Msk                               /*!<Filter bit 12 */
#define CAN_F4R2_FB13_Pos      (13U)
#define CAN_F4R2_FB13_Msk      (0x1UL << CAN_F4R2_FB13_Pos)                     /*!< 0x00002000 */
#define CAN_F4R2_FB13          CAN_F4R2_FB13_Msk                               /*!<Filter bit 13 */
#define CAN_F4R2_FB14_Pos      (14U)
#define CAN_F4R2_FB14_Msk      (0x1UL << CAN_F4R2_FB14_Pos)                     /*!< 0x00004000 */
#define CAN_F4R2_FB14          CAN_F4R2_FB14_Msk                               /*!<Filter bit 14 */
#define CAN_F4R2_FB15_Pos      (15U)
#define CAN_F4R2_FB15_Msk      (0x1UL << CAN_F4R2_FB15_Pos)                     /*!< 0x00008000 */
#define CAN_F4R2_FB15          CAN_F4R2_FB15_Msk                               /*!<Filter bit 15 */
#define CAN_F4R2_FB16_Pos      (16U)
#define CAN_F4R2_FB16_Msk      (0x1UL << CAN_F4R2_FB16_Pos)                     /*!< 0x00010000 */
#define CAN_F4R2_FB16          CAN_F4R2_FB16_Msk                               /*!<Filter bit 16 */
#define CAN_F4R2_FB17_Pos      (17U)
#define CAN_F4R2_FB17_Msk      (0x1UL << CAN_F4R2_FB17_Pos)                     /*!< 0x00020000 */
#define CAN_F4R2_FB17          CAN_F4R2_FB17_Msk                               /*!<Filter bit 17 */
#define CAN_F4R2_FB18_Pos      (18U)
#define CAN_F4R2_FB18_Msk      (0x1UL << CAN_F4R2_FB18_Pos)                     /*!< 0x00040000 */
#define CAN_F4R2_FB18          CAN_F4R2_FB18_Msk                               /*!<Filter bit 18 */
#define CAN_F4R2_FB19_Pos      (19U)
#define CAN_F4R2_FB19_Msk      (0x1UL << CAN_F4R2_FB19_Pos)                     /*!< 0x00080000 */
#define CAN_F4R2_FB19          CAN_F4R2_FB19_Msk                               /*!<Filter bit 19 */
#define CAN_F4R2_FB20_Pos      (20U)
#define CAN_F4R2_FB20_Msk      (0x1UL << CAN_F4R2_FB20_Pos)                     /*!< 0x00100000 */
#define CAN_F4R2_FB20          CAN_F4R2_FB20_Msk                               /*!<Filter bit 20 */
#define CAN_F4R2_FB21_Pos      (21U)
#define CAN_F4R2_FB21_Msk      (0x1UL << CAN_F4R2_FB21_Pos)                     /*!< 0x00200000 */
#define CAN_F4R2_FB21          CAN_F4R2_FB21_Msk                               /*!<Filter bit 21 */
#define CAN_F4R2_FB22_Pos      (22U)
#define CAN_F4R2_FB22_Msk      (0x1UL << CAN_F4R2_FB22_Pos)                     /*!< 0x00400000 */
#define CAN_F4R2_FB22          CAN_F4R2_FB22_Msk                               /*!<Filter bit 22 */
#define CAN_F4R2_FB23_Pos      (23U)
#define CAN_F4R2_FB23_Msk      (0x1UL << CAN_F4R2_FB23_Pos)                     /*!< 0x00800000 */
#define CAN_F4R2_FB23          CAN_F4R2_FB23_Msk                               /*!<Filter bit 23 */
#define CAN_F4R2_FB24_Pos      (24U)
#define CAN_F4R2_FB24_Msk      (0x1UL << CAN_F4R2_FB24_Pos)                     /*!< 0x01000000 */
#define CAN_F4R2_FB24          CAN_F4R2_FB24_Msk                               /*!<Filter bit 24 */
#define CAN_F4R2_FB25_Pos      (25U)
#define CAN_F4R2_FB25_Msk      (0x1UL << CAN_F4R2_FB25_Pos)                     /*!< 0x02000000 */
#define CAN_F4R2_FB25          CAN_F4R2_FB25_Msk                               /*!<Filter bit 25 */
#define CAN_F4R2_FB26_Pos      (26U)
#define CAN_F4R2_FB26_Msk      (0x1UL << CAN_F4R2_FB26_Pos)                     /*!< 0x04000000 */
#define CAN_F4R2_FB26          CAN_F4R2_FB26_Msk                               /*!<Filter bit 26 */
#define CAN_F4R2_FB27_Pos      (27U)
#define CAN_F4R2_FB27_Msk      (0x1UL << CAN_F4R2_FB27_Pos)                     /*!< 0x08000000 */
#define CAN_F4R2_FB27          CAN_F4R2_FB27_Msk                               /*!<Filter bit 27 */
#define CAN_F4R2_FB28_Pos      (28U)
#define CAN_F4R2_FB28_Msk      (0x1UL << CAN_F4R2_FB28_Pos)                     /*!< 0x10000000 */
#define CAN_F4R2_FB28          CAN_F4R2_FB28_Msk                               /*!<Filter bit 28 */
#define CAN_F4R2_FB29_Pos      (29U)
#define CAN_F4R2_FB29_Msk      (0x1UL << CAN_F4R2_FB29_Pos)                     /*!< 0x20000000 */
#define CAN_F4R2_FB29          CAN_F4R2_FB29_Msk                               /*!<Filter bit 29 */
#define CAN_F4R2_FB30_Pos      (30U)
#define CAN_F4R2_FB30_Msk      (0x1UL << CAN_F4R2_FB30_Pos)                     /*!< 0x40000000 */
#define CAN_F4R2_FB30          CAN_F4R2_FB30_Msk                               /*!<Filter bit 30 */
#define CAN_F4R2_FB31_Pos      (31U)
#define CAN_F4R2_FB31_Msk      (0x1UL << CAN_F4R2_FB31_Pos)                     /*!< 0x80000000 */
#define CAN_F4R2_FB31          CAN_F4R2_FB31_Msk                               /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F5R2 register  *******************/
#define CAN_F5R2_FB0_Pos       (0U)
#define CAN_F5R2_FB0_Msk       (0x1UL << CAN_F5R2_FB0_Pos)                      /*!< 0x00000001 */
#define CAN_F5R2_FB0           CAN_F5R2_FB0_Msk                                /*!<Filter bit 0 */
#define CAN_F5R2_FB1_Pos       (1U)
#define CAN_F5R2_FB1_Msk       (0x1UL << CAN_F5R2_FB1_Pos)                      /*!< 0x00000002 */
#define CAN_F5R2_FB1           CAN_F5R2_FB1_Msk                                /*!<Filter bit 1 */
#define CAN_F5R2_FB2_Pos       (2U)
#define CAN_F5R2_FB2_Msk       (0x1UL << CAN_F5R2_FB2_Pos)                      /*!< 0x00000004 */
#define CAN_F5R2_FB2           CAN_F5R2_FB2_Msk                                /*!<Filter bit 2 */
#define CAN_F5R2_FB3_Pos       (3U)
#define CAN_F5R2_FB3_Msk       (0x1UL << CAN_F5R2_FB3_Pos)                      /*!< 0x00000008 */
#define CAN_F5R2_FB3           CAN_F5R2_FB3_Msk                                /*!<Filter bit 3 */
#define CAN_F5R2_FB4_Pos       (4U)
#define CAN_F5R2_FB4_Msk       (0x1UL << CAN_F5R2_FB4_Pos)                      /*!< 0x00000010 */
#define CAN_F5R2_FB4           CAN_F5R2_FB4_Msk                                /*!<Filter bit 4 */
#define CAN_F5R2_FB5_Pos       (5U)
#define CAN_F5R2_FB5_Msk       (0x1UL << CAN_F5R2_FB5_Pos)                      /*!< 0x00000020 */
#define CAN_F5R2_FB5           CAN_F5R2_FB5_Msk                                /*!<Filter bit 5 */
#define CAN_F5R2_FB6_Pos       (6U)
#define CAN_F5R2_FB6_Msk       (0x1UL << CAN_F5R2_FB6_Pos)                      /*!< 0x00000040 */
#define CAN_F5R2_FB6           CAN_F5R2_FB6_Msk                                /*!<Filter bit 6 */
#define CAN_F5R2_FB7_Pos       (7U)
#define CAN_F5R2_FB7_Msk       (0x1UL << CAN_F5R2_FB7_Pos)                      /*!< 0x00000080 */
#define CAN_F5R2_FB7           CAN_F5R2_FB7_Msk                                /*!<Filter bit 7 */
#define CAN_F5R2_FB8_Pos       (8U)
#define CAN_F5R2_FB8_Msk       (0x1UL << CAN_F5R2_FB8_Pos)                      /*!< 0x00000100 */
#define CAN_F5R2_FB8           CAN_F5R2_FB8_Msk                                /*!<Filter bit 8 */
#define CAN_F5R2_FB9_Pos       (9U)
#define CAN_F5R2_FB9_Msk       (0x1UL << CAN_F5R2_FB9_Pos)                      /*!< 0x00000200 */
#define CAN_F5R2_FB9           CAN_F5R2_FB9_Msk                                /*!<Filter bit 9 */
#define CAN_F5R2_FB10_Pos      (10U)
#define CAN_F5R2_FB10_Msk      (0x1UL << CAN_F5R2_FB10_Pos)                     /*!< 0x00000400 */
#define CAN_F5R2_FB10          CAN_F5R2_FB10_Msk                               /*!<Filter bit 10 */
#define CAN_F5R2_FB11_Pos      (11U)
#define CAN_F5R2_FB11_Msk      (0x1UL << CAN_F5R2_FB11_Pos)                     /*!< 0x00000800 */
#define CAN_F5R2_FB11          CAN_F5R2_FB11_Msk                               /*!<Filter bit 11 */
#define CAN_F5R2_FB12_Pos      (12U)
#define CAN_F5R2_FB12_Msk      (0x1UL << CAN_F5R2_FB12_Pos)                     /*!< 0x00001000 */
#define CAN_F5R2_FB12          CAN_F5R2_FB12_Msk                               /*!<Filter bit 12 */
#define CAN_F5R2_FB13_Pos      (13U)
#define CAN_F5R2_FB13_Msk      (0x1UL << CAN_F5R2_FB13_Pos)                     /*!< 0x00002000 */
#define CAN_F5R2_FB13          CAN_F5R2_FB13_Msk                               /*!<Filter bit 13 */
#define CAN_F5R2_FB14_Pos      (14U)
#define CAN_F5R2_FB14_Msk      (0x1UL << CAN_F5R2_FB14_Pos)                     /*!< 0x00004000 */
#define CAN_F5R2_FB14          CAN_F5R2_FB14_Msk                               /*!<Filter bit 14 */
#define CAN_F5R2_FB15_Pos      (15U)
#define CAN_F5R2_FB15_Msk      (0x1UL << CAN_F5R2_FB15_Pos)                     /*!< 0x00008000 */
#define CAN_F5R2_FB15          CAN_F5R2_FB15_Msk                               /*!<Filter bit 15 */
#define CAN_F5R2_FB16_Pos      (16U)
#define CAN_F5R2_FB16_Msk      (0x1UL << CAN_F5R2_FB16_Pos)                     /*!< 0x00010000 */
#define CAN_F5R2_FB16          CAN_F5R2_FB16_Msk                               /*!<Filter bit 16 */
#define CAN_F5R2_FB17_Pos      (17U)
#define CAN_F5R2_FB17_Msk      (0x1UL << CAN_F5R2_FB17_Pos)                     /*!< 0x00020000 */
#define CAN_F5R2_FB17          CAN_F5R2_FB17_Msk                               /*!<Filter bit 17 */
#define CAN_F5R2_FB18_Pos      (18U)
#define CAN_F5R2_FB18_Msk      (0x1UL << CAN_F5R2_FB18_Pos)                     /*!< 0x00040000 */
#define CAN_F5R2_FB18          CAN_F5R2_FB18_Msk                               /*!<Filter bit 18 */
#define CAN_F5R2_FB19_Pos      (19U)
#define CAN_F5R2_FB19_Msk      (0x1UL << CAN_F5R2_FB19_Pos)                     /*!< 0x00080000 */
#define CAN_F5R2_FB19          CAN_F5R2_FB19_Msk                               /*!<Filter bit 19 */
#define CAN_F5R2_FB20_Pos      (20U)
#define CAN_F5R2_FB20_Msk      (0x1UL << CAN_F5R2_FB20_Pos)                     /*!< 0x00100000 */
#define CAN_F5R2_FB20          CAN_F5R2_FB20_Msk                               /*!<Filter bit 20 */
#define CAN_F5R2_FB21_Pos      (21U)
#define CAN_F5R2_FB21_Msk      (0x1UL << CAN_F5R2_FB21_Pos)                     /*!< 0x00200000 */
#define CAN_F5R2_FB21          CAN_F5R2_FB21_Msk                               /*!<Filter bit 21 */
#define CAN_F5R2_FB22_Pos      (22U)
#define CAN_F5R2_FB22_Msk      (0x1UL << CAN_F5R2_FB22_Pos)                     /*!< 0x00400000 */
#define CAN_F5R2_FB22          CAN_F5R2_FB22_Msk                               /*!<Filter bit 22 */
#define CAN_F5R2_FB23_Pos      (23U)
#define CAN_F5R2_FB23_Msk      (0x1UL << CAN_F5R2_FB23_Pos)                     /*!< 0x00800000 */
#define CAN_F5R2_FB23          CAN_F5R2_FB23_Msk                               /*!<Filter bit 23 */
#define CAN_F5R2_FB24_Pos      (24U)
#define CAN_F5R2_FB24_Msk      (0x1UL << CAN_F5R2_FB24_Pos)                     /*!< 0x01000000 */
#define CAN_F5R2_FB24          CAN_F5R2_FB24_Msk                               /*!<Filter bit 24 */
#define CAN_F5R2_FB25_Pos      (25U)
#define CAN_F5R2_FB25_Msk      (0x1UL << CAN_F5R2_FB25_Pos)                     /*!< 0x02000000 */
#define CAN_F5R2_FB25          CAN_F5R2_FB25_Msk                               /*!<Filter bit 25 */
#define CAN_F5R2_FB26_Pos      (26U)
#define CAN_F5R2_FB26_Msk      (0x1UL << CAN_F5R2_FB26_Pos)                     /*!< 0x04000000 */
#define CAN_F5R2_FB26          CAN_F5R2_FB26_Msk                               /*!<Filter bit 26 */
#define CAN_F5R2_FB27_Pos      (27U)
#define CAN_F5R2_FB27_Msk      (0x1UL << CAN_F5R2_FB27_Pos)                     /*!< 0x08000000 */
#define CAN_F5R2_FB27          CAN_F5R2_FB27_Msk                               /*!<Filter bit 27 */
#define CAN_F5R2_FB28_Pos      (28U)
#define CAN_F5R2_FB28_Msk      (0x1UL << CAN_F5R2_FB28_Pos)                     /*!< 0x10000000 */
#define CAN_F5R2_FB28          CAN_F5R2_FB28_Msk                               /*!<Filter bit 28 */
#define CAN_F5R2_FB29_Pos      (29U)
#define CAN_F5R2_FB29_Msk      (0x1UL << CAN_F5R2_FB29_Pos)                     /*!< 0x20000000 */
#define CAN_F5R2_FB29          CAN_F5R2_FB29_Msk                               /*!<Filter bit 29 */
#define CAN_F5R2_FB30_Pos      (30U)
#define CAN_F5R2_FB30_Msk      (0x1UL << CAN_F5R2_FB30_Pos)                     /*!< 0x40000000 */
#define CAN_F5R2_FB30          CAN_F5R2_FB30_Msk                               /*!<Filter bit 30 */
#define CAN_F5R2_FB31_Pos      (31U)
#define CAN_F5R2_FB31_Msk      (0x1UL << CAN_F5R2_FB31_Pos)                     /*!< 0x80000000 */
#define CAN_F5R2_FB31          CAN_F5R2_FB31_Msk                               /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F6R2 register  *******************/
#define CAN_F6R2_FB0_Pos       (0U)
#define CAN_F6R2_FB0_Msk       (0x1UL << CAN_F6R2_FB0_Pos)                      /*!< 0x00000001 */
#define CAN_F6R2_FB0           CAN_F6R2_FB0_Msk                                /*!<Filter bit 0 */
#define CAN_F6R2_FB1_Pos       (1U)
#define CAN_F6R2_FB1_Msk       (0x1UL << CAN_F6R2_FB1_Pos)                      /*!< 0x00000002 */
#define CAN_F6R2_FB1           CAN_F6R2_FB1_Msk                                /*!<Filter bit 1 */
#define CAN_F6R2_FB2_Pos       (2U)
#define CAN_F6R2_FB2_Msk       (0x1UL << CAN_F6R2_FB2_Pos)                      /*!< 0x00000004 */
#define CAN_F6R2_FB2           CAN_F6R2_FB2_Msk                                /*!<Filter bit 2 */
#define CAN_F6R2_FB3_Pos       (3U)
#define CAN_F6R2_FB3_Msk       (0x1UL << CAN_F6R2_FB3_Pos)                      /*!< 0x00000008 */
#define CAN_F6R2_FB3           CAN_F6R2_FB3_Msk                                /*!<Filter bit 3 */
#define CAN_F6R2_FB4_Pos       (4U)
#define CAN_F6R2_FB4_Msk       (0x1UL << CAN_F6R2_FB4_Pos)                      /*!< 0x00000010 */
#define CAN_F6R2_FB4           CAN_F6R2_FB4_Msk                                /*!<Filter bit 4 */
#define CAN_F6R2_FB5_Pos       (5U)
#define CAN_F6R2_FB5_Msk       (0x1UL << CAN_F6R2_FB5_Pos)                      /*!< 0x00000020 */
#define CAN_F6R2_FB5           CAN_F6R2_FB5_Msk                                /*!<Filter bit 5 */
#define CAN_F6R2_FB6_Pos       (6U)
#define CAN_F6R2_FB6_Msk       (0x1UL << CAN_F6R2_FB6_Pos)                      /*!< 0x00000040 */
#define CAN_F6R2_FB6           CAN_F6R2_FB6_Msk                                /*!<Filter bit 6 */
#define CAN_F6R2_FB7_Pos       (7U)
#define CAN_F6R2_FB7_Msk       (0x1UL << CAN_F6R2_FB7_Pos)                      /*!< 0x00000080 */
#define CAN_F6R2_FB7           CAN_F6R2_FB7_Msk                                /*!<Filter bit 7 */
#define CAN_F6R2_FB8_Pos       (8U)
#define CAN_F6R2_FB8_Msk       (0x1UL << CAN_F6R2_FB8_Pos)                      /*!< 0x00000100 */
#define CAN_F6R2_FB8           CAN_F6R2_FB8_Msk                                /*!<Filter bit 8 */
#define CAN_F6R2_FB9_Pos       (9U)
#define CAN_F6R2_FB9_Msk       (0x1UL << CAN_F6R2_FB9_Pos)                      /*!< 0x00000200 */
#define CAN_F6R2_FB9           CAN_F6R2_FB9_Msk                                /*!<Filter bit 9 */
#define CAN_F6R2_FB10_Pos      (10U)
#define CAN_F6R2_FB10_Msk      (0x1UL << CAN_F6R2_FB10_Pos)                     /*!< 0x00000400 */
#define CAN_F6R2_FB10          CAN_F6R2_FB10_Msk                               /*!<Filter bit 10 */
#define CAN_F6R2_FB11_Pos      (11U)
#define CAN_F6R2_FB11_Msk      (0x1UL << CAN_F6R2_FB11_Pos)                     /*!< 0x00000800 */
#define CAN_F6R2_FB11          CAN_F6R2_FB11_Msk                               /*!<Filter bit 11 */
#define CAN_F6R2_FB12_Pos      (12U)
#define CAN_F6R2_FB12_Msk      (0x1UL << CAN_F6R2_FB12_Pos)                     /*!< 0x00001000 */
#define CAN_F6R2_FB12          CAN_F6R2_FB12_Msk                               /*!<Filter bit 12 */
#define CAN_F6R2_FB13_Pos      (13U)
#define CAN_F6R2_FB13_Msk      (0x1UL << CAN_F6R2_FB13_Pos)                     /*!< 0x00002000 */
#define CAN_F6R2_FB13          CAN_F6R2_FB13_Msk                               /*!<Filter bit 13 */
#define CAN_F6R2_FB14_Pos      (14U)
#define CAN_F6R2_FB14_Msk      (0x1UL << CAN_F6R2_FB14_Pos)                     /*!< 0x00004000 */
#define CAN_F6R2_FB14          CAN_F6R2_FB14_Msk                               /*!<Filter bit 14 */
#define CAN_F6R2_FB15_Pos      (15U)
#define CAN_F6R2_FB15_Msk      (0x1UL << CAN_F6R2_FB15_Pos)                     /*!< 0x00008000 */
#define CAN_F6R2_FB15          CAN_F6R2_FB15_Msk                               /*!<Filter bit 15 */
#define CAN_F6R2_FB16_Pos      (16U)
#define CAN_F6R2_FB16_Msk      (0x1UL << CAN_F6R2_FB16_Pos)                     /*!< 0x00010000 */
#define CAN_F6R2_FB16          CAN_F6R2_FB16_Msk                               /*!<Filter bit 16 */
#define CAN_F6R2_FB17_Pos      (17U)
#define CAN_F6R2_FB17_Msk      (0x1UL << CAN_F6R2_FB17_Pos)                     /*!< 0x00020000 */
#define CAN_F6R2_FB17          CAN_F6R2_FB17_Msk                               /*!<Filter bit 17 */
#define CAN_F6R2_FB18_Pos      (18U)
#define CAN_F6R2_FB18_Msk      (0x1UL << CAN_F6R2_FB18_Pos)                     /*!< 0x00040000 */
#define CAN_F6R2_FB18          CAN_F6R2_FB18_Msk                               /*!<Filter bit 18 */
#define CAN_F6R2_FB19_Pos      (19U)
#define CAN_F6R2_FB19_Msk      (0x1UL << CAN_F6R2_FB19_Pos)                     /*!< 0x00080000 */
#define CAN_F6R2_FB19          CAN_F6R2_FB19_Msk                               /*!<Filter bit 19 */
#define CAN_F6R2_FB20_Pos      (20U)
#define CAN_F6R2_FB20_Msk      (0x1UL << CAN_F6R2_FB20_Pos)                     /*!< 0x00100000 */
#define CAN_F6R2_FB20          CAN_F6R2_FB20_Msk                               /*!<Filter bit 20 */
#define CAN_F6R2_FB21_Pos      (21U)
#define CAN_F6R2_FB21_Msk      (0x1UL << CAN_F6R2_FB21_Pos)                     /*!< 0x00200000 */
#define CAN_F6R2_FB21          CAN_F6R2_FB21_Msk                               /*!<Filter bit 21 */
#define CAN_F6R2_FB22_Pos      (22U)
#define CAN_F6R2_FB22_Msk      (0x1UL << CAN_F6R2_FB22_Pos)                     /*!< 0x00400000 */
#define CAN_F6R2_FB22          CAN_F6R2_FB22_Msk                               /*!<Filter bit 22 */
#define CAN_F6R2_FB23_Pos      (23U)
#define CAN_F6R2_FB23_Msk      (0x1UL << CAN_F6R2_FB23_Pos)                     /*!< 0x00800000 */
#define CAN_F6R2_FB23          CAN_F6R2_FB23_Msk                               /*!<Filter bit 23 */
#define CAN_F6R2_FB24_Pos      (24U)
#define CAN_F6R2_FB24_Msk      (0x1UL << CAN_F6R2_FB24_Pos)                     /*!< 0x01000000 */
#define CAN_F6R2_FB24          CAN_F6R2_FB24_Msk                               /*!<Filter bit 24 */
#define CAN_F6R2_FB25_Pos      (25U)
#define CAN_F6R2_FB25_Msk      (0x1UL << CAN_F6R2_FB25_Pos)                     /*!< 0x02000000 */
#define CAN_F6R2_FB25          CAN_F6R2_FB25_Msk                               /*!<Filter bit 25 */
#define CAN_F6R2_FB26_Pos      (26U)
#define CAN_F6R2_FB26_Msk      (0x1UL << CAN_F6R2_FB26_Pos)                     /*!< 0x04000000 */
#define CAN_F6R2_FB26          CAN_F6R2_FB26_Msk                               /*!<Filter bit 26 */
#define CAN_F6R2_FB27_Pos      (27U)
#define CAN_F6R2_FB27_Msk      (0x1UL << CAN_F6R2_FB27_Pos)                     /*!< 0x08000000 */
#define CAN_F6R2_FB27          CAN_F6R2_FB27_Msk                               /*!<Filter bit 27 */
#define CAN_F6R2_FB28_Pos      (28U)
#define CAN_F6R2_FB28_Msk      (0x1UL << CAN_F6R2_FB28_Pos)                     /*!< 0x10000000 */
#define CAN_F6R2_FB28          CAN_F6R2_FB28_Msk                               /*!<Filter bit 28 */
#define CAN_F6R2_FB29_Pos      (29U)
#define CAN_F6R2_FB29_Msk      (0x1UL << CAN_F6R2_FB29_Pos)                     /*!< 0x20000000 */
#define CAN_F6R2_FB29          CAN_F6R2_FB29_Msk                               /*!<Filter bit 29 */
#define CAN_F6R2_FB30_Pos      (30U)
#define CAN_F6R2_FB30_Msk      (0x1UL << CAN_F6R2_FB30_Pos)                     /*!< 0x40000000 */
#define CAN_F6R2_FB30          CAN_F6R2_FB30_Msk                               /*!<Filter bit 30 */
#define CAN_F6R2_FB31_Pos      (31U)
#define CAN_F6R2_FB31_Msk      (0x1UL << CAN_F6R2_FB31_Pos)                     /*!< 0x80000000 */
#define CAN_F6R2_FB31          CAN_F6R2_FB31_Msk                               /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F7R2 register  *******************/
#define CAN_F7R2_FB0_Pos       (0U)
#define CAN_F7R2_FB0_Msk       (0x1UL << CAN_F7R2_FB0_Pos)                      /*!< 0x00000001 */
#define CAN_F7R2_FB0           CAN_F7R2_FB0_Msk                                /*!<Filter bit 0 */
#define CAN_F7R2_FB1_Pos       (1U)
#define CAN_F7R2_FB1_Msk       (0x1UL << CAN_F7R2_FB1_Pos)                      /*!< 0x00000002 */
#define CAN_F7R2_FB1           CAN_F7R2_FB1_Msk                                /*!<Filter bit 1 */
#define CAN_F7R2_FB2_Pos       (2U)
#define CAN_F7R2_FB2_Msk       (0x1UL << CAN_F7R2_FB2_Pos)                      /*!< 0x00000004 */
#define CAN_F7R2_FB2           CAN_F7R2_FB2_Msk                                /*!<Filter bit 2 */
#define CAN_F7R2_FB3_Pos       (3U)
#define CAN_F7R2_FB3_Msk       (0x1UL << CAN_F7R2_FB3_Pos)                      /*!< 0x00000008 */
#define CAN_F7R2_FB3           CAN_F7R2_FB3_Msk                                /*!<Filter bit 3 */
#define CAN_F7R2_FB4_Pos       (4U)
#define CAN_F7R2_FB4_Msk       (0x1UL << CAN_F7R2_FB4_Pos)                      /*!< 0x00000010 */
#define CAN_F7R2_FB4           CAN_F7R2_FB4_Msk                                /*!<Filter bit 4 */
#define CAN_F7R2_FB5_Pos       (5U)
#define CAN_F7R2_FB5_Msk       (0x1UL << CAN_F7R2_FB5_Pos)                      /*!< 0x00000020 */
#define CAN_F7R2_FB5           CAN_F7R2_FB5_Msk                                /*!<Filter bit 5 */
#define CAN_F7R2_FB6_Pos       (6U)
#define CAN_F7R2_FB6_Msk       (0x1UL << CAN_F7R2_FB6_Pos)                      /*!< 0x00000040 */
#define CAN_F7R2_FB6           CAN_F7R2_FB6_Msk                                /*!<Filter bit 6 */
#define CAN_F7R2_FB7_Pos       (7U)
#define CAN_F7R2_FB7_Msk       (0x1UL << CAN_F7R2_FB7_Pos)                      /*!< 0x00000080 */
#define CAN_F7R2_FB7           CAN_F7R2_FB7_Msk                                /*!<Filter bit 7 */
#define CAN_F7R2_FB8_Pos       (8U)
#define CAN_F7R2_FB8_Msk       (0x1UL << CAN_F7R2_FB8_Pos)                      /*!< 0x00000100 */
#define CAN_F7R2_FB8           CAN_F7R2_FB8_Msk                                /*!<Filter bit 8 */
#define CAN_F7R2_FB9_Pos       (9U)
#define CAN_F7R2_FB9_Msk       (0x1UL << CAN_F7R2_FB9_Pos)                      /*!< 0x00000200 */
#define CAN_F7R2_FB9           CAN_F7R2_FB9_Msk                                /*!<Filter bit 9 */
#define CAN_F7R2_FB10_Pos      (10U)
#define CAN_F7R2_FB10_Msk      (0x1UL << CAN_F7R2_FB10_Pos)                     /*!< 0x00000400 */
#define CAN_F7R2_FB10          CAN_F7R2_FB10_Msk                               /*!<Filter bit 10 */
#define CAN_F7R2_FB11_Pos      (11U)
#define CAN_F7R2_FB11_Msk      (0x1UL << CAN_F7R2_FB11_Pos)                     /*!< 0x00000800 */
#define CAN_F7R2_FB11          CAN_F7R2_FB11_Msk                               /*!<Filter bit 11 */
#define CAN_F7R2_FB12_Pos      (12U)
#define CAN_F7R2_FB12_Msk      (0x1UL << CAN_F7R2_FB12_Pos)                     /*!< 0x00001000 */
#define CAN_F7R2_FB12          CAN_F7R2_FB12_Msk                               /*!<Filter bit 12 */
#define CAN_F7R2_FB13_Pos      (13U)
#define CAN_F7R2_FB13_Msk      (0x1UL << CAN_F7R2_FB13_Pos)                     /*!< 0x00002000 */
#define CAN_F7R2_FB13          CAN_F7R2_FB13_Msk                               /*!<Filter bit 13 */
#define CAN_F7R2_FB14_Pos      (14U)
#define CAN_F7R2_FB14_Msk      (0x1UL << CAN_F7R2_FB14_Pos)                     /*!< 0x00004000 */
#define CAN_F7R2_FB14          CAN_F7R2_FB14_Msk                               /*!<Filter bit 14 */
#define CAN_F7R2_FB15_Pos      (15U)
#define CAN_F7R2_FB15_Msk      (0x1UL << CAN_F7R2_FB15_Pos)                     /*!< 0x00008000 */
#define CAN_F7R2_FB15          CAN_F7R2_FB15_Msk                               /*!<Filter bit 15 */
#define CAN_F7R2_FB16_Pos      (16U)
#define CAN_F7R2_FB16_Msk      (0x1UL << CAN_F7R2_FB16_Pos)                     /*!< 0x00010000 */
#define CAN_F7R2_FB16          CAN_F7R2_FB16_Msk                               /*!<Filter bit 16 */
#define CAN_F7R2_FB17_Pos      (17U)
#define CAN_F7R2_FB17_Msk      (0x1UL << CAN_F7R2_FB17_Pos)                     /*!< 0x00020000 */
#define CAN_F7R2_FB17          CAN_F7R2_FB17_Msk                               /*!<Filter bit 17 */
#define CAN_F7R2_FB18_Pos      (18U)
#define CAN_F7R2_FB18_Msk      (0x1UL << CAN_F7R2_FB18_Pos)                     /*!< 0x00040000 */
#define CAN_F7R2_FB18          CAN_F7R2_FB18_Msk                               /*!<Filter bit 18 */
#define CAN_F7R2_FB19_Pos      (19U)
#define CAN_F7R2_FB19_Msk      (0x1UL << CAN_F7R2_FB19_Pos)                     /*!< 0x00080000 */
#define CAN_F7R2_FB19          CAN_F7R2_FB19_Msk                               /*!<Filter bit 19 */
#define CAN_F7R2_FB20_Pos      (20U)
#define CAN_F7R2_FB20_Msk      (0x1UL << CAN_F7R2_FB20_Pos)                     /*!< 0x00100000 */
#define CAN_F7R2_FB20          CAN_F7R2_FB20_Msk                               /*!<Filter bit 20 */
#define CAN_F7R2_FB21_Pos      (21U)
#define CAN_F7R2_FB21_Msk      (0x1UL << CAN_F7R2_FB21_Pos)                     /*!< 0x00200000 */
#define CAN_F7R2_FB21          CAN_F7R2_FB21_Msk                               /*!<Filter bit 21 */
#define CAN_F7R2_FB22_Pos      (22U)
#define CAN_F7R2_FB22_Msk      (0x1UL << CAN_F7R2_FB22_Pos)                     /*!< 0x00400000 */
#define CAN_F7R2_FB22          CAN_F7R2_FB22_Msk                               /*!<Filter bit 22 */
#define CAN_F7R2_FB23_Pos      (23U)
#define CAN_F7R2_FB23_Msk      (0x1UL << CAN_F7R2_FB23_Pos)                     /*!< 0x00800000 */
#define CAN_F7R2_FB23          CAN_F7R2_FB23_Msk                               /*!<Filter bit 23 */
#define CAN_F7R2_FB24_Pos      (24U)
#define CAN_F7R2_FB24_Msk      (0x1UL << CAN_F7R2_FB24_Pos)                     /*!< 0x01000000 */
#define CAN_F7R2_FB24          CAN_F7R2_FB24_Msk                               /*!<Filter bit 24 */
#define CAN_F7R2_FB25_Pos      (25U)
#define CAN_F7R2_FB25_Msk      (0x1UL << CAN_F7R2_FB25_Pos)                     /*!< 0x02000000 */
#define CAN_F7R2_FB25          CAN_F7R2_FB25_Msk                               /*!<Filter bit 25 */
#define CAN_F7R2_FB26_Pos      (26U)
#define CAN_F7R2_FB26_Msk      (0x1UL << CAN_F7R2_FB26_Pos)                     /*!< 0x04000000 */
#define CAN_F7R2_FB26          CAN_F7R2_FB26_Msk                               /*!<Filter bit 26 */
#define CAN_F7R2_FB27_Pos      (27U)
#define CAN_F7R2_FB27_Msk      (0x1UL << CAN_F7R2_FB27_Pos)                     /*!< 0x08000000 */
#define CAN_F7R2_FB27          CAN_F7R2_FB27_Msk                               /*!<Filter bit 27 */
#define CAN_F7R2_FB28_Pos      (28U)
#define CAN_F7R2_FB28_Msk      (0x1UL << CAN_F7R2_FB28_Pos)                     /*!< 0x10000000 */
#define CAN_F7R2_FB28          CAN_F7R2_FB28_Msk                               /*!<Filter bit 28 */
#define CAN_F7R2_FB29_Pos      (29U)
#define CAN_F7R2_FB29_Msk      (0x1UL << CAN_F7R2_FB29_Pos)                     /*!< 0x20000000 */
#define CAN_F7R2_FB29          CAN_F7R2_FB29_Msk                               /*!<Filter bit 29 */
#define CAN_F7R2_FB30_Pos      (30U)
#define CAN_F7R2_FB30_Msk      (0x1UL << CAN_F7R2_FB30_Pos)                     /*!< 0x40000000 */
#define CAN_F7R2_FB30          CAN_F7R2_FB30_Msk                               /*!<Filter bit 30 */
#define CAN_F7R2_FB31_Pos      (31U)
#define CAN_F7R2_FB31_Msk      (0x1UL << CAN_F7R2_FB31_Pos)                     /*!< 0x80000000 */
#define CAN_F7R2_FB31          CAN_F7R2_FB31_Msk                               /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F8R2 register  *******************/
#define CAN_F8R2_FB0_Pos       (0U)
#define CAN_F8R2_FB0_Msk       (0x1UL << CAN_F8R2_FB0_Pos)                      /*!< 0x00000001 */
#define CAN_F8R2_FB0           CAN_F8R2_FB0_Msk                                /*!<Filter bit 0 */
#define CAN_F8R2_FB1_Pos       (1U)
#define CAN_F8R2_FB1_Msk       (0x1UL << CAN_F8R2_FB1_Pos)                      /*!< 0x00000002 */
#define CAN_F8R2_FB1           CAN_F8R2_FB1_Msk                                /*!<Filter bit 1 */
#define CAN_F8R2_FB2_Pos       (2U)
#define CAN_F8R2_FB2_Msk       (0x1UL << CAN_F8R2_FB2_Pos)                      /*!< 0x00000004 */
#define CAN_F8R2_FB2           CAN_F8R2_FB2_Msk                                /*!<Filter bit 2 */
#define CAN_F8R2_FB3_Pos       (3U)
#define CAN_F8R2_FB3_Msk       (0x1UL << CAN_F8R2_FB3_Pos)                      /*!< 0x00000008 */
#define CAN_F8R2_FB3           CAN_F8R2_FB3_Msk                                /*!<Filter bit 3 */
#define CAN_F8R2_FB4_Pos       (4U)
#define CAN_F8R2_FB4_Msk       (0x1UL << CAN_F8R2_FB4_Pos)                      /*!< 0x00000010 */
#define CAN_F8R2_FB4           CAN_F8R2_FB4_Msk                                /*!<Filter bit 4 */
#define CAN_F8R2_FB5_Pos       (5U)
#define CAN_F8R2_FB5_Msk       (0x1UL << CAN_F8R2_FB5_Pos)                      /*!< 0x00000020 */
#define CAN_F8R2_FB5           CAN_F8R2_FB5_Msk                                /*!<Filter bit 5 */
#define CAN_F8R2_FB6_Pos       (6U)
#define CAN_F8R2_FB6_Msk       (0x1UL << CAN_F8R2_FB6_Pos)                      /*!< 0x00000040 */
#define CAN_F8R2_FB6           CAN_F8R2_FB6_Msk                                /*!<Filter bit 6 */
#define CAN_F8R2_FB7_Pos       (7U)
#define CAN_F8R2_FB7_Msk       (0x1UL << CAN_F8R2_FB7_Pos)                      /*!< 0x00000080 */
#define CAN_F8R2_FB7           CAN_F8R2_FB7_Msk                                /*!<Filter bit 7 */
#define CAN_F8R2_FB8_Pos       (8U)
#define CAN_F8R2_FB8_Msk       (0x1UL << CAN_F8R2_FB8_Pos)                      /*!< 0x00000100 */
#define CAN_F8R2_FB8           CAN_F8R2_FB8_Msk                                /*!<Filter bit 8 */
#define CAN_F8R2_FB9_Pos       (9U)
#define CAN_F8R2_FB9_Msk       (0x1UL << CAN_F8R2_FB9_Pos)                      /*!< 0x00000200 */
#define CAN_F8R2_FB9           CAN_F8R2_FB9_Msk                                /*!<Filter bit 9 */
#define CAN_F8R2_FB10_Pos      (10U)
#define CAN_F8R2_FB10_Msk      (0x1UL << CAN_F8R2_FB10_Pos)                     /*!< 0x00000400 */
#define CAN_F8R2_FB10          CAN_F8R2_FB10_Msk                               /*!<Filter bit 10 */
#define CAN_F8R2_FB11_Pos      (11U)
#define CAN_F8R2_FB11_Msk      (0x1UL << CAN_F8R2_FB11_Pos)                     /*!< 0x00000800 */
#define CAN_F8R2_FB11          CAN_F8R2_FB11_Msk                               /*!<Filter bit 11 */
#define CAN_F8R2_FB12_Pos      (12U)
#define CAN_F8R2_FB12_Msk      (0x1UL << CAN_F8R2_FB12_Pos)                     /*!< 0x00001000 */
#define CAN_F8R2_FB12          CAN_F8R2_FB12_Msk                               /*!<Filter bit 12 */
#define CAN_F8R2_FB13_Pos      (13U)
#define CAN_F8R2_FB13_Msk      (0x1UL << CAN_F8R2_FB13_Pos)                     /*!< 0x00002000 */
#define CAN_F8R2_FB13          CAN_F8R2_FB13_Msk                               /*!<Filter bit 13 */
#define CAN_F8R2_FB14_Pos      (14U)
#define CAN_F8R2_FB14_Msk      (0x1UL << CAN_F8R2_FB14_Pos)                     /*!< 0x00004000 */
#define CAN_F8R2_FB14          CAN_F8R2_FB14_Msk                               /*!<Filter bit 14 */
#define CAN_F8R2_FB15_Pos      (15U)
#define CAN_F8R2_FB15_Msk      (0x1UL << CAN_F8R2_FB15_Pos)                     /*!< 0x00008000 */
#define CAN_F8R2_FB15          CAN_F8R2_FB15_Msk                               /*!<Filter bit 15 */
#define CAN_F8R2_FB16_Pos      (16U)
#define CAN_F8R2_FB16_Msk      (0x1UL << CAN_F8R2_FB16_Pos)                     /*!< 0x00010000 */
#define CAN_F8R2_FB16          CAN_F8R2_FB16_Msk                               /*!<Filter bit 16 */
#define CAN_F8R2_FB17_Pos      (17U)
#define CAN_F8R2_FB17_Msk      (0x1UL << CAN_F8R2_FB17_Pos)                     /*!< 0x00020000 */
#define CAN_F8R2_FB17          CAN_F8R2_FB17_Msk                               /*!<Filter bit 17 */
#define CAN_F8R2_FB18_Pos      (18U)
#define CAN_F8R2_FB18_Msk      (0x1UL << CAN_F8R2_FB18_Pos)                     /*!< 0x00040000 */
#define CAN_F8R2_FB18          CAN_F8R2_FB18_Msk                               /*!<Filter bit 18 */
#define CAN_F8R2_FB19_Pos      (19U)
#define CAN_F8R2_FB19_Msk      (0x1UL << CAN_F8R2_FB19_Pos)                     /*!< 0x00080000 */
#define CAN_F8R2_FB19          CAN_F8R2_FB19_Msk                               /*!<Filter bit 19 */
#define CAN_F8R2_FB20_Pos      (20U)
#define CAN_F8R2_FB20_Msk      (0x1UL << CAN_F8R2_FB20_Pos)                     /*!< 0x00100000 */
#define CAN_F8R2_FB20          CAN_F8R2_FB20_Msk                               /*!<Filter bit 20 */
#define CAN_F8R2_FB21_Pos      (21U)
#define CAN_F8R2_FB21_Msk      (0x1UL << CAN_F8R2_FB21_Pos)                     /*!< 0x00200000 */
#define CAN_F8R2_FB21          CAN_F8R2_FB21_Msk                               /*!<Filter bit 21 */
#define CAN_F8R2_FB22_Pos      (22U)
#define CAN_F8R2_FB22_Msk      (0x1UL << CAN_F8R2_FB22_Pos)                     /*!< 0x00400000 */
#define CAN_F8R2_FB22          CAN_F8R2_FB22_Msk                               /*!<Filter bit 22 */
#define CAN_F8R2_FB23_Pos      (23U)
#define CAN_F8R2_FB23_Msk      (0x1UL << CAN_F8R2_FB23_Pos)                     /*!< 0x00800000 */
#define CAN_F8R2_FB23          CAN_F8R2_FB23_Msk                               /*!<Filter bit 23 */
#define CAN_F8R2_FB24_Pos      (24U)
#define CAN_F8R2_FB24_Msk      (0x1UL << CAN_F8R2_FB24_Pos)                     /*!< 0x01000000 */
#define CAN_F8R2_FB24          CAN_F8R2_FB24_Msk                               /*!<Filter bit 24 */
#define CAN_F8R2_FB25_Pos      (25U)
#define CAN_F8R2_FB25_Msk      (0x1UL << CAN_F8R2_FB25_Pos)                     /*!< 0x02000000 */
#define CAN_F8R2_FB25          CAN_F8R2_FB25_Msk                               /*!<Filter bit 25 */
#define CAN_F8R2_FB26_Pos      (26U)
#define CAN_F8R2_FB26_Msk      (0x1UL << CAN_F8R2_FB26_Pos)                     /*!< 0x04000000 */
#define CAN_F8R2_FB26          CAN_F8R2_FB26_Msk                               /*!<Filter bit 26 */
#define CAN_F8R2_FB27_Pos      (27U)
#define CAN_F8R2_FB27_Msk      (0x1UL << CAN_F8R2_FB27_Pos)                     /*!< 0x08000000 */
#define CAN_F8R2_FB27          CAN_F8R2_FB27_Msk                               /*!<Filter bit 27 */
#define CAN_F8R2_FB28_Pos      (28U)
#define CAN_F8R2_FB28_Msk      (0x1UL << CAN_F8R2_FB28_Pos)                     /*!< 0x10000000 */
#define CAN_F8R2_FB28          CAN_F8R2_FB28_Msk                               /*!<Filter bit 28 */
#define CAN_F8R2_FB29_Pos      (29U)
#define CAN_F8R2_FB29_Msk      (0x1UL << CAN_F8R2_FB29_Pos)                     /*!< 0x20000000 */
#define CAN_F8R2_FB29          CAN_F8R2_FB29_Msk                               /*!<Filter bit 29 */
#define CAN_F8R2_FB30_Pos      (30U)
#define CAN_F8R2_FB30_Msk      (0x1UL << CAN_F8R2_FB30_Pos)                     /*!< 0x40000000 */
#define CAN_F8R2_FB30          CAN_F8R2_FB30_Msk                               /*!<Filter bit 30 */
#define CAN_F8R2_FB31_Pos      (31U)
#define CAN_F8R2_FB31_Msk      (0x1UL << CAN_F8R2_FB31_Pos)                     /*!< 0x80000000 */
#define CAN_F8R2_FB31          CAN_F8R2_FB31_Msk                               /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F9R2 register  *******************/
#define CAN_F9R2_FB0_Pos       (0U)
#define CAN_F9R2_FB0_Msk       (0x1UL << CAN_F9R2_FB0_Pos)                      /*!< 0x00000001 */
#define CAN_F9R2_FB0           CAN_F9R2_FB0_Msk                                /*!<Filter bit 0 */
#define CAN_F9R2_FB1_Pos       (1U)
#define CAN_F9R2_FB1_Msk       (0x1UL << CAN_F9R2_FB1_Pos)                      /*!< 0x00000002 */
#define CAN_F9R2_FB1           CAN_F9R2_FB1_Msk                                /*!<Filter bit 1 */
#define CAN_F9R2_FB2_Pos       (2U)
#define CAN_F9R2_FB2_Msk       (0x1UL << CAN_F9R2_FB2_Pos)                      /*!< 0x00000004 */
#define CAN_F9R2_FB2           CAN_F9R2_FB2_Msk                                /*!<Filter bit 2 */
#define CAN_F9R2_FB3_Pos       (3U)
#define CAN_F9R2_FB3_Msk       (0x1UL << CAN_F9R2_FB3_Pos)                      /*!< 0x00000008 */
#define CAN_F9R2_FB3           CAN_F9R2_FB3_Msk                                /*!<Filter bit 3 */
#define CAN_F9R2_FB4_Pos       (4U)
#define CAN_F9R2_FB4_Msk       (0x1UL << CAN_F9R2_FB4_Pos)                      /*!< 0x00000010 */
#define CAN_F9R2_FB4           CAN_F9R2_FB4_Msk                                /*!<Filter bit 4 */
#define CAN_F9R2_FB5_Pos       (5U)
#define CAN_F9R2_FB5_Msk       (0x1UL << CAN_F9R2_FB5_Pos)                      /*!< 0x00000020 */
#define CAN_F9R2_FB5           CAN_F9R2_FB5_Msk                                /*!<Filter bit 5 */
#define CAN_F9R2_FB6_Pos       (6U)
#define CAN_F9R2_FB6_Msk       (0x1UL << CAN_F9R2_FB6_Pos)                      /*!< 0x00000040 */
#define CAN_F9R2_FB6           CAN_F9R2_FB6_Msk                                /*!<Filter bit 6 */
#define CAN_F9R2_FB7_Pos       (7U)
#define CAN_F9R2_FB7_Msk       (0x1UL << CAN_F9R2_FB7_Pos)                      /*!< 0x00000080 */
#define CAN_F9R2_FB7           CAN_F9R2_FB7_Msk                                /*!<Filter bit 7 */
#define CAN_F9R2_FB8_Pos       (8U)
#define CAN_F9R2_FB8_Msk       (0x1UL << CAN_F9R2_FB8_Pos)                      /*!< 0x00000100 */
#define CAN_F9R2_FB8           CAN_F9R2_FB8_Msk                                /*!<Filter bit 8 */
#define CAN_F9R2_FB9_Pos       (9U)
#define CAN_F9R2_FB9_Msk       (0x1UL << CAN_F9R2_FB9_Pos)                      /*!< 0x00000200 */
#define CAN_F9R2_FB9           CAN_F9R2_FB9_Msk                                /*!<Filter bit 9 */
#define CAN_F9R2_FB10_Pos      (10U)
#define CAN_F9R2_FB10_Msk      (0x1UL << CAN_F9R2_FB10_Pos)                     /*!< 0x00000400 */
#define CAN_F9R2_FB10          CAN_F9R2_FB10_Msk                               /*!<Filter bit 10 */
#define CAN_F9R2_FB11_Pos      (11U)
#define CAN_F9R2_FB11_Msk      (0x1UL << CAN_F9R2_FB11_Pos)                     /*!< 0x00000800 */
#define CAN_F9R2_FB11          CAN_F9R2_FB11_Msk                               /*!<Filter bit 11 */
#define CAN_F9R2_FB12_Pos      (12U)
#define CAN_F9R2_FB12_Msk      (0x1UL << CAN_F9R2_FB12_Pos)                     /*!< 0x00001000 */
#define CAN_F9R2_FB12          CAN_F9R2_FB12_Msk                               /*!<Filter bit 12 */
#define CAN_F9R2_FB13_Pos      (13U)
#define CAN_F9R2_FB13_Msk      (0x1UL << CAN_F9R2_FB13_Pos)                     /*!< 0x00002000 */
#define CAN_F9R2_FB13          CAN_F9R2_FB13_Msk                               /*!<Filter bit 13 */
#define CAN_F9R2_FB14_Pos      (14U)
#define CAN_F9R2_FB14_Msk      (0x1UL << CAN_F9R2_FB14_Pos)                     /*!< 0x00004000 */
#define CAN_F9R2_FB14          CAN_F9R2_FB14_Msk                               /*!<Filter bit 14 */
#define CAN_F9R2_FB15_Pos      (15U)
#define CAN_F9R2_FB15_Msk      (0x1UL << CAN_F9R2_FB15_Pos)                     /*!< 0x00008000 */
#define CAN_F9R2_FB15          CAN_F9R2_FB15_Msk                               /*!<Filter bit 15 */
#define CAN_F9R2_FB16_Pos      (16U)
#define CAN_F9R2_FB16_Msk      (0x1UL << CAN_F9R2_FB16_Pos)                     /*!< 0x00010000 */
#define CAN_F9R2_FB16          CAN_F9R2_FB16_Msk                               /*!<Filter bit 16 */
#define CAN_F9R2_FB17_Pos      (17U)
#define CAN_F9R2_FB17_Msk      (0x1UL << CAN_F9R2_FB17_Pos)                     /*!< 0x00020000 */
#define CAN_F9R2_FB17          CAN_F9R2_FB17_Msk                               /*!<Filter bit 17 */
#define CAN_F9R2_FB18_Pos      (18U)
#define CAN_F9R2_FB18_Msk      (0x1UL << CAN_F9R2_FB18_Pos)                     /*!< 0x00040000 */
#define CAN_F9R2_FB18          CAN_F9R2_FB18_Msk                               /*!<Filter bit 18 */
#define CAN_F9R2_FB19_Pos      (19U)
#define CAN_F9R2_FB19_Msk      (0x1UL << CAN_F9R2_FB19_Pos)                     /*!< 0x00080000 */
#define CAN_F9R2_FB19          CAN_F9R2_FB19_Msk                               /*!<Filter bit 19 */
#define CAN_F9R2_FB20_Pos      (20U)
#define CAN_F9R2_FB20_Msk      (0x1UL << CAN_F9R2_FB20_Pos)                     /*!< 0x00100000 */
#define CAN_F9R2_FB20          CAN_F9R2_FB20_Msk                               /*!<Filter bit 20 */
#define CAN_F9R2_FB21_Pos      (21U)
#define CAN_F9R2_FB21_Msk      (0x1UL << CAN_F9R2_FB21_Pos)                     /*!< 0x00200000 */
#define CAN_F9R2_FB21          CAN_F9R2_FB21_Msk                               /*!<Filter bit 21 */
#define CAN_F9R2_FB22_Pos      (22U)
#define CAN_F9R2_FB22_Msk      (0x1UL << CAN_F9R2_FB22_Pos)                     /*!< 0x00400000 */
#define CAN_F9R2_FB22          CAN_F9R2_FB22_Msk                               /*!<Filter bit 22 */
#define CAN_F9R2_FB23_Pos      (23U)
#define CAN_F9R2_FB23_Msk      (0x1UL << CAN_F9R2_FB23_Pos)                     /*!< 0x00800000 */
#define CAN_F9R2_FB23          CAN_F9R2_FB23_Msk                               /*!<Filter bit 23 */
#define CAN_F9R2_FB24_Pos      (24U)
#define CAN_F9R2_FB24_Msk      (0x1UL << CAN_F9R2_FB24_Pos)                     /*!< 0x01000000 */
#define CAN_F9R2_FB24          CAN_F9R2_FB24_Msk                               /*!<Filter bit 24 */
#define CAN_F9R2_FB25_Pos      (25U)
#define CAN_F9R2_FB25_Msk      (0x1UL << CAN_F9R2_FB25_Pos)                     /*!< 0x02000000 */
#define CAN_F9R2_FB25          CAN_F9R2_FB25_Msk                               /*!<Filter bit 25 */
#define CAN_F9R2_FB26_Pos      (26U)
#define CAN_F9R2_FB26_Msk      (0x1UL << CAN_F9R2_FB26_Pos)                     /*!< 0x04000000 */
#define CAN_F9R2_FB26          CAN_F9R2_FB26_Msk                               /*!<Filter bit 26 */
#define CAN_F9R2_FB27_Pos      (27U)
#define CAN_F9R2_FB27_Msk      (0x1UL << CAN_F9R2_FB27_Pos)                     /*!< 0x08000000 */
#define CAN_F9R2_FB27          CAN_F9R2_FB27_Msk                               /*!<Filter bit 27 */
#define CAN_F9R2_FB28_Pos      (28U)
#define CAN_F9R2_FB28_Msk      (0x1UL << CAN_F9R2_FB28_Pos)                     /*!< 0x10000000 */
#define CAN_F9R2_FB28          CAN_F9R2_FB28_Msk                               /*!<Filter bit 28 */
#define CAN_F9R2_FB29_Pos      (29U)
#define CAN_F9R2_FB29_Msk      (0x1UL << CAN_F9R2_FB29_Pos)                     /*!< 0x20000000 */
#define CAN_F9R2_FB29          CAN_F9R2_FB29_Msk                               /*!<Filter bit 29 */
#define CAN_F9R2_FB30_Pos      (30U)
#define CAN_F9R2_FB30_Msk      (0x1UL << CAN_F9R2_FB30_Pos)                     /*!< 0x40000000 */
#define CAN_F9R2_FB30          CAN_F9R2_FB30_Msk                               /*!<Filter bit 30 */
#define CAN_F9R2_FB31_Pos      (31U)
#define CAN_F9R2_FB31_Msk      (0x1UL << CAN_F9R2_FB31_Pos)                     /*!< 0x80000000 */
#define CAN_F9R2_FB31          CAN_F9R2_FB31_Msk                               /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F10R2 register  ******************/
#define CAN_F10R2_FB0_Pos      (0U)
#define CAN_F10R2_FB0_Msk      (0x1UL << CAN_F10R2_FB0_Pos)                     /*!< 0x00000001 */
#define CAN_F10R2_FB0          CAN_F10R2_FB0_Msk                               /*!<Filter bit 0 */
#define CAN_F10R2_FB1_Pos      (1U)
#define CAN_F10R2_FB1_Msk      (0x1UL << CAN_F10R2_FB1_Pos)                     /*!< 0x00000002 */
#define CAN_F10R2_FB1          CAN_F10R2_FB1_Msk                               /*!<Filter bit 1 */
#define CAN_F10R2_FB2_Pos      (2U)
#define CAN_F10R2_FB2_Msk      (0x1UL << CAN_F10R2_FB2_Pos)                     /*!< 0x00000004 */
#define CAN_F10R2_FB2          CAN_F10R2_FB2_Msk                               /*!<Filter bit 2 */
#define CAN_F10R2_FB3_Pos      (3U)
#define CAN_F10R2_FB3_Msk      (0x1UL << CAN_F10R2_FB3_Pos)                     /*!< 0x00000008 */
#define CAN_F10R2_FB3          CAN_F10R2_FB3_Msk                               /*!<Filter bit 3 */
#define CAN_F10R2_FB4_Pos      (4U)
#define CAN_F10R2_FB4_Msk      (0x1UL << CAN_F10R2_FB4_Pos)                     /*!< 0x00000010 */
#define CAN_F10R2_FB4          CAN_F10R2_FB4_Msk                               /*!<Filter bit 4 */
#define CAN_F10R2_FB5_Pos      (5U)
#define CAN_F10R2_FB5_Msk      (0x1UL << CAN_F10R2_FB5_Pos)                     /*!< 0x00000020 */
#define CAN_F10R2_FB5          CAN_F10R2_FB5_Msk                               /*!<Filter bit 5 */
#define CAN_F10R2_FB6_Pos      (6U)
#define CAN_F10R2_FB6_Msk      (0x1UL << CAN_F10R2_FB6_Pos)                     /*!< 0x00000040 */
#define CAN_F10R2_FB6          CAN_F10R2_FB6_Msk                               /*!<Filter bit 6 */
#define CAN_F10R2_FB7_Pos      (7U)
#define CAN_F10R2_FB7_Msk      (0x1UL << CAN_F10R2_FB7_Pos)                     /*!< 0x00000080 */
#define CAN_F10R2_FB7          CAN_F10R2_FB7_Msk                               /*!<Filter bit 7 */
#define CAN_F10R2_FB8_Pos      (8U)
#define CAN_F10R2_FB8_Msk      (0x1UL << CAN_F10R2_FB8_Pos)                     /*!< 0x00000100 */
#define CAN_F10R2_FB8          CAN_F10R2_FB8_Msk                               /*!<Filter bit 8 */
#define CAN_F10R2_FB9_Pos      (9U)
#define CAN_F10R2_FB9_Msk      (0x1UL << CAN_F10R2_FB9_Pos)                     /*!< 0x00000200 */
#define CAN_F10R2_FB9          CAN_F10R2_FB9_Msk                               /*!<Filter bit 9 */
#define CAN_F10R2_FB10_Pos     (10U)
#define CAN_F10R2_FB10_Msk     (0x1UL << CAN_F10R2_FB10_Pos)                    /*!< 0x00000400 */
#define CAN_F10R2_FB10         CAN_F10R2_FB10_Msk                              /*!<Filter bit 10 */
#define CAN_F10R2_FB11_Pos     (11U)
#define CAN_F10R2_FB11_Msk     (0x1UL << CAN_F10R2_FB11_Pos)                    /*!< 0x00000800 */
#define CAN_F10R2_FB11         CAN_F10R2_FB11_Msk                              /*!<Filter bit 11 */
#define CAN_F10R2_FB12_Pos     (12U)
#define CAN_F10R2_FB12_Msk     (0x1UL << CAN_F10R2_FB12_Pos)                    /*!< 0x00001000 */
#define CAN_F10R2_FB12         CAN_F10R2_FB12_Msk                              /*!<Filter bit 12 */
#define CAN_F10R2_FB13_Pos     (13U)
#define CAN_F10R2_FB13_Msk     (0x1UL << CAN_F10R2_FB13_Pos)                    /*!< 0x00002000 */
#define CAN_F10R2_FB13         CAN_F10R2_FB13_Msk                              /*!<Filter bit 13 */
#define CAN_F10R2_FB14_Pos     (14U)
#define CAN_F10R2_FB14_Msk     (0x1UL << CAN_F10R2_FB14_Pos)                    /*!< 0x00004000 */
#define CAN_F10R2_FB14         CAN_F10R2_FB14_Msk                              /*!<Filter bit 14 */
#define CAN_F10R2_FB15_Pos     (15U)
#define CAN_F10R2_FB15_Msk     (0x1UL << CAN_F10R2_FB15_Pos)                    /*!< 0x00008000 */
#define CAN_F10R2_FB15         CAN_F10R2_FB15_Msk                              /*!<Filter bit 15 */
#define CAN_F10R2_FB16_Pos     (16U)
#define CAN_F10R2_FB16_Msk     (0x1UL << CAN_F10R2_FB16_Pos)                    /*!< 0x00010000 */
#define CAN_F10R2_FB16         CAN_F10R2_FB16_Msk                              /*!<Filter bit 16 */
#define CAN_F10R2_FB17_Pos     (17U)
#define CAN_F10R2_FB17_Msk     (0x1UL << CAN_F10R2_FB17_Pos)                    /*!< 0x00020000 */
#define CAN_F10R2_FB17         CAN_F10R2_FB17_Msk                              /*!<Filter bit 17 */
#define CAN_F10R2_FB18_Pos     (18U)
#define CAN_F10R2_FB18_Msk     (0x1UL << CAN_F10R2_FB18_Pos)                    /*!< 0x00040000 */
#define CAN_F10R2_FB18         CAN_F10R2_FB18_Msk                              /*!<Filter bit 18 */
#define CAN_F10R2_FB19_Pos     (19U)
#define CAN_F10R2_FB19_Msk     (0x1UL << CAN_F10R2_FB19_Pos)                    /*!< 0x00080000 */
#define CAN_F10R2_FB19         CAN_F10R2_FB19_Msk                              /*!<Filter bit 19 */
#define CAN_F10R2_FB20_Pos     (20U)
#define CAN_F10R2_FB20_Msk     (0x1UL << CAN_F10R2_FB20_Pos)                    /*!< 0x00100000 */
#define CAN_F10R2_FB20         CAN_F10R2_FB20_Msk                              /*!<Filter bit 20 */
#define CAN_F10R2_FB21_Pos     (21U)
#define CAN_F10R2_FB21_Msk     (0x1UL << CAN_F10R2_FB21_Pos)                    /*!< 0x00200000 */
#define CAN_F10R2_FB21         CAN_F10R2_FB21_Msk                              /*!<Filter bit 21 */
#define CAN_F10R2_FB22_Pos     (22U)
#define CAN_F10R2_FB22_Msk     (0x1UL << CAN_F10R2_FB22_Pos)                    /*!< 0x00400000 */
#define CAN_F10R2_FB22         CAN_F10R2_FB22_Msk                              /*!<Filter bit 22 */
#define CAN_F10R2_FB23_Pos     (23U)
#define CAN_F10R2_FB23_Msk     (0x1UL << CAN_F10R2_FB23_Pos)                    /*!< 0x00800000 */
#define CAN_F10R2_FB23         CAN_F10R2_FB23_Msk                              /*!<Filter bit 23 */
#define CAN_F10R2_FB24_Pos     (24U)
#define CAN_F10R2_FB24_Msk     (0x1UL << CAN_F10R2_FB24_Pos)                    /*!< 0x01000000 */
#define CAN_F10R2_FB24         CAN_F10R2_FB24_Msk                              /*!<Filter bit 24 */
#define CAN_F10R2_FB25_Pos     (25U)
#define CAN_F10R2_FB25_Msk     (0x1UL << CAN_F10R2_FB25_Pos)                    /*!< 0x02000000 */
#define CAN_F10R2_FB25         CAN_F10R2_FB25_Msk                              /*!<Filter bit 25 */
#define CAN_F10R2_FB26_Pos     (26U)
#define CAN_F10R2_FB26_Msk     (0x1UL << CAN_F10R2_FB26_Pos)                    /*!< 0x04000000 */
#define CAN_F10R2_FB26         CAN_F10R2_FB26_Msk                              /*!<Filter bit 26 */
#define CAN_F10R2_FB27_Pos     (27U)
#define CAN_F10R2_FB27_Msk     (0x1UL << CAN_F10R2_FB27_Pos)                    /*!< 0x08000000 */
#define CAN_F10R2_FB27         CAN_F10R2_FB27_Msk                              /*!<Filter bit 27 */
#define CAN_F10R2_FB28_Pos     (28U)
#define CAN_F10R2_FB28_Msk     (0x1UL << CAN_F10R2_FB28_Pos)                    /*!< 0x10000000 */
#define CAN_F10R2_FB28         CAN_F10R2_FB28_Msk                              /*!<Filter bit 28 */
#define CAN_F10R2_FB29_Pos     (29U)
#define CAN_F10R2_FB29_Msk     (0x1UL << CAN_F10R2_FB29_Pos)                    /*!< 0x20000000 */
#define CAN_F10R2_FB29         CAN_F10R2_FB29_Msk                              /*!<Filter bit 29 */
#define CAN_F10R2_FB30_Pos     (30U)
#define CAN_F10R2_FB30_Msk     (0x1UL << CAN_F10R2_FB30_Pos)                    /*!< 0x40000000 */
#define CAN_F10R2_FB30         CAN_F10R2_FB30_Msk                              /*!<Filter bit 30 */
#define CAN_F10R2_FB31_Pos     (31U)
#define CAN_F10R2_FB31_Msk     (0x1UL << CAN_F10R2_FB31_Pos)                    /*!< 0x80000000 */
#define CAN_F10R2_FB31         CAN_F10R2_FB31_Msk                              /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F11R2 register  ******************/
#define CAN_F11R2_FB0_Pos      (0U)
#define CAN_F11R2_FB0_Msk      (0x1UL << CAN_F11R2_FB0_Pos)                     /*!< 0x00000001 */
#define CAN_F11R2_FB0          CAN_F11R2_FB0_Msk                               /*!<Filter bit 0 */
#define CAN_F11R2_FB1_Pos      (1U)
#define CAN_F11R2_FB1_Msk      (0x1UL << CAN_F11R2_FB1_Pos)                     /*!< 0x00000002 */
#define CAN_F11R2_FB1          CAN_F11R2_FB1_Msk                               /*!<Filter bit 1 */
#define CAN_F11R2_FB2_Pos      (2U)
#define CAN_F11R2_FB2_Msk      (0x1UL << CAN_F11R2_FB2_Pos)                     /*!< 0x00000004 */
#define CAN_F11R2_FB2          CAN_F11R2_FB2_Msk                               /*!<Filter bit 2 */
#define CAN_F11R2_FB3_Pos      (3U)
#define CAN_F11R2_FB3_Msk      (0x1UL << CAN_F11R2_FB3_Pos)                     /*!< 0x00000008 */
#define CAN_F11R2_FB3          CAN_F11R2_FB3_Msk                               /*!<Filter bit 3 */
#define CAN_F11R2_FB4_Pos      (4U)
#define CAN_F11R2_FB4_Msk      (0x1UL << CAN_F11R2_FB4_Pos)                     /*!< 0x00000010 */
#define CAN_F11R2_FB4          CAN_F11R2_FB4_Msk                               /*!<Filter bit 4 */
#define CAN_F11R2_FB5_Pos      (5U)
#define CAN_F11R2_FB5_Msk      (0x1UL << CAN_F11R2_FB5_Pos)                     /*!< 0x00000020 */
#define CAN_F11R2_FB5          CAN_F11R2_FB5_Msk                               /*!<Filter bit 5 */
#define CAN_F11R2_FB6_Pos      (6U)
#define CAN_F11R2_FB6_Msk      (0x1UL << CAN_F11R2_FB6_Pos)                     /*!< 0x00000040 */
#define CAN_F11R2_FB6          CAN_F11R2_FB6_Msk                               /*!<Filter bit 6 */
#define CAN_F11R2_FB7_Pos      (7U)
#define CAN_F11R2_FB7_Msk      (0x1UL << CAN_F11R2_FB7_Pos)                     /*!< 0x00000080 */
#define CAN_F11R2_FB7          CAN_F11R2_FB7_Msk                               /*!<Filter bit 7 */
#define CAN_F11R2_FB8_Pos      (8U)
#define CAN_F11R2_FB8_Msk      (0x1UL << CAN_F11R2_FB8_Pos)                     /*!< 0x00000100 */
#define CAN_F11R2_FB8          CAN_F11R2_FB8_Msk                               /*!<Filter bit 8 */
#define CAN_F11R2_FB9_Pos      (9U)
#define CAN_F11R2_FB9_Msk      (0x1UL << CAN_F11R2_FB9_Pos)                     /*!< 0x00000200 */
#define CAN_F11R2_FB9          CAN_F11R2_FB9_Msk                               /*!<Filter bit 9 */
#define CAN_F11R2_FB10_Pos     (10U)
#define CAN_F11R2_FB10_Msk     (0x1UL << CAN_F11R2_FB10_Pos)                    /*!< 0x00000400 */
#define CAN_F11R2_FB10         CAN_F11R2_FB10_Msk                              /*!<Filter bit 10 */
#define CAN_F11R2_FB11_Pos     (11U)
#define CAN_F11R2_FB11_Msk     (0x1UL << CAN_F11R2_FB11_Pos)                    /*!< 0x00000800 */
#define CAN_F11R2_FB11         CAN_F11R2_FB11_Msk                              /*!<Filter bit 11 */
#define CAN_F11R2_FB12_Pos     (12U)
#define CAN_F11R2_FB12_Msk     (0x1UL << CAN_F11R2_FB12_Pos)                    /*!< 0x00001000 */
#define CAN_F11R2_FB12         CAN_F11R2_FB12_Msk                              /*!<Filter bit 12 */
#define CAN_F11R2_FB13_Pos     (13U)
#define CAN_F11R2_FB13_Msk     (0x1UL << CAN_F11R2_FB13_Pos)                    /*!< 0x00002000 */
#define CAN_F11R2_FB13         CAN_F11R2_FB13_Msk                              /*!<Filter bit 13 */
#define CAN_F11R2_FB14_Pos     (14U)
#define CAN_F11R2_FB14_Msk     (0x1UL << CAN_F11R2_FB14_Pos)                    /*!< 0x00004000 */
#define CAN_F11R2_FB14         CAN_F11R2_FB14_Msk                              /*!<Filter bit 14 */
#define CAN_F11R2_FB15_Pos     (15U)
#define CAN_F11R2_FB15_Msk     (0x1UL << CAN_F11R2_FB15_Pos)                    /*!< 0x00008000 */
#define CAN_F11R2_FB15         CAN_F11R2_FB15_Msk                              /*!<Filter bit 15 */
#define CAN_F11R2_FB16_Pos     (16U)
#define CAN_F11R2_FB16_Msk     (0x1UL << CAN_F11R2_FB16_Pos)                    /*!< 0x00010000 */
#define CAN_F11R2_FB16         CAN_F11R2_FB16_Msk                              /*!<Filter bit 16 */
#define CAN_F11R2_FB17_Pos     (17U)
#define CAN_F11R2_FB17_Msk     (0x1UL << CAN_F11R2_FB17_Pos)                    /*!< 0x00020000 */
#define CAN_F11R2_FB17         CAN_F11R2_FB17_Msk                              /*!<Filter bit 17 */
#define CAN_F11R2_FB18_Pos     (18U)
#define CAN_F11R2_FB18_Msk     (0x1UL << CAN_F11R2_FB18_Pos)                    /*!< 0x00040000 */
#define CAN_F11R2_FB18         CAN_F11R2_FB18_Msk                              /*!<Filter bit 18 */
#define CAN_F11R2_FB19_Pos     (19U)
#define CAN_F11R2_FB19_Msk     (0x1UL << CAN_F11R2_FB19_Pos)                    /*!< 0x00080000 */
#define CAN_F11R2_FB19         CAN_F11R2_FB19_Msk                              /*!<Filter bit 19 */
#define CAN_F11R2_FB20_Pos     (20U)
#define CAN_F11R2_FB20_Msk     (0x1UL << CAN_F11R2_FB20_Pos)                    /*!< 0x00100000 */
#define CAN_F11R2_FB20         CAN_F11R2_FB20_Msk                              /*!<Filter bit 20 */
#define CAN_F11R2_FB21_Pos     (21U)
#define CAN_F11R2_FB21_Msk     (0x1UL << CAN_F11R2_FB21_Pos)                    /*!< 0x00200000 */
#define CAN_F11R2_FB21         CAN_F11R2_FB21_Msk                              /*!<Filter bit 21 */
#define CAN_F11R2_FB22_Pos     (22U)
#define CAN_F11R2_FB22_Msk     (0x1UL << CAN_F11R2_FB22_Pos)                    /*!< 0x00400000 */
#define CAN_F11R2_FB22         CAN_F11R2_FB22_Msk                              /*!<Filter bit 22 */
#define CAN_F11R2_FB23_Pos     (23U)
#define CAN_F11R2_FB23_Msk     (0x1UL << CAN_F11R2_FB23_Pos)                    /*!< 0x00800000 */
#define CAN_F11R2_FB23         CAN_F11R2_FB23_Msk                              /*!<Filter bit 23 */
#define CAN_F11R2_FB24_Pos     (24U)
#define CAN_F11R2_FB24_Msk     (0x1UL << CAN_F11R2_FB24_Pos)                    /*!< 0x01000000 */
#define CAN_F11R2_FB24         CAN_F11R2_FB24_Msk                              /*!<Filter bit 24 */
#define CAN_F11R2_FB25_Pos     (25U)
#define CAN_F11R2_FB25_Msk     (0x1UL << CAN_F11R2_FB25_Pos)                    /*!< 0x02000000 */
#define CAN_F11R2_FB25         CAN_F11R2_FB25_Msk                              /*!<Filter bit 25 */
#define CAN_F11R2_FB26_Pos     (26U)
#define CAN_F11R2_FB26_Msk     (0x1UL << CAN_F11R2_FB26_Pos)                    /*!< 0x04000000 */
#define CAN_F11R2_FB26         CAN_F11R2_FB26_Msk                              /*!<Filter bit 26 */
#define CAN_F11R2_FB27_Pos     (27U)
#define CAN_F11R2_FB27_Msk     (0x1UL << CAN_F11R2_FB27_Pos)                    /*!< 0x08000000 */
#define CAN_F11R2_FB27         CAN_F11R2_FB27_Msk                              /*!<Filter bit 27 */
#define CAN_F11R2_FB28_Pos     (28U)
#define CAN_F11R2_FB28_Msk     (0x1UL << CAN_F11R2_FB28_Pos)                    /*!< 0x10000000 */
#define CAN_F11R2_FB28         CAN_F11R2_FB28_Msk                              /*!<Filter bit 28 */
#define CAN_F11R2_FB29_Pos     (29U)
#define CAN_F11R2_FB29_Msk     (0x1UL << CAN_F11R2_FB29_Pos)                    /*!< 0x20000000 */
#define CAN_F11R2_FB29         CAN_F11R2_FB29_Msk                              /*!<Filter bit 29 */
#define CAN_F11R2_FB30_Pos     (30U)
#define CAN_F11R2_FB30_Msk     (0x1UL << CAN_F11R2_FB30_Pos)                    /*!< 0x40000000 */
#define CAN_F11R2_FB30         CAN_F11R2_FB30_Msk                              /*!<Filter bit 30 */
#define CAN_F11R2_FB31_Pos     (31U)
#define CAN_F11R2_FB31_Msk     (0x1UL << CAN_F11R2_FB31_Pos)                    /*!< 0x80000000 */
#define CAN_F11R2_FB31         CAN_F11R2_FB31_Msk                              /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F12R2 register  ******************/
#define CAN_F12R2_FB0_Pos      (0U)
#define CAN_F12R2_FB0_Msk      (0x1UL << CAN_F12R2_FB0_Pos)                     /*!< 0x00000001 */
#define CAN_F12R2_FB0          CAN_F12R2_FB0_Msk                               /*!<Filter bit 0 */
#define CAN_F12R2_FB1_Pos      (1U)
#define CAN_F12R2_FB1_Msk      (0x1UL << CAN_F12R2_FB1_Pos)                     /*!< 0x00000002 */
#define CAN_F12R2_FB1          CAN_F12R2_FB1_Msk                               /*!<Filter bit 1 */
#define CAN_F12R2_FB2_Pos      (2U)
#define CAN_F12R2_FB2_Msk      (0x1UL << CAN_F12R2_FB2_Pos)                     /*!< 0x00000004 */
#define CAN_F12R2_FB2          CAN_F12R2_FB2_Msk                               /*!<Filter bit 2 */
#define CAN_F12R2_FB3_Pos      (3U)
#define CAN_F12R2_FB3_Msk      (0x1UL << CAN_F12R2_FB3_Pos)                     /*!< 0x00000008 */
#define CAN_F12R2_FB3          CAN_F12R2_FB3_Msk                               /*!<Filter bit 3 */
#define CAN_F12R2_FB4_Pos      (4U)
#define CAN_F12R2_FB4_Msk      (0x1UL << CAN_F12R2_FB4_Pos)                     /*!< 0x00000010 */
#define CAN_F12R2_FB4          CAN_F12R2_FB4_Msk                               /*!<Filter bit 4 */
#define CAN_F12R2_FB5_Pos      (5U)
#define CAN_F12R2_FB5_Msk      (0x1UL << CAN_F12R2_FB5_Pos)                     /*!< 0x00000020 */
#define CAN_F12R2_FB5          CAN_F12R2_FB5_Msk                               /*!<Filter bit 5 */
#define CAN_F12R2_FB6_Pos      (6U)
#define CAN_F12R2_FB6_Msk      (0x1UL << CAN_F12R2_FB6_Pos)                     /*!< 0x00000040 */
#define CAN_F12R2_FB6          CAN_F12R2_FB6_Msk                               /*!<Filter bit 6 */
#define CAN_F12R2_FB7_Pos      (7U)
#define CAN_F12R2_FB7_Msk      (0x1UL << CAN_F12R2_FB7_Pos)                     /*!< 0x00000080 */
#define CAN_F12R2_FB7          CAN_F12R2_FB7_Msk                               /*!<Filter bit 7 */
#define CAN_F12R2_FB8_Pos      (8U)
#define CAN_F12R2_FB8_Msk      (0x1UL << CAN_F12R2_FB8_Pos)                     /*!< 0x00000100 */
#define CAN_F12R2_FB8          CAN_F12R2_FB8_Msk                               /*!<Filter bit 8 */
#define CAN_F12R2_FB9_Pos      (9U)
#define CAN_F12R2_FB9_Msk      (0x1UL << CAN_F12R2_FB9_Pos)                     /*!< 0x00000200 */
#define CAN_F12R2_FB9          CAN_F12R2_FB9_Msk                               /*!<Filter bit 9 */
#define CAN_F12R2_FB10_Pos     (10U)
#define CAN_F12R2_FB10_Msk     (0x1UL << CAN_F12R2_FB10_Pos)                    /*!< 0x00000400 */
#define CAN_F12R2_FB10         CAN_F12R2_FB10_Msk                              /*!<Filter bit 10 */
#define CAN_F12R2_FB11_Pos     (11U)
#define CAN_F12R2_FB11_Msk     (0x1UL << CAN_F12R2_FB11_Pos)                    /*!< 0x00000800 */
#define CAN_F12R2_FB11         CAN_F12R2_FB11_Msk                              /*!<Filter bit 11 */
#define CAN_F12R2_FB12_Pos     (12U)
#define CAN_F12R2_FB12_Msk     (0x1UL << CAN_F12R2_FB12_Pos)                    /*!< 0x00001000 */
#define CAN_F12R2_FB12         CAN_F12R2_FB12_Msk                              /*!<Filter bit 12 */
#define CAN_F12R2_FB13_Pos     (13U)
#define CAN_F12R2_FB13_Msk     (0x1UL << CAN_F12R2_FB13_Pos)                    /*!< 0x00002000 */
#define CAN_F12R2_FB13         CAN_F12R2_FB13_Msk                              /*!<Filter bit 13 */
#define CAN_F12R2_FB14_Pos     (14U)
#define CAN_F12R2_FB14_Msk     (0x1UL << CAN_F12R2_FB14_Pos)                    /*!< 0x00004000 */
#define CAN_F12R2_FB14         CAN_F12R2_FB14_Msk                              /*!<Filter bit 14 */
#define CAN_F12R2_FB15_Pos     (15U)
#define CAN_F12R2_FB15_Msk     (0x1UL << CAN_F12R2_FB15_Pos)                    /*!< 0x00008000 */
#define CAN_F12R2_FB15         CAN_F12R2_FB15_Msk                              /*!<Filter bit 15 */
#define CAN_F12R2_FB16_Pos     (16U)
#define CAN_F12R2_FB16_Msk     (0x1UL << CAN_F12R2_FB16_Pos)                    /*!< 0x00010000 */
#define CAN_F12R2_FB16         CAN_F12R2_FB16_Msk                              /*!<Filter bit 16 */
#define CAN_F12R2_FB17_Pos     (17U)
#define CAN_F12R2_FB17_Msk     (0x1UL << CAN_F12R2_FB17_Pos)                    /*!< 0x00020000 */
#define CAN_F12R2_FB17         CAN_F12R2_FB17_Msk                              /*!<Filter bit 17 */
#define CAN_F12R2_FB18_Pos     (18U)
#define CAN_F12R2_FB18_Msk     (0x1UL << CAN_F12R2_FB18_Pos)                    /*!< 0x00040000 */
#define CAN_F12R2_FB18         CAN_F12R2_FB18_Msk                              /*!<Filter bit 18 */
#define CAN_F12R2_FB19_Pos     (19U)
#define CAN_F12R2_FB19_Msk     (0x1UL << CAN_F12R2_FB19_Pos)                    /*!< 0x00080000 */
#define CAN_F12R2_FB19         CAN_F12R2_FB19_Msk                              /*!<Filter bit 19 */
#define CAN_F12R2_FB20_Pos     (20U)
#define CAN_F12R2_FB20_Msk     (0x1UL << CAN_F12R2_FB20_Pos)                    /*!< 0x00100000 */
#define CAN_F12R2_FB20         CAN_F12R2_FB20_Msk                              /*!<Filter bit 20 */
#define CAN_F12R2_FB21_Pos     (21U)
#define CAN_F12R2_FB21_Msk     (0x1UL << CAN_F12R2_FB21_Pos)                    /*!< 0x00200000 */
#define CAN_F12R2_FB21         CAN_F12R2_FB21_Msk                              /*!<Filter bit 21 */
#define CAN_F12R2_FB22_Pos     (22U)
#define CAN_F12R2_FB22_Msk     (0x1UL << CAN_F12R2_FB22_Pos)                    /*!< 0x00400000 */
#define CAN_F12R2_FB22         CAN_F12R2_FB22_Msk                              /*!<Filter bit 22 */
#define CAN_F12R2_FB23_Pos     (23U)
#define CAN_F12R2_FB23_Msk     (0x1UL << CAN_F12R2_FB23_Pos)                    /*!< 0x00800000 */
#define CAN_F12R2_FB23         CAN_F12R2_FB23_Msk                              /*!<Filter bit 23 */
#define CAN_F12R2_FB24_Pos     (24U)
#define CAN_F12R2_FB24_Msk     (0x1UL << CAN_F12R2_FB24_Pos)                    /*!< 0x01000000 */
#define CAN_F12R2_FB24         CAN_F12R2_FB24_Msk                              /*!<Filter bit 24 */
#define CAN_F12R2_FB25_Pos     (25U)
#define CAN_F12R2_FB25_Msk     (0x1UL << CAN_F12R2_FB25_Pos)                    /*!< 0x02000000 */
#define CAN_F12R2_FB25         CAN_F12R2_FB25_Msk                              /*!<Filter bit 25 */
#define CAN_F12R2_FB26_Pos     (26U)
#define CAN_F12R2_FB26_Msk     (0x1UL << CAN_F12R2_FB26_Pos)                    /*!< 0x04000000 */
#define CAN_F12R2_FB26         CAN_F12R2_FB26_Msk                              /*!<Filter bit 26 */
#define CAN_F12R2_FB27_Pos     (27U)
#define CAN_F12R2_FB27_Msk     (0x1UL << CAN_F12R2_FB27_Pos)                    /*!< 0x08000000 */
#define CAN_F12R2_FB27         CAN_F12R2_FB27_Msk                              /*!<Filter bit 27 */
#define CAN_F12R2_FB28_Pos     (28U)
#define CAN_F12R2_FB28_Msk     (0x1UL << CAN_F12R2_FB28_Pos)                    /*!< 0x10000000 */
#define CAN_F12R2_FB28         CAN_F12R2_FB28_Msk                              /*!<Filter bit 28 */
#define CAN_F12R2_FB29_Pos     (29U)
#define CAN_F12R2_FB29_Msk     (0x1UL << CAN_F12R2_FB29_Pos)                    /*!< 0x20000000 */
#define CAN_F12R2_FB29         CAN_F12R2_FB29_Msk                              /*!<Filter bit 29 */
#define CAN_F12R2_FB30_Pos     (30U)
#define CAN_F12R2_FB30_Msk     (0x1UL << CAN_F12R2_FB30_Pos)                    /*!< 0x40000000 */
#define CAN_F12R2_FB30         CAN_F12R2_FB30_Msk                              /*!<Filter bit 30 */
#define CAN_F12R2_FB31_Pos     (31U)
#define CAN_F12R2_FB31_Msk     (0x1UL << CAN_F12R2_FB31_Pos)                    /*!< 0x80000000 */
#define CAN_F12R2_FB31         CAN_F12R2_FB31_Msk                              /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F13R2 register  ******************/
#define CAN_F13R2_FB0_Pos      (0U)
#define CAN_F13R2_FB0_Msk      (0x1UL << CAN_F13R2_FB0_Pos)                     /*!< 0x00000001 */
#define CAN_F13R2_FB0          CAN_F13R2_FB0_Msk                               /*!<Filter bit 0 */
#define CAN_F13R2_FB1_Pos      (1U)
#define CAN_F13R2_FB1_Msk      (0x1UL << CAN_F13R2_FB1_Pos)                     /*!< 0x00000002 */
#define CAN_F13R2_FB1          CAN_F13R2_FB1_Msk                               /*!<Filter bit 1 */
#define CAN_F13R2_FB2_Pos      (2U)
#define CAN_F13R2_FB2_Msk      (0x1UL << CAN_F13R2_FB2_Pos)                     /*!< 0x00000004 */
#define CAN_F13R2_FB2          CAN_F13R2_FB2_Msk                               /*!<Filter bit 2 */
#define CAN_F13R2_FB3_Pos      (3U)
#define CAN_F13R2_FB3_Msk      (0x1UL << CAN_F13R2_FB3_Pos)                     /*!< 0x00000008 */
#define CAN_F13R2_FB3          CAN_F13R2_FB3_Msk                               /*!<Filter bit 3 */
#define CAN_F13R2_FB4_Pos      (4U)
#define CAN_F13R2_FB4_Msk      (0x1UL << CAN_F13R2_FB4_Pos)                     /*!< 0x00000010 */
#define CAN_F13R2_FB4          CAN_F13R2_FB4_Msk                               /*!<Filter bit 4 */
#define CAN_F13R2_FB5_Pos      (5U)
#define CAN_F13R2_FB5_Msk      (0x1UL << CAN_F13R2_FB5_Pos)                     /*!< 0x00000020 */
#define CAN_F13R2_FB5          CAN_F13R2_FB5_Msk                               /*!<Filter bit 5 */
#define CAN_F13R2_FB6_Pos      (6U)
#define CAN_F13R2_FB6_Msk      (0x1UL << CAN_F13R2_FB6_Pos)                     /*!< 0x00000040 */
#define CAN_F13R2_FB6          CAN_F13R2_FB6_Msk                               /*!<Filter bit 6 */
#define CAN_F13R2_FB7_Pos      (7U)
#define CAN_F13R2_FB7_Msk      (0x1UL << CAN_F13R2_FB7_Pos)                     /*!< 0x00000080 */
#define CAN_F13R2_FB7          CAN_F13R2_FB7_Msk                               /*!<Filter bit 7 */
#define CAN_F13R2_FB8_Pos      (8U)
#define CAN_F13R2_FB8_Msk      (0x1UL << CAN_F13R2_FB8_Pos)                     /*!< 0x00000100 */
#define CAN_F13R2_FB8          CAN_F13R2_FB8_Msk                               /*!<Filter bit 8 */
#define CAN_F13R2_FB9_Pos      (9U)
#define CAN_F13R2_FB9_Msk      (0x1UL << CAN_F13R2_FB9_Pos)                     /*!< 0x00000200 */
#define CAN_F13R2_FB9          CAN_F13R2_FB9_Msk                               /*!<Filter bit 9 */
#define CAN_F13R2_FB10_Pos     (10U)
#define CAN_F13R2_FB10_Msk     (0x1UL << CAN_F13R2_FB10_Pos)                    /*!< 0x00000400 */
#define CAN_F13R2_FB10         CAN_F13R2_FB10_Msk                              /*!<Filter bit 10 */
#define CAN_F13R2_FB11_Pos     (11U)
#define CAN_F13R2_FB11_Msk     (0x1UL << CAN_F13R2_FB11_Pos)                    /*!< 0x00000800 */
#define CAN_F13R2_FB11         CAN_F13R2_FB11_Msk                              /*!<Filter bit 11 */
#define CAN_F13R2_FB12_Pos     (12U)
#define CAN_F13R2_FB12_Msk     (0x1UL << CAN_F13R2_FB12_Pos)                    /*!< 0x00001000 */
#define CAN_F13R2_FB12         CAN_F13R2_FB12_Msk                              /*!<Filter bit 12 */
#define CAN_F13R2_FB13_Pos     (13U)
#define CAN_F13R2_FB13_Msk     (0x1UL << CAN_F13R2_FB13_Pos)                    /*!< 0x00002000 */
#define CAN_F13R2_FB13         CAN_F13R2_FB13_Msk                              /*!<Filter bit 13 */
#define CAN_F13R2_FB14_Pos     (14U)
#define CAN_F13R2_FB14_Msk     (0x1UL << CAN_F13R2_FB14_Pos)                    /*!< 0x00004000 */
#define CAN_F13R2_FB14         CAN_F13R2_FB14_Msk                              /*!<Filter bit 14 */
#define CAN_F13R2_FB15_Pos     (15U)
#define CAN_F13R2_FB15_Msk     (0x1UL << CAN_F13R2_FB15_Pos)                    /*!< 0x00008000 */
#define CAN_F13R2_FB15         CAN_F13R2_FB15_Msk                              /*!<Filter bit 15 */
#define CAN_F13R2_FB16_Pos     (16U)
#define CAN_F13R2_FB16_Msk     (0x1UL << CAN_F13R2_FB16_Pos)                    /*!< 0x00010000 */
#define CAN_F13R2_FB16         CAN_F13R2_FB16_Msk                              /*!<Filter bit 16 */
#define CAN_F13R2_FB17_Pos     (17U)
#define CAN_F13R2_FB17_Msk     (0x1UL << CAN_F13R2_FB17_Pos)                    /*!< 0x00020000 */
#define CAN_F13R2_FB17         CAN_F13R2_FB17_Msk                              /*!<Filter bit 17 */
#define CAN_F13R2_FB18_Pos     (18U)
#define CAN_F13R2_FB18_Msk     (0x1UL << CAN_F13R2_FB18_Pos)                    /*!< 0x00040000 */
#define CAN_F13R2_FB18         CAN_F13R2_FB18_Msk                              /*!<Filter bit 18 */
#define CAN_F13R2_FB19_Pos     (19U)
#define CAN_F13R2_FB19_Msk     (0x1UL << CAN_F13R2_FB19_Pos)                    /*!< 0x00080000 */
#define CAN_F13R2_FB19         CAN_F13R2_FB19_Msk                              /*!<Filter bit 19 */
#define CAN_F13R2_FB20_Pos     (20U)
#define CAN_F13R2_FB20_Msk     (0x1UL << CAN_F13R2_FB20_Pos)                    /*!< 0x00100000 */
#define CAN_F13R2_FB20         CAN_F13R2_FB20_Msk                              /*!<Filter bit 20 */
#define CAN_F13R2_FB21_Pos     (21U)
#define CAN_F13R2_FB21_Msk     (0x1UL << CAN_F13R2_FB21_Pos)                    /*!< 0x00200000 */
#define CAN_F13R2_FB21         CAN_F13R2_FB21_Msk                              /*!<Filter bit 21 */
#define CAN_F13R2_FB22_Pos     (22U)
#define CAN_F13R2_FB22_Msk     (0x1UL << CAN_F13R2_FB22_Pos)                    /*!< 0x00400000 */
#define CAN_F13R2_FB22         CAN_F13R2_FB22_Msk                              /*!<Filter bit 22 */
#define CAN_F13R2_FB23_Pos     (23U)
#define CAN_F13R2_FB23_Msk     (0x1UL << CAN_F13R2_FB23_Pos)                    /*!< 0x00800000 */
#define CAN_F13R2_FB23         CAN_F13R2_FB23_Msk                              /*!<Filter bit 23 */
#define CAN_F13R2_FB24_Pos     (24U)
#define CAN_F13R2_FB24_Msk     (0x1UL << CAN_F13R2_FB24_Pos)                    /*!< 0x01000000 */
#define CAN_F13R2_FB24         CAN_F13R2_FB24_Msk                              /*!<Filter bit 24 */
#define CAN_F13R2_FB25_Pos     (25U)
#define CAN_F13R2_FB25_Msk     (0x1UL << CAN_F13R2_FB25_Pos)                    /*!< 0x02000000 */
#define CAN_F13R2_FB25         CAN_F13R2_FB25_Msk                              /*!<Filter bit 25 */
#define CAN_F13R2_FB26_Pos     (26U)
#define CAN_F13R2_FB26_Msk     (0x1UL << CAN_F13R2_FB26_Pos)                    /*!< 0x04000000 */
#define CAN_F13R2_FB26         CAN_F13R2_FB26_Msk                              /*!<Filter bit 26 */
#define CAN_F13R2_FB27_Pos     (27U)
#define CAN_F13R2_FB27_Msk     (0x1UL << CAN_F13R2_FB27_Pos)                    /*!< 0x08000000 */
#define CAN_F13R2_FB27         CAN_F13R2_FB27_Msk                              /*!<Filter bit 27 */
#define CAN_F13R2_FB28_Pos     (28U)
#define CAN_F13R2_FB28_Msk     (0x1UL << CAN_F13R2_FB28_Pos)                    /*!< 0x10000000 */
#define CAN_F13R2_FB28         CAN_F13R2_FB28_Msk                              /*!<Filter bit 28 */
#define CAN_F13R2_FB29_Pos     (29U)
#define CAN_F13R2_FB29_Msk     (0x1UL << CAN_F13R2_FB29_Pos)                    /*!< 0x20000000 */
#define CAN_F13R2_FB29         CAN_F13R2_FB29_Msk                              /*!<Filter bit 29 */
#define CAN_F13R2_FB30_Pos     (30U)
#define CAN_F13R2_FB30_Msk     (0x1UL << CAN_F13R2_FB30_Pos)                    /*!< 0x40000000 */
#define CAN_F13R2_FB30         CAN_F13R2_FB30_Msk                              /*!<Filter bit 30 */
#define CAN_F13R2_FB31_Pos     (31U)
#define CAN_F13R2_FB31_Msk     (0x1UL << CAN_F13R2_FB31_Pos)                    /*!< 0x80000000 */
#define CAN_F13R2_FB31         CAN_F13R2_FB31_Msk                              /*!<Filter bit 31 */

/* CAN filters Legacy aliases */
#define CAN_FM1R_FBM14_Pos     (14U)
#define CAN_FM1R_FBM14_Msk     (0x1UL << CAN_FM1R_FBM14_Pos)                    /*!< 0x00004000 */
#define CAN_FM1R_FBM14         CAN_FM1R_FBM14_Msk                              /*!<Filter Init Mode bit 14 */
#define CAN_FM1R_FBM15_Pos     (15U)
#define CAN_FM1R_FBM15_Msk     (0x1UL << CAN_FM1R_FBM15_Pos)                    /*!< 0x00008000 */
#define CAN_FM1R_FBM15         CAN_FM1R_FBM15_Msk                              /*!<Filter Init Mode bit 15 */
#define CAN_FM1R_FBM16_Pos     (16U)
#define CAN_FM1R_FBM16_Msk     (0x1UL << CAN_FM1R_FBM16_Pos)                    /*!< 0x00010000 */
#define CAN_FM1R_FBM16         CAN_FM1R_FBM16_Msk                              /*!<Filter Init Mode bit 16 */
#define CAN_FM1R_FBM17_Pos     (17U)
#define CAN_FM1R_FBM17_Msk     (0x1UL << CAN_FM1R_FBM17_Pos)                    /*!< 0x00020000 */
#define CAN_FM1R_FBM17         CAN_FM1R_FBM17_Msk                              /*!<Filter Init Mode bit 17 */
#define CAN_FM1R_FBM18_Pos     (18U)
#define CAN_FM1R_FBM18_Msk     (0x1UL << CAN_FM1R_FBM18_Pos)                    /*!< 0x00040000 */
#define CAN_FM1R_FBM18         CAN_FM1R_FBM18_Msk                              /*!<Filter Init Mode bit 18 */
#define CAN_FM1R_FBM19_Pos     (19U)
#define CAN_FM1R_FBM19_Msk     (0x1UL << CAN_FM1R_FBM19_Pos)                    /*!< 0x00080000 */
#define CAN_FM1R_FBM19         CAN_FM1R_FBM19_Msk                              /*!<Filter Init Mode bit 19 */
#define CAN_FM1R_FBM20_Pos     (20U)
#define CAN_FM1R_FBM20_Msk     (0x1UL << CAN_FM1R_FBM20_Pos)                    /*!< 0x00100000 */
#define CAN_FM1R_FBM20         CAN_FM1R_FBM20_Msk                              /*!<Filter Init Mode bit 20 */
#define CAN_FM1R_FBM21_Pos     (21U)
#define CAN_FM1R_FBM21_Msk     (0x1UL << CAN_FM1R_FBM21_Pos)                    /*!< 0x00200000 */
#define CAN_FM1R_FBM21         CAN_FM1R_FBM21_Msk                              /*!<Filter Init Mode bit 21 */
#define CAN_FM1R_FBM22_Pos     (22U)
#define CAN_FM1R_FBM22_Msk     (0x1UL << CAN_FM1R_FBM22_Pos)                    /*!< 0x00400000 */
#define CAN_FM1R_FBM22         CAN_FM1R_FBM22_Msk                              /*!<Filter Init Mode bit 22 */
#define CAN_FM1R_FBM23_Pos     (23U)
#define CAN_FM1R_FBM23_Msk     (0x1UL << CAN_FM1R_FBM23_Pos)                    /*!< 0x00800000 */
#define CAN_FM1R_FBM23         CAN_FM1R_FBM23_Msk                              /*!<Filter Init Mode bit 23 */
#define CAN_FM1R_FBM24_Pos     (24U)
#define CAN_FM1R_FBM24_Msk     (0x1UL << CAN_FM1R_FBM24_Pos)                    /*!< 0x01000000 */
#define CAN_FM1R_FBM24         CAN_FM1R_FBM24_Msk                              /*!<Filter Init Mode bit 24 */
#define CAN_FM1R_FBM25_Pos     (25U)
#define CAN_FM1R_FBM25_Msk     (0x1UL << CAN_FM1R_FBM25_Pos)                    /*!< 0x02000000 */
#define CAN_FM1R_FBM25         CAN_FM1R_FBM25_Msk                              /*!<Filter Init Mode bit 25 */
#define CAN_FM1R_FBM26_Pos     (26U)
#define CAN_FM1R_FBM26_Msk     (0x1UL << CAN_FM1R_FBM26_Pos)                    /*!< 0x04000000 */
#define CAN_FM1R_FBM26         CAN_FM1R_FBM26_Msk                              /*!<Filter Init Mode bit 26 */
#define CAN_FM1R_FBM27_Pos     (27U)
#define CAN_FM1R_FBM27_Msk     (0x1UL << CAN_FM1R_FBM27_Pos)                    /*!< 0x08000000 */
#define CAN_FM1R_FBM27         CAN_FM1R_FBM27_Msk                              /*!<Filter Init Mode bit 27 */

#define CAN_FS1R_FSC14_Pos     (14U)
#define CAN_FS1R_FSC14_Msk     (0x1UL << CAN_FS1R_FSC14_Pos)                    /*!< 0x00004000 */
#define CAN_FS1R_FSC14         CAN_FS1R_FSC14_Msk                              /*!<Filter Scale Configuration bit 14 */
#define CAN_FS1R_FSC15_Pos     (15U)
#define CAN_FS1R_FSC15_Msk     (0x1UL << CAN_FS1R_FSC15_Pos)                    /*!< 0x00008000 */
#define CAN_FS1R_FSC15         CAN_FS1R_FSC15_Msk                              /*!<Filter Scale Configuration bit 15 */
#define CAN_FS1R_FSC16_Pos     (16U)
#define CAN_FS1R_FSC16_Msk     (0x1UL << CAN_FS1R_FSC16_Pos)                    /*!< 0x00010000 */
#define CAN_FS1R_FSC16         CAN_FS1R_FSC16_Msk                              /*!<Filter Scale Configuration bit 16 */
#define CAN_FS1R_FSC17_Pos     (17U)
#define CAN_FS1R_FSC17_Msk     (0x1UL << CAN_FS1R_FSC17_Pos)                    /*!< 0x00020000 */
#define CAN_FS1R_FSC17         CAN_FS1R_FSC17_Msk                              /*!<Filter Scale Configuration bit 17 */
#define CAN_FS1R_FSC18_Pos     (18U)
#define CAN_FS1R_FSC18_Msk     (0x1UL << CAN_FS1R_FSC18_Pos)                    /*!< 0x00040000 */
#define CAN_FS1R_FSC18         CAN_FS1R_FSC18_Msk                              /*!<Filter Scale Configuration bit 18 */
#define CAN_FS1R_FSC19_Pos     (19U)
#define CAN_FS1R_FSC19_Msk     (0x1UL << CAN_FS1R_FSC19_Pos)                    /*!< 0x00080000 */
#define CAN_FS1R_FSC19         CAN_FS1R_FSC19_Msk                              /*!<Filter Scale Configuration bit 19 */
#define CAN_FS1R_FSC20_Pos     (20U)
#define CAN_FS1R_FSC20_Msk     (0x1UL << CAN_FS1R_FSC20_Pos)                    /*!< 0x00100000 */
#define CAN_FS1R_FSC20         CAN_FS1R_FSC20_Msk                              /*!<Filter Scale Configuration bit 20 */
#define CAN_FS1R_FSC21_Pos     (21U)
#define CAN_FS1R_FSC21_Msk     (0x1UL << CAN_FS1R_FSC21_Pos)                    /*!< 0x00200000 */
#define CAN_FS1R_FSC21         CAN_FS1R_FSC21_Msk                              /*!<Filter Scale Configuration bit 21 */
#define CAN_FS1R_FSC22_Pos     (22U)
#define CAN_FS1R_FSC22_Msk     (0x1UL << CAN_FS1R_FSC22_Pos)                    /*!< 0x00400000 */
#define CAN_FS1R_FSC22         CAN_FS1R_FSC22_Msk                              /*!<Filter Scale Configuration bit 22 */
#define CAN_FS1R_FSC23_Pos     (23U)
#define CAN_FS1R_FSC23_Msk     (0x1UL << CAN_FS1R_FSC23_Pos)                    /*!< 0x00800000 */
#define CAN_FS1R_FSC23         CAN_FS1R_FSC23_Msk                              /*!<Filter Scale Configuration bit 23 */
#define CAN_FS1R_FSC24_Pos     (24U)
#define CAN_FS1R_FSC24_Msk     (0x1UL << CAN_FS1R_FSC24_Pos)                    /*!< 0x01000000 */
#define CAN_FS1R_FSC24         CAN_FS1R_FSC24_Msk                              /*!<Filter Scale Configuration bit 24 */
#define CAN_FS1R_FSC25_Pos     (25U)
#define CAN_FS1R_FSC25_Msk     (0x1UL << CAN_FS1R_FSC25_Pos)                    /*!< 0x02000000 */
#define CAN_FS1R_FSC25         CAN_FS1R_FSC25_Msk                              /*!<Filter Scale Configuration bit 25 */
#define CAN_FS1R_FSC26_Pos     (26U)
#define CAN_FS1R_FSC26_Msk     (0x1UL << CAN_FS1R_FSC26_Pos)                    /*!< 0x04000000 */
#define CAN_FS1R_FSC26         CAN_FS1R_FSC26_Msk                              /*!<Filter Scale Configuration bit 26 */
#define CAN_FS1R_FSC27_Pos     (27U)
#define CAN_FS1R_FSC27_Msk     (0x1UL << CAN_FS1R_FSC27_Pos)                    /*!< 0x08000000 */
#define CAN_FS1R_FSC27         CAN_FS1R_FSC27_Msk                              /*!<Filter Scale Configuration bit 27 */

#define CAN_FFA1R_FFA14_Pos    (14U)
#define CAN_FFA1R_FFA14_Msk    (0x1UL << CAN_FFA1R_FFA14_Pos)                   /*!< 0x00004000 */
#define CAN_FFA1R_FFA14        CAN_FFA1R_FFA14_Msk                             /*!<Filter FIFO Assignment bit 14 */
#define CAN_FFA1R_FFA15_Pos    (15U)
#define CAN_FFA1R_FFA15_Msk    (0x1UL << CAN_FFA1R_FFA15_Pos)                   /*!< 0x00008000 */
#define CAN_FFA1R_FFA15        CAN_FFA1R_FFA15_Msk                             /*!<Filter FIFO Assignment bit 15 */
#define CAN_FFA1R_FFA16_Pos    (16U)
#define CAN_FFA1R_FFA16_Msk    (0x1UL << CAN_FFA1R_FFA16_Pos)                   /*!< 0x00010000 */
#define CAN_FFA1R_FFA16        CAN_FFA1R_FFA16_Msk                             /*!<Filter FIFO Assignment bit 16 */
#define CAN_FFA1R_FFA17_Pos    (17U)
#define CAN_FFA1R_FFA17_Msk    (0x1UL << CAN_FFA1R_FFA17_Pos)                   /*!< 0x00020000 */
#define CAN_FFA1R_FFA17        CAN_FFA1R_FFA17_Msk                             /*!<Filter FIFO Assignment bit 17 */
#define CAN_FFA1R_FFA18_Pos    (18U)
#define CAN_FFA1R_FFA18_Msk    (0x1UL << CAN_FFA1R_FFA18_Pos)                   /*!< 0x00040000 */
#define CAN_FFA1R_FFA18        CAN_FFA1R_FFA18_Msk                             /*!<Filter FIFO Assignment bit 18 */
#define CAN_FFA1R_FFA19_Pos    (19U)
#define CAN_FFA1R_FFA19_Msk    (0x1UL << CAN_FFA1R_FFA19_Pos)                   /*!< 0x00080000 */
#define CAN_FFA1R_FFA19        CAN_FFA1R_FFA19_Msk                             /*!<Filter FIFO Assignment bit 19 */
#define CAN_FFA1R_FFA20_Pos    (20U)
#define CAN_FFA1R_FFA20_Msk    (0x1UL << CAN_FFA1R_FFA20_Pos)                   /*!< 0x00100000 */
#define CAN_FFA1R_FFA20        CAN_FFA1R_FFA20_Msk                             /*!<Filter FIFO Assignment bit 20 */
#define CAN_FFA1R_FFA21_Pos    (21U)
#define CAN_FFA1R_FFA21_Msk    (0x1UL << CAN_FFA1R_FFA21_Pos)                   /*!< 0x00200000 */
#define CAN_FFA1R_FFA21        CAN_FFA1R_FFA21_Msk                             /*!<Filter FIFO Assignment bit 21 */
#define CAN_FFA1R_FFA22_Pos    (22U)
#define CAN_FFA1R_FFA22_Msk    (0x1UL << CAN_FFA1R_FFA22_Pos)                   /*!< 0x00400000 */
#define CAN_FFA1R_FFA22        CAN_FFA1R_FFA22_Msk                             /*!<Filter FIFO Assignment bit 22 */
#define CAN_FFA1R_FFA23_Pos    (23U)
#define CAN_FFA1R_FFA23_Msk    (0x1UL << CAN_FFA1R_FFA23_Pos)                   /*!< 0x00800000 */
#define CAN_FFA1R_FFA23        CAN_FFA1R_FFA23_Msk                             /*!<Filter FIFO Assignment bit 23 */
#define CAN_FFA1R_FFA24_Pos    (24U)
#define CAN_FFA1R_FFA24_Msk    (0x1UL << CAN_FFA1R_FFA24_Pos)                   /*!< 0x01000000 */
#define CAN_FFA1R_FFA24        CAN_FFA1R_FFA24_Msk                             /*!<Filter FIFO Assignment bit 24 */
#define CAN_FFA1R_FFA25_Pos    (25U)
#define CAN_FFA1R_FFA25_Msk    (0x1UL << CAN_FFA1R_FFA25_Pos)                   /*!< 0x02000000 */
#define CAN_FFA1R_FFA25        CAN_FFA1R_FFA25_Msk                             /*!<Filter FIFO Assignment bit 25 */
#define CAN_FFA1R_FFA26_Pos    (26U)
#define CAN_FFA1R_FFA26_Msk    (0x1UL << CAN_FFA1R_FFA26_Pos)                   /*!< 0x04000000 */
#define CAN_FFA1R_FFA26        CAN_FFA1R_FFA26_Msk                             /*!<Filter FIFO Assignment bit 26 */
#define CAN_FFA1R_FFA27_Pos    (27U)
#define CAN_FFA1R_FFA27_Msk    (0x1UL << CAN_FFA1R_FFA27_Pos)                   /*!< 0x08000000 */
#define CAN_FFA1R_FFA27        CAN_FFA1R_FFA27_Msk                             /*!<Filter FIFO Assignment bit 27 */

#define CAN_FA1R_FACT14_Pos    (14U)
#define CAN_FA1R_FACT14_Msk    (0x1UL << CAN_FA1R_FACT14_Pos)                   /*!< 0x00004000 */
#define CAN_FA1R_FACT14        CAN_FA1R_FACT14_Msk                             /*!<Filter Active bit 14 */
#define CAN_FA1R_FACT15_Pos    (15U)
#define CAN_FA1R_FACT15_Msk    (0x1UL << CAN_FA1R_FACT15_Pos)                   /*!< 0x00008000 */
#define CAN_FA1R_FACT15        CAN_FA1R_FACT15_Msk                             /*!<Filter Active bit 15 */
#define CAN_FA1R_FACT16_Pos    (16U)
#define CAN_FA1R_FACT16_Msk    (0x1UL << CAN_FA1R_FACT16_Pos)                   /*!< 0x00010000 */
#define CAN_FA1R_FACT16        CAN_FA1R_FACT16_Msk                             /*!<Filter Active bit 16 */
#define CAN_FA1R_FACT17_Pos    (17U)
#define CAN_FA1R_FACT17_Msk    (0x1UL << CAN_FA1R_FACT17_Pos)                   /*!< 0x00020000 */
#define CAN_FA1R_FACT17        CAN_FA1R_FACT17_Msk                             /*!<Filter Active bit 17 */
#define CAN_FA1R_FACT18_Pos    (18U)
#define CAN_FA1R_FACT18_Msk    (0x1UL << CAN_FA1R_FACT18_Pos)                   /*!< 0x00040000 */
#define CAN_FA1R_FACT18        CAN_FA1R_FACT18_Msk                             /*!<Filter Active bit 18 */
#define CAN_FA1R_FACT19_Pos    (19U)
#define CAN_FA1R_FACT19_Msk    (0x1UL << CAN_FA1R_FACT19_Pos)                   /*!< 0x00080000 */
#define CAN_FA1R_FACT19        CAN_FA1R_FACT19_Msk                             /*!<Filter Active bit 19 */
#define CAN_FA1R_FACT20_Pos    (20U)
#define CAN_FA1R_FACT20_Msk    (0x1UL << CAN_FA1R_FACT20_Pos)                   /*!< 0x00100000 */
#define CAN_FA1R_FACT20        CAN_FA1R_FACT20_Msk                             /*!<Filter Active bit 20 */
#define CAN_FA1R_FACT21_Pos    (21U)
#define CAN_FA1R_FACT21_Msk    (0x1UL << CAN_FA1R_FACT21_Pos)                   /*!< 0x00200000 */
#define CAN_FA1R_FACT21        CAN_FA1R_FACT21_Msk                             /*!<Filter Active bit 21 */
#define CAN_FA1R_FACT22_Pos    (22U)
#define CAN_FA1R_FACT22_Msk    (0x1UL << CAN_FA1R_FACT22_Pos)                   /*!< 0x00400000 */
#define CAN_FA1R_FACT22        CAN_FA1R_FACT22_Msk                             /*!<Filter Active bit 22 */
#define CAN_FA1R_FACT23_Pos    (23U)
#define CAN_FA1R_FACT23_Msk    (0x1UL << CAN_FA1R_FACT23_Pos)                   /*!< 0x00800000 */
#define CAN_FA1R_FACT23        CAN_FA1R_FACT23_Msk                             /*!<Filter Active bit 23 */
#define CAN_FA1R_FACT24_Pos    (24U)
#define CAN_FA1R_FACT24_Msk    (0x1UL << CAN_FA1R_FACT24_Pos)                   /*!< 0x01000000 */
#define CAN_FA1R_FACT24        CAN_FA1R_FACT24_Msk                             /*!<Filter Active bit 24 */
#define CAN_FA1R_FACT25_Pos    (25U)
#define CAN_FA1R_FACT25_Msk    (0x1UL << CAN_FA1R_FACT25_Pos)                   /*!< 0x02000000 */
#define CAN_FA1R_FACT25        CAN_FA1R_FACT25_Msk                             /*!<Filter Active bit 25 */
#define CAN_FA1R_FACT26_Pos    (26U)
#define CAN_FA1R_FACT26_Msk    (0x1UL << CAN_FA1R_FACT26_Pos)                   /*!< 0x04000000 */
#define CAN_FA1R_FACT26        CAN_FA1R_FACT26_Msk                             /*!<Filter Active bit 26 */
#define CAN_FA1R_FACT27_Pos    (27U)
#define CAN_FA1R_FACT27_Msk    (0x1UL << CAN_FA1R_FACT27_Pos)                   /*!< 0x08000000 */
#define CAN_FA1R_FACT27        CAN_FA1R_FACT27_Msk                             /*!<Filter Active bit 27 */

/******************************************************************************/
/*                                                                            */
/*                                 HDMI-CEC (CEC)                             */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for CEC_CR register  *********************/
#define CEC_CR_CECEN_Pos         (0U)
#define CEC_CR_CECEN_Msk         (0x1UL << CEC_CR_CECEN_Pos)                    /*!< 0x00000001 */
#define CEC_CR_CECEN             CEC_CR_CECEN_Msk                              /*!< CEC Enable                         */
#define CEC_CR_TXSOM_Pos         (1U)
#define CEC_CR_TXSOM_Msk         (0x1UL << CEC_CR_TXSOM_Pos)                    /*!< 0x00000002 */
#define CEC_CR_TXSOM             CEC_CR_TXSOM_Msk                              /*!< CEC Tx Start Of Message            */
#define CEC_CR_TXEOM_Pos         (2U)
#define CEC_CR_TXEOM_Msk         (0x1UL << CEC_CR_TXEOM_Pos)                    /*!< 0x00000004 */
#define CEC_CR_TXEOM             CEC_CR_TXEOM_Msk                              /*!< CEC Tx End Of Message              */

/*******************  Bit definition for CEC_CFGR register  *******************/
#define CEC_CFGR_SFT_Pos         (0U)
#define CEC_CFGR_SFT_Msk         (0x7UL << CEC_CFGR_SFT_Pos)                    /*!< 0x00000007 */
#define CEC_CFGR_SFT             CEC_CFGR_SFT_Msk                              /*!< CEC Signal Free Time               */
#define CEC_CFGR_RXTOL_Pos       (3U)
#define CEC_CFGR_RXTOL_Msk       (0x1UL << CEC_CFGR_RXTOL_Pos)                  /*!< 0x00000008 */
#define CEC_CFGR_RXTOL           CEC_CFGR_RXTOL_Msk                            /*!< CEC Tolerance                      */
#define CEC_CFGR_BRESTP_Pos      (4U)
#define CEC_CFGR_BRESTP_Msk      (0x1UL << CEC_CFGR_BRESTP_Pos)                 /*!< 0x00000010 */
#define CEC_CFGR_BRESTP          CEC_CFGR_BRESTP_Msk                           /*!< CEC Rx Stop                        */
#define CEC_CFGR_BREGEN_Pos      (5U)
#define CEC_CFGR_BREGEN_Msk      (0x1UL << CEC_CFGR_BREGEN_Pos)                 /*!< 0x00000020 */
#define CEC_CFGR_BREGEN          CEC_CFGR_BREGEN_Msk                           /*!< CEC Bit Rising Error generation    */
#define CEC_CFGR_LBPEGEN_Pos     (6U)
#define CEC_CFGR_LBPEGEN_Msk     (0x1UL << CEC_CFGR_LBPEGEN_Pos)                /*!< 0x00000040 */
#define CEC_CFGR_LBPEGEN         CEC_CFGR_LBPEGEN_Msk                          /*!< CEC Long Bit Period Error gener.   */
#define CEC_CFGR_BRDNOGEN_Pos    (7U)
#define CEC_CFGR_BRDNOGEN_Msk    (0x1UL << CEC_CFGR_BRDNOGEN_Pos)               /*!< 0x00000080 */
#define CEC_CFGR_BRDNOGEN        CEC_CFGR_BRDNOGEN_Msk                         /*!< CEC Broadcast No Error generation  */
#define CEC_CFGR_SFTOPT_Pos      (8U)
#define CEC_CFGR_SFTOPT_Msk      (0x1UL << CEC_CFGR_SFTOPT_Pos)                 /*!< 0x00000100 */
#define CEC_CFGR_SFTOPT          CEC_CFGR_SFTOPT_Msk                           /*!< CEC Signal Free Time optional      */
#define CEC_CFGR_OAR_Pos         (16U)
#define CEC_CFGR_OAR_Msk         (0x7FFFUL << CEC_CFGR_OAR_Pos)                 /*!< 0x7FFF0000 */
#define CEC_CFGR_OAR             CEC_CFGR_OAR_Msk                              /*!< CEC Own Address                    */
#define CEC_CFGR_LSTN_Pos        (31U)
#define CEC_CFGR_LSTN_Msk        (0x1UL << CEC_CFGR_LSTN_Pos)                   /*!< 0x80000000 */
#define CEC_CFGR_LSTN            CEC_CFGR_LSTN_Msk                             /*!< CEC Listen mode                    */

/*******************  Bit definition for CEC_TXDR register  *******************/
#define CEC_TXDR_TXD_Pos         (0U)
#define CEC_TXDR_TXD_Msk         (0xFFUL << CEC_TXDR_TXD_Pos)                   /*!< 0x000000FF */
#define CEC_TXDR_TXD             CEC_TXDR_TXD_Msk                              /*!< CEC Tx Data                        */

/*******************  Bit definition for CEC_RXDR register  *******************/
#define CEC_RXDR_RXD_Pos         (0U)
#define CEC_RXDR_RXD_Msk         (0xFFUL << CEC_RXDR_RXD_Pos)                   /*!< 0x000000FF */
#define CEC_RXDR_RXD             CEC_RXDR_RXD_Msk                              /*!< CEC Rx Data                        */
/* Legacy aliases */
#define CEC_TXDR_RXD_Pos         CEC_RXDR_RXD_Pos    
#define CEC_TXDR_RXD_Msk         CEC_RXDR_RXD_Msk
#define CEC_TXDR_RXD             CEC_RXDR_RXD
/*******************  Bit definition for CEC_ISR register  ********************/
#define CEC_ISR_RXBR_Pos         (0U)
#define CEC_ISR_RXBR_Msk         (0x1UL << CEC_ISR_RXBR_Pos)                    /*!< 0x00000001 */
#define CEC_ISR_RXBR             CEC_ISR_RXBR_Msk                              /*!< CEC Rx-Byte Received                   */
#define CEC_ISR_RXEND_Pos        (1U)
#define CEC_ISR_RXEND_Msk        (0x1UL << CEC_ISR_RXEND_Pos)                   /*!< 0x00000002 */
#define CEC_ISR_RXEND            CEC_ISR_RXEND_Msk                             /*!< CEC End Of Reception                   */
#define CEC_ISR_RXOVR_Pos        (2U)
#define CEC_ISR_RXOVR_Msk        (0x1UL << CEC_ISR_RXOVR_Pos)                   /*!< 0x00000004 */
#define CEC_ISR_RXOVR            CEC_ISR_RXOVR_Msk                             /*!< CEC Rx-Overrun                         */
#define CEC_ISR_BRE_Pos          (3U)
#define CEC_ISR_BRE_Msk          (0x1UL << CEC_ISR_BRE_Pos)                     /*!< 0x00000008 */
#define CEC_ISR_BRE              CEC_ISR_BRE_Msk                               /*!< CEC Rx Bit Rising Error                */
#define CEC_ISR_SBPE_Pos         (4U)
#define CEC_ISR_SBPE_Msk         (0x1UL << CEC_ISR_SBPE_Pos)                    /*!< 0x00000010 */
#define CEC_ISR_SBPE             CEC_ISR_SBPE_Msk                              /*!< CEC Rx Short Bit period Error          */
#define CEC_ISR_LBPE_Pos         (5U)
#define CEC_ISR_LBPE_Msk         (0x1UL << CEC_ISR_LBPE_Pos)                    /*!< 0x00000020 */
#define CEC_ISR_LBPE             CEC_ISR_LBPE_Msk                              /*!< CEC Rx Long Bit period Error           */
#define CEC_ISR_RXACKE_Pos       (6U)
#define CEC_ISR_RXACKE_Msk       (0x1UL << CEC_ISR_RXACKE_Pos)                  /*!< 0x00000040 */
#define CEC_ISR_RXACKE           CEC_ISR_RXACKE_Msk                            /*!< CEC Rx Missing Acknowledge             */
#define CEC_ISR_ARBLST_Pos       (7U)
#define CEC_ISR_ARBLST_Msk       (0x1UL << CEC_ISR_ARBLST_Pos)                  /*!< 0x00000080 */
#define CEC_ISR_ARBLST           CEC_ISR_ARBLST_Msk                            /*!< CEC Arbitration Lost                   */
#define CEC_ISR_TXBR_Pos         (8U)
#define CEC_ISR_TXBR_Msk         (0x1UL << CEC_ISR_TXBR_Pos)                    /*!< 0x00000100 */
#define CEC_ISR_TXBR             CEC_ISR_TXBR_Msk                              /*!< CEC Tx Byte Request                    */
#define CEC_ISR_TXEND_Pos        (9U)
#define CEC_ISR_TXEND_Msk        (0x1UL << CEC_ISR_TXEND_Pos)                   /*!< 0x00000200 */
#define CEC_ISR_TXEND            CEC_ISR_TXEND_Msk                             /*!< CEC End of Transmission                */
#define CEC_ISR_TXUDR_Pos        (10U)
#define CEC_ISR_TXUDR_Msk        (0x1UL << CEC_ISR_TXUDR_Pos)                   /*!< 0x00000400 */
#define CEC_ISR_TXUDR            CEC_ISR_TXUDR_Msk                             /*!< CEC Tx-Buffer Underrun                 */
#define CEC_ISR_TXERR_Pos        (11U)
#define CEC_ISR_TXERR_Msk        (0x1UL << CEC_ISR_TXERR_Pos)                   /*!< 0x00000800 */
#define CEC_ISR_TXERR            CEC_ISR_TXERR_Msk                             /*!< CEC Tx-Error                           */
#define CEC_ISR_TXACKE_Pos       (12U)
#define CEC_ISR_TXACKE_Msk       (0x1UL << CEC_ISR_TXACKE_Pos)                  /*!< 0x00001000 */
#define CEC_ISR_TXACKE           CEC_ISR_TXACKE_Msk                            /*!< CEC Tx Missing Acknowledge             */

/*******************  Bit definition for CEC_IER register  ********************/
#define CEC_IER_RXBRIE_Pos       (0U)
#define CEC_IER_RXBRIE_Msk       (0x1UL << CEC_IER_RXBRIE_Pos)                  /*!< 0x00000001 */
#define CEC_IER_RXBRIE           CEC_IER_RXBRIE_Msk                            /*!< CEC Rx-Byte Received IT Enable         */
#define CEC_IER_RXENDIE_Pos      (1U)
#define CEC_IER_RXENDIE_Msk      (0x1UL << CEC_IER_RXENDIE_Pos)                 /*!< 0x00000002 */
#define CEC_IER_RXENDIE          CEC_IER_RXENDIE_Msk                           /*!< CEC End Of Reception IT Enable         */
#define CEC_IER_RXOVRIE_Pos      (2U)
#define CEC_IER_RXOVRIE_Msk      (0x1UL << CEC_IER_RXOVRIE_Pos)                 /*!< 0x00000004 */
#define CEC_IER_RXOVRIE          CEC_IER_RXOVRIE_Msk                           /*!< CEC Rx-Overrun IT Enable               */
#define CEC_IER_BREIE_Pos        (3U)
#define CEC_IER_BREIE_Msk        (0x1UL << CEC_IER_BREIE_Pos)                   /*!< 0x00000008 */
#define CEC_IER_BREIE            CEC_IER_BREIE_Msk                             /*!< CEC Rx Bit Rising Error IT Enable      */
#define CEC_IER_SBPEIE_Pos       (4U)
#define CEC_IER_SBPEIE_Msk       (0x1UL << CEC_IER_SBPEIE_Pos)                  /*!< 0x00000010 */
#define CEC_IER_SBPEIE           CEC_IER_SBPEIE_Msk                            /*!< CEC Rx Short Bit period Error IT Enable*/
#define CEC_IER_LBPEIE_Pos       (5U)
#define CEC_IER_LBPEIE_Msk       (0x1UL << CEC_IER_LBPEIE_Pos)                  /*!< 0x00000020 */
#define CEC_IER_LBPEIE           CEC_IER_LBPEIE_Msk                            /*!< CEC Rx Long Bit period Error IT Enable */
#define CEC_IER_RXACKEIE_Pos     (6U)
#define CEC_IER_RXACKEIE_Msk     (0x1UL << CEC_IER_RXACKEIE_Pos)                /*!< 0x00000040 */
#define CEC_IER_RXACKEIE         CEC_IER_RXACKEIE_Msk                          /*!< CEC Rx Missing Acknowledge IT Enable   */
#define CEC_IER_ARBLSTIE_Pos     (7U)
#define CEC_IER_ARBLSTIE_Msk     (0x1UL << CEC_IER_ARBLSTIE_Pos)                /*!< 0x00000080 */
#define CEC_IER_ARBLSTIE         CEC_IER_ARBLSTIE_Msk                          /*!< CEC Arbitration Lost IT Enable         */
#define CEC_IER_TXBRIE_Pos       (8U)
#define CEC_IER_TXBRIE_Msk       (0x1UL << CEC_IER_TXBRIE_Pos)                  /*!< 0x00000100 */
#define CEC_IER_TXBRIE           CEC_IER_TXBRIE_Msk                            /*!< CEC Tx Byte Request  IT Enable         */
#define CEC_IER_TXENDIE_Pos      (9U)
#define CEC_IER_TXENDIE_Msk      (0x1UL << CEC_IER_TXENDIE_Pos)                 /*!< 0x00000200 */
#define CEC_IER_TXENDIE          CEC_IER_TXENDIE_Msk                           /*!< CEC End of Transmission IT Enable      */
#define CEC_IER_TXUDRIE_Pos      (10U)
#define CEC_IER_TXUDRIE_Msk      (0x1UL << CEC_IER_TXUDRIE_Pos)                 /*!< 0x00000400 */
#define CEC_IER_TXUDRIE          CEC_IER_TXUDRIE_Msk                           /*!< CEC Tx-Buffer Underrun IT Enable       */
#define CEC_IER_TXERRIE_Pos      (11U)
#define CEC_IER_TXERRIE_Msk      (0x1UL << CEC_IER_TXERRIE_Pos)                 /*!< 0x00000800 */
#define CEC_IER_TXERRIE          CEC_IER_TXERRIE_Msk                           /*!< CEC Tx-Error IT Enable                 */
#define CEC_IER_TXACKEIE_Pos     (12U)
#define CEC_IER_TXACKEIE_Msk     (0x1UL << CEC_IER_TXACKEIE_Pos)                /*!< 0x00001000 */
#define CEC_IER_TXACKEIE         CEC_IER_TXACKEIE_Msk                          /*!< CEC Tx Missing Acknowledge IT Enable   */

/******************************************************************************/
/*                                                                            */
/*                       CRC calculation unit (CRC)                           */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for CRC_DR register  *********************/
#define CRC_DR_DR_Pos            (0U)
#define CRC_DR_DR_Msk            (0xFFFFFFFFUL << CRC_DR_DR_Pos)                /*!< 0xFFFFFFFF */
#define CRC_DR_DR                CRC_DR_DR_Msk                                 /*!< Data register bits */

/*******************  Bit definition for CRC_IDR register  ********************/
#define CRC_IDR_IDR              ((uint8_t)0xFFU)                              /*!< General-purpose 8-bit data register bits */

/********************  Bit definition for CRC_CR register  ********************/
#define CRC_CR_RESET_Pos         (0U)
#define CRC_CR_RESET_Msk         (0x1UL << CRC_CR_RESET_Pos)                    /*!< 0x00000001 */
#define CRC_CR_RESET             CRC_CR_RESET_Msk                              /*!< RESET the CRC computation unit bit */
#define CRC_CR_REV_IN_Pos        (5U)
#define CRC_CR_REV_IN_Msk        (0x3UL << CRC_CR_REV_IN_Pos)                   /*!< 0x00000060 */
#define CRC_CR_REV_IN            CRC_CR_REV_IN_Msk                             /*!< REV_IN Reverse Input Data bits */
#define CRC_CR_REV_IN_0          (0x1UL << CRC_CR_REV_IN_Pos)                   /*!< 0x00000020 */
#define CRC_CR_REV_IN_1          (0x2UL << CRC_CR_REV_IN_Pos)                   /*!< 0x00000040 */
#define CRC_CR_REV_OUT_Pos       (7U)
#define CRC_CR_REV_OUT_Msk       (0x1UL << CRC_CR_REV_OUT_Pos)                  /*!< 0x00000080 */
#define CRC_CR_REV_OUT           CRC_CR_REV_OUT_Msk                            /*!< REV_OUT Reverse Output Data bits */

/*******************  Bit definition for CRC_INIT register  *******************/
#define CRC_INIT_INIT_Pos        (0U)
#define CRC_INIT_INIT_Msk        (0xFFFFFFFFUL << CRC_INIT_INIT_Pos)            /*!< 0xFFFFFFFF */
#define CRC_INIT_INIT            CRC_INIT_INIT_Msk                             /*!< Initial CRC value bits */

/******************************************************************************/
/*                                                                            */
/*                          CRS Clock Recovery System                         */
/******************************************************************************/

/*******************  Bit definition for CRS_CR register  *********************/
#define CRS_CR_SYNCOKIE_Pos       (0U)
#define CRS_CR_SYNCOKIE_Msk       (0x1UL << CRS_CR_SYNCOKIE_Pos)                /*!< 0x00000001 */
#define CRS_CR_SYNCOKIE           CRS_CR_SYNCOKIE_Msk                          /* SYNC event OK interrupt enable        */
#define CRS_CR_SYNCWARNIE_Pos     (1U)
#define CRS_CR_SYNCWARNIE_Msk     (0x1UL << CRS_CR_SYNCWARNIE_Pos)              /*!< 0x00000002 */
#define CRS_CR_SYNCWARNIE         CRS_CR_SYNCWARNIE_Msk                        /* SYNC warning interrupt enable         */
#define CRS_CR_ERRIE_Pos          (2U)
#define CRS_CR_ERRIE_Msk          (0x1UL << CRS_CR_ERRIE_Pos)                   /*!< 0x00000004 */
#define CRS_CR_ERRIE              CRS_CR_ERRIE_Msk                             /* SYNC error interrupt enable           */
#define CRS_CR_ESYNCIE_Pos        (3U)
#define CRS_CR_ESYNCIE_Msk        (0x1UL << CRS_CR_ESYNCIE_Pos)                 /*!< 0x00000008 */
#define CRS_CR_ESYNCIE            CRS_CR_ESYNCIE_Msk                           /* Expected SYNC(ESYNCF) interrupt Enable*/
#define CRS_CR_CEN_Pos            (5U)
#define CRS_CR_CEN_Msk            (0x1UL << CRS_CR_CEN_Pos)                     /*!< 0x00000020 */
#define CRS_CR_CEN                CRS_CR_CEN_Msk                               /* Frequency error counter enable        */
#define CRS_CR_AUTOTRIMEN_Pos     (6U)
#define CRS_CR_AUTOTRIMEN_Msk     (0x1UL << CRS_CR_AUTOTRIMEN_Pos)              /*!< 0x00000040 */
#define CRS_CR_AUTOTRIMEN         CRS_CR_AUTOTRIMEN_Msk                        /* Automatic trimming enable             */
#define CRS_CR_SWSYNC_Pos         (7U)
#define CRS_CR_SWSYNC_Msk         (0x1UL << CRS_CR_SWSYNC_Pos)                  /*!< 0x00000080 */
#define CRS_CR_SWSYNC             CRS_CR_SWSYNC_Msk                            /* A Software SYNC event is generated    */
#define CRS_CR_TRIM_Pos           (8U)
#define CRS_CR_TRIM_Msk           (0x3FUL << CRS_CR_TRIM_Pos)                   /*!< 0x00003F00 */
#define CRS_CR_TRIM               CRS_CR_TRIM_Msk                              /* HSI48 oscillator smooth trimming      */

/*******************  Bit definition for CRS_CFGR register  *********************/
#define CRS_CFGR_RELOAD_Pos       (0U)
#define CRS_CFGR_RELOAD_Msk       (0xFFFFUL << CRS_CFGR_RELOAD_Pos)             /*!< 0x0000FFFF */
#define CRS_CFGR_RELOAD           CRS_CFGR_RELOAD_Msk                          /* Counter reload value               */
#define CRS_CFGR_FELIM_Pos        (16U)
#define CRS_CFGR_FELIM_Msk        (0xFFUL << CRS_CFGR_FELIM_Pos)                /*!< 0x00FF0000 */
#define CRS_CFGR_FELIM            CRS_CFGR_FELIM_Msk                           /* Frequency error limit              */

#define CRS_CFGR_SYNCDIV_Pos      (24U)
#define CRS_CFGR_SYNCDIV_Msk      (0x7UL << CRS_CFGR_SYNCDIV_Pos)               /*!< 0x07000000 */
#define CRS_CFGR_SYNCDIV          CRS_CFGR_SYNCDIV_Msk                         /* SYNC divider                       */
#define CRS_CFGR_SYNCDIV_0        (0x1UL << CRS_CFGR_SYNCDIV_Pos)               /*!< 0x01000000 */
#define CRS_CFGR_SYNCDIV_1        (0x2UL << CRS_CFGR_SYNCDIV_Pos)               /*!< 0x02000000 */
#define CRS_CFGR_SYNCDIV_2        (0x4UL << CRS_CFGR_SYNCDIV_Pos)               /*!< 0x04000000 */

#define CRS_CFGR_SYNCSRC_Pos      (28U)
#define CRS_CFGR_SYNCSRC_Msk      (0x3UL << CRS_CFGR_SYNCSRC_Pos)               /*!< 0x30000000 */
#define CRS_CFGR_SYNCSRC          CRS_CFGR_SYNCSRC_Msk                         /* SYNC signal source selection       */
#define CRS_CFGR_SYNCSRC_0        (0x1UL << CRS_CFGR_SYNCSRC_Pos)               /*!< 0x10000000 */
#define CRS_CFGR_SYNCSRC_1        (0x2UL << CRS_CFGR_SYNCSRC_Pos)               /*!< 0x20000000 */

#define CRS_CFGR_SYNCPOL_Pos      (31U)
#define CRS_CFGR_SYNCPOL_Msk      (0x1UL << CRS_CFGR_SYNCPOL_Pos)               /*!< 0x80000000 */
#define CRS_CFGR_SYNCPOL          CRS_CFGR_SYNCPOL_Msk                         /* SYNC polarity selection            */

/*******************  Bit definition for CRS_ISR register  *********************/
#define CRS_ISR_SYNCOKF_Pos       (0U)
#define CRS_ISR_SYNCOKF_Msk       (0x1UL << CRS_ISR_SYNCOKF_Pos)                /*!< 0x00000001 */
#define CRS_ISR_SYNCOKF           CRS_ISR_SYNCOKF_Msk                          /* SYNC event OK flag             */
#define CRS_ISR_SYNCWARNF_Pos     (1U)
#define CRS_ISR_SYNCWARNF_Msk     (0x1UL << CRS_ISR_SYNCWARNF_Pos)              /*!< 0x00000002 */
#define CRS_ISR_SYNCWARNF         CRS_ISR_SYNCWARNF_Msk                        /* SYNC warning                   */
#define CRS_ISR_ERRF_Pos          (2U)
#define CRS_ISR_ERRF_Msk          (0x1UL << CRS_ISR_ERRF_Pos)                   /*!< 0x00000004 */
#define CRS_ISR_ERRF              CRS_ISR_ERRF_Msk                             /* SYNC error flag                */
#define CRS_ISR_ESYNCF_Pos        (3U)
#define CRS_ISR_ESYNCF_Msk        (0x1UL << CRS_ISR_ESYNCF_Pos)                 /*!< 0x00000008 */
#define CRS_ISR_ESYNCF            CRS_ISR_ESYNCF_Msk                           /* Expected SYNC flag             */
#define CRS_ISR_SYNCERR_Pos       (8U)
#define CRS_ISR_SYNCERR_Msk       (0x1UL << CRS_ISR_SYNCERR_Pos)                /*!< 0x00000100 */
#define CRS_ISR_SYNCERR           CRS_ISR_SYNCERR_Msk                          /* SYNC error                     */
#define CRS_ISR_SYNCMISS_Pos      (9U)
#define CRS_ISR_SYNCMISS_Msk      (0x1UL << CRS_ISR_SYNCMISS_Pos)               /*!< 0x00000200 */
#define CRS_ISR_SYNCMISS          CRS_ISR_SYNCMISS_Msk                         /* SYNC missed                    */
#define CRS_ISR_TRIMOVF_Pos       (10U)
#define CRS_ISR_TRIMOVF_Msk       (0x1UL << CRS_ISR_TRIMOVF_Pos)                /*!< 0x00000400 */
#define CRS_ISR_TRIMOVF           CRS_ISR_TRIMOVF_Msk                          /* Trimming overflow or underflow */
#define CRS_ISR_FEDIR_Pos         (15U)
#define CRS_ISR_FEDIR_Msk         (0x1UL << CRS_ISR_FEDIR_Pos)                  /*!< 0x00008000 */
#define CRS_ISR_FEDIR             CRS_ISR_FEDIR_Msk                            /* Frequency error direction      */
#define CRS_ISR_FECAP_Pos         (16U)
#define CRS_ISR_FECAP_Msk         (0xFFFFUL << CRS_ISR_FECAP_Pos)               /*!< 0xFFFF0000 */
#define CRS_ISR_FECAP             CRS_ISR_FECAP_Msk                            /* Frequency error capture        */

/*******************  Bit definition for CRS_ICR register  *********************/
#define CRS_ICR_SYNCOKC_Pos       (0U)
#define CRS_ICR_SYNCOKC_Msk       (0x1UL << CRS_ICR_SYNCOKC_Pos)                /*!< 0x00000001 */
#define CRS_ICR_SYNCOKC           CRS_ICR_SYNCOKC_Msk                          /* SYNC event OK clear flag     */
#define CRS_ICR_SYNCWARNC_Pos     (1U)
#define CRS_ICR_SYNCWARNC_Msk     (0x1UL << CRS_ICR_SYNCWARNC_Pos)              /*!< 0x00000002 */
#define CRS_ICR_SYNCWARNC         CRS_ICR_SYNCWARNC_Msk                        /* SYNC warning clear flag      */
#define CRS_ICR_ERRC_Pos          (2U)
#define CRS_ICR_ERRC_Msk          (0x1UL << CRS_ICR_ERRC_Pos)                   /*!< 0x00000004 */
#define CRS_ICR_ERRC              CRS_ICR_ERRC_Msk                             /* Error clear flag        */
#define CRS_ICR_ESYNCC_Pos        (3U)
#define CRS_ICR_ESYNCC_Msk        (0x1UL << CRS_ICR_ESYNCC_Pos)                 /*!< 0x00000008 */
#define CRS_ICR_ESYNCC            CRS_ICR_ESYNCC_Msk                           /* Expected SYNC clear flag     */

/******************************************************************************/
/*                                                                            */
/*                           Debug MCU (DBGMCU)                               */
/*                                                                            */
/******************************************************************************/

/****************  Bit definition for DBGMCU_IDCODE register  *****************/
#define DBGMCU_IDCODE_DEV_ID_Pos                     (0U)
#define DBGMCU_IDCODE_DEV_ID_Msk                     (0xFFFUL << DBGMCU_IDCODE_DEV_ID_Pos) /*!< 0x00000FFF */
#define DBGMCU_IDCODE_DEV_ID                         DBGMCU_IDCODE_DEV_ID_Msk  /*!< Device Identifier */

#define DBGMCU_IDCODE_REV_ID_Pos                     (16U)
#define DBGMCU_IDCODE_REV_ID_Msk                     (0xFFFFUL << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0xFFFF0000 */
#define DBGMCU_IDCODE_REV_ID                         DBGMCU_IDCODE_REV_ID_Msk  /*!< REV_ID[15:0] bits (Revision Identifier) */
#define DBGMCU_IDCODE_REV_ID_0                       (0x0001UL << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0x00010000 */
#define DBGMCU_IDCODE_REV_ID_1                       (0x0002UL << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0x00020000 */
#define DBGMCU_IDCODE_REV_ID_2                       (0x0004UL << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0x00040000 */
#define DBGMCU_IDCODE_REV_ID_3                       (0x0008UL << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0x00080000 */
#define DBGMCU_IDCODE_REV_ID_4                       (0x0010UL << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0x00100000 */
#define DBGMCU_IDCODE_REV_ID_5                       (0x0020UL << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0x00200000 */
#define DBGMCU_IDCODE_REV_ID_6                       (0x0040UL << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0x00400000 */
#define DBGMCU_IDCODE_REV_ID_7                       (0x0080UL << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0x00800000 */
#define DBGMCU_IDCODE_REV_ID_8                       (0x0100UL << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0x01000000 */
#define DBGMCU_IDCODE_REV_ID_9                       (0x0200UL << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0x02000000 */
#define DBGMCU_IDCODE_REV_ID_10                      (0x0400UL << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0x04000000 */
#define DBGMCU_IDCODE_REV_ID_11                      (0x0800UL << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0x08000000 */
#define DBGMCU_IDCODE_REV_ID_12                      (0x1000UL << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0x10000000 */
#define DBGMCU_IDCODE_REV_ID_13                      (0x2000UL << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0x20000000 */
#define DBGMCU_IDCODE_REV_ID_14                      (0x4000UL << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0x40000000 */
#define DBGMCU_IDCODE_REV_ID_15                      (0x8000UL << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0x80000000 */

/******************  Bit definition for DBGMCU_CR register  *******************/
#define DBGMCU_CR_DBG_STOP_Pos                       (1U)
#define DBGMCU_CR_DBG_STOP_Msk                       (0x1UL << DBGMCU_CR_DBG_STOP_Pos) /*!< 0x00000002 */
#define DBGMCU_CR_DBG_STOP                           DBGMCU_CR_DBG_STOP_Msk    /*!< Debug Stop Mode */
#define DBGMCU_CR_DBG_STANDBY_Pos                    (2U)
#define DBGMCU_CR_DBG_STANDBY_Msk                    (0x1UL << DBGMCU_CR_DBG_STANDBY_Pos) /*!< 0x00000004 */
#define DBGMCU_CR_DBG_STANDBY                        DBGMCU_CR_DBG_STANDBY_Msk /*!< Debug Standby mode */

/******************  Bit definition for DBGMCU_APB1_FZ register  **************/
#define DBGMCU_APB1_FZ_DBG_TIM2_STOP_Pos             (0U)
#define DBGMCU_APB1_FZ_DBG_TIM2_STOP_Msk             (0x1UL << DBGMCU_APB1_FZ_DBG_TIM2_STOP_Pos) /*!< 0x00000001 */
#define DBGMCU_APB1_FZ_DBG_TIM2_STOP                 DBGMCU_APB1_FZ_DBG_TIM2_STOP_Msk /*!< TIM2 counter stopped when core is halted */
#define DBGMCU_APB1_FZ_DBG_TIM3_STOP_Pos             (1U)
#define DBGMCU_APB1_FZ_DBG_TIM3_STOP_Msk             (0x1UL << DBGMCU_APB1_FZ_DBG_TIM3_STOP_Pos) /*!< 0x00000002 */
#define DBGMCU_APB1_FZ_DBG_TIM3_STOP                 DBGMCU_APB1_FZ_DBG_TIM3_STOP_Msk /*!< TIM3 counter stopped when core is halted */
#define DBGMCU_APB1_FZ_DBG_TIM14_STOP_Pos            (8U)
#define DBGMCU_APB1_FZ_DBG_TIM14_STOP_Msk            (0x1UL << DBGMCU_APB1_FZ_DBG_TIM14_STOP_Pos) /*!< 0x00000100 */
#define DBGMCU_APB1_FZ_DBG_TIM14_STOP                DBGMCU_APB1_FZ_DBG_TIM14_STOP_Msk /*!< TIM14 counter stopped when core is halted */
#define DBGMCU_APB1_FZ_DBG_RTC_STOP_Pos              (10U)
#define DBGMCU_APB1_FZ_DBG_RTC_STOP_Msk              (0x1UL << DBGMCU_APB1_FZ_DBG_RTC_STOP_Pos) /*!< 0x00000400 */
#define DBGMCU_APB1_FZ_DBG_RTC_STOP                  DBGMCU_APB1_FZ_DBG_RTC_STOP_Msk /*!< RTC Calendar frozen when core is halted */
#define DBGMCU_APB1_FZ_DBG_WWDG_STOP_Pos             (11U)
#define DBGMCU_APB1_FZ_DBG_WWDG_STOP_Msk             (0x1UL << DBGMCU_APB1_FZ_DBG_WWDG_STOP_Pos) /*!< 0x00000800 */
#define DBGMCU_APB1_FZ_DBG_WWDG_STOP                 DBGMCU_APB1_FZ_DBG_WWDG_STOP_Msk /*!< Debug Window Watchdog stopped when Core is halted */
#define DBGMCU_APB1_FZ_DBG_IWDG_STOP_Pos             (12U)
#define DBGMCU_APB1_FZ_DBG_IWDG_STOP_Msk             (0x1UL << DBGMCU_APB1_FZ_DBG_IWDG_STOP_Pos) /*!< 0x00001000 */
#define DBGMCU_APB1_FZ_DBG_IWDG_STOP                 DBGMCU_APB1_FZ_DBG_IWDG_STOP_Msk /*!< Debug Independent Watchdog stopped when Core is halted */
#define DBGMCU_APB1_FZ_DBG_I2C1_SMBUS_TIMEOUT_Pos    (21U)
#define DBGMCU_APB1_FZ_DBG_I2C1_SMBUS_TIMEOUT_Msk    (0x1UL << DBGMCU_APB1_FZ_DBG_I2C1_SMBUS_TIMEOUT_Pos) /*!< 0x00200000 */
#define DBGMCU_APB1_FZ_DBG_I2C1_SMBUS_TIMEOUT        DBGMCU_APB1_FZ_DBG_I2C1_SMBUS_TIMEOUT_Msk /*!< I2C1 SMBUS timeout mode stopped when Core is halted */
#define DBGMCU_APB1_FZ_DBG_CAN_STOP_Pos              (25U)
#define DBGMCU_APB1_FZ_DBG_CAN_STOP_Msk              (0x1UL << DBGMCU_APB1_FZ_DBG_CAN_STOP_Pos) /*!< 0x02000000 */
#define DBGMCU_APB1_FZ_DBG_CAN_STOP                  DBGMCU_APB1_FZ_DBG_CAN_STOP_Msk /*!< CAN debug stopped when Core is halted  */

/******************  Bit definition for DBGMCU_APB2_FZ register  **************/
#define DBGMCU_APB2_FZ_DBG_TIM1_STOP_Pos             (11U)
#define DBGMCU_APB2_FZ_DBG_TIM1_STOP_Msk             (0x1UL << DBGMCU_APB2_FZ_DBG_TIM1_STOP_Pos) /*!< 0x00000800 */
#define DBGMCU_APB2_FZ_DBG_TIM1_STOP                 DBGMCU_APB2_FZ_DBG_TIM1_STOP_Msk /*!< TIM1 counter stopped when core is halted */
#define DBGMCU_APB2_FZ_DBG_TIM16_STOP_Pos            (17U)
#define DBGMCU_APB2_FZ_DBG_TIM16_STOP_Msk            (0x1UL << DBGMCU_APB2_FZ_DBG_TIM16_STOP_Pos) /*!< 0x00020000 */
#define DBGMCU_APB2_FZ_DBG_TIM16_STOP                DBGMCU_APB2_FZ_DBG_TIM16_STOP_Msk /*!< TIM16 counter stopped when core is halted */
#define DBGMCU_APB2_FZ_DBG_TIM17_STOP_Pos            (18U)
#define DBGMCU_APB2_FZ_DBG_TIM17_STOP_Msk            (0x1UL << DBGMCU_APB2_FZ_DBG_TIM17_STOP_Pos) /*!< 0x00040000 */
#define DBGMCU_APB2_FZ_DBG_TIM17_STOP                DBGMCU_APB2_FZ_DBG_TIM17_STOP_Msk /*!< TIM17 counter stopped when core is halted */

/******************************************************************************/
/*                                                                            */
/*                           DMA Controller (DMA)                             */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for DMA_ISR register  ********************/
#define DMA_ISR_GIF1_Pos       (0U)
#define DMA_ISR_GIF1_Msk       (0x1UL << DMA_ISR_GIF1_Pos)                      /*!< 0x00000001 */
#define DMA_ISR_GIF1           DMA_ISR_GIF1_Msk                                /*!< Channel 1 Global interrupt flag    */
#define DMA_ISR_TCIF1_Pos      (1U)
#define DMA_ISR_TCIF1_Msk      (0x1UL << DMA_ISR_TCIF1_Pos)                     /*!< 0x00000002 */
#define DMA_ISR_TCIF1          DMA_ISR_TCIF1_Msk                               /*!< Channel 1 Transfer Complete flag   */
#define DMA_ISR_HTIF1_Pos      (2U)
#define DMA_ISR_HTIF1_Msk      (0x1UL << DMA_ISR_HTIF1_Pos)                     /*!< 0x00000004 */
#define DMA_ISR_HTIF1          DMA_ISR_HTIF1_Msk                               /*!< Channel 1 Half Transfer flag       */
#define DMA_ISR_TEIF1_Pos      (3U)
#define DMA_ISR_TEIF1_Msk      (0x1UL << DMA_ISR_TEIF1_Pos)                     /*!< 0x00000008 */
#define DMA_ISR_TEIF1          DMA_ISR_TEIF1_Msk                               /*!< Channel 1 Transfer Error flag      */
#define DMA_ISR_GIF2_Pos       (4U)
#define DMA_ISR_GIF2_Msk       (0x1UL << DMA_ISR_GIF2_Pos)                      /*!< 0x00000010 */
#define DMA_ISR_GIF2           DMA_ISR_GIF2_Msk                                /*!< Channel 2 Global interrupt flag    */
#define DMA_ISR_TCIF2_Pos      (5U)
#define DMA_ISR_TCIF2_Msk      (0x1UL << DMA_ISR_TCIF2_Pos)                     /*!< 0x00000020 */
#define DMA_ISR_TCIF2          DMA_ISR_TCIF2_Msk                               /*!< Channel 2 Transfer Complete flag   */
#define DMA_ISR_HTIF2_Pos      (6U)
#define DMA_ISR_HTIF2_Msk      (0x1UL << DMA_ISR_HTIF2_Pos)                     /*!< 0x00000040 */
#define DMA_ISR_HTIF2          DMA_ISR_HTIF2_Msk                               /*!< Channel 2 Half Transfer flag       */
#define DMA_ISR_TEIF2_Pos      (7U)
#define DMA_ISR_TEIF2_Msk      (0x1UL << DMA_ISR_TEIF2_Pos)                     /*!< 0x00000080 */
#define DMA_ISR_TEIF2          DMA_ISR_TEIF2_Msk                               /*!< Channel 2 Transfer Error flag      */
#define DMA_ISR_GIF3_Pos       (8U)
#define DMA_ISR_GIF3_Msk       (0x1UL << DMA_ISR_GIF3_Pos)                      /*!< 0x00000100 */
#define DMA_ISR_GIF3           DMA_ISR_GIF3_Msk                                /*!< Channel 3 Global interrupt flag    */
#define DMA_ISR_TCIF3_Pos      (9U)
#define DMA_ISR_TCIF3_Msk      (0x1UL << DMA_ISR_TCIF3_Pos)                     /*!< 0x00000200 */
#define DMA_ISR_TCIF3          DMA_ISR_TCIF3_Msk                               /*!< Channel 3 Transfer Complete flag   */
#define DMA_ISR_HTIF3_Pos      (10U)
#define DMA_ISR_HTIF3_Msk      (0x1UL << DMA_ISR_HTIF3_Pos)                     /*!< 0x00000400 */
#define DMA_ISR_HTIF3          DMA_ISR_HTIF3_Msk                               /*!< Channel 3 Half Transfer flag       */
#define DMA_ISR_TEIF3_Pos      (11U)
#define DMA_ISR_TEIF3_Msk      (0x1UL << DMA_ISR_TEIF3_Pos)                     /*!< 0x00000800 */
#define DMA_ISR_TEIF3          DMA_ISR_TEIF3_Msk                               /*!< Channel 3 Transfer Error flag      */
#define DMA_ISR_GIF4_Pos       (12U)
#define DMA_ISR_GIF4_Msk       (0x1UL << DMA_ISR_GIF4_Pos)                      /*!< 0x00001000 */
#define DMA_ISR_GIF4           DMA_ISR_GIF4_Msk                                /*!< Channel 4 Global interrupt flag    */
#define DMA_ISR_TCIF4_Pos      (13U)
#define DMA_ISR_TCIF4_Msk      (0x1UL << DMA_ISR_TCIF4_Pos)                     /*!< 0x00002000 */
#define DMA_ISR_TCIF4          DMA_ISR_TCIF4_Msk                               /*!< Channel 4 Transfer Complete flag   */
#define DMA_ISR_HTIF4_Pos      (14U)
#define DMA_ISR_HTIF4_Msk      (0x1UL << DMA_ISR_HTIF4_Pos)                     /*!< 0x00004000 */
#define DMA_ISR_HTIF4          DMA_ISR_HTIF4_Msk                               /*!< Channel 4 Half Transfer flag       */
#define DMA_ISR_TEIF4_Pos      (15U)
#define DMA_ISR_TEIF4_Msk      (0x1UL << DMA_ISR_TEIF4_Pos)                     /*!< 0x00008000 */
#define DMA_ISR_TEIF4          DMA_ISR_TEIF4_Msk                               /*!< Channel 4 Transfer Error flag      */
#define DMA_ISR_GIF5_Pos       (16U)
#define DMA_ISR_GIF5_Msk       (0x1UL << DMA_ISR_GIF5_Pos)                      /*!< 0x00010000 */
#define DMA_ISR_GIF5           DMA_ISR_GIF5_Msk                                /*!< Channel 5 Global interrupt flag    */
#define DMA_ISR_TCIF5_Pos      (17U)
#define DMA_ISR_TCIF5_Msk      (0x1UL << DMA_ISR_TCIF5_Pos)                     /*!< 0x00020000 */
#define DMA_ISR_TCIF5          DMA_ISR_TCIF5_Msk                               /*!< Channel 5 Transfer Complete flag   */
#define DMA_ISR_HTIF5_Pos      (18U)
#define DMA_ISR_HTIF5_Msk      (0x1UL << DMA_ISR_HTIF5_Pos)                     /*!< 0x00040000 */
#define DMA_ISR_HTIF5          DMA_ISR_HTIF5_Msk                               /*!< Channel 5 Half Transfer flag       */
#define DMA_ISR_TEIF5_Pos      (19U)
#define DMA_ISR_TEIF5_Msk      (0x1UL << DMA_ISR_TEIF5_Pos)                     /*!< 0x00080000 */
#define DMA_ISR_TEIF5          DMA_ISR_TEIF5_Msk                               /*!< Channel 5 Transfer Error flag      */
#define DMA_ISR_GIF6_Pos       (20U)
#define DMA_ISR_GIF6_Msk       (0x1UL << DMA_ISR_GIF6_Pos)                      /*!< 0x00100000 */
#define DMA_ISR_GIF6           DMA_ISR_GIF6_Msk                                /*!< Channel 6 Global interrupt flag    */
#define DMA_ISR_TCIF6_Pos      (21U)
#define DMA_ISR_TCIF6_Msk      (0x1UL << DMA_ISR_TCIF6_Pos)                     /*!< 0x00200000 */
#define DMA_ISR_TCIF6          DMA_ISR_TCIF6_Msk                               /*!< Channel 6 Transfer Complete flag   */
#define DMA_ISR_HTIF6_Pos      (22U)
#define DMA_ISR_HTIF6_Msk      (0x1UL << DMA_ISR_HTIF6_Pos)                     /*!< 0x00400000 */
#define DMA_ISR_HTIF6          DMA_ISR_HTIF6_Msk                               /*!< Channel 6 Half Transfer flag       */
#define DMA_ISR_TEIF6_Pos      (23U)
#define DMA_ISR_TEIF6_Msk      (0x1UL << DMA_ISR_TEIF6_Pos)                     /*!< 0x00800000 */
#define DMA_ISR_TEIF6          DMA_ISR_TEIF6_Msk                               /*!< Channel 6 Transfer Error flag      */
#define DMA_ISR_GIF7_Pos       (24U)
#define DMA_ISR_GIF7_Msk       (0x1UL << DMA_ISR_GIF7_Pos)                      /*!< 0x01000000 */
#define DMA_ISR_GIF7           DMA_ISR_GIF7_Msk                                /*!< Channel 7 Global interrupt flag    */
#define DMA_ISR_TCIF7_Pos      (25U)
#define DMA_ISR_TCIF7_Msk      (0x1UL << DMA_ISR_TCIF7_Pos)                     /*!< 0x02000000 */
#define DMA_ISR_TCIF7          DMA_ISR_TCIF7_Msk                               /*!< Channel 7 Transfer Complete flag   */
#define DMA_ISR_HTIF7_Pos      (26U)
#define DMA_ISR_HTIF7_Msk      (0x1UL << DMA_ISR_HTIF7_Pos)                     /*!< 0x04000000 */
#define DMA_ISR_HTIF7          DMA_ISR_HTIF7_Msk                               /*!< Channel 7 Half Transfer flag       */
#define DMA_ISR_TEIF7_Pos      (27U)
#define DMA_ISR_TEIF7_Msk      (0x1UL << DMA_ISR_TEIF7_Pos)                     /*!< 0x08000000 */
#define DMA_ISR_TEIF7          DMA_ISR_TEIF7_Msk                               /*!< Channel 7 Transfer Error flag      */

/*******************  Bit definition for DMA_IFCR register  *******************/
#define DMA_IFCR_CGIF1_Pos     (0U)
#define DMA_IFCR_CGIF1_Msk     (0x1UL << DMA_IFCR_CGIF1_Pos)                    /*!< 0x00000001 */
#define DMA_IFCR_CGIF1         DMA_IFCR_CGIF1_Msk                              /*!< Channel 1 Global interrupt clear    */
#define DMA_IFCR_CTCIF1_Pos    (1U)
#define DMA_IFCR_CTCIF1_Msk    (0x1UL << DMA_IFCR_CTCIF1_Pos)                   /*!< 0x00000002 */
#define DMA_IFCR_CTCIF1        DMA_IFCR_CTCIF1_Msk                             /*!< Channel 1 Transfer Complete clear   */
#define DMA_IFCR_CHTIF1_Pos    (2U)
#define DMA_IFCR_CHTIF1_Msk    (0x1UL << DMA_IFCR_CHTIF1_Pos)                   /*!< 0x00000004 */
#define DMA_IFCR_CHTIF1        DMA_IFCR_CHTIF1_Msk                             /*!< Channel 1 Half Transfer clear       */
#define DMA_IFCR_CTEIF1_Pos    (3U)
#define DMA_IFCR_CTEIF1_Msk    (0x1UL << DMA_IFCR_CTEIF1_Pos)                   /*!< 0x00000008 */
#define DMA_IFCR_CTEIF1        DMA_IFCR_CTEIF1_Msk                             /*!< Channel 1 Transfer Error clear      */
#define DMA_IFCR_CGIF2_Pos     (4U)
#define DMA_IFCR_CGIF2_Msk     (0x1UL << DMA_IFCR_CGIF2_Pos)                    /*!< 0x00000010 */
#define DMA_IFCR_CGIF2         DMA_IFCR_CGIF2_Msk                              /*!< Channel 2 Global interrupt clear    */
#define DMA_IFCR_CTCIF2_Pos    (5U)
#define DMA_IFCR_CTCIF2_Msk    (0x1UL << DMA_IFCR_CTCIF2_Pos)                   /*!< 0x00000020 */
#define DMA_IFCR_CTCIF2        DMA_IFCR_CTCIF2_Msk                             /*!< Channel 2 Transfer Complete clear   */
#define DMA_IFCR_CHTIF2_Pos    (6U)
#define DMA_IFCR_CHTIF2_Msk    (0x1UL << DMA_IFCR_CHTIF2_Pos)                   /*!< 0x00000040 */
#define DMA_IFCR_CHTIF2        DMA_IFCR_CHTIF2_Msk                             /*!< Channel 2 Half Transfer clear       */
#define DMA_IFCR_CTEIF2_Pos    (7U)
#define DMA_IFCR_CTEIF2_Msk    (0x1UL << DMA_IFCR_CTEIF2_Pos)                   /*!< 0x00000080 */
#define DMA_IFCR_CTEIF2        DMA_IFCR_CTEIF2_Msk                             /*!< Channel 2 Transfer Error clear      */
#define DMA_IFCR_CGIF3_Pos     (8U)
#define DMA_IFCR_CGIF3_Msk     (0x1UL << DMA_IFCR_CGIF3_Pos)                    /*!< 0x00000100 */
#define DMA_IFCR_CGIF3         DMA_IFCR_CGIF3_Msk                              /*!< Channel 3 Global interrupt clear    */
#define DMA_IFCR_CTCIF3_Pos    (9U)
#define DMA_IFCR_CTCIF3_Msk    (0x1UL << DMA_IFCR_CTCIF3_Pos)                   /*!< 0x00000200 */
#define DMA_IFCR_CTCIF3        DMA_IFCR_CTCIF3_Msk                             /*!< Channel 3 Transfer Complete clear   */
#define DMA_IFCR_CHTIF3_Pos    (10U)
#define DMA_IFCR_CHTIF3_Msk    (0x1UL << DMA_IFCR_CHTIF3_Pos)                   /*!< 0x00000400 */
#define DMA_IFCR_CHTIF3        DMA_IFCR_CHTIF3_Msk                             /*!< Channel 3 Half Transfer clear       */
#define DMA_IFCR_CTEIF3_Pos    (11U)
#define DMA_IFCR_CTEIF3_Msk    (0x1UL << DMA_IFCR_CTEIF3_Pos)                   /*!< 0x00000800 */
#define DMA_IFCR_CTEIF3        DMA_IFCR_CTEIF3_Msk                             /*!< Channel 3 Transfer Error clear      */
#define DMA_IFCR_CGIF4_Pos     (12U)
#define DMA_IFCR_CGIF4_Msk     (0x1UL << DMA_IFCR_CGIF4_Pos)                    /*!< 0x00001000 */
#define DMA_IFCR_CGIF4         DMA_IFCR_CGIF4_Msk                              /*!< Channel 4 Global interrupt clear    */
#define DMA_IFCR_CTCIF4_Pos    (13U)
#define DMA_IFCR_CTCIF4_Msk    (0x1UL << DMA_IFCR_CTCIF4_Pos)                   /*!< 0x00002000 */
#define DMA_IFCR_CTCIF4        DMA_IFCR_CTCIF4_Msk                             /*!< Channel 4 Transfer Complete clear   */
#define DMA_IFCR_CHTIF4_Pos    (14U)
#define DMA_IFCR_CHTIF4_Msk    (0x1UL << DMA_IFCR_CHTIF4_Pos)                   /*!< 0x00004000 */
#define DMA_IFCR_CHTIF4        DMA_IFCR_CHTIF4_Msk                             /*!< Channel 4 Half Transfer clear       */
#define DMA_IFCR_CTEIF4_Pos    (15U)
#define DMA_IFCR_CTEIF4_Msk    (0x1UL << DMA_IFCR_CTEIF4_Pos)                   /*!< 0x00008000 */
#define DMA_IFCR_CTEIF4        DMA_IFCR_CTEIF4_Msk                             /*!< Channel 4 Transfer Error clear      */
#define DMA_IFCR_CGIF5_Pos     (16U)
#define DMA_IFCR_CGIF5_Msk     (0x1UL << DMA_IFCR_CGIF5_Pos)                    /*!< 0x00010000 */
#define DMA_IFCR_CGIF5         DMA_IFCR_CGIF5_Msk                              /*!< Channel 5 Global interrupt clear    */
#define DMA_IFCR_CTCIF5_Pos    (17U)
#define DMA_IFCR_CTCIF5_Msk    (0x1UL << DMA_IFCR_CTCIF5_Pos)                   /*!< 0x00020000 */
#define DMA_IFCR_CTCIF5        DMA_IFCR_CTCIF5_Msk                             /*!< Channel 5 Transfer Complete clear   */
#define DMA_IFCR_CHTIF5_Pos    (18U)
#define DMA_IFCR_CHTIF5_Msk    (0x1UL << DMA_IFCR_CHTIF5_Pos)                   /*!< 0x00040000 */
#define DMA_IFCR_CHTIF5        DMA_IFCR_CHTIF5_Msk                             /*!< Channel 5 Half Transfer clear       */
#define DMA_IFCR_CTEIF5_Pos    (19U)
#define DMA_IFCR_CTEIF5_Msk    (0x1UL << DMA_IFCR_CTEIF5_Pos)                   /*!< 0x00080000 */
#define DMA_IFCR_CTEIF5        DMA_IFCR_CTEIF5_Msk                             /*!< Channel 5 Transfer Error clear      */
#define DMA_IFCR_CGIF6_Pos     (20U)
#define DMA_IFCR_CGIF6_Msk     (0x1UL << DMA_IFCR_CGIF6_Pos)                    /*!< 0x00100000 */
#define DMA_IFCR_CGIF6         DMA_IFCR_CGIF6_Msk                              /*!< Channel 6 Global interrupt clear    */
#define DMA_IFCR_CTCIF6_Pos    (21U)
#define DMA_IFCR_CTCIF6_Msk    (0x1UL << DMA_IFCR_CTCIF6_Pos)                   /*!< 0x00200000 */
#define DMA_IFCR_CTCIF6        DMA_IFCR_CTCIF6_Msk                             /*!< Channel 6 Transfer Complete clear   */
#define DMA_IFCR_CHTIF6_Pos    (22U)
#define DMA_IFCR_CHTIF6_Msk    (0x1UL << DMA_IFCR_CHTIF6_Pos)                   /*!< 0x00400000 */
#define DMA_IFCR_CHTIF6        DMA_IFCR_CHTIF6_Msk                             /*!< Channel 6 Half Transfer clear       */
#define DMA_IFCR_CTEIF6_Pos    (23U)
#define DMA_IFCR_CTEIF6_Msk    (0x1UL << DMA_IFCR_CTEIF6_Pos)                   /*!< 0x00800000 */
#define DMA_IFCR_CTEIF6        DMA_IFCR_CTEIF6_Msk                             /*!< Channel 6 Transfer Error clear      */
#define DMA_IFCR_CGIF7_Pos     (24U)
#define DMA_IFCR_CGIF7_Msk     (0x1UL << DMA_IFCR_CGIF7_Pos)                    /*!< 0x01000000 */
#define DMA_IFCR_CGIF7         DMA_IFCR_CGIF7_Msk                              /*!< Channel 7 Global interrupt clear    */
#define DMA_IFCR_CTCIF7_Pos    (25U)
#define DMA_IFCR_CTCIF7_Msk    (0x1UL << DMA_IFCR_CTCIF7_Pos)                   /*!< 0x02000000 */
#define DMA_IFCR_CTCIF7        DMA_IFCR_CTCIF7_Msk                             /*!< Channel 7 Transfer Complete clear   */
#define DMA_IFCR_CHTIF7_Pos    (26U)
#define DMA_IFCR_CHTIF7_Msk    (0x1UL << DMA_IFCR_CHTIF7_Pos)                   /*!< 0x04000000 */
#define DMA_IFCR_CHTIF7        DMA_IFCR_CHTIF7_Msk                             /*!< Channel 7 Half Transfer clear       */
#define DMA_IFCR_CTEIF7_Pos    (27U)
#define DMA_IFCR_CTEIF7_Msk    (0x1UL << DMA_IFCR_CTEIF7_Pos)                   /*!< 0x08000000 */
#define DMA_IFCR_CTEIF7        DMA_IFCR_CTEIF7_Msk                             /*!< Channel 7 Transfer Error clear      */

/*******************  Bit definition for DMA_CCR register  ********************/
#define DMA_CCR_EN_Pos         (0U)
#define DMA_CCR_EN_Msk         (0x1UL << DMA_CCR_EN_Pos)                        /*!< 0x00000001 */
#define DMA_CCR_EN             DMA_CCR_EN_Msk                                  /*!< Channel enable                      */
#define DMA_CCR_TCIE_Pos       (1U)
#define DMA_CCR_TCIE_Msk       (0x1UL << DMA_CCR_TCIE_Pos)                      /*!< 0x00000002 */
#define DMA_CCR_TCIE           DMA_CCR_TCIE_Msk                                /*!< Transfer complete interrupt enable  */
#define DMA_CCR_HTIE_Pos       (2U)
#define DMA_CCR_HTIE_Msk       (0x1UL << DMA_CCR_HTIE_Pos)                      /*!< 0x00000004 */
#define DMA_CCR_HTIE           DMA_CCR_HTIE_Msk                                /*!< Half Transfer interrupt enable      */
#define DMA_CCR_TEIE_Pos       (3U)
#define DMA_CCR_TEIE_Msk       (0x1UL << DMA_CCR_TEIE_Pos)                      /*!< 0x00000008 */
#define DMA_CCR_TEIE           DMA_CCR_TEIE_Msk                                /*!< Transfer error interrupt enable     */
#define DMA_CCR_DIR_Pos        (4U)
#define DMA_CCR_DIR_Msk        (0x1UL << DMA_CCR_DIR_Pos)                       /*!< 0x00000010 */
#define DMA_CCR_DIR            DMA_CCR_DIR_Msk                                 /*!< Data transfer direction             */
#define DMA_CCR_CIRC_Pos       (5U)
#define DMA_CCR_CIRC_Msk       (0x1UL << DMA_CCR_CIRC_Pos)                      /*!< 0x00000020 */
#define DMA_CCR_CIRC           DMA_CCR_CIRC_Msk                                /*!< Circular mode                       */
#define DMA_CCR_PINC_Pos       (6U)
#define DMA_CCR_PINC_Msk       (0x1UL << DMA_CCR_PINC_Pos)                      /*!< 0x00000040 */
#define DMA_CCR_PINC           DMA_CCR_PINC_Msk                                /*!< Peripheral increment mode           */
#define DMA_CCR_MINC_Pos       (7U)
#define DMA_CCR_MINC_Msk       (0x1UL << DMA_CCR_MINC_Pos)                      /*!< 0x00000080 */
#define DMA_CCR_MINC           DMA_CCR_MINC_Msk                                /*!< Memory increment mode               */

#define DMA_CCR_PSIZE_Pos      (8U)
#define DMA_CCR_PSIZE_Msk      (0x3UL << DMA_CCR_PSIZE_Pos)                     /*!< 0x00000300 */
#define DMA_CCR_PSIZE          DMA_CCR_PSIZE_Msk                               /*!< PSIZE[1:0] bits (Peripheral size)   */
#define DMA_CCR_PSIZE_0        (0x1UL << DMA_CCR_PSIZE_Pos)                     /*!< 0x00000100 */
#define DMA_CCR_PSIZE_1        (0x2UL << DMA_CCR_PSIZE_Pos)                     /*!< 0x00000200 */

#define DMA_CCR_MSIZE_Pos      (10U)
#define DMA_CCR_MSIZE_Msk      (0x3UL << DMA_CCR_MSIZE_Pos)                     /*!< 0x00000C00 */
#define DMA_CCR_MSIZE          DMA_CCR_MSIZE_Msk                               /*!< MSIZE[1:0] bits (Memory size)       */
#define DMA_CCR_MSIZE_0        (0x1UL << DMA_CCR_MSIZE_Pos)                     /*!< 0x00000400 */
#define DMA_CCR_MSIZE_1        (0x2UL << DMA_CCR_MSIZE_Pos)                     /*!< 0x00000800 */

#define DMA_CCR_PL_Pos         (12U)
#define DMA_CCR_PL_Msk         (0x3UL << DMA_CCR_PL_Pos)                        /*!< 0x00003000 */
#define DMA_CCR_PL             DMA_CCR_PL_Msk                                  /*!< PL[1:0] bits(Channel Priority level)*/
#define DMA_CCR_PL_0           (0x1UL << DMA_CCR_PL_Pos)                        /*!< 0x00001000 */
#define DMA_CCR_PL_1           (0x2UL << DMA_CCR_PL_Pos)                        /*!< 0x00002000 */

#define DMA_CCR_MEM2MEM_Pos    (14U)
#define DMA_CCR_MEM2MEM_Msk    (0x1UL << DMA_CCR_MEM2MEM_Pos)                   /*!< 0x00004000 */
#define DMA_CCR_MEM2MEM        DMA_CCR_MEM2MEM_Msk                             /*!< Memory to memory mode               */

/******************  Bit definition for DMA_CNDTR register  *******************/
#define DMA_CNDTR_NDT_Pos      (0U)
#define DMA_CNDTR_NDT_Msk      (0xFFFFUL << DMA_CNDTR_NDT_Pos)                  /*!< 0x0000FFFF */
#define DMA_CNDTR_NDT          DMA_CNDTR_NDT_Msk                               /*!< Number of data to Transfer          */

/******************  Bit definition for DMA_CPAR register  ********************/
#define DMA_CPAR_PA_Pos        (0U)
#define DMA_CPAR_PA_Msk        (0xFFFFFFFFUL << DMA_CPAR_PA_Pos)                /*!< 0xFFFFFFFF */
#define DMA_CPAR_PA            DMA_CPAR_PA_Msk                                 /*!< Peripheral Address                  */

/******************  Bit definition for DMA_CMAR register  ********************/
#define DMA_CMAR_MA_Pos        (0U)
#define DMA_CMAR_MA_Msk        (0xFFFFFFFFUL << DMA_CMAR_MA_Pos)                /*!< 0xFFFFFFFF */
#define DMA_CMAR_MA            DMA_CMAR_MA_Msk                                 /*!< Memory Address                      */

/******************************************************************************/
/*                                                                            */
/*                 External Interrupt/Event Controller (EXTI)                 */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for EXTI_IMR register  *******************/
#define EXTI_IMR_MR0_Pos          (0U)
#define EXTI_IMR_MR0_Msk          (0x1UL << EXTI_IMR_MR0_Pos)                   /*!< 0x00000001 */
#define EXTI_IMR_MR0              EXTI_IMR_MR0_Msk                             /*!< Interrupt Mask on line 0  */
#define EXTI_IMR_MR1_Pos          (1U)
#define EXTI_IMR_MR1_Msk          (0x1UL << EXTI_IMR_MR1_Pos)                   /*!< 0x00000002 */
#define EXTI_IMR_MR1              EXTI_IMR_MR1_Msk                             /*!< Interrupt Mask on line 1  */
#define EXTI_IMR_MR2_Pos          (2U)
#define EXTI_IMR_MR2_Msk          (0x1UL << EXTI_IMR_MR2_Pos)                   /*!< 0x00000004 */
#define EXTI_IMR_MR2              EXTI_IMR_MR2_Msk                             /*!< Interrupt Mask on line 2  */
#define EXTI_IMR_MR3_Pos          (3U)
#define EXTI_IMR_MR3_Msk          (0x1UL << EXTI_IMR_MR3_Pos)                   /*!< 0x00000008 */
#define EXTI_IMR_MR3              EXTI_IMR_MR3_Msk                             /*!< Interrupt Mask on line 3  */
#define EXTI_IMR_MR4_Pos          (4U)
#define EXTI_IMR_MR4_Msk          (0x1UL << EXTI_IMR_MR4_Pos)                   /*!< 0x00000010 */
#define EXTI_IMR_MR4              EXTI_IMR_MR4_Msk                             /*!< Interrupt Mask on line 4  */
#define EXTI_IMR_MR5_Pos          (5U)
#define EXTI_IMR_MR5_Msk          (0x1UL << EXTI_IMR_MR5_Pos)                   /*!< 0x00000020 */
#define EXTI_IMR_MR5              EXTI_IMR_MR5_Msk                             /*!< Interrupt Mask on line 5  */
#define EXTI_IMR_MR6_Pos          (6U)
#define EXTI_IMR_MR6_Msk          (0x1UL << EXTI_IMR_MR6_Pos)                   /*!< 0x00000040 */
#define EXTI_IMR_MR6              EXTI_IMR_MR6_Msk                             /*!< Interrupt Mask on line 6  */
#define EXTI_IMR_MR7_Pos          (7U)
#define EXTI_IMR_MR7_Msk          (0x1UL << EXTI_IMR_MR7_Pos)                   /*!< 0x00000080 */
#define EXTI_IMR_MR7              EXTI_IMR_MR7_Msk                             /*!< Interrupt Mask on line 7  */
#define EXTI_IMR_MR8_Pos          (8U)
#define EXTI_IMR_MR8_Msk          (0x1UL << EXTI_IMR_MR8_Pos)                   /*!< 0x00000100 */
#define EXTI_IMR_MR8              EXTI_IMR_MR8_Msk                             /*!< Interrupt Mask on line 8  */
#define EXTI_IMR_MR9_Pos          (9U)
#define EXTI_IMR_MR9_Msk          (0x1UL << EXTI_IMR_MR9_Pos)                   /*!< 0x00000200 */
#define EXTI_IMR_MR9              EXTI_IMR_MR9_Msk                             /*!< Interrupt Mask on line 9  */
#define EXTI_IMR_MR10_Pos         (10U)
#define EXTI_IMR_MR10_Msk         (0x1UL << EXTI_IMR_MR10_Pos)                  /*!< 0x00000400 */
#define EXTI_IMR_MR10             EXTI_IMR_MR10_Msk                            /*!< Interrupt Mask on line 10 */
#define EXTI_IMR_MR11_Pos         (11U)
#define EXTI_IMR_MR11_Msk         (0x1UL << EXTI_IMR_MR11_Pos)                  /*!< 0x00000800 */
#define EXTI_IMR_MR11             EXTI_IMR_MR11_Msk                            /*!< Interrupt Mask on line 11 */
#define EXTI_IMR_MR12_Pos         (12U)
#define EXTI_IMR_MR12_Msk         (0x1UL << EXTI_IMR_MR12_Pos)                  /*!< 0x00001000 */
#define EXTI_IMR_MR12             EXTI_IMR_MR12_Msk                            /*!< Interrupt Mask on line 12 */
#define EXTI_IMR_MR13_Pos         (13U)
#define EXTI_IMR_MR13_Msk         (0x1UL << EXTI_IMR_MR13_Pos)                  /*!< 0x00002000 */
#define EXTI_IMR_MR13             EXTI_IMR_MR13_Msk                            /*!< Interrupt Mask on line 13 */
#define EXTI_IMR_MR14_Pos         (14U)
#define EXTI_IMR_MR14_Msk         (0x1UL << EXTI_IMR_MR14_Pos)                  /*!< 0x00004000 */
#define EXTI_IMR_MR14             EXTI_IMR_MR14_Msk                            /*!< Interrupt Mask on line 14 */
#define EXTI_IMR_MR15_Pos         (15U)
#define EXTI_IMR_MR15_Msk         (0x1UL << EXTI_IMR_MR15_Pos)                  /*!< 0x00008000 */
#define EXTI_IMR_MR15             EXTI_IMR_MR15_Msk                            /*!< Interrupt Mask on line 15 */
#define EXTI_IMR_MR16_Pos         (16U)
#define EXTI_IMR_MR16_Msk         (0x1UL << EXTI_IMR_MR16_Pos)                  /*!< 0x00010000 */
#define EXTI_IMR_MR16             EXTI_IMR_MR16_Msk                            /*!< Interrupt Mask on line 16 */
#define EXTI_IMR_MR17_Pos         (17U)
#define EXTI_IMR_MR17_Msk         (0x1UL << EXTI_IMR_MR17_Pos)                  /*!< 0x00020000 */
#define EXTI_IMR_MR17             EXTI_IMR_MR17_Msk                            /*!< Interrupt Mask on line 17 */
#define EXTI_IMR_MR18_Pos         (18U)
#define EXTI_IMR_MR18_Msk         (0x1UL << EXTI_IMR_MR18_Pos)                  /*!< 0x00040000 */
#define EXTI_IMR_MR18             EXTI_IMR_MR18_Msk                            /*!< Interrupt Mask on line 18 */
#define EXTI_IMR_MR19_Pos         (19U)
#define EXTI_IMR_MR19_Msk         (0x1UL << EXTI_IMR_MR19_Pos)                  /*!< 0x00080000 */
#define EXTI_IMR_MR19             EXTI_IMR_MR19_Msk                            /*!< Interrupt Mask on line 19 */
#define EXTI_IMR_MR21_Pos         (21U)
#define EXTI_IMR_MR21_Msk         (0x1UL << EXTI_IMR_MR21_Pos)                  /*!< 0x00200000 */
#define EXTI_IMR_MR21             EXTI_IMR_MR21_Msk                            /*!< Interrupt Mask on line 21 */
#define EXTI_IMR_MR22_Pos         (22U)
#define EXTI_IMR_MR22_Msk         (0x1UL << EXTI_IMR_MR22_Pos)                  /*!< 0x00400000 */
#define EXTI_IMR_MR22             EXTI_IMR_MR22_Msk                            /*!< Interrupt Mask on line 22 */
#define EXTI_IMR_MR23_Pos         (23U)
#define EXTI_IMR_MR23_Msk         (0x1UL << EXTI_IMR_MR23_Pos)                  /*!< 0x00800000 */
#define EXTI_IMR_MR23             EXTI_IMR_MR23_Msk                            /*!< Interrupt Mask on line 23 */
#define EXTI_IMR_MR25_Pos         (25U)
#define EXTI_IMR_MR25_Msk         (0x1UL << EXTI_IMR_MR25_Pos)                  /*!< 0x02000000 */
#define EXTI_IMR_MR25             EXTI_IMR_MR25_Msk                            /*!< Interrupt Mask on line 25 */
#define EXTI_IMR_MR27_Pos         (27U)
#define EXTI_IMR_MR27_Msk         (0x1UL << EXTI_IMR_MR27_Pos)                  /*!< 0x08000000 */
#define EXTI_IMR_MR27             EXTI_IMR_MR27_Msk                            /*!< Interrupt Mask on line 27 */
#define EXTI_IMR_MR31_Pos         (31U)
#define EXTI_IMR_MR31_Msk         (0x1UL << EXTI_IMR_MR31_Pos)                  /*!< 0x80000000 */
#define EXTI_IMR_MR31             EXTI_IMR_MR31_Msk                            /*!< Interrupt Mask on line 31 */

/* References Defines */
#define  EXTI_IMR_IM0 EXTI_IMR_MR0
#define  EXTI_IMR_IM1 EXTI_IMR_MR1
#define  EXTI_IMR_IM2 EXTI_IMR_MR2
#define  EXTI_IMR_IM3 EXTI_IMR_MR3
#define  EXTI_IMR_IM4 EXTI_IMR_MR4
#define  EXTI_IMR_IM5 EXTI_IMR_MR5
#define  EXTI_IMR_IM6 EXTI_IMR_MR6
#define  EXTI_IMR_IM7 EXTI_IMR_MR7
#define  EXTI_IMR_IM8 EXTI_IMR_MR8
#define  EXTI_IMR_IM9 EXTI_IMR_MR9
#define  EXTI_IMR_IM10 EXTI_IMR_MR10
#define  EXTI_IMR_IM11 EXTI_IMR_MR11
#define  EXTI_IMR_IM12 EXTI_IMR_MR12
#define  EXTI_IMR_IM13 EXTI_IMR_MR13
#define  EXTI_IMR_IM14 EXTI_IMR_MR14
#define  EXTI_IMR_IM15 EXTI_IMR_MR15
#define  EXTI_IMR_IM16 EXTI_IMR_MR16
#define  EXTI_IMR_IM17 EXTI_IMR_MR17
#define  EXTI_IMR_IM18 EXTI_IMR_MR18
#define  EXTI_IMR_IM19 EXTI_IMR_MR19
#define  EXTI_IMR_IM21 EXTI_IMR_MR21
#define  EXTI_IMR_IM22 EXTI_IMR_MR22
#define  EXTI_IMR_IM23 EXTI_IMR_MR23
#define  EXTI_IMR_IM25 EXTI_IMR_MR25
#define  EXTI_IMR_IM27 EXTI_IMR_MR27
#define  EXTI_IMR_IM31 EXTI_IMR_MR31

#define EXTI_IMR_IM_Pos           (0U)
#define EXTI_IMR_IM_Msk           (0x8AEFFFFFUL << EXTI_IMR_IM_Pos)             /*!< 0x8AEFFFFF */
#define EXTI_IMR_IM               EXTI_IMR_IM_Msk                              /*!< Interrupt Mask All */


/******************  Bit definition for EXTI_EMR register  ********************/
#define EXTI_EMR_MR0_Pos          (0U)
#define EXTI_EMR_MR0_Msk          (0x1UL << EXTI_EMR_MR0_Pos)                   /*!< 0x00000001 */
#define EXTI_EMR_MR0              EXTI_EMR_MR0_Msk                             /*!< Event Mask on line 0  */
#define EXTI_EMR_MR1_Pos          (1U)
#define EXTI_EMR_MR1_Msk          (0x1UL << EXTI_EMR_MR1_Pos)                   /*!< 0x00000002 */
#define EXTI_EMR_MR1              EXTI_EMR_MR1_Msk                             /*!< Event Mask on line 1  */
#define EXTI_EMR_MR2_Pos          (2U)
#define EXTI_EMR_MR2_Msk          (0x1UL << EXTI_EMR_MR2_Pos)                   /*!< 0x00000004 */
#define EXTI_EMR_MR2              EXTI_EMR_MR2_Msk                             /*!< Event Mask on line 2  */
#define EXTI_EMR_MR3_Pos          (3U)
#define EXTI_EMR_MR3_Msk          (0x1UL << EXTI_EMR_MR3_Pos)                   /*!< 0x00000008 */
#define EXTI_EMR_MR3              EXTI_EMR_MR3_Msk                             /*!< Event Mask on line 3  */
#define EXTI_EMR_MR4_Pos          (4U)
#define EXTI_EMR_MR4_Msk          (0x1UL << EXTI_EMR_MR4_Pos)                   /*!< 0x00000010 */
#define EXTI_EMR_MR4              EXTI_EMR_MR4_Msk                             /*!< Event Mask on line 4  */
#define EXTI_EMR_MR5_Pos          (5U)
#define EXTI_EMR_MR5_Msk          (0x1UL << EXTI_EMR_MR5_Pos)                   /*!< 0x00000020 */
#define EXTI_EMR_MR5              EXTI_EMR_MR5_Msk                             /*!< Event Mask on line 5  */
#define EXTI_EMR_MR6_Pos          (6U)
#define EXTI_EMR_MR6_Msk          (0x1UL << EXTI_EMR_MR6_Pos)                   /*!< 0x00000040 */
#define EXTI_EMR_MR6              EXTI_EMR_MR6_Msk                             /*!< Event Mask on line 6  */
#define EXTI_EMR_MR7_Pos          (7U)
#define EXTI_EMR_MR7_Msk          (0x1UL << EXTI_EMR_MR7_Pos)                   /*!< 0x00000080 */
#define EXTI_EMR_MR7              EXTI_EMR_MR7_Msk                             /*!< Event Mask on line 7  */
#define EXTI_EMR_MR8_Pos          (8U)
#define EXTI_EMR_MR8_Msk          (0x1UL << EXTI_EMR_MR8_Pos)                   /*!< 0x00000100 */
#define EXTI_EMR_MR8              EXTI_EMR_MR8_Msk                             /*!< Event Mask on line 8  */
#define EXTI_EMR_MR9_Pos          (9U)
#define EXTI_EMR_MR9_Msk          (0x1UL << EXTI_EMR_MR9_Pos)                   /*!< 0x00000200 */
#define EXTI_EMR_MR9              EXTI_EMR_MR9_Msk                             /*!< Event Mask on line 9  */
#define EXTI_EMR_MR10_Pos         (10U)
#define EXTI_EMR_MR10_Msk         (0x1UL << EXTI_EMR_MR10_Pos)                  /*!< 0x00000400 */
#define EXTI_EMR_MR10             EXTI_EMR_MR10_Msk                            /*!< Event Mask on line 10 */
#define EXTI_EMR_MR11_Pos         (11U)
#define EXTI_EMR_MR11_Msk         (0x1UL << EXTI_EMR_MR11_Pos)                  /*!< 0x00000800 */
#define EXTI_EMR_MR11             EXTI_EMR_MR11_Msk                            /*!< Event Mask on line 11 */
#define EXTI_EMR_MR12_Pos         (12U)
#define EXTI_EMR_MR12_Msk         (0x1UL << EXTI_EMR_MR12_Pos)                  /*!< 0x00001000 */
#define EXTI_EMR_MR12             EXTI_EMR_MR12_Msk                            /*!< Event Mask on line 12 */
#define EXTI_EMR_MR13_Pos         (13U)
#define EXTI_EMR_MR13_Msk         (0x1UL << EXTI_EMR_MR13_Pos)                  /*!< 0x00002000 */
#define EXTI_EMR_MR13             EXTI_EMR_MR13_Msk                            /*!< Event Mask on line 13 */
#define EXTI_EMR_MR14_Pos         (14U)
#define EXTI_EMR_MR14_Msk         (0x1UL << EXTI_EMR_MR14_Pos)                  /*!< 0x00004000 */
#define EXTI_EMR_MR14             EXTI_EMR_MR14_Msk                            /*!< Event Mask on line 14 */
#define EXTI_EMR_MR15_Pos         (15U)
#define EXTI_EMR_MR15_Msk         (0x1UL << EXTI_EMR_MR15_Pos)                  /*!< 0x00008000 */
#define EXTI_EMR_MR15             EXTI_EMR_MR15_Msk                            /*!< Event Mask on line 15 */
#define EXTI_EMR_MR16_Pos         (16U)
#define EXTI_EMR_MR16_Msk         (0x1UL << EXTI_EMR_MR16_Pos)                  /*!< 0x00010000 */
#define EXTI_EMR_MR16             EXTI_EMR_MR16_Msk                            /*!< Event Mask on line 16 */
#define EXTI_EMR_MR17_Pos         (17U)
#define EXTI_EMR_MR17_Msk         (0x1UL << EXTI_EMR_MR17_Pos)                  /*!< 0x00020000 */
#define EXTI_EMR_MR17             EXTI_EMR_MR17_Msk                            /*!< Event Mask on line 17 */
#define EXTI_EMR_MR18_Pos         (18U)
#define EXTI_EMR_MR18_Msk         (0x1UL << EXTI_EMR_MR18_Pos)                  /*!< 0x00040000 */
#define EXTI_EMR_MR18             EXTI_EMR_MR18_Msk                            /*!< Event Mask on line 18 */
#define EXTI_EMR_MR19_Pos         (19U)
#define EXTI_EMR_MR19_Msk         (0x1UL << EXTI_EMR_MR19_Pos)                  /*!< 0x00080000 */
#define EXTI_EMR_MR19             EXTI_EMR_MR19_Msk                            /*!< Event Mask on line 19 */
#define EXTI_EMR_MR21_Pos         (21U)
#define EXTI_EMR_MR21_Msk         (0x1UL << EXTI_EMR_MR21_Pos)                  /*!< 0x00200000 */
#define EXTI_EMR_MR21             EXTI_EMR_MR21_Msk                            /*!< Event Mask on line 21 */
#define EXTI_EMR_MR22_Pos         (22U)
#define EXTI_EMR_MR22_Msk         (0x1UL << EXTI_EMR_MR22_Pos)                  /*!< 0x00400000 */
#define EXTI_EMR_MR22             EXTI_EMR_MR22_Msk                            /*!< Event Mask on line 22 */
#define EXTI_EMR_MR23_Pos         (23U)
#define EXTI_EMR_MR23_Msk         (0x1UL << EXTI_EMR_MR23_Pos)                  /*!< 0x00800000 */
#define EXTI_EMR_MR23             EXTI_EMR_MR23_Msk                            /*!< Event Mask on line 23 */
#define EXTI_EMR_MR25_Pos         (25U)
#define EXTI_EMR_MR25_Msk         (0x1UL << EXTI_EMR_MR25_Pos)                  /*!< 0x02000000 */
#define EXTI_EMR_MR25             EXTI_EMR_MR25_Msk                            /*!< Event Mask on line 25 */
#define EXTI_EMR_MR27_Pos         (27U)
#define EXTI_EMR_MR27_Msk         (0x1UL << EXTI_EMR_MR27_Pos)                  /*!< 0x08000000 */
#define EXTI_EMR_MR27             EXTI_EMR_MR27_Msk                            /*!< Event Mask on line 27 */
#define EXTI_EMR_MR31_Pos         (31U)
#define EXTI_EMR_MR31_Msk         (0x1UL << EXTI_EMR_MR31_Pos)                  /*!< 0x80000000 */
#define EXTI_EMR_MR31             EXTI_EMR_MR31_Msk                            /*!< Event Mask on line 31 */

/* References Defines */
#define  EXTI_EMR_EM0 EXTI_EMR_MR0
#define  EXTI_EMR_EM1 EXTI_EMR_MR1
#define  EXTI_EMR_EM2 EXTI_EMR_MR2
#define  EXTI_EMR_EM3 EXTI_EMR_MR3
#define  EXTI_EMR_EM4 EXTI_EMR_MR4
#define  EXTI_EMR_EM5 EXTI_EMR_MR5
#define  EXTI_EMR_EM6 EXTI_EMR_MR6
#define  EXTI_EMR_EM7 EXTI_EMR_MR7
#define  EXTI_EMR_EM8 EXTI_EMR_MR8
#define  EXTI_EMR_EM9 EXTI_EMR_MR9
#define  EXTI_EMR_EM10 EXTI_EMR_MR10
#define  EXTI_EMR_EM11 EXTI_EMR_MR11
#define  EXTI_EMR_EM12 EXTI_EMR_MR12
#define  EXTI_EMR_EM13 EXTI_EMR_MR13
#define  EXTI_EMR_EM14 EXTI_EMR_MR14
#define  EXTI_EMR_EM15 EXTI_EMR_MR15
#define  EXTI_EMR_EM16 EXTI_EMR_MR16
#define  EXTI_EMR_EM17 EXTI_EMR_MR17
#define  EXTI_EMR_EM18 EXTI_EMR_MR18
#define  EXTI_EMR_EM19 EXTI_EMR_MR19
#define  EXTI_EMR_EM21 EXTI_EMR_MR21
#define  EXTI_EMR_EM22 EXTI_EMR_MR22
#define  EXTI_EMR_EM23 EXTI_EMR_MR23
#define  EXTI_EMR_EM25 EXTI_EMR_MR25
#define  EXTI_EMR_EM27 EXTI_EMR_MR27
#define  EXTI_EMR_EM31 EXTI_EMR_MR31

/*******************  Bit definition for EXTI_RTSR register  ******************/
#define EXTI_RTSR_TR0_Pos         (0U)
#define EXTI_RTSR_TR0_Msk         (0x1UL << EXTI_RTSR_TR0_Pos)                  /*!< 0x00000001 */
#define EXTI_RTSR_TR0             EXTI_RTSR_TR0_Msk                            /*!< Rising trigger event configuration bit of line 0 */
#define EXTI_RTSR_TR1_Pos         (1U)
#define EXTI_RTSR_TR1_Msk         (0x1UL << EXTI_RTSR_TR1_Pos)                  /*!< 0x00000002 */
#define EXTI_RTSR_TR1             EXTI_RTSR_TR1_Msk                            /*!< Rising trigger event configuration bit of line 1 */
#define EXTI_RTSR_TR2_Pos         (2U)
#define EXTI_RTSR_TR2_Msk         (0x1UL << EXTI_RTSR_TR2_Pos)                  /*!< 0x00000004 */
#define EXTI_RTSR_TR2             EXTI_RTSR_TR2_Msk                            /*!< Rising trigger event configuration bit of line 2 */
#define EXTI_RTSR_TR3_Pos         (3U)
#define EXTI_RTSR_TR3_Msk         (0x1UL << EXTI_RTSR_TR3_Pos)                  /*!< 0x00000008 */
#define EXTI_RTSR_TR3             EXTI_RTSR_TR3_Msk                            /*!< Rising trigger event configuration bit of line 3 */
#define EXTI_RTSR_TR4_Pos         (4U)
#define EXTI_RTSR_TR4_Msk         (0x1UL << EXTI_RTSR_TR4_Pos)                  /*!< 0x00000010 */
#define EXTI_RTSR_TR4             EXTI_RTSR_TR4_Msk                            /*!< Rising trigger event configuration bit of line 4 */
#define EXTI_RTSR_TR5_Pos         (5U)
#define EXTI_RTSR_TR5_Msk         (0x1UL << EXTI_RTSR_TR5_Pos)                  /*!< 0x00000020 */
#define EXTI_RTSR_TR5             EXTI_RTSR_TR5_Msk                            /*!< Rising trigger event configuration bit of line 5 */
#define EXTI_RTSR_TR6_Pos         (6U)
#define EXTI_RTSR_TR6_Msk         (0x1UL << EXTI_RTSR_TR6_Pos)                  /*!< 0x00000040 */
#define EXTI_RTSR_TR6             EXTI_RTSR_TR6_Msk                            /*!< Rising trigger event configuration bit of line 6 */
#define EXTI_RTSR_TR7_Pos         (7U)
#define EXTI_RTSR_TR7_Msk         (0x1UL << EXTI_RTSR_TR7_Pos)                  /*!< 0x00000080 */
#define EXTI_RTSR_TR7             EXTI_RTSR_TR7_Msk                            /*!< Rising trigger event configuration bit of line 7 */
#define EXTI_RTSR_TR8_Pos         (8U)
#define EXTI_RTSR_TR8_Msk         (0x1UL << EXTI_RTSR_TR8_Pos)                  /*!< 0x00000100 */
#define EXTI_RTSR_TR8             EXTI_RTSR_TR8_Msk                            /*!< Rising trigger event configuration bit of line 8 */
#define EXTI_RTSR_TR9_Pos         (9U)
#define EXTI_RTSR_TR9_Msk         (0x1UL << EXTI_RTSR_TR9_Pos)                  /*!< 0x00000200 */
#define EXTI_RTSR_TR9             EXTI_RTSR_TR9_Msk                            /*!< Rising trigger event configuration bit of line 9 */
#define EXTI_RTSR_TR10_Pos        (10U)
#define EXTI_RTSR_TR10_Msk        (0x1UL << EXTI_RTSR_TR10_Pos)                 /*!< 0x00000400 */
#define EXTI_RTSR_TR10            EXTI_RTSR_TR10_Msk                           /*!< Rising trigger event configuration bit of line 10 */
#define EXTI_RTSR_TR11_Pos        (11U)
#define EXTI_RTSR_TR11_Msk        (0x1UL << EXTI_RTSR_TR11_Pos)                 /*!< 0x00000800 */
#define EXTI_RTSR_TR11            EXTI_RTSR_TR11_Msk                           /*!< Rising trigger event configuration bit of line 11 */
#define EXTI_RTSR_TR12_Pos        (12U)
#define EXTI_RTSR_TR12_Msk        (0x1UL << EXTI_RTSR_TR12_Pos)                 /*!< 0x00001000 */
#define EXTI_RTSR_TR12            EXTI_RTSR_TR12_Msk                           /*!< Rising trigger event configuration bit of line 12 */
#define EXTI_RTSR_TR13_Pos        (13U)
#define EXTI_RTSR_TR13_Msk        (0x1UL << EXTI_RTSR_TR13_Pos)                 /*!< 0x00002000 */
#define EXTI_RTSR_TR13            EXTI_RTSR_TR13_Msk                           /*!< Rising trigger event configuration bit of line 13 */
#define EXTI_RTSR_TR14_Pos        (14U)
#define EXTI_RTSR_TR14_Msk        (0x1UL << EXTI_RTSR_TR14_Pos)                 /*!< 0x00004000 */
#define EXTI_RTSR_TR14            EXTI_RTSR_TR14_Msk                           /*!< Rising trigger event configuration bit of line 14 */
#define EXTI_RTSR_TR15_Pos        (15U)
#define EXTI_RTSR_TR15_Msk        (0x1UL << EXTI_RTSR_TR15_Pos)                 /*!< 0x00008000 */
#define EXTI_RTSR_TR15            EXTI_RTSR_TR15_Msk                           /*!< Rising trigger event configuration bit of line 15 */
#define EXTI_RTSR_TR16_Pos        (16U)
#define EXTI_RTSR_TR16_Msk        (0x1UL << EXTI_RTSR_TR16_Pos)                 /*!< 0x00010000 */
#define EXTI_RTSR_TR16            EXTI_RTSR_TR16_Msk                           /*!< Rising trigger event configuration bit of line 16 */
#define EXTI_RTSR_TR17_Pos        (17U)
#define EXTI_RTSR_TR17_Msk        (0x1UL << EXTI_RTSR_TR17_Pos)                 /*!< 0x00020000 */
#define EXTI_RTSR_TR17            EXTI_RTSR_TR17_Msk                           /*!< Rising trigger event configuration bit of line 17 */
#define EXTI_RTSR_TR19_Pos        (19U)
#define EXTI_RTSR_TR19_Msk        (0x1UL << EXTI_RTSR_TR19_Pos)                 /*!< 0x00080000 */
#define EXTI_RTSR_TR19            EXTI_RTSR_TR19_Msk                           /*!< Rising trigger event configuration bit of line 19 */
#define EXTI_RTSR_TR21_Pos        (21U)
#define EXTI_RTSR_TR21_Msk        (0x1UL << EXTI_RTSR_TR21_Pos)                 /*!< 0x00200000 */
#define EXTI_RTSR_TR21            EXTI_RTSR_TR21_Msk                           /*!< Rising trigger event configuration bit of line 21 */
#define EXTI_RTSR_TR22_Pos        (22U)
#define EXTI_RTSR_TR22_Msk        (0x1UL << EXTI_RTSR_TR22_Pos)                 /*!< 0x00400000 */
#define EXTI_RTSR_TR22            EXTI_RTSR_TR22_Msk                           /*!< Rising trigger event configuration bit of line 22 */
#define EXTI_RTSR_TR31_Pos        (31U)
#define EXTI_RTSR_TR31_Msk        (0x1UL << EXTI_RTSR_TR31_Pos)                 /*!< 0x80000000 */
#define EXTI_RTSR_TR31            EXTI_RTSR_TR31_Msk                           /*!< Rising trigger event configuration bit of line 31 */

/* References Defines */
#define EXTI_RTSR_RT0 EXTI_RTSR_TR0
#define EXTI_RTSR_RT1 EXTI_RTSR_TR1
#define EXTI_RTSR_RT2 EXTI_RTSR_TR2
#define EXTI_RTSR_RT3 EXTI_RTSR_TR3
#define EXTI_RTSR_RT4 EXTI_RTSR_TR4
#define EXTI_RTSR_RT5 EXTI_RTSR_TR5
#define EXTI_RTSR_RT6 EXTI_RTSR_TR6
#define EXTI_RTSR_RT7 EXTI_RTSR_TR7
#define EXTI_RTSR_RT8 EXTI_RTSR_TR8
#define EXTI_RTSR_RT9 EXTI_RTSR_TR9
#define EXTI_RTSR_RT10 EXTI_RTSR_TR10
#define EXTI_RTSR_RT11 EXTI_RTSR_TR11
#define EXTI_RTSR_RT12 EXTI_RTSR_TR12
#define EXTI_RTSR_RT13 EXTI_RTSR_TR13
#define EXTI_RTSR_RT14 EXTI_RTSR_TR14
#define EXTI_RTSR_RT15 EXTI_RTSR_TR15
#define EXTI_RTSR_RT16 EXTI_RTSR_TR16
#define EXTI_RTSR_RT17 EXTI_RTSR_TR17
#define EXTI_RTSR_RT19 EXTI_RTSR_TR19
#define EXTI_RTSR_RT21 EXTI_RTSR_TR21
#define EXTI_RTSR_RT22 EXTI_RTSR_TR22
#define  EXTI_RTSR_RT31 EXTI_RTSR_TR31

/*******************  Bit definition for EXTI_FTSR register *******************/
#define EXTI_FTSR_TR0_Pos         (0U)
#define EXTI_FTSR_TR0_Msk         (0x1UL << EXTI_FTSR_TR0_Pos)                  /*!< 0x00000001 */
#define EXTI_FTSR_TR0             EXTI_FTSR_TR0_Msk                            /*!< Falling trigger event configuration bit of line 0 */
#define EXTI_FTSR_TR1_Pos         (1U)
#define EXTI_FTSR_TR1_Msk         (0x1UL << EXTI_FTSR_TR1_Pos)                  /*!< 0x00000002 */
#define EXTI_FTSR_TR1             EXTI_FTSR_TR1_Msk                            /*!< Falling trigger event configuration bit of line 1 */
#define EXTI_FTSR_TR2_Pos         (2U)
#define EXTI_FTSR_TR2_Msk         (0x1UL << EXTI_FTSR_TR2_Pos)                  /*!< 0x00000004 */
#define EXTI_FTSR_TR2             EXTI_FTSR_TR2_Msk                            /*!< Falling trigger event configuration bit of line 2 */
#define EXTI_FTSR_TR3_Pos         (3U)
#define EXTI_FTSR_TR3_Msk         (0x1UL << EXTI_FTSR_TR3_Pos)                  /*!< 0x00000008 */
#define EXTI_FTSR_TR3             EXTI_FTSR_TR3_Msk                            /*!< Falling trigger event configuration bit of line 3 */
#define EXTI_FTSR_TR4_Pos         (4U)
#define EXTI_FTSR_TR4_Msk         (0x1UL << EXTI_FTSR_TR4_Pos)                  /*!< 0x00000010 */
#define EXTI_FTSR_TR4             EXTI_FTSR_TR4_Msk                            /*!< Falling trigger event configuration bit of line 4 */
#define EXTI_FTSR_TR5_Pos         (5U)
#define EXTI_FTSR_TR5_Msk         (0x1UL << EXTI_FTSR_TR5_Pos)                  /*!< 0x00000020 */
#define EXTI_FTSR_TR5             EXTI_FTSR_TR5_Msk                            /*!< Falling trigger event configuration bit of line 5 */
#define EXTI_FTSR_TR6_Pos         (6U)
#define EXTI_FTSR_TR6_Msk         (0x1UL << EXTI_FTSR_TR6_Pos)                  /*!< 0x00000040 */
#define EXTI_FTSR_TR6             EXTI_FTSR_TR6_Msk                            /*!< Falling trigger event configuration bit of line 6 */
#define EXTI_FTSR_TR7_Pos         (7U)
#define EXTI_FTSR_TR7_Msk         (0x1UL << EXTI_FTSR_TR7_Pos)                  /*!< 0x00000080 */
#define EXTI_FTSR_TR7             EXTI_FTSR_TR7_Msk                            /*!< Falling trigger event configuration bit of line 7 */
#define EXTI_FTSR_TR8_Pos         (8U)
#define EXTI_FTSR_TR8_Msk         (0x1UL << EXTI_FTSR_TR8_Pos)                  /*!< 0x00000100 */
#define EXTI_FTSR_TR8             EXTI_FTSR_TR8_Msk                            /*!< Falling trigger event configuration bit of line 8 */
#define EXTI_FTSR_TR9_Pos         (9U)
#define EXTI_FTSR_TR9_Msk         (0x1UL << EXTI_FTSR_TR9_Pos)                  /*!< 0x00000200 */
#define EXTI_FTSR_TR9             EXTI_FTSR_TR9_Msk                            /*!< Falling trigger event configuration bit of line 9 */
#define EXTI_FTSR_TR10_Pos        (10U)
#define EXTI_FTSR_TR10_Msk        (0x1UL << EXTI_FTSR_TR10_Pos)                 /*!< 0x00000400 */
#define EXTI_FTSR_TR10            EXTI_FTSR_TR10_Msk                           /*!< Falling trigger event configuration bit of line 10 */
#define EXTI_FTSR_TR11_Pos        (11U)
#define EXTI_FTSR_TR11_Msk        (0x1UL << EXTI_FTSR_TR11_Pos)                 /*!< 0x00000800 */
#define EXTI_FTSR_TR11            EXTI_FTSR_TR11_Msk                           /*!< Falling trigger event configuration bit of line 11 */
#define EXTI_FTSR_TR12_Pos        (12U)
#define EXTI_FTSR_TR12_Msk        (0x1UL << EXTI_FTSR_TR12_Pos)                 /*!< 0x00001000 */
#define EXTI_FTSR_TR12            EXTI_FTSR_TR12_Msk                           /*!< Falling trigger event configuration bit of line 12 */
#define EXTI_FTSR_TR13_Pos        (13U)
#define EXTI_FTSR_TR13_Msk        (0x1UL << EXTI_FTSR_TR13_Pos)                 /*!< 0x00002000 */
#define EXTI_FTSR_TR13            EXTI_FTSR_TR13_Msk                           /*!< Falling trigger event configuration bit of line 13 */
#define EXTI_FTSR_TR14_Pos        (14U)
#define EXTI_FTSR_TR14_Msk        (0x1UL << EXTI_FTSR_TR14_Pos)                 /*!< 0x00004000 */
#define EXTI_FTSR_TR14            EXTI_FTSR_TR14_Msk                           /*!< Falling trigger event configuration bit of line 14 */
#define EXTI_FTSR_TR15_Pos        (15U)
#define EXTI_FTSR_TR15_Msk        (0x1UL << EXTI_FTSR_TR15_Pos)                 /*!< 0x00008000 */
#define EXTI_FTSR_TR15            EXTI_FTSR_TR15_Msk                           /*!< Falling trigger event configuration bit of line 15 */
#define EXTI_FTSR_TR16_Pos        (16U)
#define EXTI_FTSR_TR16_Msk        (0x1UL << EXTI_FTSR_TR16_Pos)                 /*!< 0x00010000 */
#define EXTI_FTSR_TR16            EXTI_FTSR_TR16_Msk                           /*!< Falling trigger event configuration bit of line 16 */
#define EXTI_FTSR_TR17_Pos        (17U)
#define EXTI_FTSR_TR17_Msk        (0x1UL << EXTI_FTSR_TR17_Pos)                 /*!< 0x00020000 */
#define EXTI_FTSR_TR17            EXTI_FTSR_TR17_Msk                           /*!< Falling trigger event configuration bit of line 17 */
#define EXTI_FTSR_TR19_Pos        (19U)
#define EXTI_FTSR_TR19_Msk        (0x1UL << EXTI_FTSR_TR19_Pos)                 /*!< 0x00080000 */
#define EXTI_FTSR_TR19            EXTI_FTSR_TR19_Msk                           /*!< Falling trigger event configuration bit of line 19 */
#define EXTI_FTSR_TR21_Pos        (21U)
#define EXTI_FTSR_TR21_Msk        (0x1UL << EXTI_FTSR_TR21_Pos)                 /*!< 0x00200000 */
#define EXTI_FTSR_TR21            EXTI_FTSR_TR21_Msk                           /*!< Falling trigger event configuration bit of line 21 */
#define EXTI_FTSR_TR22_Pos        (22U)
#define EXTI_FTSR_TR22_Msk        (0x1UL << EXTI_FTSR_TR22_Pos)                 /*!< 0x00400000 */
#define EXTI_FTSR_TR22            EXTI_FTSR_TR22_Msk                           /*!< Falling trigger event configuration bit of line 22 */
#define EXTI_FTSR_TR31_Pos        (31U)
#define EXTI_FTSR_TR31_Msk        (0x1UL << EXTI_FTSR_TR31_Pos)                 /*!< 0x80000000 */
#define EXTI_FTSR_TR31            EXTI_FTSR_TR31_Msk                           /*!< Falling trigger event configuration bit of line 31 */

/* References Defines */
#define EXTI_FTSR_FT0 EXTI_FTSR_TR0
#define EXTI_FTSR_FT1 EXTI_FTSR_TR1
#define EXTI_FTSR_FT2 EXTI_FTSR_TR2
#define EXTI_FTSR_FT3 EXTI_FTSR_TR3
#define EXTI_FTSR_FT4 EXTI_FTSR_TR4
#define EXTI_FTSR_FT5 EXTI_FTSR_TR5
#define EXTI_FTSR_FT6 EXTI_FTSR_TR6
#define EXTI_FTSR_FT7 EXTI_FTSR_TR7
#define EXTI_FTSR_FT8 EXTI_FTSR_TR8
#define EXTI_FTSR_FT9 EXTI_FTSR_TR9
#define EXTI_FTSR_FT10 EXTI_FTSR_TR10
#define EXTI_FTSR_FT11 EXTI_FTSR_TR11
#define EXTI_FTSR_FT12 EXTI_FTSR_TR12
#define EXTI_FTSR_FT13 EXTI_FTSR_TR13
#define EXTI_FTSR_FT14 EXTI_FTSR_TR14
#define EXTI_FTSR_FT15 EXTI_FTSR_TR15
#define EXTI_FTSR_FT16 EXTI_FTSR_TR16
#define EXTI_FTSR_FT17 EXTI_FTSR_TR17
#define EXTI_FTSR_FT19 EXTI_FTSR_TR19
#define EXTI_FTSR_FT21 EXTI_FTSR_TR21
#define EXTI_FTSR_FT22 EXTI_FTSR_TR22
#define EXTI_FTSR_FT31 EXTI_FTSR_TR31

/******************* Bit definition for EXTI_SWIER register *******************/
#define EXTI_SWIER_SWIER0_Pos     (0U)
#define EXTI_SWIER_SWIER0_Msk     (0x1UL << EXTI_SWIER_SWIER0_Pos)              /*!< 0x00000001 */
#define EXTI_SWIER_SWIER0         EXTI_SWIER_SWIER0_Msk                        /*!< Software Interrupt on line 0  */
#define EXTI_SWIER_SWIER1_Pos     (1U)
#define EXTI_SWIER_SWIER1_Msk     (0x1UL << EXTI_SWIER_SWIER1_Pos)              /*!< 0x00000002 */
#define EXTI_SWIER_SWIER1         EXTI_SWIER_SWIER1_Msk                        /*!< Software Interrupt on line 1  */
#define EXTI_SWIER_SWIER2_Pos     (2U)
#define EXTI_SWIER_SWIER2_Msk     (0x1UL << EXTI_SWIER_SWIER2_Pos)              /*!< 0x00000004 */
#define EXTI_SWIER_SWIER2         EXTI_SWIER_SWIER2_Msk                        /*!< Software Interrupt on line 2  */
#define EXTI_SWIER_SWIER3_Pos     (3U)
#define EXTI_SWIER_SWIER3_Msk     (0x1UL << EXTI_SWIER_SWIER3_Pos)              /*!< 0x00000008 */
#define EXTI_SWIER_SWIER3         EXTI_SWIER_SWIER3_Msk                        /*!< Software Interrupt on line 3  */
#define EXTI_SWIER_SWIER4_Pos     (4U)
#define EXTI_SWIER_SWIER4_Msk     (0x1UL << EXTI_SWIER_SWIER4_Pos)              /*!< 0x00000010 */
#define EXTI_SWIER_SWIER4         EXTI_SWIER_SWIER4_Msk                        /*!< Software Interrupt on line 4  */
#define EXTI_SWIER_SWIER5_Pos     (5U)
#define EXTI_SWIER_SWIER5_Msk     (0x1UL << EXTI_SWIER_SWIER5_Pos)              /*!< 0x00000020 */
#define EXTI_SWIER_SWIER5         EXTI_SWIER_SWIER5_Msk                        /*!< Software Interrupt on line 5  */
#define EXTI_SWIER_SWIER6_Pos     (6U)
#define EXTI_SWIER_SWIER6_Msk     (0x1UL << EXTI_SWIER_SWIER6_Pos)              /*!< 0x00000040 */
#define EXTI_SWIER_SWIER6         EXTI_SWIER_SWIER6_Msk                        /*!< Software Interrupt on line 6  */
#define EXTI_SWIER_SWIER7_Pos     (7U)
#define EXTI_SWIER_SWIER7_Msk     (0x1UL << EXTI_SWIER_SWIER7_Pos)              /*!< 0x00000080 */
#define EXTI_SWIER_SWIER7         EXTI_SWIER_SWIER7_Msk                        /*!< Software Interrupt on line 7  */
#define EXTI_SWIER_SWIER8_Pos     (8U)
#define EXTI_SWIER_SWIER8_Msk     (0x1UL << EXTI_SWIER_SWIER8_Pos)              /*!< 0x00000100 */
#define EXTI_SWIER_SWIER8         EXTI_SWIER_SWIER8_Msk                        /*!< Software Interrupt on line 8  */
#define EXTI_SWIER_SWIER9_Pos     (9U)
#define EXTI_SWIER_SWIER9_Msk     (0x1UL << EXTI_SWIER_SWIER9_Pos)              /*!< 0x00000200 */
#define EXTI_SWIER_SWIER9         EXTI_SWIER_SWIER9_Msk                        /*!< Software Interrupt on line 9  */
#define EXTI_SWIER_SWIER10_Pos    (10U)
#define EXTI_SWIER_SWIER10_Msk    (0x1UL << EXTI_SWIER_SWIER10_Pos)             /*!< 0x00000400 */
#define EXTI_SWIER_SWIER10        EXTI_SWIER_SWIER10_Msk                       /*!< Software Interrupt on line 10 */
#define EXTI_SWIER_SWIER11_Pos    (11U)
#define EXTI_SWIER_SWIER11_Msk    (0x1UL << EXTI_SWIER_SWIER11_Pos)             /*!< 0x00000800 */
#define EXTI_SWIER_SWIER11        EXTI_SWIER_SWIER11_Msk                       /*!< Software Interrupt on line 11 */
#define EXTI_SWIER_SWIER12_Pos    (12U)
#define EXTI_SWIER_SWIER12_Msk    (0x1UL << EXTI_SWIER_SWIER12_Pos)             /*!< 0x00001000 */
#define EXTI_SWIER_SWIER12        EXTI_SWIER_SWIER12_Msk                       /*!< Software Interrupt on line 12 */
#define EXTI_SWIER_SWIER13_Pos    (13U)
#define EXTI_SWIER_SWIER13_Msk    (0x1UL << EXTI_SWIER_SWIER13_Pos)             /*!< 0x00002000 */
#define EXTI_SWIER_SWIER13        EXTI_SWIER_SWIER13_Msk                       /*!< Software Interrupt on line 13 */
#define EXTI_SWIER_SWIER14_Pos    (14U)
#define EXTI_SWIER_SWIER14_Msk    (0x1UL << EXTI_SWIER_SWIER14_Pos)             /*!< 0x00004000 */
#define EXTI_SWIER_SWIER14        EXTI_SWIER_SWIER14_Msk                       /*!< Software Interrupt on line 14 */
#define EXTI_SWIER_SWIER15_Pos    (15U)
#define EXTI_SWIER_SWIER15_Msk    (0x1UL << EXTI_SWIER_SWIER15_Pos)             /*!< 0x00008000 */
#define EXTI_SWIER_SWIER15        EXTI_SWIER_SWIER15_Msk                       /*!< Software Interrupt on line 15 */
#define EXTI_SWIER_SWIER16_Pos    (16U)
#define EXTI_SWIER_SWIER16_Msk    (0x1UL << EXTI_SWIER_SWIER16_Pos)             /*!< 0x00010000 */
#define EXTI_SWIER_SWIER16        EXTI_SWIER_SWIER16_Msk                       /*!< Software Interrupt on line 16 */
#define EXTI_SWIER_SWIER17_Pos    (17U)
#define EXTI_SWIER_SWIER17_Msk    (0x1UL << EXTI_SWIER_SWIER17_Pos)             /*!< 0x00020000 */
#define EXTI_SWIER_SWIER17        EXTI_SWIER_SWIER17_Msk                       /*!< Software Interrupt on line 17 */
#define EXTI_SWIER_SWIER19_Pos    (19U)
#define EXTI_SWIER_SWIER19_Msk    (0x1UL << EXTI_SWIER_SWIER19_Pos)             /*!< 0x00080000 */
#define EXTI_SWIER_SWIER19        EXTI_SWIER_SWIER19_Msk                       /*!< Software Interrupt on line 19 */
#define EXTI_SWIER_SWIER21_Pos    (21U)
#define EXTI_SWIER_SWIER21_Msk    (0x1UL << EXTI_SWIER_SWIER21_Pos)             /*!< 0x00200000 */
#define EXTI_SWIER_SWIER21        EXTI_SWIER_SWIER21_Msk                       /*!< Software Interrupt on line 21 */
#define EXTI_SWIER_SWIER22_Pos    (22U)
#define EXTI_SWIER_SWIER22_Msk    (0x1UL << EXTI_SWIER_SWIER22_Pos)             /*!< 0x00400000 */
#define EXTI_SWIER_SWIER22        EXTI_SWIER_SWIER22_Msk                       /*!< Software Interrupt on line 22 */
#define EXTI_SWIER_SWIER31_Pos    (31U)
#define EXTI_SWIER_SWIER31_Msk    (0x1UL << EXTI_SWIER_SWIER31_Pos)             /*!< 0x80000000 */
#define EXTI_SWIER_SWIER31        EXTI_SWIER_SWIER31_Msk                       /*!< Software Interrupt on line 31 */

/* References Defines */
#define EXTI_SWIER_SWI0 EXTI_SWIER_SWIER0
#define EXTI_SWIER_SWI1 EXTI_SWIER_SWIER1
#define EXTI_SWIER_SWI2 EXTI_SWIER_SWIER2
#define EXTI_SWIER_SWI3 EXTI_SWIER_SWIER3
#define EXTI_SWIER_SWI4 EXTI_SWIER_SWIER4
#define EXTI_SWIER_SWI5 EXTI_SWIER_SWIER5
#define EXTI_SWIER_SWI6 EXTI_SWIER_SWIER6
#define EXTI_SWIER_SWI7 EXTI_SWIER_SWIER7
#define EXTI_SWIER_SWI8 EXTI_SWIER_SWIER8
#define EXTI_SWIER_SWI9 EXTI_SWIER_SWIER9
#define EXTI_SWIER_SWI10 EXTI_SWIER_SWIER10
#define EXTI_SWIER_SWI11 EXTI_SWIER_SWIER11
#define EXTI_SWIER_SWI12 EXTI_SWIER_SWIER12
#define EXTI_SWIER_SWI13 EXTI_SWIER_SWIER13
#define EXTI_SWIER_SWI14 EXTI_SWIER_SWIER14
#define EXTI_SWIER_SWI15 EXTI_SWIER_SWIER15
#define EXTI_SWIER_SWI16 EXTI_SWIER_SWIER16
#define EXTI_SWIER_SWI17 EXTI_SWIER_SWIER17
#define EXTI_SWIER_SWI19 EXTI_SWIER_SWIER19
#define EXTI_SWIER_SWI21 EXTI_SWIER_SWIER21
#define EXTI_SWIER_SWI22 EXTI_SWIER_SWIER22
#define  EXTI_SWIER_SWI31 EXTI_SWIER_SWIER31

/******************  Bit definition for EXTI_PR register  *********************/
#define EXTI_PR_PR0_Pos           (0U)
#define EXTI_PR_PR0_Msk           (0x1UL << EXTI_PR_PR0_Pos)                    /*!< 0x00000001 */
#define EXTI_PR_PR0               EXTI_PR_PR0_Msk                              /*!< Pending bit 0  */
#define EXTI_PR_PR1_Pos           (1U)
#define EXTI_PR_PR1_Msk           (0x1UL << EXTI_PR_PR1_Pos)                    /*!< 0x00000002 */
#define EXTI_PR_PR1               EXTI_PR_PR1_Msk                              /*!< Pending bit 1  */
#define EXTI_PR_PR2_Pos           (2U)
#define EXTI_PR_PR2_Msk           (0x1UL << EXTI_PR_PR2_Pos)                    /*!< 0x00000004 */
#define EXTI_PR_PR2               EXTI_PR_PR2_Msk                              /*!< Pending bit 2  */
#define EXTI_PR_PR3_Pos           (3U)
#define EXTI_PR_PR3_Msk           (0x1UL << EXTI_PR_PR3_Pos)                    /*!< 0x00000008 */
#define EXTI_PR_PR3               EXTI_PR_PR3_Msk                              /*!< Pending bit 3  */
#define EXTI_PR_PR4_Pos           (4U)
#define EXTI_PR_PR4_Msk           (0x1UL << EXTI_PR_PR4_Pos)                    /*!< 0x00000010 */
#define EXTI_PR_PR4               EXTI_PR_PR4_Msk                              /*!< Pending bit 4  */
#define EXTI_PR_PR5_Pos           (5U)
#define EXTI_PR_PR5_Msk           (0x1UL << EXTI_PR_PR5_Pos)                    /*!< 0x00000020 */
#define EXTI_PR_PR5               EXTI_PR_PR5_Msk                              /*!< Pending bit 5  */
#define EXTI_PR_PR6_Pos           (6U)
#define EXTI_PR_PR6_Msk           (0x1UL << EXTI_PR_PR6_Pos)                    /*!< 0x00000040 */
#define EXTI_PR_PR6               EXTI_PR_PR6_Msk                              /*!< Pending bit 6  */
#define EXTI_PR_PR7_Pos           (7U)
#define EXTI_PR_PR7_Msk           (0x1UL << EXTI_PR_PR7_Pos)                    /*!< 0x00000080 */
#define EXTI_PR_PR7               EXTI_PR_PR7_Msk                              /*!< Pending bit 7  */
#define EXTI_PR_PR8_Pos           (8U)
#define EXTI_PR_PR8_Msk           (0x1UL << EXTI_PR_PR8_Pos)                    /*!< 0x00000100 */
#define EXTI_PR_PR8               EXTI_PR_PR8_Msk                              /*!< Pending bit 8  */
#define EXTI_PR_PR9_Pos           (9U)
#define EXTI_PR_PR9_Msk           (0x1UL << EXTI_PR_PR9_Pos)                    /*!< 0x00000200 */
#define EXTI_PR_PR9               EXTI_PR_PR9_Msk                              /*!< Pending bit 9  */
#define EXTI_PR_PR10_Pos          (10U)
#define EXTI_PR_PR10_Msk          (0x1UL << EXTI_PR_PR10_Pos)                   /*!< 0x00000400 */
#define EXTI_PR_PR10              EXTI_PR_PR10_Msk                             /*!< Pending bit 10 */
#define EXTI_PR_PR11_Pos          (11U)
#define EXTI_PR_PR11_Msk          (0x1UL << EXTI_PR_PR11_Pos)                   /*!< 0x00000800 */
#define EXTI_PR_PR11              EXTI_PR_PR11_Msk                             /*!< Pending bit 11 */
#define EXTI_PR_PR12_Pos          (12U)
#define EXTI_PR_PR12_Msk          (0x1UL << EXTI_PR_PR12_Pos)                   /*!< 0x00001000 */
#define EXTI_PR_PR12              EXTI_PR_PR12_Msk                             /*!< Pending bit 12 */
#define EXTI_PR_PR13_Pos          (13U)
#define EXTI_PR_PR13_Msk          (0x1UL << EXTI_PR_PR13_Pos)                   /*!< 0x00002000 */
#define EXTI_PR_PR13              EXTI_PR_PR13_Msk                             /*!< Pending bit 13 */
#define EXTI_PR_PR14_Pos          (14U)
#define EXTI_PR_PR14_Msk          (0x1UL << EXTI_PR_PR14_Pos)                   /*!< 0x00004000 */
#define EXTI_PR_PR14              EXTI_PR_PR14_Msk                             /*!< Pending bit 14 */
#define EXTI_PR_PR15_Pos          (15U)
#define EXTI_PR_PR15_Msk          (0x1UL << EXTI_PR_PR15_Pos)                   /*!< 0x00008000 */
#define EXTI_PR_PR15              EXTI_PR_PR15_Msk                             /*!< Pending bit 15 */
#define EXTI_PR_PR16_Pos          (16U)
#define EXTI_PR_PR16_Msk          (0x1UL << EXTI_PR_PR16_Pos)                   /*!< 0x00010000 */
#define EXTI_PR_PR16              EXTI_PR_PR16_Msk                             /*!< Pending bit 16 */
#define EXTI_PR_PR17_Pos          (17U)
#define EXTI_PR_PR17_Msk          (0x1UL << EXTI_PR_PR17_Pos)                   /*!< 0x00020000 */
#define EXTI_PR_PR17              EXTI_PR_PR17_Msk                             /*!< Pending bit 17 */
#define EXTI_PR_PR19_Pos          (19U)
#define EXTI_PR_PR19_Msk          (0x1UL << EXTI_PR_PR19_Pos)                   /*!< 0x00080000 */
#define EXTI_PR_PR19              EXTI_PR_PR19_Msk                             /*!< Pending bit 19 */
#define EXTI_PR_PR21_Pos          (21U)
#define EXTI_PR_PR21_Msk          (0x1UL << EXTI_PR_PR21_Pos)                   /*!< 0x00200000 */
#define EXTI_PR_PR21              EXTI_PR_PR21_Msk                             /*!< Pending bit 21 */
#define EXTI_PR_PR22_Pos          (22U)
#define EXTI_PR_PR22_Msk          (0x1UL << EXTI_PR_PR22_Pos)                   /*!< 0x00400000 */
#define EXTI_PR_PR22              EXTI_PR_PR22_Msk                             /*!< Pending bit 22 */
#define EXTI_PR_PR31_Pos          (31U)
#define EXTI_PR_PR31_Msk          (0x1UL << EXTI_PR_PR31_Pos)                   /*!< 0x80000000 */
#define EXTI_PR_PR31              EXTI_PR_PR31_Msk                             /*!< Pending bit 31 */

/* References Defines */
#define EXTI_PR_PIF0 EXTI_PR_PR0
#define EXTI_PR_PIF1 EXTI_PR_PR1
#define EXTI_PR_PIF2 EXTI_PR_PR2
#define EXTI_PR_PIF3 EXTI_PR_PR3
#define EXTI_PR_PIF4 EXTI_PR_PR4
#define EXTI_PR_PIF5 EXTI_PR_PR5
#define EXTI_PR_PIF6 EXTI_PR_PR6
#define EXTI_PR_PIF7 EXTI_PR_PR7
#define EXTI_PR_PIF8 EXTI_PR_PR8
#define EXTI_PR_PIF9 EXTI_PR_PR9
#define EXTI_PR_PIF10 EXTI_PR_PR10
#define EXTI_PR_PIF11 EXTI_PR_PR11
#define EXTI_PR_PIF12 EXTI_PR_PR12
#define EXTI_PR_PIF13 EXTI_PR_PR13
#define EXTI_PR_PIF14 EXTI_PR_PR14
#define EXTI_PR_PIF15 EXTI_PR_PR15
#define EXTI_PR_PIF16 EXTI_PR_PR16
#define EXTI_PR_PIF17 EXTI_PR_PR17
#define EXTI_PR_PIF19 EXTI_PR_PR19
#define EXTI_PR_PIF21 EXTI_PR_PR21
#define EXTI_PR_PIF22 EXTI_PR_PR22
#define EXTI_PR_PIF31 EXTI_PR_PR31

/******************************************************************************/
/*                                                                            */
/*                      FLASH and Option Bytes Registers                      */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for FLASH_ACR register  ******************/
#define FLASH_ACR_LATENCY_Pos             (0U)
#define FLASH_ACR_LATENCY_Msk             (0x1UL << FLASH_ACR_LATENCY_Pos)      /*!< 0x00000001 */
#define FLASH_ACR_LATENCY                 FLASH_ACR_LATENCY_Msk                /*!< LATENCY bit (Latency) */

#define FLASH_ACR_PRFTBE_Pos              (4U)
#define FLASH_ACR_PRFTBE_Msk              (0x1UL << FLASH_ACR_PRFTBE_Pos)       /*!< 0x00000010 */
#define FLASH_ACR_PRFTBE                  FLASH_ACR_PRFTBE_Msk                 /*!< Prefetch Buffer Enable */
#define FLASH_ACR_PRFTBS_Pos              (5U)
#define FLASH_ACR_PRFTBS_Msk              (0x1UL << FLASH_ACR_PRFTBS_Pos)       /*!< 0x00000020 */
#define FLASH_ACR_PRFTBS                  FLASH_ACR_PRFTBS_Msk                 /*!< Prefetch Buffer Status */

/******************  Bit definition for FLASH_KEYR register  ******************/
#define FLASH_KEYR_FKEYR_Pos              (0U)
#define FLASH_KEYR_FKEYR_Msk              (0xFFFFFFFFUL << FLASH_KEYR_FKEYR_Pos) /*!< 0xFFFFFFFF */
#define FLASH_KEYR_FKEYR                  FLASH_KEYR_FKEYR_Msk                 /*!< FPEC Key */

/*****************  Bit definition for FLASH_OPTKEYR register  ****************/
#define FLASH_OPTKEYR_OPTKEYR_Pos         (0U)
#define FLASH_OPTKEYR_OPTKEYR_Msk         (0xFFFFFFFFUL << FLASH_OPTKEYR_OPTKEYR_Pos) /*!< 0xFFFFFFFF */
#define FLASH_OPTKEYR_OPTKEYR             FLASH_OPTKEYR_OPTKEYR_Msk            /*!< Option Byte Key */

/******************  FLASH Keys  **********************************************/
#define FLASH_KEY1_Pos                    (0U)
#define FLASH_KEY1_Msk                    (0x45670123UL << FLASH_KEY1_Pos)      /*!< 0x45670123 */
#define FLASH_KEY1                        FLASH_KEY1_Msk                       /*!< Flash program erase key1 */
#define FLASH_KEY2_Pos                    (0U)
#define FLASH_KEY2_Msk                    (0xCDEF89ABUL << FLASH_KEY2_Pos)      /*!< 0xCDEF89AB */
#define FLASH_KEY2                        FLASH_KEY2_Msk                       /*!< Flash program erase key2: used with FLASH_PEKEY1
                                                                                to unlock the write access to the FPEC. */

#define FLASH_OPTKEY1_Pos                 (0U)
#define FLASH_OPTKEY1_Msk                 (0x45670123UL << FLASH_OPTKEY1_Pos)   /*!< 0x45670123 */
#define FLASH_OPTKEY1                     FLASH_OPTKEY1_Msk                    /*!< Flash option key1 */
#define FLASH_OPTKEY2_Pos                 (0U)
#define FLASH_OPTKEY2_Msk                 (0xCDEF89ABUL << FLASH_OPTKEY2_Pos)   /*!< 0xCDEF89AB */
#define FLASH_OPTKEY2                     FLASH_OPTKEY2_Msk                    /*!< Flash option key2: used with FLASH_OPTKEY1 to
                                                                                unlock the write access to the option byte block */

/******************  Bit definition for FLASH_SR register  *******************/
#define FLASH_SR_BSY_Pos                  (0U)
#define FLASH_SR_BSY_Msk                  (0x1UL << FLASH_SR_BSY_Pos)           /*!< 0x00000001 */
#define FLASH_SR_BSY                      FLASH_SR_BSY_Msk                     /*!< Busy */
#define FLASH_SR_PGERR_Pos                (2U)
#define FLASH_SR_PGERR_Msk                (0x1UL << FLASH_SR_PGERR_Pos)         /*!< 0x00000004 */
#define FLASH_SR_PGERR                    FLASH_SR_PGERR_Msk                   /*!< Programming Error */
#define FLASH_SR_WRPRTERR_Pos             (4U)
#define FLASH_SR_WRPRTERR_Msk             (0x1UL << FLASH_SR_WRPRTERR_Pos)      /*!< 0x00000010 */
#define FLASH_SR_WRPRTERR                 FLASH_SR_WRPRTERR_Msk                /*!< Write Protection Error */
#define FLASH_SR_EOP_Pos                  (5U)
#define FLASH_SR_EOP_Msk                  (0x1UL << FLASH_SR_EOP_Pos)           /*!< 0x00000020 */
#define FLASH_SR_EOP                      FLASH_SR_EOP_Msk                     /*!< End of operation */
#define  FLASH_SR_WRPERR                     FLASH_SR_WRPRTERR             /*!< Legacy of Write Protection Error */

/*******************  Bit definition for FLASH_CR register  *******************/
#define FLASH_CR_PG_Pos                   (0U)
#define FLASH_CR_PG_Msk                   (0x1UL << FLASH_CR_PG_Pos)            /*!< 0x00000001 */
#define FLASH_CR_PG                       FLASH_CR_PG_Msk                      /*!< Programming */
#define FLASH_CR_PER_Pos                  (1U)
#define FLASH_CR_PER_Msk                  (0x1UL << FLASH_CR_PER_Pos)           /*!< 0x00000002 */
#define FLASH_CR_PER                      FLASH_CR_PER_Msk                     /*!< Page Erase */
#define FLASH_CR_MER_Pos                  (2U)
#define FLASH_CR_MER_Msk                  (0x1UL << FLASH_CR_MER_Pos)           /*!< 0x00000004 */
#define FLASH_CR_MER                      FLASH_CR_MER_Msk                     /*!< Mass Erase */
#define FLASH_CR_OPTPG_Pos                (4U)
#define FLASH_CR_OPTPG_Msk                (0x1UL << FLASH_CR_OPTPG_Pos)         /*!< 0x00000010 */
#define FLASH_CR_OPTPG                    FLASH_CR_OPTPG_Msk                   /*!< Option Byte Programming */
#define FLASH_CR_OPTER_Pos                (5U)
#define FLASH_CR_OPTER_Msk                (0x1UL << FLASH_CR_OPTER_Pos)         /*!< 0x00000020 */
#define FLASH_CR_OPTER                    FLASH_CR_OPTER_Msk                   /*!< Option Byte Erase */
#define FLASH_CR_STRT_Pos                 (6U)
#define FLASH_CR_STRT_Msk                 (0x1UL << FLASH_CR_STRT_Pos)          /*!< 0x00000040 */
#define FLASH_CR_STRT                     FLASH_CR_STRT_Msk                    /*!< Start */
#define FLASH_CR_LOCK_Pos                 (7U)
#define FLASH_CR_LOCK_Msk                 (0x1UL << FLASH_CR_LOCK_Pos)          /*!< 0x00000080 */
#define FLASH_CR_LOCK                     FLASH_CR_LOCK_Msk                    /*!< Lock */
#define FLASH_CR_OPTWRE_Pos               (9U)
#define FLASH_CR_OPTWRE_Msk               (0x1UL << FLASH_CR_OPTWRE_Pos)        /*!< 0x00000200 */
#define FLASH_CR_OPTWRE                   FLASH_CR_OPTWRE_Msk                  /*!< Option Bytes Write Enable */
#define FLASH_CR_ERRIE_Pos                (10U)
#define FLASH_CR_ERRIE_Msk                (0x1UL << FLASH_CR_ERRIE_Pos)         /*!< 0x00000400 */
#define FLASH_CR_ERRIE                    FLASH_CR_ERRIE_Msk                   /*!< Error Interrupt Enable */
#define FLASH_CR_EOPIE_Pos                (12U)
#define FLASH_CR_EOPIE_Msk                (0x1UL << FLASH_CR_EOPIE_Pos)         /*!< 0x00001000 */
#define FLASH_CR_EOPIE                    FLASH_CR_EOPIE_Msk                   /*!< End of operation interrupt enable */
#define FLASH_CR_OBL_LAUNCH_Pos           (13U)
#define FLASH_CR_OBL_LAUNCH_Msk           (0x1UL << FLASH_CR_OBL_LAUNCH_Pos)    /*!< 0x00002000 */
#define FLASH_CR_OBL_LAUNCH               FLASH_CR_OBL_LAUNCH_Msk              /*!< Option Bytes Loader Launch */

/*******************  Bit definition for FLASH_AR register  *******************/
#define FLASH_AR_FAR_Pos                  (0U)
#define FLASH_AR_FAR_Msk                  (0xFFFFFFFFUL << FLASH_AR_FAR_Pos)    /*!< 0xFFFFFFFF */
#define FLASH_AR_FAR                      FLASH_AR_FAR_Msk                     /*!< Flash Address */

/******************  Bit definition for FLASH_OBR register  *******************/
#define FLASH_OBR_OPTERR_Pos              (0U)
#define FLASH_OBR_OPTERR_Msk              (0x1UL << FLASH_OBR_OPTERR_Pos)       /*!< 0x00000001 */
#define FLASH_OBR_OPTERR                  FLASH_OBR_OPTERR_Msk                 /*!< Option Byte Error */
#define FLASH_OBR_RDPRT1_Pos              (1U)
#define FLASH_OBR_RDPRT1_Msk              (0x1UL << FLASH_OBR_RDPRT1_Pos)       /*!< 0x00000002 */
#define FLASH_OBR_RDPRT1                  FLASH_OBR_RDPRT1_Msk                 /*!< Read protection Level 1 */
#define FLASH_OBR_RDPRT2_Pos              (2U)
#define FLASH_OBR_RDPRT2_Msk              (0x1UL << FLASH_OBR_RDPRT2_Pos)       /*!< 0x00000004 */
#define FLASH_OBR_RDPRT2                  FLASH_OBR_RDPRT2_Msk                 /*!< Read protection Level 2 */

#define FLASH_OBR_USER_Pos                (8U)
#define FLASH_OBR_USER_Msk                (0xFFUL << FLASH_OBR_USER_Pos)        /*!< 0x0000FF00 */
#define FLASH_OBR_USER                    FLASH_OBR_USER_Msk                   /*!< User Option Bytes */
#define FLASH_OBR_IWDG_SW_Pos             (8U)
#define FLASH_OBR_IWDG_SW_Msk             (0x1UL << FLASH_OBR_IWDG_SW_Pos)      /*!< 0x00000100 */
#define FLASH_OBR_IWDG_SW                 FLASH_OBR_IWDG_SW_Msk                /*!< IWDG SW */
#define FLASH_OBR_nRST_STOP_Pos           (9U)
#define FLASH_OBR_nRST_STOP_Msk           (0x1UL << FLASH_OBR_nRST_STOP_Pos)    /*!< 0x00000200 */
#define FLASH_OBR_nRST_STOP               FLASH_OBR_nRST_STOP_Msk              /*!< nRST_STOP */
#define FLASH_OBR_nRST_STDBY_Pos          (10U)
#define FLASH_OBR_nRST_STDBY_Msk          (0x1UL << FLASH_OBR_nRST_STDBY_Pos)   /*!< 0x00000400 */
#define FLASH_OBR_nRST_STDBY              FLASH_OBR_nRST_STDBY_Msk             /*!< nRST_STDBY */
#define FLASH_OBR_nBOOT0_Pos              (11U)
#define FLASH_OBR_nBOOT0_Msk              (0x1UL << FLASH_OBR_nBOOT0_Pos)       /*!< 0x00000800 */
#define FLASH_OBR_nBOOT0                  FLASH_OBR_nBOOT0_Msk                 /*!< nBOOT0 */
#define FLASH_OBR_nBOOT1_Pos              (12U)
#define FLASH_OBR_nBOOT1_Msk              (0x1UL << FLASH_OBR_nBOOT1_Pos)       /*!< 0x00001000 */
#define FLASH_OBR_nBOOT1                  FLASH_OBR_nBOOT1_Msk                 /*!< nBOOT1 */
#define FLASH_OBR_VDDA_MONITOR_Pos        (13U)
#define FLASH_OBR_VDDA_MONITOR_Msk        (0x1UL << FLASH_OBR_VDDA_MONITOR_Pos) /*!< 0x00002000 */
#define FLASH_OBR_VDDA_MONITOR            FLASH_OBR_VDDA_MONITOR_Msk           /*!< VDDA power supply supervisor */
#define FLASH_OBR_RAM_PARITY_CHECK_Pos    (14U)
#define FLASH_OBR_RAM_PARITY_CHECK_Msk    (0x1UL << FLASH_OBR_RAM_PARITY_CHECK_Pos) /*!< 0x00004000 */
#define FLASH_OBR_RAM_PARITY_CHECK        FLASH_OBR_RAM_PARITY_CHECK_Msk       /*!< RAM parity check */
#define FLASH_OBR_BOOT_SEL_Pos            (15U)
#define FLASH_OBR_BOOT_SEL_Msk            (0x1UL << FLASH_OBR_BOOT_SEL_Pos)     /*!< 0x00008000 */
#define FLASH_OBR_BOOT_SEL                FLASH_OBR_BOOT_SEL_Msk               /*!< BOOT selection */
#define FLASH_OBR_DATA0_Pos               (16U)
#define FLASH_OBR_DATA0_Msk               (0xFFUL << FLASH_OBR_DATA0_Pos)       /*!< 0x00FF0000 */
#define FLASH_OBR_DATA0                   FLASH_OBR_DATA0_Msk                  /*!< Data0 */
#define FLASH_OBR_DATA1_Pos               (24U)
#define FLASH_OBR_DATA1_Msk               (0xFFUL << FLASH_OBR_DATA1_Pos)       /*!< 0xFF000000 */
#define FLASH_OBR_DATA1                   FLASH_OBR_DATA1_Msk                  /*!< Data1 */

/* Old BOOT1 bit definition, maintained for legacy purpose */
#define FLASH_OBR_BOOT1                      FLASH_OBR_nBOOT1

/* Old OBR_VDDA bit definition, maintained for legacy purpose */
#define FLASH_OBR_VDDA_ANALOG                FLASH_OBR_VDDA_MONITOR

/******************  Bit definition for FLASH_WRPR register  ******************/
#define FLASH_WRPR_WRP_Pos                (0U)
#define FLASH_WRPR_WRP_Msk                (0xFFFFUL << FLASH_WRPR_WRP_Pos)      /*!< 0x0000FFFF */
#define FLASH_WRPR_WRP                    FLASH_WRPR_WRP_Msk                   /*!< Write Protect */

/*----------------------------------------------------------------------------*/

/******************  Bit definition for OB_RDP register  **********************/
#define OB_RDP_RDP_Pos       (0U)
#define OB_RDP_RDP_Msk       (0xFFUL << OB_RDP_RDP_Pos)                         /*!< 0x000000FF */
#define OB_RDP_RDP           OB_RDP_RDP_Msk                                    /*!< Read protection option byte */
#define OB_RDP_nRDP_Pos      (8U)
#define OB_RDP_nRDP_Msk      (0xFFUL << OB_RDP_nRDP_Pos)                        /*!< 0x0000FF00 */
#define OB_RDP_nRDP          OB_RDP_nRDP_Msk                                   /*!< Read protection complemented option byte */

/******************  Bit definition for OB_USER register  *********************/
#define OB_USER_USER_Pos     (16U)
#define OB_USER_USER_Msk     (0xFFUL << OB_USER_USER_Pos)                       /*!< 0x00FF0000 */
#define OB_USER_USER         OB_USER_USER_Msk                                  /*!< User option byte */
#define OB_USER_nUSER_Pos    (24U)
#define OB_USER_nUSER_Msk    (0xFFUL << OB_USER_nUSER_Pos)                      /*!< 0xFF000000 */
#define OB_USER_nUSER        OB_USER_nUSER_Msk                                 /*!< User complemented option byte */

/******************  Bit definition for OB_WRP0 register  *********************/
#define OB_WRP0_WRP0_Pos     (0U)
#define OB_WRP0_WRP0_Msk     (0xFFUL << OB_WRP0_WRP0_Pos)                       /*!< 0x000000FF */
#define OB_WRP0_WRP0         OB_WRP0_WRP0_Msk                                  /*!< Flash memory write protection option bytes */
#define OB_WRP0_nWRP0_Pos    (8U)
#define OB_WRP0_nWRP0_Msk    (0xFFUL << OB_WRP0_nWRP0_Pos)                      /*!< 0x0000FF00 */
#define OB_WRP0_nWRP0        OB_WRP0_nWRP0_Msk                                 /*!< Flash memory write protection complemented option bytes */

/******************************************************************************/
/*                                                                            */
/*                       General Purpose IOs (GPIO)                           */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for GPIO_MODER register  *****************/
#define GPIO_MODER_MODER0_Pos           (0U)
#define GPIO_MODER_MODER0_Msk           (0x3UL << GPIO_MODER_MODER0_Pos)        /*!< 0x00000003 */
#define GPIO_MODER_MODER0               GPIO_MODER_MODER0_Msk
#define GPIO_MODER_MODER0_0             (0x1UL << GPIO_MODER_MODER0_Pos)        /*!< 0x00000001 */
#define GPIO_MODER_MODER0_1             (0x2UL << GPIO_MODER_MODER0_Pos)        /*!< 0x00000002 */
#define GPIO_MODER_MODER1_Pos           (2U)
#define GPIO_MODER_MODER1_Msk           (0x3UL << GPIO_MODER_MODER1_Pos)        /*!< 0x0000000C */
#define GPIO_MODER_MODER1               GPIO_MODER_MODER1_Msk
#define GPIO_MODER_MODER1_0             (0x1UL << GPIO_MODER_MODER1_Pos)        /*!< 0x00000004 */
#define GPIO_MODER_MODER1_1             (0x2UL << GPIO_MODER_MODER1_Pos)        /*!< 0x00000008 */
#define GPIO_MODER_MODER2_Pos           (4U)
#define GPIO_MODER_MODER2_Msk           (0x3UL << GPIO_MODER_MODER2_Pos)        /*!< 0x00000030 */
#define GPIO_MODER_MODER2               GPIO_MODER_MODER2_Msk
#define GPIO_MODER_MODER2_0             (0x1UL << GPIO_MODER_MODER2_Pos)        /*!< 0x00000010 */
#define GPIO_MODER_MODER2_1             (0x2UL << GPIO_MODER_MODER2_Pos)        /*!< 0x00000020 */
#define GPIO_MODER_MODER3_Pos           (6U)
#define GPIO_MODER_MODER3_Msk           (0x3UL << GPIO_MODER_MODER3_Pos)        /*!< 0x000000C0 */
#define GPIO_MODER_MODER3               GPIO_MODER_MODER3_Msk
#define GPIO_MODER_MODER3_0             (0x1UL << GPIO_MODER_MODER3_Pos)        /*!< 0x00000040 */
#define GPIO_MODER_MODER3_1             (0x2UL << GPIO_MODER_MODER3_Pos)        /*!< 0x00000080 */
#define GPIO_MODER_MODER4_Pos           (8U)
#define GPIO_MODER_MODER4_Msk           (0x3UL << GPIO_MODER_MODER4_Pos)        /*!< 0x00000300 */
#define GPIO_MODER_MODER4               GPIO_MODER_MODER4_Msk
#define GPIO_MODER_MODER4_0             (0x1UL << GPIO_MODER_MODER4_Pos)        /*!< 0x00000100 */
#define GPIO_MODER_MODER4_1             (0x2UL << GPIO_MODER_MODER4_Pos)        /*!< 0x00000200 */
#define GPIO_MODER_MODER5_Pos           (10U)
#define GPIO_MODER_MODER5_Msk           (0x3UL << GPIO_MODER_MODER5_Pos)        /*!< 0x00000C00 */
#define GPIO_MODER_MODER5               GPIO_MODER_MODER5_Msk
#define GPIO_MODER_MODER5_0             (0x1UL << GPIO_MODER_MODER5_Pos)        /*!< 0x00000400 */
#define GPIO_MODER_MODER5_1             (0x2UL << GPIO_MODER_MODER5_Pos)        /*!< 0x00000800 */
#define GPIO_MODER_MODER6_Pos           (12U)
#define GPIO_MODER_MODER6_Msk           (0x3UL << GPIO_MODER_MODER6_Pos)        /*!< 0x00003000 */
#define GPIO_MODER_MODER6               GPIO_MODER_MODER6_Msk
#define GPIO_MODER_MODER6_0             (0x1UL << GPIO_MODER_MODER6_Pos)        /*!< 0x00001000 */
#define GPIO_MODER_MODER6_1             (0x2UL << GPIO_MODER_MODER6_Pos)        /*!< 0x00002000 */
#define GPIO_MODER_MODER7_Pos           (14U)
#define GPIO_MODER_MODER7_Msk           (0x3UL << GPIO_MODER_MODER7_Pos)        /*!< 0x0000C000 */
#define GPIO_MODER_MODER7               GPIO_MODER_MODER7_Msk
#define GPIO_MODER_MODER7_0             (0x1UL << GPIO_MODER_MODER7_Pos)        /*!< 0x00004000 */
#define GPIO_MODER_MODER7_1             (0x2UL << GPIO_MODER_MODER7_Pos)        /*!< 0x00008000 */
#define GPIO_MODER_MODER8_Pos           (16U)
#define GPIO_MODER_MODER8_Msk           (0x3UL << GPIO_MODER_MODER8_Pos)        /*!< 0x00030000 */
#define GPIO_MODER_MODER8               GPIO_MODER_MODER8_Msk
#define GPIO_MODER_MODER8_0             (0x1UL << GPIO_MODER_MODER8_Pos)        /*!< 0x00010000 */
#define GPIO_MODER_MODER8_1             (0x2UL << GPIO_MODER_MODER8_Pos)        /*!< 0x00020000 */
#define GPIO_MODER_MODER9_Pos           (18U)
#define GPIO_MODER_MODER9_Msk           (0x3UL << GPIO_MODER_MODER9_Pos)        /*!< 0x000C0000 */
#define GPIO_MODER_MODER9               GPIO_MODER_MODER9_Msk
#define GPIO_MODER_MODER9_0             (0x1UL << GPIO_MODER_MODER9_Pos)        /*!< 0x00040000 */
#define GPIO_MODER_MODER9_1             (0x2UL << GPIO_MODER_MODER9_Pos)        /*!< 0x00080000 */
#define GPIO_MODER_MODER10_Pos          (20U)
#define GPIO_MODER_MODER10_Msk          (0x3UL << GPIO_MODER_MODER10_Pos)       /*!< 0x00300000 */
#define GPIO_MODER_MODER10              GPIO_MODER_MODER10_Msk
#define GPIO_MODER_MODER10_0            (0x1UL << GPIO_MODER_MODER10_Pos)       /*!< 0x00100000 */
#define GPIO_MODER_MODER10_1            (0x2UL << GPIO_MODER_MODER10_Pos)       /*!< 0x00200000 */
#define GPIO_MODER_MODER11_Pos          (22U)
#define GPIO_MODER_MODER11_Msk          (0x3UL << GPIO_MODER_MODER11_Pos)       /*!< 0x00C00000 */
#define GPIO_MODER_MODER11              GPIO_MODER_MODER11_Msk
#define GPIO_MODER_MODER11_0            (0x1UL << GPIO_MODER_MODER11_Pos)       /*!< 0x00400000 */
#define GPIO_MODER_MODER11_1            (0x2UL << GPIO_MODER_MODER11_Pos)       /*!< 0x00800000 */
#define GPIO_MODER_MODER12_Pos          (24U)
#define GPIO_MODER_MODER12_Msk          (0x3UL << GPIO_MODER_MODER12_Pos)       /*!< 0x03000000 */
#define GPIO_MODER_MODER12              GPIO_MODER_MODER12_Msk
#define GPIO_MODER_MODER12_0            (0x1UL << GPIO_MODER_MODER12_Pos)       /*!< 0x01000000 */
#define GPIO_MODER_MODER12_1            (0x2UL << GPIO_MODER_MODER12_Pos)       /*!< 0x02000000 */
#define GPIO_MODER_MODER13_Pos          (26U)
#define GPIO_MODER_MODER13_Msk          (0x3UL << GPIO_MODER_MODER13_Pos)       /*!< 0x0C000000 */
#define GPIO_MODER_MODER13              GPIO_MODER_MODER13_Msk
#define GPIO_MODER_MODER13_0            (0x1UL << GPIO_MODER_MODER13_Pos)       /*!< 0x04000000 */
#define GPIO_MODER_MODER13_1            (0x2UL << GPIO_MODER_MODER13_Pos)       /*!< 0x08000000 */
#define GPIO_MODER_MODER14_Pos          (28U)
#define GPIO_MODER_MODER14_Msk          (0x3UL << GPIO_MODER_MODER14_Pos)       /*!< 0x30000000 */
#define GPIO_MODER_MODER14              GPIO_MODER_MODER14_Msk
#define GPIO_MODER_MODER14_0            (0x1UL << GPIO_MODER_MODER14_Pos)       /*!< 0x10000000 */
#define GPIO_MODER_MODER14_1            (0x2UL << GPIO_MODER_MODER14_Pos)       /*!< 0x20000000 */
#define GPIO_MODER_MODER15_Pos          (30U)
#define GPIO_MODER_MODER15_Msk          (0x3UL << GPIO_MODER_MODER15_Pos)       /*!< 0xC0000000 */
#define GPIO_MODER_MODER15              GPIO_MODER_MODER15_Msk
#define GPIO_MODER_MODER15_0            (0x1UL << GPIO_MODER_MODER15_Pos)       /*!< 0x40000000 */
#define GPIO_MODER_MODER15_1            (0x2UL << GPIO_MODER_MODER15_Pos)       /*!< 0x80000000 */

/******************  Bit definition for GPIO_OTYPER register  *****************/
#define GPIO_OTYPER_OT_0                (0x00000001U)
#define GPIO_OTYPER_OT_1                (0x00000002U)
#define GPIO_OTYPER_OT_2                (0x00000004U)
#define GPIO_OTYPER_OT_3                (0x00000008U)
#define GPIO_OTYPER_OT_4                (0x00000010U)
#define GPIO_OTYPER_OT_5                (0x00000020U)
#define GPIO_OTYPER_OT_6                (0x00000040U)
#define GPIO_OTYPER_OT_7                (0x00000080U)
#define GPIO_OTYPER_OT_8                (0x00000100U)
#define GPIO_OTYPER_OT_9                (0x00000200U)
#define GPIO_OTYPER_OT_10               (0x00000400U)
#define GPIO_OTYPER_OT_11               (0x00000800U)
#define GPIO_OTYPER_OT_12               (0x00001000U)
#define GPIO_OTYPER_OT_13               (0x00002000U)
#define GPIO_OTYPER_OT_14               (0x00004000U)
#define GPIO_OTYPER_OT_15               (0x00008000U)

/****************  Bit definition for GPIO_OSPEEDR register  ******************/
#define GPIO_OSPEEDR_OSPEEDR0_Pos       (0U)
#define GPIO_OSPEEDR_OSPEEDR0_Msk       (0x3UL << GPIO_OSPEEDR_OSPEEDR0_Pos)    /*!< 0x00000003 */
#define GPIO_OSPEEDR_OSPEEDR0           GPIO_OSPEEDR_OSPEEDR0_Msk
#define GPIO_OSPEEDR_OSPEEDR0_0         (0x1UL << GPIO_OSPEEDR_OSPEEDR0_Pos)    /*!< 0x00000001 */
#define GPIO_OSPEEDR_OSPEEDR0_1         (0x2UL << GPIO_OSPEEDR_OSPEEDR0_Pos)    /*!< 0x00000002 */
#define GPIO_OSPEEDR_OSPEEDR1_Pos       (2U)
#define GPIO_OSPEEDR_OSPEEDR1_Msk       (0x3UL << GPIO_OSPEEDR_OSPEEDR1_Pos)    /*!< 0x0000000C */
#define GPIO_OSPEEDR_OSPEEDR1           GPIO_OSPEEDR_OSPEEDR1_Msk
#define GPIO_OSPEEDR_OSPEEDR1_0         (0x1UL << GPIO_OSPEEDR_OSPEEDR1_Pos)    /*!< 0x00000004 */
#define GPIO_OSPEEDR_OSPEEDR1_1         (0x2UL << GPIO_OSPEEDR_OSPEEDR1_Pos)    /*!< 0x00000008 */
#define GPIO_OSPEEDR_OSPEEDR2_Pos       (4U)
#define GPIO_OSPEEDR_OSPEEDR2_Msk       (0x3UL << GPIO_OSPEEDR_OSPEEDR2_Pos)    /*!< 0x00000030 */
#define GPIO_OSPEEDR_OSPEEDR2           GPIO_OSPEEDR_OSPEEDR2_Msk
#define GPIO_OSPEEDR_OSPEEDR2_0         (0x1UL << GPIO_OSPEEDR_OSPEEDR2_Pos)    /*!< 0x00000010 */
#define GPIO_OSPEEDR_OSPEEDR2_1         (0x2UL << GPIO_OSPEEDR_OSPEEDR2_Pos)    /*!< 0x00000020 */
#define GPIO_OSPEEDR_OSPEEDR3_Pos       (6U)
#define GPIO_OSPEEDR_OSPEEDR3_Msk       (0x3UL << GPIO_OSPEEDR_OSPEEDR3_Pos)    /*!< 0x000000C0 */
#define GPIO_OSPEEDR_OSPEEDR3           GPIO_OSPEEDR_OSPEEDR3_Msk
#define GPIO_OSPEEDR_OSPEEDR3_0         (0x1UL << GPIO_OSPEEDR_OSPEEDR3_Pos)    /*!< 0x00000040 */
#define GPIO_OSPEEDR_OSPEEDR3_1         (0x2UL << GPIO_OSPEEDR_OSPEEDR3_Pos)    /*!< 0x00000080 */
#define GPIO_OSPEEDR_OSPEEDR4_Pos       (8U)
#define GPIO_OSPEEDR_OSPEEDR4_Msk       (0x3UL << GPIO_OSPEEDR_OSPEEDR4_Pos)    /*!< 0x00000300 */
#define GPIO_OSPEEDR_OSPEEDR4           GPIO_OSPEEDR_OSPEEDR4_Msk
#define GPIO_OSPEEDR_OSPEEDR4_0         (0x1UL << GPIO_OSPEEDR_OSPEEDR4_Pos)    /*!< 0x00000100 */
#define GPIO_OSPEEDR_OSPEEDR4_1         (0x2UL << GPIO_OSPEEDR_OSPEEDR4_Pos)    /*!< 0x00000200 */
#define GPIO_OSPEEDR_OSPEEDR5_Pos       (10U)
#define GPIO_OSPEEDR_OSPEEDR5_Msk       (0x3UL << GPIO_OSPEEDR_OSPEEDR5_Pos)    /*!< 0x00000C00 */
#define GPIO_OSPEEDR_OSPEEDR5           GPIO_OSPEEDR_OSPEEDR5_Msk
#define GPIO_OSPEEDR_OSPEEDR5_0         (0x1UL << GPIO_OSPEEDR_OSPEEDR5_Pos)    /*!< 0x00000400 */
#define GPIO_OSPEEDR_OSPEEDR5_1         (0x2UL << GPIO_OSPEEDR_OSPEEDR5_Pos)    /*!< 0x00000800 */
#define GPIO_OSPEEDR_OSPEEDR6_Pos       (12U)
#define GPIO_OSPEEDR_OSPEEDR6_Msk       (0x3UL << GPIO_OSPEEDR_OSPEEDR6_Pos)    /*!< 0x00003000 */
#define GPIO_OSPEEDR_OSPEEDR6           GPIO_OSPEEDR_OSPEEDR6_Msk
#define GPIO_OSPEEDR_OSPEEDR6_0         (0x1UL << GPIO_OSPEEDR_OSPEEDR6_Pos)    /*!< 0x00001000 */
#define GPIO_OSPEEDR_OSPEEDR6_1         (0x2UL << GPIO_OSPEEDR_OSPEEDR6_Pos)    /*!< 0x00002000 */
#define GPIO_OSPEEDR_OSPEEDR7_Pos       (14U)
#define GPIO_OSPEEDR_OSPEEDR7_Msk       (0x3UL << GPIO_OSPEEDR_OSPEEDR7_Pos)    /*!< 0x0000C000 */
#define GPIO_OSPEEDR_OSPEEDR7           GPIO_OSPEEDR_OSPEEDR7_Msk
#define GPIO_OSPEEDR_OSPEEDR7_0         (0x1UL << GPIO_OSPEEDR_OSPEEDR7_Pos)    /*!< 0x00004000 */
#define GPIO_OSPEEDR_OSPEEDR7_1         (0x2UL << GPIO_OSPEEDR_OSPEEDR7_Pos)    /*!< 0x00008000 */
#define GPIO_OSPEEDR_OSPEEDR8_Pos       (16U)
#define GPIO_OSPEEDR_OSPEEDR8_Msk       (0x3UL << GPIO_OSPEEDR_OSPEEDR8_Pos)    /*!< 0x00030000 */
#define GPIO_OSPEEDR_OSPEEDR8           GPIO_OSPEEDR_OSPEEDR8_Msk
#define GPIO_OSPEEDR_OSPEEDR8_0         (0x1UL << GPIO_OSPEEDR_OSPEEDR8_Pos)    /*!< 0x00010000 */
#define GPIO_OSPEEDR_OSPEEDR8_1         (0x2UL << GPIO_OSPEEDR_OSPEEDR8_Pos)    /*!< 0x00020000 */
#define GPIO_OSPEEDR_OSPEEDR9_Pos       (18U)
#define GPIO_OSPEEDR_OSPEEDR9_Msk       (0x3UL << GPIO_OSPEEDR_OSPEEDR9_Pos)    /*!< 0x000C0000 */
#define GPIO_OSPEEDR_OSPEEDR9           GPIO_OSPEEDR_OSPEEDR9_Msk
#define GPIO_OSPEEDR_OSPEEDR9_0         (0x1UL << GPIO_OSPEEDR_OSPEEDR9_Pos)    /*!< 0x00040000 */
#define GPIO_OSPEEDR_OSPEEDR9_1         (0x2UL << GPIO_OSPEEDR_OSPEEDR9_Pos)    /*!< 0x00080000 */
#define GPIO_OSPEEDR_OSPEEDR10_Pos      (20U)
#define GPIO_OSPEEDR_OSPEEDR10_Msk      (0x3UL << GPIO_OSPEEDR_OSPEEDR10_Pos)   /*!< 0x00300000 */
#define GPIO_OSPEEDR_OSPEEDR10          GPIO_OSPEEDR_OSPEEDR10_Msk
#define GPIO_OSPEEDR_OSPEEDR10_0        (0x1UL << GPIO_OSPEEDR_OSPEEDR10_Pos)   /*!< 0x00100000 */
#define GPIO_OSPEEDR_OSPEEDR10_1        (0x2UL << GPIO_OSPEEDR_OSPEEDR10_Pos)   /*!< 0x00200000 */
#define GPIO_OSPEEDR_OSPEEDR11_Pos      (22U)
#define GPIO_OSPEEDR_OSPEEDR11_Msk      (0x3UL << GPIO_OSPEEDR_OSPEEDR11_Pos)   /*!< 0x00C00000 */
#define GPIO_OSPEEDR_OSPEEDR11          GPIO_OSPEEDR_OSPEEDR11_Msk
#define GPIO_OSPEEDR_OSPEEDR11_0        (0x1UL << GPIO_OSPEEDR_OSPEEDR11_Pos)   /*!< 0x00400000 */
#define GPIO_OSPEEDR_OSPEEDR11_1        (0x2UL << GPIO_OSPEEDR_OSPEEDR11_Pos)   /*!< 0x00800000 */
#define GPIO_OSPEEDR_OSPEEDR12_Pos      (24U)
#define GPIO_OSPEEDR_OSPEEDR12_Msk      (0x3UL << GPIO_OSPEEDR_OSPEEDR12_Pos)   /*!< 0x03000000 */
#define GPIO_OSPEEDR_OSPEEDR12          GPIO_OSPEEDR_OSPEEDR12_Msk
#define GPIO_OSPEEDR_OSPEEDR12_0        (0x1UL << GPIO_OSPEEDR_OSPEEDR12_Pos)   /*!< 0x01000000 */
#define GPIO_OSPEEDR_OSPEEDR12_1        (0x2UL << GPIO_OSPEEDR_OSPEEDR12_Pos)   /*!< 0x02000000 */
#define GPIO_OSPEEDR_OSPEEDR13_Pos      (26U)
#define GPIO_OSPEEDR_OSPEEDR13_Msk      (0x3UL << GPIO_OSPEEDR_OSPEEDR13_Pos)   /*!< 0x0C000000 */
#define GPIO_OSPEEDR_OSPEEDR13          GPIO_OSPEEDR_OSPEEDR13_Msk
#define GPIO_OSPEEDR_OSPEEDR13_0        (0x1UL << GPIO_OSPEEDR_OSPEEDR13_Pos)   /*!< 0x04000000 */
#define GPIO_OSPEEDR_OSPEEDR13_1        (0x2UL << GPIO_OSPEEDR_OSPEEDR13_Pos)   /*!< 0x08000000 */
#define GPIO_OSPEEDR_OSPEEDR14_Pos      (28U)
#define GPIO_OSPEEDR_OSPEEDR14_Msk      (0x3UL << GPIO_OSPEEDR_OSPEEDR14_Pos)   /*!< 0x30000000 */
#define GPIO_OSPEEDR_OSPEEDR14          GPIO_OSPEEDR_OSPEEDR14_Msk
#define GPIO_OSPEEDR_OSPEEDR14_0        (0x1UL << GPIO_OSPEEDR_OSPEEDR14_Pos)   /*!< 0x10000000 */
#define GPIO_OSPEEDR_OSPEEDR14_1        (0x2UL << GPIO_OSPEEDR_OSPEEDR14_Pos)   /*!< 0x20000000 */
#define GPIO_OSPEEDR_OSPEEDR15_Pos      (30U)
#define GPIO_OSPEEDR_OSPEEDR15_Msk      (0x3UL << GPIO_OSPEEDR_OSPEEDR15_Pos)   /*!< 0xC0000000 */
#define GPIO_OSPEEDR_OSPEEDR15          GPIO_OSPEEDR_OSPEEDR15_Msk
#define GPIO_OSPEEDR_OSPEEDR15_0        (0x1UL << GPIO_OSPEEDR_OSPEEDR15_Pos)   /*!< 0x40000000 */
#define GPIO_OSPEEDR_OSPEEDR15_1        (0x2UL << GPIO_OSPEEDR_OSPEEDR15_Pos)   /*!< 0x80000000 */

/* Old Bit definition for GPIO_OSPEEDR register maintained for legacy purpose */
#define GPIO_OSPEEDER_OSPEEDR0     GPIO_OSPEEDR_OSPEEDR0
#define GPIO_OSPEEDER_OSPEEDR0_0   GPIO_OSPEEDR_OSPEEDR0_0
#define GPIO_OSPEEDER_OSPEEDR0_1   GPIO_OSPEEDR_OSPEEDR0_1
#define GPIO_OSPEEDER_OSPEEDR1     GPIO_OSPEEDR_OSPEEDR1
#define GPIO_OSPEEDER_OSPEEDR1_0   GPIO_OSPEEDR_OSPEEDR1_0
#define GPIO_OSPEEDER_OSPEEDR1_1   GPIO_OSPEEDR_OSPEEDR1_1
#define GPIO_OSPEEDER_OSPEEDR2     GPIO_OSPEEDR_OSPEEDR2
#define GPIO_OSPEEDER_OSPEEDR2_0   GPIO_OSPEEDR_OSPEEDR2_0
#define GPIO_OSPEEDER_OSPEEDR2_1   GPIO_OSPEEDR_OSPEEDR2_1
#define GPIO_OSPEEDER_OSPEEDR3     GPIO_OSPEEDR_OSPEEDR3
#define GPIO_OSPEEDER_OSPEEDR3_0   GPIO_OSPEEDR_OSPEEDR3_0
#define GPIO_OSPEEDER_OSPEEDR3_1   GPIO_OSPEEDR_OSPEEDR3_1
#define GPIO_OSPEEDER_OSPEEDR4     GPIO_OSPEEDR_OSPEEDR4
#define GPIO_OSPEEDER_OSPEEDR4_0   GPIO_OSPEEDR_OSPEEDR4_0
#define GPIO_OSPEEDER_OSPEEDR4_1   GPIO_OSPEEDR_OSPEEDR4_1
#define GPIO_OSPEEDER_OSPEEDR5     GPIO_OSPEEDR_OSPEEDR5
#define GPIO_OSPEEDER_OSPEEDR5_0   GPIO_OSPEEDR_OSPEEDR5_0
#define GPIO_OSPEEDER_OSPEEDR5_1   GPIO_OSPEEDR_OSPEEDR5_1
#define GPIO_OSPEEDER_OSPEEDR6     GPIO_OSPEEDR_OSPEEDR6
#define GPIO_OSPEEDER_OSPEEDR6_0   GPIO_OSPEEDR_OSPEEDR6_0
#define GPIO_OSPEEDER_OSPEEDR6_1   GPIO_OSPEEDR_OSPEEDR6_1
#define GPIO_OSPEEDER_OSPEEDR7     GPIO_OSPEEDR_OSPEEDR7
#define GPIO_OSPEEDER_OSPEEDR7_0   GPIO_OSPEEDR_OSPEEDR7_0
#define GPIO_OSPEEDER_OSPEEDR7_1   GPIO_OSPEEDR_OSPEEDR7_1
#define GPIO_OSPEEDER_OSPEEDR8     GPIO_OSPEEDR_OSPEEDR8
#define GPIO_OSPEEDER_OSPEEDR8_0   GPIO_OSPEEDR_OSPEEDR8_0
#define GPIO_OSPEEDER_OSPEEDR8_1   GPIO_OSPEEDR_OSPEEDR8_1
#define GPIO_OSPEEDER_OSPEEDR9     GPIO_OSPEEDR_OSPEEDR9
#define GPIO_OSPEEDER_OSPEEDR9_0   GPIO_OSPEEDR_OSPEEDR9_0
#define GPIO_OSPEEDER_OSPEEDR9_1   GPIO_OSPEEDR_OSPEEDR9_1
#define GPIO_OSPEEDER_OSPEEDR10    GPIO_OSPEEDR_OSPEEDR10
#define GPIO_OSPEEDER_OSPEEDR10_0  GPIO_OSPEEDR_OSPEEDR10_0
#define GPIO_OSPEEDER_OSPEEDR10_1  GPIO_OSPEEDR_OSPEEDR10_1
#define GPIO_OSPEEDER_OSPEEDR11    GPIO_OSPEEDR_OSPEEDR11
#define GPIO_OSPEEDER_OSPEEDR11_0  GPIO_OSPEEDR_OSPEEDR11_0
#define GPIO_OSPEEDER_OSPEEDR11_1  GPIO_OSPEEDR_OSPEEDR11_1
#define GPIO_OSPEEDER_OSPEEDR12    GPIO_OSPEEDR_OSPEEDR12
#define GPIO_OSPEEDER_OSPEEDR12_0  GPIO_OSPEEDR_OSPEEDR12_0
#define GPIO_OSPEEDER_OSPEEDR12_1  GPIO_OSPEEDR_OSPEEDR12_1
#define GPIO_OSPEEDER_OSPEEDR13    GPIO_OSPEEDR_OSPEEDR13
#define GPIO_OSPEEDER_OSPEEDR13_0  GPIO_OSPEEDR_OSPEEDR13_0
#define GPIO_OSPEEDER_OSPEEDR13_1  GPIO_OSPEEDR_OSPEEDR13_1
#define GPIO_OSPEEDER_OSPEEDR14    GPIO_OSPEEDR_OSPEEDR14
#define GPIO_OSPEEDER_OSPEEDR14_0  GPIO_OSPEEDR_OSPEEDR14_0
#define GPIO_OSPEEDER_OSPEEDR14_1  GPIO_OSPEEDR_OSPEEDR14_1
#define GPIO_OSPEEDER_OSPEEDR15    GPIO_OSPEEDR_OSPEEDR15
#define GPIO_OSPEEDER_OSPEEDR15_0  GPIO_OSPEEDR_OSPEEDR15_0
#define GPIO_OSPEEDER_OSPEEDR15_1  GPIO_OSPEEDR_OSPEEDR15_1

/*******************  Bit definition for GPIO_PUPDR register ******************/
#define GPIO_PUPDR_PUPDR0_Pos           (0U)
#define GPIO_PUPDR_PUPDR0_Msk           (0x3UL << GPIO_PUPDR_PUPDR0_Pos)        /*!< 0x00000003 */
#define GPIO_PUPDR_PUPDR0               GPIO_PUPDR_PUPDR0_Msk
#define GPIO_PUPDR_PUPDR0_0             (0x1UL << GPIO_PUPDR_PUPDR0_Pos)        /*!< 0x00000001 */
#define GPIO_PUPDR_PUPDR0_1             (0x2UL << GPIO_PUPDR_PUPDR0_Pos)        /*!< 0x00000002 */
#define GPIO_PUPDR_PUPDR1_Pos           (2U)
#define GPIO_PUPDR_PUPDR1_Msk           (0x3UL << GPIO_PUPDR_PUPDR1_Pos)        /*!< 0x0000000C */
#define GPIO_PUPDR_PUPDR1               GPIO_PUPDR_PUPDR1_Msk
#define GPIO_PUPDR_PUPDR1_0             (0x1UL << GPIO_PUPDR_PUPDR1_Pos)        /*!< 0x00000004 */
#define GPIO_PUPDR_PUPDR1_1             (0x2UL << GPIO_PUPDR_PUPDR1_Pos)        /*!< 0x00000008 */
#define GPIO_PUPDR_PUPDR2_Pos           (4U)
#define GPIO_PUPDR_PUPDR2_Msk           (0x3UL << GPIO_PUPDR_PUPDR2_Pos)        /*!< 0x00000030 */
#define GPIO_PUPDR_PUPDR2               GPIO_PUPDR_PUPDR2_Msk
#define GPIO_PUPDR_PUPDR2_0             (0x1UL << GPIO_PUPDR_PUPDR2_Pos)        /*!< 0x00000010 */
#define GPIO_PUPDR_PUPDR2_1             (0x2UL << GPIO_PUPDR_PUPDR2_Pos)        /*!< 0x00000020 */
#define GPIO_PUPDR_PUPDR3_Pos           (6U)
#define GPIO_PUPDR_PUPDR3_Msk           (0x3UL << GPIO_PUPDR_PUPDR3_Pos)        /*!< 0x000000C0 */
#define GPIO_PUPDR_PUPDR3               GPIO_PUPDR_PUPDR3_Msk
#define GPIO_PUPDR_PUPDR3_0             (0x1UL << GPIO_PUPDR_PUPDR3_Pos)        /*!< 0x00000040 */
#define GPIO_PUPDR_PUPDR3_1             (0x2UL << GPIO_PUPDR_PUPDR3_Pos)        /*!< 0x00000080 */
#define GPIO_PUPDR_PUPDR4_Pos           (8U)
#define GPIO_PUPDR_PUPDR4_Msk           (0x3UL << GPIO_PUPDR_PUPDR4_Pos)        /*!< 0x00000300 */
#define GPIO_PUPDR_PUPDR4               GPIO_PUPDR_PUPDR4_Msk
#define GPIO_PUPDR_PUPDR4_0             (0x1UL << GPIO_PUPDR_PUPDR4_Pos)        /*!< 0x00000100 */
#define GPIO_PUPDR_PUPDR4_1             (0x2UL << GPIO_PUPDR_PUPDR4_Pos)        /*!< 0x00000200 */
#define GPIO_PUPDR_PUPDR5_Pos           (10U)
#define GPIO_PUPDR_PUPDR5_Msk           (0x3UL << GPIO_PUPDR_PUPDR5_Pos)        /*!< 0x00000C00 */
#define GPIO_PUPDR_PUPDR5               GPIO_PUPDR_PUPDR5_Msk
#define GPIO_PUPDR_PUPDR5_0             (0x1UL << GPIO_PUPDR_PUPDR5_Pos)        /*!< 0x00000400 */
#define GPIO_PUPDR_PUPDR5_1             (0x2UL << GPIO_PUPDR_PUPDR5_Pos)        /*!< 0x00000800 */
#define GPIO_PUPDR_PUPDR6_Pos           (12U)
#define GPIO_PUPDR_PUPDR6_Msk           (0x3UL << GPIO_PUPDR_PUPDR6_Pos)        /*!< 0x00003000 */
#define GPIO_PUPDR_PUPDR6               GPIO_PUPDR_PUPDR6_Msk
#define GPIO_PUPDR_PUPDR6_0             (0x1UL << GPIO_PUPDR_PUPDR6_Pos)        /*!< 0x00001000 */
#define GPIO_PUPDR_PUPDR6_1             (0x2UL << GPIO_PUPDR_PUPDR6_Pos)        /*!< 0x00002000 */
#define GPIO_PUPDR_PUPDR7_Pos           (14U)
#define GPIO_PUPDR_PUPDR7_Msk           (0x3UL << GPIO_PUPDR_PUPDR7_Pos)        /*!< 0x0000C000 */
#define GPIO_PUPDR_PUPDR7               GPIO_PUPDR_PUPDR7_Msk
#define GPIO_PUPDR_PUPDR7_0             (0x1UL << GPIO_PUPDR_PUPDR7_Pos)        /*!< 0x00004000 */
#define GPIO_PUPDR_PUPDR7_1             (0x2UL << GPIO_PUPDR_PUPDR7_Pos)        /*!< 0x00008000 */
#define GPIO_PUPDR_PUPDR8_Pos           (16U)
#define GPIO_PUPDR_PUPDR8_Msk           (0x3UL << GPIO_PUPDR_PUPDR8_Pos)        /*!< 0x00030000 */
#define GPIO_PUPDR_PUPDR8               GPIO_PUPDR_PUPDR8_Msk
#define GPIO_PUPDR_PUPDR8_0             (0x1UL << GPIO_PUPDR_PUPDR8_Pos)        /*!< 0x00010000 */
#define GPIO_PUPDR_PUPDR8_1             (0x2UL << GPIO_PUPDR_PUPDR8_Pos)        /*!< 0x00020000 */
#define GPIO_PUPDR_PUPDR9_Pos           (18U)
#define GPIO_PUPDR_PUPDR9_Msk           (0x3UL << GPIO_PUPDR_PUPDR9_Pos)        /*!< 0x000C0000 */
#define GPIO_PUPDR_PUPDR9               GPIO_PUPDR_PUPDR9_Msk
#define GPIO_PUPDR_PUPDR9_0             (0x1UL << GPIO_PUPDR_PUPDR9_Pos)        /*!< 0x00040000 */
#define GPIO_PUPDR_PUPDR9_1             (0x2UL << GPIO_PUPDR_PUPDR9_Pos)        /*!< 0x00080000 */
#define GPIO_PUPDR_PUPDR10_Pos          (20U)
#define GPIO_PUPDR_PUPDR10_Msk          (0x3UL << GPIO_PUPDR_PUPDR10_Pos)       /*!< 0x00300000 */
#define GPIO_PUPDR_PUPDR10              GPIO_PUPDR_PUPDR10_Msk
#define GPIO_PUPDR_PUPDR10_0            (0x1UL << GPIO_PUPDR_PUPDR10_Pos)       /*!< 0x00100000 */
#define GPIO_PUPDR_PUPDR10_1            (0x2UL << GPIO_PUPDR_PUPDR10_Pos)       /*!< 0x00200000 */
#define GPIO_PUPDR_PUPDR11_Pos          (22U)
#define GPIO_PUPDR_PUPDR11_Msk          (0x3UL << GPIO_PUPDR_PUPDR11_Pos)       /*!< 0x00C00000 */
#define GPIO_PUPDR_PUPDR11              GPIO_PUPDR_PUPDR11_Msk
#define GPIO_PUPDR_PUPDR11_0            (0x1UL << GPIO_PUPDR_PUPDR11_Pos)       /*!< 0x00400000 */
#define GPIO_PUPDR_PUPDR11_1            (0x2UL << GPIO_PUPDR_PUPDR11_Pos)       /*!< 0x00800000 */
#define GPIO_PUPDR_PUPDR12_Pos          (24U)
#define GPIO_PUPDR_PUPDR12_Msk          (0x3UL << GPIO_PUPDR_PUPDR12_Pos)       /*!< 0x03000000 */
#define GPIO_PUPDR_PUPDR12              GPIO_PUPDR_PUPDR12_Msk
#define GPIO_PUPDR_PUPDR12_0            (0x1UL << GPIO_PUPDR_PUPDR12_Pos)       /*!< 0x01000000 */
#define GPIO_PUPDR_PUPDR12_1            (0x2UL << GPIO_PUPDR_PUPDR12_Pos)       /*!< 0x02000000 */
#define GPIO_PUPDR_PUPDR13_Pos          (26U)
#define GPIO_PUPDR_PUPDR13_Msk          (0x3UL << GPIO_PUPDR_PUPDR13_Pos)       /*!< 0x0C000000 */
#define GPIO_PUPDR_PUPDR13              GPIO_PUPDR_PUPDR13_Msk
#define GPIO_PUPDR_PUPDR13_0            (0x1UL << GPIO_PUPDR_PUPDR13_Pos)       /*!< 0x04000000 */
#define GPIO_PUPDR_PUPDR13_1            (0x2UL << GPIO_PUPDR_PUPDR13_Pos)       /*!< 0x08000000 */
#define GPIO_PUPDR_PUPDR14_Pos          (28U)
#define GPIO_PUPDR_PUPDR14_Msk          (0x3UL << GPIO_PUPDR_PUPDR14_Pos)       /*!< 0x30000000 */
#define GPIO_PUPDR_PUPDR14              GPIO_PUPDR_PUPDR14_Msk
#define GPIO_PUPDR_PUPDR14_0            (0x1UL << GPIO_PUPDR_PUPDR14_Pos)       /*!< 0x10000000 */
#define GPIO_PUPDR_PUPDR14_1            (0x2UL << GPIO_PUPDR_PUPDR14_Pos)       /*!< 0x20000000 */
#define GPIO_PUPDR_PUPDR15_Pos          (30U)
#define GPIO_PUPDR_PUPDR15_Msk          (0x3UL << GPIO_PUPDR_PUPDR15_Pos)       /*!< 0xC0000000 */
#define GPIO_PUPDR_PUPDR15              GPIO_PUPDR_PUPDR15_Msk
#define GPIO_PUPDR_PUPDR15_0            (0x1UL << GPIO_PUPDR_PUPDR15_Pos)       /*!< 0x40000000 */
#define GPIO_PUPDR_PUPDR15_1            (0x2UL << GPIO_PUPDR_PUPDR15_Pos)       /*!< 0x80000000 */

/*******************  Bit definition for GPIO_IDR register  *******************/
#define GPIO_IDR_0                      (0x00000001U)
#define GPIO_IDR_1                      (0x00000002U)
#define GPIO_IDR_2                      (0x00000004U)
#define GPIO_IDR_3                      (0x00000008U)
#define GPIO_IDR_4                      (0x00000010U)
#define GPIO_IDR_5                      (0x00000020U)
#define GPIO_IDR_6                      (0x00000040U)
#define GPIO_IDR_7                      (0x00000080U)
#define GPIO_IDR_8                      (0x00000100U)
#define GPIO_IDR_9                      (0x00000200U)
#define GPIO_IDR_10                     (0x00000400U)
#define GPIO_IDR_11                     (0x00000800U)
#define GPIO_IDR_12                     (0x00001000U)
#define GPIO_IDR_13                     (0x00002000U)
#define GPIO_IDR_14                     (0x00004000U)
#define GPIO_IDR_15                     (0x00008000U)

/******************  Bit definition for GPIO_ODR register  ********************/
#define GPIO_ODR_0                      (0x00000001U)
#define GPIO_ODR_1                      (0x00000002U)
#define GPIO_ODR_2                      (0x00000004U)
#define GPIO_ODR_3                      (0x00000008U)
#define GPIO_ODR_4                      (0x00000010U)
#define GPIO_ODR_5                      (0x00000020U)
#define GPIO_ODR_6                      (0x00000040U)
#define GPIO_ODR_7                      (0x00000080U)
#define GPIO_ODR_8                      (0x00000100U)
#define GPIO_ODR_9                      (0x00000200U)
#define GPIO_ODR_10                     (0x00000400U)
#define GPIO_ODR_11                     (0x00000800U)
#define GPIO_ODR_12                     (0x00001000U)
#define GPIO_ODR_13                     (0x00002000U)
#define GPIO_ODR_14                     (0x00004000U)
#define GPIO_ODR_15                     (0x00008000U)

/****************** Bit definition for GPIO_BSRR register  ********************/
#define GPIO_BSRR_BS_0                  (0x00000001U)
#define GPIO_BSRR_BS_1                  (0x00000002U)
#define GPIO_BSRR_BS_2                  (0x00000004U)
#define GPIO_BSRR_BS_3                  (0x00000008U)
#define GPIO_BSRR_BS_4                  (0x00000010U)
#define GPIO_BSRR_BS_5                  (0x00000020U)
#define GPIO_BSRR_BS_6                  (0x00000040U)
#define GPIO_BSRR_BS_7                  (0x00000080U)
#define GPIO_BSRR_BS_8                  (0x00000100U)
#define GPIO_BSRR_BS_9                  (0x00000200U)
#define GPIO_BSRR_BS_10                 (0x00000400U)
#define GPIO_BSRR_BS_11                 (0x00000800U)
#define GPIO_BSRR_BS_12                 (0x00001000U)
#define GPIO_BSRR_BS_13                 (0x00002000U)
#define GPIO_BSRR_BS_14                 (0x00004000U)
#define GPIO_BSRR_BS_15                 (0x00008000U)
#define GPIO_BSRR_BR_0                  (0x00010000U)
#define GPIO_BSRR_BR_1                  (0x00020000U)
#define GPIO_BSRR_BR_2                  (0x00040000U)
#define GPIO_BSRR_BR_3                  (0x00080000U)
#define GPIO_BSRR_BR_4                  (0x00100000U)
#define GPIO_BSRR_BR_5                  (0x00200000U)
#define GPIO_BSRR_BR_6                  (0x00400000U)
#define GPIO_BSRR_BR_7                  (0x00800000U)
#define GPIO_BSRR_BR_8                  (0x01000000U)
#define GPIO_BSRR_BR_9                  (0x02000000U)
#define GPIO_BSRR_BR_10                 (0x04000000U)
#define GPIO_BSRR_BR_11                 (0x08000000U)
#define GPIO_BSRR_BR_12                 (0x10000000U)
#define GPIO_BSRR_BR_13                 (0x20000000U)
#define GPIO_BSRR_BR_14                 (0x40000000U)
#define GPIO_BSRR_BR_15                 (0x80000000U)

/****************** Bit definition for GPIO_LCKR register  ********************/
#define GPIO_LCKR_LCK0_Pos              (0U)
#define GPIO_LCKR_LCK0_Msk              (0x1UL << GPIO_LCKR_LCK0_Pos)           /*!< 0x00000001 */
#define GPIO_LCKR_LCK0                  GPIO_LCKR_LCK0_Msk
#define GPIO_LCKR_LCK1_Pos              (1U)
#define GPIO_LCKR_LCK1_Msk              (0x1UL << GPIO_LCKR_LCK1_Pos)           /*!< 0x00000002 */
#define GPIO_LCKR_LCK1                  GPIO_LCKR_LCK1_Msk
#define GPIO_LCKR_LCK2_Pos              (2U)
#define GPIO_LCKR_LCK2_Msk              (0x1UL << GPIO_LCKR_LCK2_Pos)           /*!< 0x00000004 */
#define GPIO_LCKR_LCK2                  GPIO_LCKR_LCK2_Msk
#define GPIO_LCKR_LCK3_Pos              (3U)
#define GPIO_LCKR_LCK3_Msk              (0x1UL << GPIO_LCKR_LCK3_Pos)           /*!< 0x00000008 */
#define GPIO_LCKR_LCK3                  GPIO_LCKR_LCK3_Msk
#define GPIO_LCKR_LCK4_Pos              (4U)
#define GPIO_LCKR_LCK4_Msk              (0x1UL << GPIO_LCKR_LCK4_Pos)           /*!< 0x00000010 */
#define GPIO_LCKR_LCK4                  GPIO_LCKR_LCK4_Msk
#define GPIO_LCKR_LCK5_Pos              (5U)
#define GPIO_LCKR_LCK5_Msk              (0x1UL << GPIO_LCKR_LCK5_Pos)           /*!< 0x00000020 */
#define GPIO_LCKR_LCK5                  GPIO_LCKR_LCK5_Msk
#define GPIO_LCKR_LCK6_Pos              (6U)
#define GPIO_LCKR_LCK6_Msk              (0x1UL << GPIO_LCKR_LCK6_Pos)           /*!< 0x00000040 */
#define GPIO_LCKR_LCK6                  GPIO_LCKR_LCK6_Msk
#define GPIO_LCKR_LCK7_Pos              (7U)
#define GPIO_LCKR_LCK7_Msk              (0x1UL << GPIO_LCKR_LCK7_Pos)           /*!< 0x00000080 */
#define GPIO_LCKR_LCK7                  GPIO_LCKR_LCK7_Msk
#define GPIO_LCKR_LCK8_Pos              (8U)
#define GPIO_LCKR_LCK8_Msk              (0x1UL << GPIO_LCKR_LCK8_Pos)           /*!< 0x00000100 */
#define GPIO_LCKR_LCK8                  GPIO_LCKR_LCK8_Msk
#define GPIO_LCKR_LCK9_Pos              (9U)
#define GPIO_LCKR_LCK9_Msk              (0x1UL << GPIO_LCKR_LCK9_Pos)           /*!< 0x00000200 */
#define GPIO_LCKR_LCK9                  GPIO_LCKR_LCK9_Msk
#define GPIO_LCKR_LCK10_Pos             (10U)
#define GPIO_LCKR_LCK10_Msk             (0x1UL << GPIO_LCKR_LCK10_Pos)          /*!< 0x00000400 */
#define GPIO_LCKR_LCK10                 GPIO_LCKR_LCK10_Msk
#define GPIO_LCKR_LCK11_Pos             (11U)
#define GPIO_LCKR_LCK11_Msk             (0x1UL << GPIO_LCKR_LCK11_Pos)          /*!< 0x00000800 */
#define GPIO_LCKR_LCK11                 GPIO_LCKR_LCK11_Msk
#define GPIO_LCKR_LCK12_Pos             (12U)
#define GPIO_LCKR_LCK12_Msk             (0x1UL << GPIO_LCKR_LCK12_Pos)          /*!< 0x00001000 */
#define GPIO_LCKR_LCK12                 GPIO_LCKR_LCK12_Msk
#define GPIO_LCKR_LCK13_Pos             (13U)
#define GPIO_LCKR_LCK13_Msk             (0x1UL << GPIO_LCKR_LCK13_Pos)          /*!< 0x00002000 */
#define GPIO_LCKR_LCK13                 GPIO_LCKR_LCK13_Msk
#define GPIO_LCKR_LCK14_Pos             (14U)
#define GPIO_LCKR_LCK14_Msk             (0x1UL << GPIO_LCKR_LCK14_Pos)          /*!< 0x00004000 */
#define GPIO_LCKR_LCK14                 GPIO_LCKR_LCK14_Msk
#define GPIO_LCKR_LCK15_Pos             (15U)
#define GPIO_LCKR_LCK15_Msk             (0x1UL << GPIO_LCKR_LCK15_Pos)          /*!< 0x00008000 */
#define GPIO_LCKR_LCK15                 GPIO_LCKR_LCK15_Msk
#define GPIO_LCKR_LCKK_Pos              (16U)
#define GPIO_LCKR_LCKK_Msk              (0x1UL << GPIO_LCKR_LCKK_Pos)           /*!< 0x00010000 */
#define GPIO_LCKR_LCKK                  GPIO_LCKR_LCKK_Msk

/****************** Bit definition for GPIO_AFRL register  ********************/
#define GPIO_AFRL_AFSEL0_Pos            (0U)
#define GPIO_AFRL_AFSEL0_Msk            (0xFUL << GPIO_AFRL_AFSEL0_Pos)         /*!< 0x0000000F */
#define GPIO_AFRL_AFSEL0                GPIO_AFRL_AFSEL0_Msk
#define GPIO_AFRL_AFSEL1_Pos            (4U)
#define GPIO_AFRL_AFSEL1_Msk            (0xFUL << GPIO_AFRL_AFSEL1_Pos)         /*!< 0x000000F0 */
#define GPIO_AFRL_AFSEL1                GPIO_AFRL_AFSEL1_Msk
#define GPIO_AFRL_AFSEL2_Pos            (8U)
#define GPIO_AFRL_AFSEL2_Msk            (0xFUL << GPIO_AFRL_AFSEL2_Pos)         /*!< 0x00000F00 */
#define GPIO_AFRL_AFSEL2                GPIO_AFRL_AFSEL2_Msk
#define GPIO_AFRL_AFSEL3_Pos            (12U)
#define GPIO_AFRL_AFSEL3_Msk            (0xFUL << GPIO_AFRL_AFSEL3_Pos)         /*!< 0x0000F000 */
#define GPIO_AFRL_AFSEL3                GPIO_AFRL_AFSEL3_Msk
#define GPIO_AFRL_AFSEL4_Pos            (16U)
#define GPIO_AFRL_AFSEL4_Msk            (0xFUL << GPIO_AFRL_AFSEL4_Pos)         /*!< 0x000F0000 */
#define GPIO_AFRL_AFSEL4                GPIO_AFRL_AFSEL4_Msk
#define GPIO_AFRL_AFSEL5_Pos            (20U)
#define GPIO_AFRL_AFSEL5_Msk            (0xFUL << GPIO_AFRL_AFSEL5_Pos)         /*!< 0x00F00000 */
#define GPIO_AFRL_AFSEL5                GPIO_AFRL_AFSEL5_Msk
#define GPIO_AFRL_AFSEL6_Pos            (24U)
#define GPIO_AFRL_AFSEL6_Msk            (0xFUL << GPIO_AFRL_AFSEL6_Pos)         /*!< 0x0F000000 */
#define GPIO_AFRL_AFSEL6                GPIO_AFRL_AFSEL6_Msk
#define GPIO_AFRL_AFSEL7_Pos            (28U)
#define GPIO_AFRL_AFSEL7_Msk            (0xFUL << GPIO_AFRL_AFSEL7_Pos)         /*!< 0xF0000000 */
#define GPIO_AFRL_AFSEL7                GPIO_AFRL_AFSEL7_Msk

/* Legacy aliases */
#define GPIO_AFRL_AFRL0_Pos             GPIO_AFRL_AFSEL0_Pos
#define GPIO_AFRL_AFRL0_Msk             GPIO_AFRL_AFSEL0_Msk
#define GPIO_AFRL_AFRL0                 GPIO_AFRL_AFSEL0
#define GPIO_AFRL_AFRL1_Pos             GPIO_AFRL_AFSEL1_Pos
#define GPIO_AFRL_AFRL1_Msk             GPIO_AFRL_AFSEL1_Msk
#define GPIO_AFRL_AFRL1                 GPIO_AFRL_AFSEL1
#define GPIO_AFRL_AFRL2_Pos             GPIO_AFRL_AFSEL2_Pos
#define GPIO_AFRL_AFRL2_Msk             GPIO_AFRL_AFSEL2_Msk
#define GPIO_AFRL_AFRL2                 GPIO_AFRL_AFSEL2
#define GPIO_AFRL_AFRL3_Pos             GPIO_AFRL_AFSEL3_Pos
#define GPIO_AFRL_AFRL3_Msk             GPIO_AFRL_AFSEL3_Msk
#define GPIO_AFRL_AFRL3                 GPIO_AFRL_AFSEL3
#define GPIO_AFRL_AFRL4_Pos             GPIO_AFRL_AFSEL4_Pos
#define GPIO_AFRL_AFRL4_Msk             GPIO_AFRL_AFSEL4_Msk
#define GPIO_AFRL_AFRL4                 GPIO_AFRL_AFSEL4
#define GPIO_AFRL_AFRL5_Pos             GPIO_AFRL_AFSEL5_Pos
#define GPIO_AFRL_AFRL5_Msk             GPIO_AFRL_AFSEL5_Msk
#define GPIO_AFRL_AFRL5                 GPIO_AFRL_AFSEL5
#define GPIO_AFRL_AFRL6_Pos             GPIO_AFRL_AFSEL6_Pos
#define GPIO_AFRL_AFRL6_Msk             GPIO_AFRL_AFSEL6_Msk
#define GPIO_AFRL_AFRL6                 GPIO_AFRL_AFSEL6
#define GPIO_AFRL_AFRL7_Pos             GPIO_AFRL_AFSEL7_Pos
#define GPIO_AFRL_AFRL7_Msk             GPIO_AFRL_AFSEL7_Msk
#define GPIO_AFRL_AFRL7                 GPIO_AFRL_AFSEL7

/****************** Bit definition for GPIO_AFRH register  ********************/
#define GPIO_AFRH_AFSEL8_Pos            (0U)
#define GPIO_AFRH_AFSEL8_Msk            (0xFUL << GPIO_AFRH_AFSEL8_Pos)         /*!< 0x0000000F */
#define GPIO_AFRH_AFSEL8                GPIO_AFRH_AFSEL8_Msk
#define GPIO_AFRH_AFSEL9_Pos            (4U)
#define GPIO_AFRH_AFSEL9_Msk            (0xFUL << GPIO_AFRH_AFSEL9_Pos)         /*!< 0x000000F0 */
#define GPIO_AFRH_AFSEL9                GPIO_AFRH_AFSEL9_Msk
#define GPIO_AFRH_AFSEL10_Pos           (8U)
#define GPIO_AFRH_AFSEL10_Msk           (0xFUL << GPIO_AFRH_AFSEL10_Pos)        /*!< 0x00000F00 */
#define GPIO_AFRH_AFSEL10               GPIO_AFRH_AFSEL10_Msk
#define GPIO_AFRH_AFSEL11_Pos           (12U)
#define GPIO_AFRH_AFSEL11_Msk           (0xFUL << GPIO_AFRH_AFSEL11_Pos)        /*!< 0x0000F000 */
#define GPIO_AFRH_AFSEL11               GPIO_AFRH_AFSEL11_Msk
#define GPIO_AFRH_AFSEL12_Pos           (16U)
#define GPIO_AFRH_AFSEL12_Msk           (0xFUL << GPIO_AFRH_AFSEL12_Pos)        /*!< 0x000F0000 */
#define GPIO_AFRH_AFSEL12               GPIO_AFRH_AFSEL12_Msk
#define GPIO_AFRH_AFSEL13_Pos           (20U)
#define GPIO_AFRH_AFSEL13_Msk           (0xFUL << GPIO_AFRH_AFSEL13_Pos)        /*!< 0x00F00000 */
#define GPIO_AFRH_AFSEL13               GPIO_AFRH_AFSEL13_Msk
#define GPIO_AFRH_AFSEL14_Pos           (24U)
#define GPIO_AFRH_AFSEL14_Msk           (0xFUL << GPIO_AFRH_AFSEL14_Pos)        /*!< 0x0F000000 */
#define GPIO_AFRH_AFSEL14               GPIO_AFRH_AFSEL14_Msk
#define GPIO_AFRH_AFSEL15_Pos           (28U)
#define GPIO_AFRH_AFSEL15_Msk           (0xFUL << GPIO_AFRH_AFSEL15_Pos)        /*!< 0xF0000000 */
#define GPIO_AFRH_AFSEL15               GPIO_AFRH_AFSEL15_Msk

/* Legacy aliases */
#define GPIO_AFRH_AFRH0_Pos             GPIO_AFRH_AFSEL8_Pos
#define GPIO_AFRH_AFRH0_Msk             GPIO_AFRH_AFSEL8_Msk
#define GPIO_AFRH_AFRH0                 GPIO_AFRH_AFSEL8
#define GPIO_AFRH_AFRH1_Pos             GPIO_AFRH_AFSEL9_Pos
#define GPIO_AFRH_AFRH1_Msk             GPIO_AFRH_AFSEL9_Msk
#define GPIO_AFRH_AFRH1                 GPIO_AFRH_AFSEL9
#define GPIO_AFRH_AFRH2_Pos             GPIO_AFRH_AFSEL10_Pos
#define GPIO_AFRH_AFRH2_Msk             GPIO_AFRH_AFSEL10_Msk
#define GPIO_AFRH_AFRH2                 GPIO_AFRH_AFSEL10
#define GPIO_AFRH_AFRH3_Pos             GPIO_AFRH_AFSEL11_Pos
#define GPIO_AFRH_AFRH3_Msk             GPIO_AFRH_AFSEL11_Msk
#define GPIO_AFRH_AFRH3                 GPIO_AFRH_AFSEL11
#define GPIO_AFRH_AFRH4_Pos             GPIO_AFRH_AFSEL12_Pos
#define GPIO_AFRH_AFRH4_Msk             GPIO_AFRH_AFSEL12_Msk
#define GPIO_AFRH_AFRH4                 GPIO_AFRH_AFSEL12
#define GPIO_AFRH_AFRH5_Pos             GPIO_AFRH_AFSEL13_Pos
#define GPIO_AFRH_AFRH5_Msk             GPIO_AFRH_AFSEL13_Msk
#define GPIO_AFRH_AFRH5                 GPIO_AFRH_AFSEL13
#define GPIO_AFRH_AFRH6_Pos             GPIO_AFRH_AFSEL14_Pos
#define GPIO_AFRH_AFRH6_Msk             GPIO_AFRH_AFSEL14_Msk
#define GPIO_AFRH_AFRH6                 GPIO_AFRH_AFSEL14
#define GPIO_AFRH_AFRH7_Pos             GPIO_AFRH_AFSEL15_Pos
#define GPIO_AFRH_AFRH7_Msk             GPIO_AFRH_AFSEL15_Msk
#define GPIO_AFRH_AFRH7                 GPIO_AFRH_AFSEL15

/****************** Bit definition for GPIO_BRR register  *********************/
#define GPIO_BRR_BR_0                   (0x00000001U)
#define GPIO_BRR_BR_1                   (0x00000002U)
#define GPIO_BRR_BR_2                   (0x00000004U)
#define GPIO_BRR_BR_3                   (0x00000008U)
#define GPIO_BRR_BR_4                   (0x00000010U)
#define GPIO_BRR_BR_5                   (0x00000020U)
#define GPIO_BRR_BR_6                   (0x00000040U)
#define GPIO_BRR_BR_7                   (0x00000080U)
#define GPIO_BRR_BR_8                   (0x00000100U)
#define GPIO_BRR_BR_9                   (0x00000200U)
#define GPIO_BRR_BR_10                  (0x00000400U)
#define GPIO_BRR_BR_11                  (0x00000800U)
#define GPIO_BRR_BR_12                  (0x00001000U)
#define GPIO_BRR_BR_13                  (0x00002000U)
#define GPIO_BRR_BR_14                  (0x00004000U)
#define GPIO_BRR_BR_15                  (0x00008000U)

/******************************************************************************/
/*                                                                            */
/*                   Inter-integrated Circuit Interface (I2C)                 */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for I2C_CR1 register  *******************/
#define I2C_CR1_PE_Pos               (0U)
#define I2C_CR1_PE_Msk               (0x1UL << I2C_CR1_PE_Pos)                  /*!< 0x00000001 */
#define I2C_CR1_PE                   I2C_CR1_PE_Msk                            /*!< Peripheral enable */
#define I2C_CR1_TXIE_Pos             (1U)
#define I2C_CR1_TXIE_Msk             (0x1UL << I2C_CR1_TXIE_Pos)                /*!< 0x00000002 */
#define I2C_CR1_TXIE                 I2C_CR1_TXIE_Msk                          /*!< TX interrupt enable */
#define I2C_CR1_RXIE_Pos             (2U)
#define I2C_CR1_RXIE_Msk             (0x1UL << I2C_CR1_RXIE_Pos)                /*!< 0x00000004 */
#define I2C_CR1_RXIE                 I2C_CR1_RXIE_Msk                          /*!< RX interrupt enable */
#define I2C_CR1_ADDRIE_Pos           (3U)
#define I2C_CR1_ADDRIE_Msk           (0x1UL << I2C_CR1_ADDRIE_Pos)              /*!< 0x00000008 */
#define I2C_CR1_ADDRIE               I2C_CR1_ADDRIE_Msk                        /*!< Address match interrupt enable */
#define I2C_CR1_NACKIE_Pos           (4U)
#define I2C_CR1_NACKIE_Msk           (0x1UL << I2C_CR1_NACKIE_Pos)              /*!< 0x00000010 */
#define I2C_CR1_NACKIE               I2C_CR1_NACKIE_Msk                        /*!< NACK received interrupt enable */
#define I2C_CR1_STOPIE_Pos           (5U)
#define I2C_CR1_STOPIE_Msk           (0x1UL << I2C_CR1_STOPIE_Pos)              /*!< 0x00000020 */
#define I2C_CR1_STOPIE               I2C_CR1_STOPIE_Msk                        /*!< STOP detection interrupt enable */
#define I2C_CR1_TCIE_Pos             (6U)
#define I2C_CR1_TCIE_Msk             (0x1UL << I2C_CR1_TCIE_Pos)                /*!< 0x00000040 */
#define I2C_CR1_TCIE                 I2C_CR1_TCIE_Msk                          /*!< Transfer complete interrupt enable */
#define I2C_CR1_ERRIE_Pos            (7U)
#define I2C_CR1_ERRIE_Msk            (0x1UL << I2C_CR1_ERRIE_Pos)               /*!< 0x00000080 */
#define I2C_CR1_ERRIE                I2C_CR1_ERRIE_Msk                         /*!< Errors interrupt enable */
#define I2C_CR1_DNF_Pos              (8U)
#define I2C_CR1_DNF_Msk              (0xFUL << I2C_CR1_DNF_Pos)                 /*!< 0x00000F00 */
#define I2C_CR1_DNF                  I2C_CR1_DNF_Msk                           /*!< Digital noise filter */
#define I2C_CR1_ANFOFF_Pos           (12U)
#define I2C_CR1_ANFOFF_Msk           (0x1UL << I2C_CR1_ANFOFF_Pos)              /*!< 0x00001000 */
#define I2C_CR1_ANFOFF               I2C_CR1_ANFOFF_Msk                        /*!< Analog noise filter OFF */
#define I2C_CR1_SWRST_Pos            (13U)
#define I2C_CR1_SWRST_Msk            (0x1UL << I2C_CR1_SWRST_Pos)               /*!< 0x00002000 */
#define I2C_CR1_SWRST                I2C_CR1_SWRST_Msk                         /*!< Software reset */
#define I2C_CR1_TXDMAEN_Pos          (14U)
#define I2C_CR1_TXDMAEN_Msk          (0x1UL << I2C_CR1_TXDMAEN_Pos)             /*!< 0x00004000 */
#define I2C_CR1_TXDMAEN              I2C_CR1_TXDMAEN_Msk                       /*!< DMA transmission requests enable */
#define I2C_CR1_RXDMAEN_Pos          (15U)
#define I2C_CR1_RXDMAEN_Msk          (0x1UL << I2C_CR1_RXDMAEN_Pos)             /*!< 0x00008000 */
#define I2C_CR1_RXDMAEN              I2C_CR1_RXDMAEN_Msk                       /*!< DMA reception requests enable */
#define I2C_CR1_SBC_Pos              (16U)
#define I2C_CR1_SBC_Msk              (0x1UL << I2C_CR1_SBC_Pos)                 /*!< 0x00010000 */
#define I2C_CR1_SBC                  I2C_CR1_SBC_Msk                           /*!< Slave byte control */
#define I2C_CR1_NOSTRETCH_Pos        (17U)
#define I2C_CR1_NOSTRETCH_Msk        (0x1UL << I2C_CR1_NOSTRETCH_Pos)           /*!< 0x00020000 */
#define I2C_CR1_NOSTRETCH            I2C_CR1_NOSTRETCH_Msk                     /*!< Clock stretching disable */
#define I2C_CR1_WUPEN_Pos            (18U)
#define I2C_CR1_WUPEN_Msk            (0x1UL << I2C_CR1_WUPEN_Pos)               /*!< 0x00040000 */
#define I2C_CR1_WUPEN                I2C_CR1_WUPEN_Msk                         /*!< Wakeup from STOP enable */
#define I2C_CR1_GCEN_Pos             (19U)
#define I2C_CR1_GCEN_Msk             (0x1UL << I2C_CR1_GCEN_Pos)                /*!< 0x00080000 */
#define I2C_CR1_GCEN                 I2C_CR1_GCEN_Msk                          /*!< General call enable */
#define I2C_CR1_SMBHEN_Pos           (20U)
#define I2C_CR1_SMBHEN_Msk           (0x1UL << I2C_CR1_SMBHEN_Pos)              /*!< 0x00100000 */
#define I2C_CR1_SMBHEN               I2C_CR1_SMBHEN_Msk                        /*!< SMBus host address enable */
#define I2C_CR1_SMBDEN_Pos           (21U)
#define I2C_CR1_SMBDEN_Msk           (0x1UL << I2C_CR1_SMBDEN_Pos)              /*!< 0x00200000 */
#define I2C_CR1_SMBDEN               I2C_CR1_SMBDEN_Msk                        /*!< SMBus device default address enable */
#define I2C_CR1_ALERTEN_Pos          (22U)
#define I2C_CR1_ALERTEN_Msk          (0x1UL << I2C_CR1_ALERTEN_Pos)             /*!< 0x00400000 */
#define I2C_CR1_ALERTEN              I2C_CR1_ALERTEN_Msk                       /*!< SMBus alert enable */
#define I2C_CR1_PECEN_Pos            (23U)
#define I2C_CR1_PECEN_Msk            (0x1UL << I2C_CR1_PECEN_Pos)               /*!< 0x00800000 */
#define I2C_CR1_PECEN                I2C_CR1_PECEN_Msk                         /*!< PEC enable */

/******************  Bit definition for I2C_CR2 register  ********************/
#define I2C_CR2_SADD_Pos             (0U)
#define I2C_CR2_SADD_Msk             (0x3FFUL << I2C_CR2_SADD_Pos)              /*!< 0x000003FF */
#define I2C_CR2_SADD                 I2C_CR2_SADD_Msk                          /*!< Slave address (master mode) */
#define I2C_CR2_RD_WRN_Pos           (10U)
#define I2C_CR2_RD_WRN_Msk           (0x1UL << I2C_CR2_RD_WRN_Pos)              /*!< 0x00000400 */
#define I2C_CR2_RD_WRN               I2C_CR2_RD_WRN_Msk                        /*!< Transfer direction (master mode) */
#define I2C_CR2_ADD10_Pos            (11U)
#define I2C_CR2_ADD10_Msk            (0x1UL << I2C_CR2_ADD10_Pos)               /*!< 0x00000800 */
#define I2C_CR2_ADD10                I2C_CR2_ADD10_Msk                         /*!< 10-bit addressing mode (master mode) */
#define I2C_CR2_HEAD10R_Pos          (12U)
#define I2C_CR2_HEAD10R_Msk          (0x1UL << I2C_CR2_HEAD10R_Pos)             /*!< 0x00001000 */
#define I2C_CR2_HEAD10R              I2C_CR2_HEAD10R_Msk                       /*!< 10-bit address header only read direction (master mode) */
#define I2C_CR2_START_Pos            (13U)
#define I2C_CR2_START_Msk            (0x1UL << I2C_CR2_START_Pos)               /*!< 0x00002000 */
#define I2C_CR2_START                I2C_CR2_START_Msk                         /*!< START generation */
#define I2C_CR2_STOP_Pos             (14U)
#define I2C_CR2_STOP_Msk             (0x1UL << I2C_CR2_STOP_Pos)                /*!< 0x00004000 */
#define I2C_CR2_STOP                 I2C_CR2_STOP_Msk                          /*!< STOP generation (master mode) */
#define I2C_CR2_NACK_Pos             (15U)
#define I2C_CR2_NACK_Msk             (0x1UL << I2C_CR2_NACK_Pos)                /*!< 0x00008000 */
#define I2C_CR2_NACK                 I2C_CR2_NACK_Msk                          /*!< NACK generation (slave mode) */
#define I2C_CR2_NBYTES_Pos           (16U)
#define I2C_CR2_NBYTES_Msk           (0xFFUL << I2C_CR2_NBYTES_Pos)             /*!< 0x00FF0000 */
#define I2C_CR2_NBYTES               I2C_CR2_NBYTES_Msk                        /*!< Number of bytes */
#define I2C_CR2_RELOAD_Pos           (24U)
#define I2C_CR2_RELOAD_Msk           (0x1UL << I2C_CR2_RELOAD_Pos)              /*!< 0x01000000 */
#define I2C_CR2_RELOAD               I2C_CR2_RELOAD_Msk                        /*!< NBYTES reload mode */
#define I2C_CR2_AUTOEND_Pos          (25U)
#define I2C_CR2_AUTOEND_Msk          (0x1UL << I2C_CR2_AUTOEND_Pos)             /*!< 0x02000000 */
#define I2C_CR2_AUTOEND              I2C_CR2_AUTOEND_Msk                       /*!< Automatic end mode (master mode) */
#define I2C_CR2_PECBYTE_Pos          (26U)
#define I2C_CR2_PECBYTE_Msk          (0x1UL << I2C_CR2_PECBYTE_Pos)             /*!< 0x04000000 */
#define I2C_CR2_PECBYTE              I2C_CR2_PECBYTE_Msk                       /*!< Packet error checking byte */

/*******************  Bit definition for I2C_OAR1 register  ******************/
#define I2C_OAR1_OA1_Pos             (0U)
#define I2C_OAR1_OA1_Msk             (0x3FFUL << I2C_OAR1_OA1_Pos)              /*!< 0x000003FF */
#define I2C_OAR1_OA1                 I2C_OAR1_OA1_Msk                          /*!< Interface own address 1 */
#define I2C_OAR1_OA1MODE_Pos         (10U)
#define I2C_OAR1_OA1MODE_Msk         (0x1UL << I2C_OAR1_OA1MODE_Pos)            /*!< 0x00000400 */
#define I2C_OAR1_OA1MODE             I2C_OAR1_OA1MODE_Msk                      /*!< Own address 1 10-bit mode */
#define I2C_OAR1_OA1EN_Pos           (15U)
#define I2C_OAR1_OA1EN_Msk           (0x1UL << I2C_OAR1_OA1EN_Pos)              /*!< 0x00008000 */
#define I2C_OAR1_OA1EN               I2C_OAR1_OA1EN_Msk                        /*!< Own address 1 enable */

/*******************  Bit definition for I2C_OAR2 register  ******************/
#define I2C_OAR2_OA2_Pos             (1U)
#define I2C_OAR2_OA2_Msk             (0x7FUL << I2C_OAR2_OA2_Pos)               /*!< 0x000000FE */
#define I2C_OAR2_OA2                 I2C_OAR2_OA2_Msk                          /*!< Interface own address 2 */
#define I2C_OAR2_OA2MSK_Pos          (8U)
#define I2C_OAR2_OA2MSK_Msk          (0x7UL << I2C_OAR2_OA2MSK_Pos)             /*!< 0x00000700 */
#define I2C_OAR2_OA2MSK              I2C_OAR2_OA2MSK_Msk                       /*!< Own address 2 masks */
#define I2C_OAR2_OA2NOMASK           (0x00000000U)                             /*!< No mask                                        */
#define I2C_OAR2_OA2MASK01_Pos       (8U)
#define I2C_OAR2_OA2MASK01_Msk       (0x1UL << I2C_OAR2_OA2MASK01_Pos)          /*!< 0x00000100 */
#define I2C_OAR2_OA2MASK01           I2C_OAR2_OA2MASK01_Msk                    /*!< OA2[1] is masked, Only OA2[7:2] are compared   */
#define I2C_OAR2_OA2MASK02_Pos       (9U)
#define I2C_OAR2_OA2MASK02_Msk       (0x1UL << I2C_OAR2_OA2MASK02_Pos)          /*!< 0x00000200 */
#define I2C_OAR2_OA2MASK02           I2C_OAR2_OA2MASK02_Msk                    /*!< OA2[2:1] is masked, Only OA2[7:3] are compared */
#define I2C_OAR2_OA2MASK03_Pos       (8U)
#define I2C_OAR2_OA2MASK03_Msk       (0x3UL << I2C_OAR2_OA2MASK03_Pos)          /*!< 0x00000300 */
#define I2C_OAR2_OA2MASK03           I2C_OAR2_OA2MASK03_Msk                    /*!< OA2[3:1] is masked, Only OA2[7:4] are compared */
#define I2C_OAR2_OA2MASK04_Pos       (10U)
#define I2C_OAR2_OA2MASK04_Msk       (0x1UL << I2C_OAR2_OA2MASK04_Pos)          /*!< 0x00000400 */
#define I2C_OAR2_OA2MASK04           I2C_OAR2_OA2MASK04_Msk                    /*!< OA2[4:1] is masked, Only OA2[7:5] are compared */
#define I2C_OAR2_OA2MASK05_Pos       (8U)
#define I2C_OAR2_OA2MASK05_Msk       (0x5UL << I2C_OAR2_OA2MASK05_Pos)          /*!< 0x00000500 */
#define I2C_OAR2_OA2MASK05           I2C_OAR2_OA2MASK05_Msk                    /*!< OA2[5:1] is masked, Only OA2[7:6] are compared */
#define I2C_OAR2_OA2MASK06_Pos       (9U)
#define I2C_OAR2_OA2MASK06_Msk       (0x3UL << I2C_OAR2_OA2MASK06_Pos)          /*!< 0x00000600 */
#define I2C_OAR2_OA2MASK06           I2C_OAR2_OA2MASK06_Msk                    /*!< OA2[6:1] is masked, Only OA2[7] are compared   */
#define I2C_OAR2_OA2MASK07_Pos       (8U)
#define I2C_OAR2_OA2MASK07_Msk       (0x7UL << I2C_OAR2_OA2MASK07_Pos)          /*!< 0x00000700 */
#define I2C_OAR2_OA2MASK07           I2C_OAR2_OA2MASK07_Msk                    /*!< OA2[7:1] is masked, No comparison is done      */
#define I2C_OAR2_OA2EN_Pos           (15U)
#define I2C_OAR2_OA2EN_Msk           (0x1UL << I2C_OAR2_OA2EN_Pos)              /*!< 0x00008000 */
#define I2C_OAR2_OA2EN               I2C_OAR2_OA2EN_Msk                        /*!< Own address 2 enable */

/*******************  Bit definition for I2C_TIMINGR register ****************/
#define I2C_TIMINGR_SCLL_Pos         (0U)
#define I2C_TIMINGR_SCLL_Msk         (0xFFUL << I2C_TIMINGR_SCLL_Pos)           /*!< 0x000000FF */
#define I2C_TIMINGR_SCLL             I2C_TIMINGR_SCLL_Msk                      /*!< SCL low period (master mode) */
#define I2C_TIMINGR_SCLH_Pos         (8U)
#define I2C_TIMINGR_SCLH_Msk         (0xFFUL << I2C_TIMINGR_SCLH_Pos)           /*!< 0x0000FF00 */
#define I2C_TIMINGR_SCLH             I2C_TIMINGR_SCLH_Msk                      /*!< SCL high period (master mode) */
#define I2C_TIMINGR_SDADEL_Pos       (16U)
#define I2C_TIMINGR_SDADEL_Msk       (0xFUL << I2C_TIMINGR_SDADEL_Pos)          /*!< 0x000F0000 */
#define I2C_TIMINGR_SDADEL           I2C_TIMINGR_SDADEL_Msk                    /*!< Data hold time */
#define I2C_TIMINGR_SCLDEL_Pos       (20U)
#define I2C_TIMINGR_SCLDEL_Msk       (0xFUL << I2C_TIMINGR_SCLDEL_Pos)          /*!< 0x00F00000 */
#define I2C_TIMINGR_SCLDEL           I2C_TIMINGR_SCLDEL_Msk                    /*!< Data setup time */
#define I2C_TIMINGR_PRESC_Pos        (28U)
#define I2C_TIMINGR_PRESC_Msk        (0xFUL << I2C_TIMINGR_PRESC_Pos)           /*!< 0xF0000000 */
#define I2C_TIMINGR_PRESC            I2C_TIMINGR_PRESC_Msk                     /*!< Timings prescaler */

/******************* Bit definition for I2C_TIMEOUTR register ****************/
#define I2C_TIMEOUTR_TIMEOUTA_Pos    (0U)
#define I2C_TIMEOUTR_TIMEOUTA_Msk    (0xFFFUL << I2C_TIMEOUTR_TIMEOUTA_Pos)     /*!< 0x00000FFF */
#define I2C_TIMEOUTR_TIMEOUTA        I2C_TIMEOUTR_TIMEOUTA_Msk                 /*!< Bus timeout A */
#define I2C_TIMEOUTR_TIDLE_Pos       (12U)
#define I2C_TIMEOUTR_TIDLE_Msk       (0x1UL << I2C_TIMEOUTR_TIDLE_Pos)          /*!< 0x00001000 */
#define I2C_TIMEOUTR_TIDLE           I2C_TIMEOUTR_TIDLE_Msk                    /*!< Idle clock timeout detection */
#define I2C_TIMEOUTR_TIMOUTEN_Pos    (15U)
#define I2C_TIMEOUTR_TIMOUTEN_Msk    (0x1UL << I2C_TIMEOUTR_TIMOUTEN_Pos)       /*!< 0x00008000 */
#define I2C_TIMEOUTR_TIMOUTEN        I2C_TIMEOUTR_TIMOUTEN_Msk                 /*!< Clock timeout enable */
#define I2C_TIMEOUTR_TIMEOUTB_Pos    (16U)
#define I2C_TIMEOUTR_TIMEOUTB_Msk    (0xFFFUL << I2C_TIMEOUTR_TIMEOUTB_Pos)     /*!< 0x0FFF0000 */
#define I2C_TIMEOUTR_TIMEOUTB        I2C_TIMEOUTR_TIMEOUTB_Msk                 /*!< Bus timeout B*/
#define I2C_TIMEOUTR_TEXTEN_Pos      (31U)
#define I2C_TIMEOUTR_TEXTEN_Msk      (0x1UL << I2C_TIMEOUTR_TEXTEN_Pos)         /*!< 0x80000000 */
#define I2C_TIMEOUTR_TEXTEN          I2C_TIMEOUTR_TEXTEN_Msk                   /*!< Extended clock timeout enable */

/******************  Bit definition for I2C_ISR register  ********************/
#define I2C_ISR_TXE_Pos              (0U)
#define I2C_ISR_TXE_Msk              (0x1UL << I2C_ISR_TXE_Pos)                 /*!< 0x00000001 */
#define I2C_ISR_TXE                  I2C_ISR_TXE_Msk                           /*!< Transmit data register empty */
#define I2C_ISR_TXIS_Pos             (1U)
#define I2C_ISR_TXIS_Msk             (0x1UL << I2C_ISR_TXIS_Pos)                /*!< 0x00000002 */
#define I2C_ISR_TXIS                 I2C_ISR_TXIS_Msk                          /*!< Transmit interrupt status */
#define I2C_ISR_RXNE_Pos             (2U)
#define I2C_ISR_RXNE_Msk             (0x1UL << I2C_ISR_RXNE_Pos)                /*!< 0x00000004 */
#define I2C_ISR_RXNE                 I2C_ISR_RXNE_Msk                          /*!< Receive data register not empty */
#define I2C_ISR_ADDR_Pos             (3U)
#define I2C_ISR_ADDR_Msk             (0x1UL << I2C_ISR_ADDR_Pos)                /*!< 0x00000008 */
#define I2C_ISR_ADDR                 I2C_ISR_ADDR_Msk                          /*!< Address matched (slave mode)*/
#define I2C_ISR_NACKF_Pos            (4U)
#define I2C_ISR_NACKF_Msk            (0x1UL << I2C_ISR_NACKF_Pos)               /*!< 0x00000010 */
#define I2C_ISR_NACKF                I2C_ISR_NACKF_Msk                         /*!< NACK received flag */
#define I2C_ISR_STOPF_Pos            (5U)
#define I2C_ISR_STOPF_Msk            (0x1UL << I2C_ISR_STOPF_Pos)               /*!< 0x00000020 */
#define I2C_ISR_STOPF                I2C_ISR_STOPF_Msk                         /*!< STOP detection flag */
#define I2C_ISR_TC_Pos               (6U)
#define I2C_ISR_TC_Msk               (0x1UL << I2C_ISR_TC_Pos)                  /*!< 0x00000040 */
#define I2C_ISR_TC                   I2C_ISR_TC_Msk                            /*!< Transfer complete (master mode) */
#define I2C_ISR_TCR_Pos              (7U)
#define I2C_ISR_TCR_Msk              (0x1UL << I2C_ISR_TCR_Pos)                 /*!< 0x00000080 */
#define I2C_ISR_TCR                  I2C_ISR_TCR_Msk                           /*!< Transfer complete reload */
#define I2C_ISR_BERR_Pos             (8U)
#define I2C_ISR_BERR_Msk             (0x1UL << I2C_ISR_BERR_Pos)                /*!< 0x00000100 */
#define I2C_ISR_BERR                 I2C_ISR_BERR_Msk                          /*!< Bus error */
#define I2C_ISR_ARLO_Pos             (9U)
#define I2C_ISR_ARLO_Msk             (0x1UL << I2C_ISR_ARLO_Pos)                /*!< 0x00000200 */
#define I2C_ISR_ARLO                 I2C_ISR_ARLO_Msk                          /*!< Arbitration lost */
#define I2C_ISR_OVR_Pos              (10U)
#define I2C_ISR_OVR_Msk              (0x1UL << I2C_ISR_OVR_Pos)                 /*!< 0x00000400 */
#define I2C_ISR_OVR                  I2C_ISR_OVR_Msk                           /*!< Overrun/Underrun */
#define I2C_ISR_PECERR_Pos           (11U)
#define I2C_ISR_PECERR_Msk           (0x1UL << I2C_ISR_PECERR_Pos)              /*!< 0x00000800 */
#define I2C_ISR_PECERR               I2C_ISR_PECERR_Msk                        /*!< PEC error in reception */
#define I2C_ISR_TIMEOUT_Pos          (12U)
#define I2C_ISR_TIMEOUT_Msk          (0x1UL << I2C_ISR_TIMEOUT_Pos)             /*!< 0x00001000 */
#define I2C_ISR_TIMEOUT              I2C_ISR_TIMEOUT_Msk                       /*!< Timeout or Tlow detection flag */
#define I2C_ISR_ALERT_Pos            (13U)
#define I2C_ISR_ALERT_Msk            (0x1UL << I2C_ISR_ALERT_Pos)               /*!< 0x00002000 */
#define I2C_ISR_ALERT                I2C_ISR_ALERT_Msk                         /*!< SMBus alert */
#define I2C_ISR_BUSY_Pos             (15U)
#define I2C_ISR_BUSY_Msk             (0x1UL << I2C_ISR_BUSY_Pos)                /*!< 0x00008000 */
#define I2C_ISR_BUSY                 I2C_ISR_BUSY_Msk                          /*!< Bus busy */
#define I2C_ISR_DIR_Pos              (16U)
#define I2C_ISR_DIR_Msk              (0x1UL << I2C_ISR_DIR_Pos)                 /*!< 0x00010000 */
#define I2C_ISR_DIR                  I2C_ISR_DIR_Msk                           /*!< Transfer direction (slave mode) */
#define I2C_ISR_ADDCODE_Pos          (17U)
#define I2C_ISR_ADDCODE_Msk          (0x7FUL << I2C_ISR_ADDCODE_Pos)            /*!< 0x00FE0000 */
#define I2C_ISR_ADDCODE              I2C_ISR_ADDCODE_Msk                       /*!< Address match code (slave mode) */

/******************  Bit definition for I2C_ICR register  ********************/
#define I2C_ICR_ADDRCF_Pos           (3U)
#define I2C_ICR_ADDRCF_Msk           (0x1UL << I2C_ICR_ADDRCF_Pos)              /*!< 0x00000008 */
#define I2C_ICR_ADDRCF               I2C_ICR_ADDRCF_Msk                        /*!< Address matched clear flag */
#define I2C_ICR_NACKCF_Pos           (4U)
#define I2C_ICR_NACKCF_Msk           (0x1UL << I2C_ICR_NACKCF_Pos)              /*!< 0x00000010 */
#define I2C_ICR_NACKCF               I2C_ICR_NACKCF_Msk                        /*!< NACK clear flag */
#define I2C_ICR_STOPCF_Pos           (5U)
#define I2C_ICR_STOPCF_Msk           (0x1UL << I2C_ICR_STOPCF_Pos)              /*!< 0x00000020 */
#define I2C_ICR_STOPCF               I2C_ICR_STOPCF_Msk                        /*!< STOP detection clear flag */
#define I2C_ICR_BERRCF_Pos           (8U)
#define I2C_ICR_BERRCF_Msk           (0x1UL << I2C_ICR_BERRCF_Pos)              /*!< 0x00000100 */
#define I2C_ICR_BERRCF               I2C_ICR_BERRCF_Msk                        /*!< Bus error clear flag */
#define I2C_ICR_ARLOCF_Pos           (9U)
#define I2C_ICR_ARLOCF_Msk           (0x1UL << I2C_ICR_ARLOCF_Pos)              /*!< 0x00000200 */
#define I2C_ICR_ARLOCF               I2C_ICR_ARLOCF_Msk                        /*!< Arbitration lost clear flag */
#define I2C_ICR_OVRCF_Pos            (10U)
#define I2C_ICR_OVRCF_Msk            (0x1UL << I2C_ICR_OVRCF_Pos)               /*!< 0x00000400 */
#define I2C_ICR_OVRCF                I2C_ICR_OVRCF_Msk                         /*!< Overrun/Underrun clear flag */
#define I2C_ICR_PECCF_Pos            (11U)
#define I2C_ICR_PECCF_Msk            (0x1UL << I2C_ICR_PECCF_Pos)               /*!< 0x00000800 */
#define I2C_ICR_PECCF                I2C_ICR_PECCF_Msk                         /*!< PAC error clear flag */
#define I2C_ICR_TIMOUTCF_Pos         (12U)
#define I2C_ICR_TIMOUTCF_Msk         (0x1UL << I2C_ICR_TIMOUTCF_Pos)            /*!< 0x00001000 */
#define I2C_ICR_TIMOUTCF             I2C_ICR_TIMOUTCF_Msk                      /*!< Timeout clear flag */
#define I2C_ICR_ALERTCF_Pos          (13U)
#define I2C_ICR_ALERTCF_Msk          (0x1UL << I2C_ICR_ALERTCF_Pos)             /*!< 0x00002000 */
#define I2C_ICR_ALERTCF              I2C_ICR_ALERTCF_Msk                       /*!< Alert clear flag */

/******************  Bit definition for I2C_PECR register  *******************/
#define I2C_PECR_PEC_Pos             (0U)
#define I2C_PECR_PEC_Msk             (0xFFUL << I2C_PECR_PEC_Pos)               /*!< 0x000000FF */
#define I2C_PECR_PEC                 I2C_PECR_PEC_Msk                          /*!< PEC register */

/******************  Bit definition for I2C_RXDR register  *********************/
#define I2C_RXDR_RXDATA_Pos          (0U)
#define I2C_RXDR_RXDATA_Msk          (0xFFUL << I2C_RXDR_RXDATA_Pos)            /*!< 0x000000FF */
#define I2C_RXDR_RXDATA              I2C_RXDR_RXDATA_Msk                       /*!< 8-bit receive data */

/******************  Bit definition for I2C_TXDR register  *******************/
#define I2C_TXDR_TXDATA_Pos          (0U)
#define I2C_TXDR_TXDATA_Msk          (0xFFUL << I2C_TXDR_TXDATA_Pos)            /*!< 0x000000FF */
#define I2C_TXDR_TXDATA              I2C_TXDR_TXDATA_Msk                       /*!< 8-bit transmit data */

/*****************************************************************************/
/*                                                                           */
/*                        Independent WATCHDOG (IWDG)                        */
/*                                                                           */
/*****************************************************************************/
/*******************  Bit definition for IWDG_KR register  *******************/
#define IWDG_KR_KEY_Pos      (0U)
#define IWDG_KR_KEY_Msk      (0xFFFFUL << IWDG_KR_KEY_Pos)                      /*!< 0x0000FFFF */
#define IWDG_KR_KEY          IWDG_KR_KEY_Msk                                   /*!< Key value (write only, read 0000h) */

/*******************  Bit definition for IWDG_PR register  *******************/
#define IWDG_PR_PR_Pos       (0U)
#define IWDG_PR_PR_Msk       (0x7UL << IWDG_PR_PR_Pos)                          /*!< 0x00000007 */
#define IWDG_PR_PR           IWDG_PR_PR_Msk                                    /*!< PR[2:0] (Prescaler divider) */
#define IWDG_PR_PR_0         (0x1UL << IWDG_PR_PR_Pos)                          /*!< 0x01 */
#define IWDG_PR_PR_1         (0x2UL << IWDG_PR_PR_Pos)                          /*!< 0x02 */
#define IWDG_PR_PR_2         (0x4UL << IWDG_PR_PR_Pos)                          /*!< 0x04 */

/*******************  Bit definition for IWDG_RLR register  ******************/
#define IWDG_RLR_RL_Pos      (0U)
#define IWDG_RLR_RL_Msk      (0xFFFUL << IWDG_RLR_RL_Pos)                       /*!< 0x00000FFF */
#define IWDG_RLR_RL          IWDG_RLR_RL_Msk                                   /*!< Watchdog counter reload value */

/*******************  Bit definition for IWDG_SR register  *******************/
#define IWDG_SR_PVU_Pos      (0U)
#define IWDG_SR_PVU_Msk      (0x1UL << IWDG_SR_PVU_Pos)                         /*!< 0x00000001 */
#define IWDG_SR_PVU          IWDG_SR_PVU_Msk                                   /*!< Watchdog prescaler value update */
#define IWDG_SR_RVU_Pos      (1U)
#define IWDG_SR_RVU_Msk      (0x1UL << IWDG_SR_RVU_Pos)                         /*!< 0x00000002 */
#define IWDG_SR_RVU          IWDG_SR_RVU_Msk                                   /*!< Watchdog counter reload value update */
#define IWDG_SR_WVU_Pos      (2U)
#define IWDG_SR_WVU_Msk      (0x1UL << IWDG_SR_WVU_Pos)                         /*!< 0x00000004 */
#define IWDG_SR_WVU          IWDG_SR_WVU_Msk                                   /*!< Watchdog counter window value update */

/*******************  Bit definition for IWDG_KR register  *******************/
#define IWDG_WINR_WIN_Pos    (0U)
#define IWDG_WINR_WIN_Msk    (0xFFFUL << IWDG_WINR_WIN_Pos)                     /*!< 0x00000FFF */
#define IWDG_WINR_WIN        IWDG_WINR_WIN_Msk                                 /*!< Watchdog counter window value */

/*****************************************************************************/
/*                                                                           */
/*                          Power Control (PWR)                              */
/*                                                                           */
/*****************************************************************************/

#define PWR_PVD_SUPPORT                       /*!< PWR feature available only on specific devices: Power Voltage Detection feature */


/********************  Bit definition for PWR_CR register  *******************/
#define PWR_CR_LPDS_Pos            (0U)
#define PWR_CR_LPDS_Msk            (0x1UL << PWR_CR_LPDS_Pos)                   /*!< 0x00000001 */
#define PWR_CR_LPDS                PWR_CR_LPDS_Msk                             /*!< Low-power Deepsleep */
#define PWR_CR_PDDS_Pos            (1U)
#define PWR_CR_PDDS_Msk            (0x1UL << PWR_CR_PDDS_Pos)                   /*!< 0x00000002 */
#define PWR_CR_PDDS                PWR_CR_PDDS_Msk                             /*!< Power Down Deepsleep */
#define PWR_CR_CWUF_Pos            (2U)
#define PWR_CR_CWUF_Msk            (0x1UL << PWR_CR_CWUF_Pos)                   /*!< 0x00000004 */
#define PWR_CR_CWUF                PWR_CR_CWUF_Msk                             /*!< Clear Wakeup Flag */
#define PWR_CR_CSBF_Pos            (3U)
#define PWR_CR_CSBF_Msk            (0x1UL << PWR_CR_CSBF_Pos)                   /*!< 0x00000008 */
#define PWR_CR_CSBF                PWR_CR_CSBF_Msk                             /*!< Clear Standby Flag */
#define PWR_CR_PVDE_Pos            (4U)
#define PWR_CR_PVDE_Msk            (0x1UL << PWR_CR_PVDE_Pos)                   /*!< 0x00000010 */
#define PWR_CR_PVDE                PWR_CR_PVDE_Msk                             /*!< Power Voltage Detector Enable */

#define PWR_CR_PLS_Pos             (5U)
#define PWR_CR_PLS_Msk             (0x7UL << PWR_CR_PLS_Pos)                    /*!< 0x000000E0 */
#define PWR_CR_PLS                 PWR_CR_PLS_Msk                              /*!< PLS[2:0] bits (PVD Level Selection) */
#define PWR_CR_PLS_0               (0x1UL << PWR_CR_PLS_Pos)                    /*!< 0x00000020 */
#define PWR_CR_PLS_1               (0x2UL << PWR_CR_PLS_Pos)                    /*!< 0x00000040 */
#define PWR_CR_PLS_2               (0x4UL << PWR_CR_PLS_Pos)                    /*!< 0x00000080 */

/*!< PVD level configuration */
#define PWR_CR_PLS_LEV0            (0x00000000U)                               /*!< PVD level 0 */
#define PWR_CR_PLS_LEV1            (0x00000020U)                               /*!< PVD level 1 */
#define PWR_CR_PLS_LEV2            (0x00000040U)                               /*!< PVD level 2 */
#define PWR_CR_PLS_LEV3            (0x00000060U)                               /*!< PVD level 3 */
#define PWR_CR_PLS_LEV4            (0x00000080U)                               /*!< PVD level 4 */
#define PWR_CR_PLS_LEV5            (0x000000A0U)                               /*!< PVD level 5 */
#define PWR_CR_PLS_LEV6            (0x000000C0U)                               /*!< PVD level 6 */
#define PWR_CR_PLS_LEV7            (0x000000E0U)                               /*!< PVD level 7 */

#define PWR_CR_DBP_Pos             (8U)
#define PWR_CR_DBP_Msk             (0x1UL << PWR_CR_DBP_Pos)                    /*!< 0x00000100 */
#define PWR_CR_DBP                 PWR_CR_DBP_Msk                              /*!< Disable Backup Domain write protection */

/*******************  Bit definition for PWR_CSR register  *******************/
#define PWR_CSR_WUF_Pos            (0U)
#define PWR_CSR_WUF_Msk            (0x1UL << PWR_CSR_WUF_Pos)                   /*!< 0x00000001 */
#define PWR_CSR_WUF                PWR_CSR_WUF_Msk                             /*!< Wakeup Flag */
#define PWR_CSR_SBF_Pos            (1U)
#define PWR_CSR_SBF_Msk            (0x1UL << PWR_CSR_SBF_Pos)                   /*!< 0x00000002 */
#define PWR_CSR_SBF                PWR_CSR_SBF_Msk                             /*!< Standby Flag */
#define PWR_CSR_PVDO_Pos           (2U)
#define PWR_CSR_PVDO_Msk           (0x1UL << PWR_CSR_PVDO_Pos)                  /*!< 0x00000004 */
#define PWR_CSR_PVDO               PWR_CSR_PVDO_Msk                            /*!< PVD Output */
#define PWR_CSR_VREFINTRDYF_Pos    (3U)
#define PWR_CSR_VREFINTRDYF_Msk    (0x1UL << PWR_CSR_VREFINTRDYF_Pos)           /*!< 0x00000008 */
#define PWR_CSR_VREFINTRDYF        PWR_CSR_VREFINTRDYF_Msk                     /*!< Internal voltage reference (VREFINT) ready flag */

