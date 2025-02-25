/**
  ******************************************************************************
  * @file    Templates/Src/stm32f0xx_it.c
  * @author  MCD Application Team
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
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
#include "main.h"
#include <stm32f0xx_hal.h>
#include <stm32f0xx_it.h>
#include "hal_gpio.h"
/******************************************************************************/
/*            Cortex-M0 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
volatile uint32_t tick_counter = 0; // Counter variable for timing
void SysTick_Handler(void) {
    HAL_IncTick();
    HAL_SYSTICK_IRQHandler(); // Keep HAL's functionality

    tick_counter++; // Increment every 1ms

    if (tick_counter >= 200) { // Every 200ms
        //HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7); // Toggle Blue LED (PC7)
        tick_counter = 0; // Reset counter
    }
}


/******************************************************************************/
/*                 STM32F0xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f0xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

void EXTI0_1_IRQHandler(void) {
    if (EXTI->PR & EXTI_PR_PR0) { // Check if EXTI0 caused the interrupt
        EXTI->PR |= EXTI_PR_PR0;  // Clear the EXTI pending flag for line 0

        // First toggle: Swap Green (PC8) and Orange (PC9) LEDs
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);

        // Long delay (~1-2 seconds) - BAD PRACTICE IN INTERRUPTS!
        for (volatile int i = 0; i < 1500000; i++);

        // Second toggle: Swap Green (PC8) and Orange (PC9) LEDs again
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
    }
}
#include "timers.h"

// TIM2 Initialization - Sets up a 4 Hz interrupt

void TIM2_IRQHandler(void) {
    if (TIM2->SR & TIM_SR_UIF) { // Check if update interrupt flag is set
        TIM2->SR &= ~TIM_SR_UIF; // Clear the update interrupt flag

        // Toggle PC8 & PC9 LEDs
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
    }
}