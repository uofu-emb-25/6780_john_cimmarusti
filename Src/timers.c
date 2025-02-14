
#include "timers.h"
#include <stm32f0xx_hal.h>
#include "hal_gpio.h"
#include "stm32f0xx.h"
#include "stm32f0xx_hal.h"



void init_TIM2(void) {
    // Enable TIM2 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // Set TIM2 prescaler and ARR for 4 Hz UEV
    TIM2->PSC = 7999;   // Prescaler: 8 MHz / 8000 → 1 kHz
    TIM2->ARR = 250;    // Auto-reload: 1 kHz / 250 = 4 Hz

    // Enable update interrupt
    TIM2->DIER |= TIM_DIER_UIE;

    // Enable TIM2 in NVIC
    
    NVIC_EnableIRQ(TIM2_IRQn);
    NVIC_SetPriority(TIM2_IRQn, 1);

    // Start TIM2
    TIM2->CR1 |= TIM_CR1_CEN;
}