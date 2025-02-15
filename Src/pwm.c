#include "pwm.h"

#include "stm32f0xx.h"
#include "pwm.h"

#include "pwm.h"

#include "pwm.h"

// TIM3 PWM Initialization (800 Hz on PA6 & PA7)
void TIM3_PWM_Init(void) {
    // Enable TIM3 clock
    // 1. Enable TIM3 Clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    // 2. Configure GPIOC Pins for TIM3 Alternate Function
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    GPIOC->MODER &= ~((3 << (6 * 2)) | (3 << (7 * 2)));  // Clear mode
    GPIOC->MODER |= (2 << (6 * 2)) | (2 << (7 * 2));     // Set AF mode
    GPIOC->AFR[0] &= ~((0xF << (6 * 4)) | (0xF << (7 * 4)));  // AF0 (TIM3_CH1 & TIM3_CH2)

    // 3. Configure TIM3 for 800 Hz PWM
    TIM3->PSC = 99;   // Prescaler: 8 MHz / 100 = 80 kHz
    TIM3->ARR = 99;   // Auto-reload: 80 kHz / 100 = 800 Hz

    // 4. Configure PWM Mode for Channels 1 & 2
    TIM3->CCMR1 &= ~(TIM_CCMR1_CC1S | TIM_CCMR1_CC2S);  // Output mode
    TIM3->CCMR1 |= (6 << TIM_CCMR1_OC1M_Pos);  // PWM Mode 2 (Red LED)
    TIM3->CCMR1 |= (6 << TIM_CCMR1_OC2M_Pos);  // PWM Mode 1 (Blue LED)
    TIM3->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;  // Enable preload

    // 5. Enable Output
    TIM3->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;

    // 6. Set Initial Duty Cycle (20% Brightness)
    TIM3->CCR1 = 100;  // 20% duty cycle for Red LED
    TIM3->CCR2 = 100;  // 20% duty cycle for Blue LED

    // 7. Start TIM3
    TIM3->CR1 |= TIM_CR1_CEN;
}

// helper function
void TIM3_GPIO_Init(void) {
    __HAL_RCC_GPIOC_CLK_ENABLE(); // Enable GPIOC clock

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7; // PC6 (Red) & PC7 (Blue)
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;  // Alternate function mode (push-pull)
    GPIO_InitStruct.Pull = GPIO_NOPULL;      // No pull-up/down resistors
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // Low speed
    GPIO_InitStruct.Alternate = GPIO_AF0_TIM3;  // Set AF0 for TIM3_CH1 & TIM3_CH2
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

