#include "stm32f0xx.h"
#include "motor_control_test.h"
#include <stm32f0xx_hal.h>
#include "main.h"
#include <assert.h> 


#include "hal_gpio.h"
void Motor_GPIO_Init(void) {
    __HAL_RCC_GPIOB_CLK_ENABLE();  // ✅ Enable GPIOB Clock

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // 📌 Configure PB4 as Output
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;  // ✅ Set as Push-Pull Output
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // ✅ Set PB4 to HIGH initially
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
}
void TIM3_motor_PWM_Init(void) {
    __HAL_RCC_TIM3_CLK_ENABLE();  // Enable Timer 3 Clock

    // 📌 Set Timer Prescaler and Auto-reload (20ms period for 50Hz PWM)
    TIM3->PSC = 799;     // Prescaler: Scale down 8MHz clock
    TIM3->ARR = 19999;   // Auto-reload: 20ms period

    // 📌 Set PWM Mode 1 on TIM3 Channel 1 (PB4)
    TIM3->CCMR1 &= ~TIM_CCMR1_OC1M;          // Clear mode bits
    TIM3->CCMR1 |= (6 << TIM_CCMR1_OC1M_Pos); // Set PWM Mode 1
    TIM3->CCMR1 |= TIM_CCMR1_OC1PE;           // Enable preload

    // 📌 Enable Output Compare Channel 1
    TIM3->CCER |= TIM_CCER_CC1E;

    // 📌 Start Timer
    TIM3->CR1 |= TIM_CR1_CEN;
 
}
