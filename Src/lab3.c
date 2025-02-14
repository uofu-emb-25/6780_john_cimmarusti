#include <stm32f0xx_hal.h>
#include "hal_gpio.h"
#include <assert.h>
#include "timers.h"



int lab3_main(void) {

    HAL_Init();
    __HAL_RCC_GPIOC_CLK_ENABLE(); // Enable GPIOC clock

    // Configure PC8 & PC9 as outputs
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // Set PC8 HIGH (LED on), PC9 LOW
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);

    init_TIM2();  // Initialize TIM2 with 4 Hz interrupt
    

}




