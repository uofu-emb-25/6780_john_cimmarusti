#include <stm32f0xx_hal.h>
#include "hal_gpio.h"
#include "pwm.h"

int lab3_main(void) {
    HAL_Init(); // Reset of all peripherals, init the Flash and Systick
    SystemClock_Config(); //Configure the system clock
    /* This example uses HAL library calls to control
    the GPIOC peripheral. You’ll be redoing this code
    with hardware register access. */

     // Enable the GPIOC clock in the RCC
  // Enable the GPIOC clock in the RCC
    // Set up a configuration struct to pass to the initialization function
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    

    // Initialize LED pins
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    My_HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    TIM3_PWM_Init(800, 20);

    // Start Timer (outside initialization)
    TIM3->CR1 |= TIM_CR1_CEN;

    while (1) {
        
    }
}

