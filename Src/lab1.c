#include <stm32f0xx_hal.h>
#include "hal_gpio.h"
int lab1_main(void) {
    HAL_Init(); // Reset of all peripherals, init the Flash and Systick
    SystemClock_Config(); //Configure the system clock
    /* This example uses HAL library calls to control
    the GPIOC peripheral. You’ll be redoing this code
    with hardware register access. */

    My_HAL_RCC_GPIOC_CLK_ENABLE(); // Enable the GPIOC clock in the RCC
    My_HAL_RCC_GPIOA_CLK_ENABLE(); // Enable the GPIOC clock in the RCC
    // Set up a configuration struct to pass to the initialization function
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Initialize LED pins
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    My_HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // Initialize button pin
        GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    My_HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    //GPIO_InitTypeDef initStr = {GPIO_PIN_6 | GPIO_PIN_7,
    //GPIO_MODE_OUTPUT_PP,
    //GPIO_SPEED_FREQ_LOW,
    //GPIO_NOPULL};
    
    //My_HAL_GPIO_Init(GPIOC, &initStr); // Initialize pins PC8 & PC9
    //My_HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET); // Start PC8 high
    //while (1) {
    //HAL_Delay(400); // Delay 200ms
    // Toggle the output state of both PC8 and PC9
    //My_HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8 | GPIO_PIN_9);
    //}
    
    My_HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 1); // Red LED ON
    My_HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 0); // Blue LED OFF

//
    while (1) {

   // if (My_HAL_GPIO_ReadPin(0)) { // If button is pressed
    // My_HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);   Toggle Red LED
    // My_HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);  // Toggle Blue LED
    //HAL_Delay(50);  // 500ms delay to make the blinking visible
    //while (My_HAL_GPIO_ReadPin(0));

        // Another debounce delay
    //    HAL_Delay(50);

    if (My_HAL_Debounce_ReadPin(GPIOC,  GPIO_PIN_0)== GPIO_PIN_SET) {
        My_HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6); // Toggle Red LED
        My_HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7); // Toggle Blue LED
    }

    HAL_Delay(6); // Small delay to prevent excessive CPU usage
     
    }
}
