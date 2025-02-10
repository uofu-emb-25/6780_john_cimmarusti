#include <stm32f0xx_hal.h>
#include "hal_gpio.h"
#include <assert.h>

void EXTI0_1_IRQHandler(void);
#define SysTick_IRQn -1  // SysTick interrupt number

int lab2_main(void) {
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
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    My_HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    __HAL_RCC_SYSCFG_CLK_ENABLE(); // Enable SYSCFG clock

    // Assertions before configuration
    assert((SYSCFG->EXTICR[0] & SYSCFG_EXTICR1_EXTI0_Msk) == 0); // Ensure EXTI0 is mapped to PA0

    // Configure PA0 as input with pull-down resistor
    
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Configure SYSCFG multiplexer to route PA0 to EXTI0
    SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI0_Msk; // Clear EXTI0 field (set to 0000 for PA0)

    // Assertions after configuration
    assert((SYSCFG->EXTICR[0] & SYSCFG_EXTICR1_EXTI0_Msk) == 0); // Ensure PA0 is mapped to EXTI0

    // Enable EXTI0 interrupt
    EXTI->IMR |= EXTI_IMR_IM0;  // Unmask EXTI0 (Enable interrupt)
    EXTI->RTSR |= EXTI_RTSR_TR0; // Rising-edge trigger for EXTI0

    // Enable NVIC for EXTI0
    HAL_NVIC_SetPriority(EXTI0_1_IRQn, 3, 0);
    //NVIC_SetPriority(EXTI0_1_IRQn, 1);
    HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
       // Change SysTick priority to 2 (medium priority)
    NVIC_SetPriority(SysTick_IRQn, 1);

    // Ensure EXTI0 interrupt has higher priority (set to 1)
    NVIC_SetPriority(EXTI0_1_IRQn, 3);
   
    //GPIO_InitTypeDef initStr = {GPIO_PIN_6 | GPIO_PIN_7,
    //GPIO_MODE_OUTPUT_PP,
    //GPIO_SPEED_FREQ_LOW,
    //GPIO_NOPULL};
      // Route PA0 to EXTI0 using SYSCFG
    SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI0_Msk; // Ensure EXTI0 is mapped to PA0

    // Enable EXTI0 interrupt generation
    EXTI->IMR |= EXTI_IMR_IM0;  // Unmask EXTI0
    EXTI->RTSR |= EXTI_RTSR_TR0; // Enable rising-edge trigger

    // Enable the EXTI0 interrupt in the NVIC
    //NVIC_EnableIRQ(EXTI0_1_IRQn); 

    // Set the priority for EXTI0 to 1 (high priority)
    //NVIC_SetPriority(EXTI0_1_IRQn, 3);
    //My_HAL_GPIO_Init(GPIOC, &initStr); // Initialize pins PC8 & PC9
    //My_HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET); // Start PC8 high
    //while (1) {
    //HAL_Delay(400); // Delay 200ms
    // Toggle the output state of both PC8 and PC9
    //My_HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8 | GPIO_PIN_9);
    //}
    
    My_HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 1); // Red LED ON
    My_HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 0); // Blue LED OFF


    while (1) {

   // if (My_HAL_GPIO_ReadPin(0)) { // If button is pressed
    // My_HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);   Toggle Red LED
    // My_HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);  // Toggle Blue LED
    //HAL_Delay(50);  // 500ms delay to make the blinking visible
    //while (My_HAL_GPIO_ReadPin(0));

        // Another debounce delay
    //    HAL_Delay(50);
     //if //(My_HAL_GPIO_ReadPin(GPIOC,  GPIO_PIN_0)== GPIO_PIN_SET)
    //    (My_HAL_GPIO_ReadPin(0)) {
        My_HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6); // Toggle Red LED
        //My_HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9); // Toggle Blue LED
    //}

    HAL_Delay(500); // Small delay to prevent excessive CPU usage

    
     
    }
}

//typedef enum {
 //   ...
//    EXTI0_1_IRQn           = 5,  // External Interrupts 0 and 1
 //   EXTI2_3_IRQn           = 6,  // External Interrupts 2 and 3
 //   EXTI4_15_IRQn          = 7,  // External Interrupts 4 to 15
//    ...
//} IRQn_Type;
