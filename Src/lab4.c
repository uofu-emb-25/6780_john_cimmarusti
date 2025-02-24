#include "stm32f0xx.h"
#include "stm32f0xx_hal.h"
#include "usart.h"

int lab4_main(void) {
    HAL_Init();

    // Initialize USART3 GPIOs (PC10 & PC11)
    USART3_GPIO_Init();

    while (1) {
        // Main loop
    }
}