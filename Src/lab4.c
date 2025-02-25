#include "stm32f0xx.h"
#include "stm32f0xx_hal.h"
#include "usart.h"

int lab4_main(void) {
    HAL_Init();
    
    // Initialize USART3 GPIOs & Peripheral
    USART3_GPIO_Init();
    USART3_Init();

    // Enable GPIOC Clock (for LEDs)
    __HAL_RCC_GPIOC_CLK_ENABLE();

    // Configure PC6, PC7, PC8, PC9 as Output (LEDs)
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    USART3_Write_String("USART Ready. Send 'R', 'B', 'G', or 'O' to toggle LEDs.\r\n");

    while (1) {
        char received = USART3_Read(); // Wait for data

        switch (received) {
            case 'r': // Toggle Red LED (PC6)
                HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
                USART3_Write_String("Red LED Toggled.\r\n");
                break;
            case 'b': // Toggle Blue LED (PC7)
                HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
                USART3_Write_String("Blue LED Toggled.\r\n");
                break;
            case 'o': // Toggle Green LED (PC8)
                HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
                USART3_Write_String("Orange LED Toggled.\r\n");
                break;
            case 'g': // Toggle Orange LED (PC9)
                HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
                USART3_Write_String("Green LED Toggled.\r\n");
                break;
            default:
                USART3_Write_String("Error: Invalid Input. Use 'R', 'B', 'G', or 'O'.\r\n");
                break;
        }
    }
}
