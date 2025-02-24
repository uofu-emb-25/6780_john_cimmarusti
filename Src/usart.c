#include "usart.h"

// Initialize GPIOs for USART3 (PC10 TX, PC11 RX)
void USART3_GPIO_Init(void) {
    __HAL_RCC_GPIOC_CLK_ENABLE();  // Enable GPIOC clock

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11; // PC10 (TX), PC11 (RX)
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;  // Alternate Function Push-Pull
    GPIO_InitStruct.Pull = GPIO_NOPULL;  // No Pull-up/down
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;  // Low speed
    GPIO_InitStruct.Alternate = GPIO_AF1_USART3;  // Select AF1 (USART3)
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}
