#include "usart.h"

// Initialize USART3 Peripheral
void USART3_Init(void) {
    __HAL_RCC_USART3_CLK_ENABLE(); // Enable USART3 clock

    // Configure USART3
    USART3->BRR = HAL_RCC_GetHCLKFreq() / 115200; // Set baud rate to 115200
    USART3->CR1 |= USART_CR1_TE | USART_CR1_RE; // Enable Transmit & Receive
    USART3->CR1 |= USART_CR1_UE; // Enable USART
}

// Wait for received data & return the byte
char USART3_Read(void) {
    while (!(USART3->ISR & USART_ISR_RXNE)); // Wait for RXNE (Read Register Not Empty)
    return USART3->RDR; // Read received byte
}

// Transmit a single character
void USART3_Write(char data) {
    while (!(USART3->ISR & USART_ISR_TXE)); // Wait until TXE (Transmit Register Empty)
    USART3->TDR = data; // Send character
}

// Transmit a string via USART
void USART3_Write_String(char *str) {
    while (*str) {
        USART3_Write(*str++);
    }
}
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