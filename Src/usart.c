#include "stm32f0xx.h"
#include "usart.h"
#include <stdarg.h>
#include <stdio.h>

#include "stm32f0xx.h" // Adjust for your STM32 series



// Initialize GPIOs for USART3 (PC10 TX, PC11 RX)
void USART3_GPIO_Init(void) {
    // Enable GPIOC Clock
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Configure PC10 as USART3_TX (Alternate Function)
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;  // Alternate function, push-pull
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_USART3; // AF1 for USART3 TX
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // Configure PC11 as USART3_RX (Alternate Function)
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;  // Alternate function, push-pull
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_USART3; // AF1 for USART3 RX
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

void USART3_Init(void) {
    // Enable Clocks
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;  // Enable GPIOB clock
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;  // Enable USART3 clock

    // Configure PB10 (TX) and PB11 (RX) as Alternate Function
    GPIOB->MODER |= (GPIO_MODER_MODER10_1 | GPIO_MODER_MODER11_1);  // Set to Alternate Function mode
    GPIOB->AFR[1] |= (4 << GPIO_AFRH_AFSEL10_Pos) | (4 << GPIO_AFRH_AFSEL11_Pos);  // AF4 for USART3

    // Configure USART3: 115200 baud rate
    USART3->BRR = 8000000 / 115200;  // Baud rate = PCLK / Baudrate
    USART3->CR1 = USART_CR1_TE | USART_CR1_RE;  // Enable TX and RX
    USART3->CR1 |= USART_CR1_UE;  // Enable USART
}

void GPIO_Init(void) {
    // Enable GPIOC Clock
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

    // Configure PC6 (Orange), PC7 (Green), PC8 (Red), and PC9 (Blue) as OUTPUT mode
    GPIOC->MODER &= ~(GPIO_MODER_MODER6 | GPIO_MODER_MODER7 | GPIO_MODER_MODER8 | GPIO_MODER_MODER9);
    GPIOC->MODER |= (GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0 | GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0);  // Set as output

    // Ensure all LEDs start OFF
    GPIOC->ODR &= ~((1 << 6) | (1 << 7) | (1 << 8) | (1 << 9));
}

void USART3_SendChar(char c) {
    while (!(USART3->ISR & USART_ISR_TXE));  // Wait until TX is empty
    USART3->TDR = c;
}
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
char USART3_ReceiveChar(void) {
    while (!(USART3->ISR & USART_ISR_RXNE));  // Wait for RX buffer to be filled
    return USART3->RDR;
}

void USART3_SendString(char *str) {
    while (*str) {
        USART3_SendChar(*str++);
    }
}
void Toggle_LED(char received) {
    switch (received) {
        case 'o':
            GPIOC->ODR ^= (1 << 8);  // Toggle Red LED (PC8)
            USART3_SendString("Orange LED toggled\r\n");
            break;
        case 'b':
            GPIOC->ODR ^= (1 << 7);  // Toggle Green LED (PC7)
            USART3_SendString("Blue LED toggled\r\n");
            break;
        case 'g':
            GPIOC->ODR ^= (1 << 9);  // Toggle Blue LED (PC9)
            USART3_SendString("Green LED toggled\r\n");
            break;
        case 'r':
            GPIOC->ODR ^= (1 << 6);  // Toggle Orange LED (PC6)
            USART3_SendString("Red LED toggled\r\n");
            break;
        default:
            USART3_SendString("Error: Invalid Key!\r\n");
            break;
    }
}
void USART3_Reset(void) {
    USART3->CR1 &= ~USART_CR1_UE;  // Disable USART3
    USART3->CR1 |= USART_CR1_UE;   // Re-enable USART3
}
void Control_LED(char led, char action) {
    uint16_t pin = 0;

    // Map LED character to GPIO pin
    switch (led) {
        case 'r': pin = (1 << 6); break;  // Red LED (PC8)
        case 'g': pin = (1 << 9); break;  // Green LED (PC7)
        case 'b': pin = (1 << 7); break;  // Blue LED (PC9)
        case 'o': pin = (1 << 8); break;  // Orange LED (PC6)
        default:
            USART3_SendString("Error: Invalid LED!\r\n");
            return;
    }
    //Toggle_LED(led);
    // Perform action based on the second character
    switch (action) {
        case '0':  // Turn OFF
            GPIOC->ODR &= ~pin;
            USART3_SendString("LED turned OFF\r\n");
            break;
        case '1':  // Turn ON
            GPIOC->ODR |= pin;
            USART3_SendString("LED turned ON\r\n");
            break;
        case '2':  // Toggle
            GPIOC->ODR ^= pin;
            USART3_SendString("LED toggled\r\n");
            break;
        default:
            USART3_SendString("Error: Invalid Command!\r\n");
            return;
    }
}
void Command_Parser(void) {
    while (1) {
        USART3_SendString("CMD? ");  // Prompt user for input

        char led = USART3_ReceiveChar();  // First character (LED)
        char action = USART3_ReceiveChar();  // Second character (Action)

        // Print received command
        char buffer[20];
        sprintf(buffer, "\r\nReceived: %c%c\r\n", led, action);
        USART3_SendString(buffer);

        // Process the command
        Control_LED(led, action);
    }
}
void USART2_GPIO_Init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();  // Enable GPIOA
    __HAL_RCC_USART2_CLK_ENABLE(); // Enable USART2

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // 📌 Configure PA2 (USART2 TX) as Alternate Function
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
UART_HandleTypeDef huart2;

void USART2_Init(void) {
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX;  // Only TX needed for printing
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart2) != HAL_OK) {
        // Handle Error
        while (1);
    }
}


int _write1(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}


