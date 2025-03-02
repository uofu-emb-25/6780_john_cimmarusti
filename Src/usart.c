#include "stm32f0xx.h"
#include "usart.h"
#include <stdarg.h>
#include <stdio.h>

#include "stm32f0xx.h" // Adjust for your STM32 series

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
