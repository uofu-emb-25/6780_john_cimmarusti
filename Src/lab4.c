#include "stm32f0xx.h"
#include "usart.h"

int lab41_main(void) {

    USART3_Reset();
    USART3_SendString("USART3 Reset Done\r\n");


    USART3_Init();  // Initialize USART3
    GPIO_Init();    // Initialize GPIO for LEDs

    USART3_SendString("USART3 Ready. Press R, G, O, or B to toggle LEDs.\r\n");

    while (1) {
        char received = USART3_ReceiveChar();  // Get user input
        Toggle_LED(received);  // Process input
    }
}
int lab4_main(void) {
    USART3_Init();  // Initialize USART3
    GPIO_Init();    // Initialize GPIO for LEDs

    USART3_SendString("USART3 Ready. Enter LED Command (e.g., R1 to turn Red ON)\r\n");

    Command_Parser();  // Start command loop
}
