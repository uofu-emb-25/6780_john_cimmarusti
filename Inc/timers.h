#ifndef USART_H
#define USART_H

#include "stm32f0xx.h"
#include "stm32f0xx_hal.h"

void USART3_GPIO_Init(void);
void USART3_Init(void);
char USART3_Read(void);
void USART3_Write(char data);
void USART3_Write_String(char *str);

#endif // USART_H
