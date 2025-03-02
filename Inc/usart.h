#ifndef USART_H
#define USART_H

void USART3_Init(void) ;
void GPIO_Init(void);
void USART3_SendChar(char c) ;
char USART3_ReceiveChar(void);
void USART3_SendString(char *str);
void Toggle_LED(char received);
void USART3_Reset(void) ;
#endif // USART_H
