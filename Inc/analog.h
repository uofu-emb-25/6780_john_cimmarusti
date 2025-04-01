#ifndef ANALOG_H
#define ANALOG_H
void Configure_GPIO_Lab6(void) ;
void Configure_ADC(void) ;
void Configure_TIM3_PWM(void) ;
uint16_t Read_ADC(void);
void Update_LEDs(uint16_t adc_value) ;



#include <stdint.h>
#endif