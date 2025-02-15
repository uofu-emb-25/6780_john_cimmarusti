#ifndef PWM_H
#define PWM_H
#include "stm32f0xx.h"
#include "stm32f0xx_hal.h"

void TIM3_PWM_Init(uint32_t pwm_freq, uint8_t duty_cycle);

#endif // PWM_H