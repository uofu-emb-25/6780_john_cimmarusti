#include <stdint.h>
#include <stm32f0xx_hal.h>
#include <stm32f0xx_hal_gpio.h>


#ifndef MOTOR_CONTROL_TEST_H
#define MOTOR_CONTROL_TEST_H



void TIM3_motor_PWM_Init(void);
void Motor_GPIO_Init(void);
#endif // PWM_H