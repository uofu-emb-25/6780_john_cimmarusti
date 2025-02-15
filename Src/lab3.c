#include <stm32f0xx_hal.h>
#include "hal_gpio.h"
#include "pwm.h"

int lab3_main(void) {
    HAL_Init();

   

    // Initialize TIM3 PWM
    TIM3_PWM_Init();

    // Start Timer (outside initialization)
    

    while (1) {
        
    }
}

