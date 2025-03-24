#include "stm32f0xx.h"
#include "motor_control_test.h"

#include <stm32f0xx_hal.h>
#include "main.h"
#include <assert.h> 


#include "hal_gpio.h"


int lab7_main(void) {
    HAL_Init();
    Motor_GPIO_Init();

    while (1) {
        // ✅ Keep PB4 HIGH
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
        
        HAL_Delay(1000);  // Wait 1 second
    }
}
