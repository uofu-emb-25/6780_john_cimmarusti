#include "stm32f0xx.h"
#include "stm32f0xx_hal.h"
#include "i2c.h"

int lab5_main(void) {
    HAL_Init();

    // Initialize I2C2 GPIOs
    //I2C2_GPIO_Init();
    I2C2_Init;
    while (1) {
        // Main loop
    }
}

