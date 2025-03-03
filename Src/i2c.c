
#include <stm32f0xx_hal.h>

#include "stm32f0xx.h"
#include "stm32f0xx_hal.h"

#include "i2c.h"

// Initialize GPIOs for I2C2 and L3GD20 Mode Selection
void I2C2_GPIO_Init(void) {
    __HAL_RCC_GPIOB_CLK_ENABLE(); // Enable GPIOB Clock
    __HAL_RCC_GPIOC_CLK_ENABLE(); // Enable GPIOC Clock

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Configure PB11 (I2C2_SDA) as Open-Drain Alternate Function
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_I2C2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Configure PB13 (I2C2_SCL) as Open-Drain Alternate Function
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Alternate = GPIO_AF1_I2C2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Configure PB14 as Output Push-Pull (Slave Address Select)
    GPIO_InitStruct.Pin = GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); // Set HIGH

    // Configure PC0 as Output Push-Pull (Mode Select)
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET); // Set HIGH
}

#include "i2c.h"

// Initialize I2C2 Peripheral for 100 kHz Standard Mode
void I2C2_Init(void) {
    __HAL_RCC_I2C2_CLK_ENABLE();  // Enable I2C2 clock in RCC

    // Reset I2C2 Peripheral
    I2C2->CR1 &= ~I2C_CR1_PE;  // Disable I2C before configuring

    // Configure TIMINGR for 100 kHz Standard Mode
    I2C2->TIMINGR = (1 << 28) | // PRESC = 1
                    (0x13 << 0) |  // SCLL = 0x13
                    (0xF << 8) |   // SCLH = 0xF
                    (0x2 << 16) |  // SDADEL = 0x2
                    (0x4 << 20);   // SCLDEL = 0x4

    // Enable I2C2 Peripheral (Set PE bit in CR1)
    I2C2->CR1 |= I2C_CR1_PE;
}
