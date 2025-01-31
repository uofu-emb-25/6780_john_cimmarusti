#include <stdint.h>
#include <stm32f0xx_hal.h>
#include <stm32f0xx_hal_gpio.h>

void My_HAL_GPIO_Init(GPIO_TypeDef  *GPIOx, GPIO_InitTypeDef *GPIO_Init){
    GPIOC->MODER &= ~(0b11 << 12); // Clear PC8 mode bits
    GPIOC->MODER &= ~(0b11 << 14); // Clear PC9 mode bits
    GPIOC->MODER |=  (0b01 << 12); // Set PC8 to output mode
   GPIOC->MODER |=  (0b01 << 14); // Set PC9 to output mode
    GPIOC->OTYPER &= ~(1 << 12); // PC8 push-pull
    GPIOC->OTYPER &= ~(1 << 14); // PC9 push-pull
    GPIOC->OSPEEDR &= ~(0b11 << 12); // Set PC8 to low speed
   GPIOC->OSPEEDR &= ~(0b11 << 14); // Set PC9 to low speed
    GPIOC->PUPDR &= ~(0b11 << 12); // No pull-up/down for PC8
    GPIOC->PUPDR &= ~(0b11 << 14); // No pull-up/down for PC
}
void My_HAL_GPIO_Init_Button(void) {
    // Set PA0 as input mode
    GPIOA->MODER &= ~GPIO_MODER_MODER0_Msk;

    // Set PA0 to low speed
    GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEEDR0_Msk;

    // Enable pull-down resistor
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR0_Msk; // Clear bits
    GPIOA->PUPDR |= (0b10 << GPIO_PUPDR_PUPDR0_Pos); // Pull-down
}
void My_HAL_GPIO_WritePin(uint16_t pin, uint8_t state) {
    if (state) {
        GPIOC->ODR |= (1 << pin);  // Set pin HIGH
    } else {
        GPIOC->ODR &= ~(1 << pin); // Set pin LOW
    }
}

//void HAL_GPIO_DeInit(GPIO_TypeDef  *GPIOx, uint32_t GPIO_Pin)
//{
//}


uint8_t My_HAL_GPIO_ReadPin(uint16_t pin) {
    return (GPIOA->IDR & (1 << pin)) ? 1 : 0;
}

//GPIO_PinState My_HAL_GPIO_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
//{
//    return (GPIOA->IDR & (1 << GPI0_Pi)) ? 1 : 0;
//}



//void My_HAL_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState){
//    GPIOC->ODR ^= (1 << 8) | (1 << 9); }// Toggle PC8 and PC9}



void My_HAL_GPIO_TogglePin(GPIO_TypeDef *GPIOx, uint16_t Pin) {
    GPIOx->ODR ^= Pin;  // Toggle the pin
}
