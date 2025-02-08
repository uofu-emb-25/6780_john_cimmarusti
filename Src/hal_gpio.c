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
void My_HAL_GPIO_Init_Button(GPIO_TypeDef* GPIOx, GPIO_InitTypeDef* GPIO_InitStruct) {
    // Configure mode
    GPIOx->MODER &= ~(0b11 << (GPIO_InitStruct->Pin * 2)); // Clear bits
    GPIOx->MODER |= (GPIO_InitStruct->Mode << (GPIO_InitStruct->Pin * 2)); // Set mode

    // Configure pull-up/pull-down resistors
    GPIOx->PUPDR &= ~(0b11 << (GPIO_InitStruct->Pin * 2)); // Clear bits
    GPIOx->PUPDR |= (GPIO_InitStruct->Pull << (GPIO_InitStruct->Pin * 2)); // Set pull mode

    // Configure output type
    if (GPIO_InitStruct->Mode == GPIO_MODE_OUTPUT_PP || GPIO_InitStruct->Mode == GPIO_MODE_AF_PP) {
        if (GPIO_InitStruct->Speed == GPIO_SPEED_FREQ_LOW) {
            GPIOx->OSPEEDR &= ~(0b11 << (GPIO_InitStruct->Pin * 2));
        } else {
            GPIOx->OSPEEDR |= (0b11 << (GPIO_InitStruct->Pin * 2));
        }
    }

    // Configure alternate function (if applicable)
    if (GPIO_InitStruct->Mode == GPIO_MODE_AF_PP || GPIO_InitStruct->Mode == GPIO_MODE_AF_OD) {
        uint32_t afrIndex = (GPIO_InitStruct->Pin > 7) ? 1 : 0;
        GPIOx->AFR[afrIndex] &= ~(0xF << ((GPIO_InitStruct->Pin % 8) * 4));
        GPIOx->AFR[afrIndex] |= (GPIO_InitStruct->Alternate << ((GPIO_InitStruct->Pin % 8) * 4));
    }
}
//void My_HAL_GPIO_Init_Button(void) {
    // Set PA0 as input mode
//    GPIOA->MODER &= ~GPIO_MODER_MODER0_Msk;

    // Set PA0 to low speed
//    GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEEDR0_Msk;

//    // Enable pull-down resistor
//    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR0_Msk; // Clear bits
//    GPIOA->PUPDR |= (0b10 << GPIO_PUPDR_PUPDR0_Pos); // Pull-down
//}
void My_HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t Pin, uint8_t State) {
    if (State) {
        GPIOx->BSRR = Pin;  // Set pin high
    } else {
        GPIOx->BSRR = (Pin << 16);  // Reset pin low
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


//void My_HAL_GPIO_TogglePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
//    GPIOC->ODR ^= (1 << 8) | (1 << 9); // Toggle PC8 and PC9
//}
void My_HAL_GPIO_TogglePin(GPIO_TypeDef *GPIOx, uint16_t Pin) {
    GPIOx->ODR ^= Pin;  // Toggle the pin
}
#define DEBOUNCE_THRESHOLD 0x7FFFFFFF // Triggers on transition to steady high

uint32_t debouncer = 0;

GPIO_PinState My_HAL_Debounce_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
    debouncer = (debouncer << 1); // Shift left every loop iteration


    if (My_HAL_GPIO_ReadPin(0) ) { 
        debouncer |= 0x01; // Set lowest bit when button is pressed
    }

    if (debouncer == DEBOUNCE_THRESHOLD) { 
        return GPIO_PIN_SET; // Button press detected once per transition
    }
    
    return GPIO_PIN_RESET; // No stable press detected
}