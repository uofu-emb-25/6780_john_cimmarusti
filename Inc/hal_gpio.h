#include <stdint.h>
#include <stm32f0xx_hal.h>
#include <stm32f0xx_hal_gpio.h>

void My_HAL_GPIO_Init(GPIO_TypeDef  *GPIOx, GPIO_InitTypeDef *GPIO_Init);
void My_HAL_GPIO_DeInit(GPIO_TypeDef  *GPIOx, uint32_t GPIO_Pin);
//void My_HAL_GPIO_Init_Button(void); 
void My_HAL_GPIO_Init_Button(GPIO_TypeDef* GPIOx, GPIO_InitTypeDef* GPIO_InitStruct);
uint8_t My_HAL_GPIO_ReadPin(uint16_t pin);
GPIO_PinState My_HAL_Debounce_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
//uint8_t My_HAL_Debounce_ReadPin(uint16_t pin);
//GPIO_PinState My_HAL_GPIO_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void My_HAL_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
void My_HAL_GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);