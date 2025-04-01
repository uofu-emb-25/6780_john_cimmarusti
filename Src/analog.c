

#include "stm32f0xx.h"
#include <stdint.h>
#include <stm32f0xx_hal.h>
#include "analog.h"

#define LED1_PIN 6  
#define LED2_PIN 7
#define LED3_PIN 8
#define LED4_PIN 9
#define ADC_CHANNEL 10  // PC0 -> ADC_IN10
// Configure GPIO for ADC and PWM Output


// Configure GPIO for ADC and PWM Output
void Configure_GPIO_Lab6(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIOAEN; // Enable GPIO clocks

    // Set PC6-PC9 as Alternate Function (AF0 for TIM3 PWM)
    GPIOC->MODER |= (2 << (LED1_PIN * 2)) | (2 << (LED2_PIN * 2)) |
                    (2 << (LED3_PIN * 2)) | (2 << (LED4_PIN * 2));

    // Set PA4 as Analog Mode (DAC_OUT1)
    GPIOA->MODER |= (3 << (4 * 2));  // PA4 = 11 (Analog mode)

    // Set PC0 as Analog Mode for ADC_IN10
    GPIOC->MODER |= (3 << (0 * 2));  // PC0 = 11 (Analog mode)
    GPIOC->PUPDR &= ~(3 << (0 * 2)); // No pull-up/pull-down
}

// Configure ADC in continuous conversion mode
void Configure_ADC(void) {
    RCC->APB2ENR |= RCC_APB2ENR_ADCEN;
    ADC1->CR |= ADC_CR_ADCAL;
    while (ADC1->CR & ADC_CR_ADCAL);

    ADC1->CFGR1 |= ADC_CFGR1_CONT | ADC_CFGR1_RES_1; // Continuous, 8-bit resolution
    ADC1->CHSELR = ADC_CHSELR_CHSEL10;               // Select channel 10
    ADC1->CR |= ADC_CR_ADEN;
    while (!(ADC1->ISR & ADC_ISR_ADRDY));
    ADC1->CR |= ADC_CR_ADSTART;
}

// Configure Timer 3 for PWM Output
void Configure_TIM3_PWM(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    TIM3->PSC = 79;
    TIM3->ARR = 255;

    TIM3->CCMR1 |= (6 << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
    TIM3->CCMR1 |= (6 << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
    TIM3->CCMR2 |= (6 << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE;
    TIM3->CCMR2 |= (6 << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE;

    TIM3->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
    TIM3->CR1 |= TIM_CR1_CEN;
}

// Read ADC value
uint16_t Read_ADC(void) {
    while (!(ADC1->ISR & ADC_ISR_EOC));
    return ADC1->DR;
}

// Update LEDs based on ADC value using PWM
void Update_LEDs(uint16_t adc_value) {
    uint8_t brightness = adc_value;  // Already 8-bit due to ADC resolution
    TIM3->CCR1 = brightness;
    TIM3->CCR2 = brightness;
    TIM3->CCR3 = brightness;
    TIM3->CCR4 = brightness;
}