#include "stm32f0xx.h"
#include <stdint.h>
#include <stm32f0xx_hal.h>


// Define LED pins and ADC channel
#define LED1_PIN 6  
#define LED2_PIN 7
#define LED3_PIN 8
#define LED4_PIN 9
#define ADC_CHANNEL 10  // PC0 -> ADC_IN10



// Lab 6 Checkoff 1: ADC to PWM LED Brightness Control
void lab61_main(void) {
    Configure_GPIO_Lab6();
    Configure_ADC();
    Configure_TIM3_PWM();

    while (1) {
        uint16_t adc_value = Read_ADC();
        Update_LEDs(adc_value);
    }
}

// Configure DAC
void Configure_DAC(void) {
    RCC->APB1ENR |= RCC_APB1ENR_DACEN;  // Enable DAC clock
    DAC->CR |= DAC_CR_EN1;  // Enable DAC Channel 1
    for (volatile int i = 0; i < 1000; i++);  // Small delay to allow stabilization
}

#include "stm32f0xx.h"

// Triangle Wave: 8-bit, 32 samples per cycle
const uint8_t triangle_wave[32] = {
    0,15,31,47,63,79,95,111,127,143,159,175,191,207,223,239,
    255,239,223,207,191,175,159,143,127,111,95,79,63,47,31,15
};

void Configure_DAC_Output(void) {
    // Enable GPIOA and DAC clocks
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_DACEN;

    // Set PA4 (DAC_OUT1) to analog mode
    GPIOA->MODER |= (3 << (4 * 2));   // Analog mode
    GPIOA->PUPDR &= ~(3 << (4 * 2));  // No pull-up/pull-down

    // Enable DAC channel 1
    DAC->CR = DAC_CR_EN1;
}

void Delay_ms(uint32_t ms) {
    // Simple delay loop (calibrated for ~48MHz)
    for (uint32_t i = 0; i < ms * 6000; i++) {
        __NOP();
    }
}

void lab6_main(void) {
    Configure_DAC_Output();

    while (1) {
        for (int i = 0; i < 32; i++) {
            DAC->DHR8R1 = triangle_wave[i];
            Delay_ms(1);  // 1 ms delay → ~31.25 Hz waveform
        }
    }
}