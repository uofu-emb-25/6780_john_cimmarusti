#include "pwm.h"

#include "stm32f0xx.h"
#include "pwm.h"

#include "pwm.h"

void TIM3_PWM_Init(uint32_t pwm_freq, uint8_t duty_cycle) {
    // 1. Enable TIM3 clock in RCC
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    // 2. Compute Prescaler and ARR
    uint32_t timer_clock = 16000000;  // Assume APB1 Timer Clock = 16 MHz
    uint32_t prescaler = 100;  // Adjusted for better duty cycle granularity
    uint32_t arr_value = (timer_clock / (prescaler * pwm_freq)) - 1;

    // Configure Prescaler and Auto-Reload Register
    TIM3->PSC = prescaler - 1;  // Set prescaler
    TIM3->ARR = arr_value;      // Set Auto-Reload Register (PWM period)

    // 3. Configure CCMR1 for PWM mode
    TIM3->CCMR1 &= ~TIM_CCMR1_CC1S;  // Set CH1 as output
    TIM3->CCMR1 &= ~TIM_CCMR1_CC2S;  // Set CH2 as output

    // Set CH1 to PWM Mode 2 (OC1M = 0110)
    TIM3->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;
    // Set CH2 to PWM Mode 1 (OC2M = 0111)
    TIM3->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_0;

    // Enable Output Compare Preload for both channels
    TIM3->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;

    // 4. Enable Output in CCER register
    TIM3->CCER |= TIM_CCER_CC1E;  // Enable CH1 output
    TIM3->CCER |= TIM_CCER_CC2E;  // Enable CH2 output

    // 5. Set Capture/Compare Registers (CCR1 & CCR2) for 20% duty cycle
    TIM3->CCR1 = (arr_value * duty_cycle) / 100;
    TIM3->CCR2 = (arr_value * duty_cycle) / 100;

    // Do NOT enable TIM3 here (per instructions)
}


