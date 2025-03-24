#include <stdint.h>
#include <stdlib.h>
#include "main.h"
#include "hal_gpio.h"
#include <stm32f0xx_hal.h>
#include "i2c.h"
//static inline void setLED(uint32_t ledPos) {
//    GPIOC->ODR = (GPIOC->ODR & ~((1 << 6) | (1 << 7) | (1 << 8) | (1 << 9))) | (1 << ledPos);
//}
#define FILTER_DEPTH 4
#define HIGH_THRESHOLD 550
#define LOW_THRESHOLD 450

int16_t xBuffer[FILTER_DEPTH] = {0}, yBuffer[FILTER_DEPTH] = {0};
int filterIndex = 0;
int lastLed = -1;
void addToFilter(int16_t x, int16_t y) {
    xBuffer[filterIndex] = x;
    yBuffer[filterIndex] = y;
    filterIndex = (filterIndex + 1) % FILTER_DEPTH;
}

int16_t getAverage(int16_t *buffer) {
    int32_t sum = 0;
    for (int i = 0; i < FILTER_DEPTH; i++) {
        sum += buffer[i];
    }
    return (int16_t)(sum / FILTER_DEPTH);
}

//static inline void setLED(uint32_t ledPos) {
//    GPIOC->ODR = (GPIOC->ODR & ~((1 << 6) | (1 << 7) | (1 << 8) | (1 << 9))) | (1 << ledPos);
//}

void stableSetLED(int16_t xVal, int16_t yVal) {
    int newLed = -1;

    if (yVal >= HIGH_THRESHOLD) {
        if (xVal >= HIGH_THRESHOLD)
            newLed = (xVal > yVal) ? 9 : 6;
        else if (xVal <= -HIGH_THRESHOLD)
            newLed = (abs(xVal) > yVal) ? 8 : 6;
        else
            newLed = 6;
    } else if (yVal <= -HIGH_THRESHOLD) {
        if (xVal <= -HIGH_THRESHOLD)
            newLed = (abs(xVal) > abs(yVal)) ? 8 : 7;
        else if (xVal >= HIGH_THRESHOLD)
            newLed = (xVal > abs(yVal)) ? 9 : 7;
        else
            newLed = 7;
    } else {
        if (abs(xVal) < LOW_THRESHOLD && abs(yVal) < LOW_THRESHOLD) {
            newLed = -1;
        } else {
            newLed = lastLed;
        }
    }

    if (newLed != lastLed) {
        if (newLed == -1)
            GPIOC->ODR &= ~((1 << 6) | (1 << 7) | (1 << 8) | (1 << 9));
        else
            setLED(newLed);
        lastLed = newLed;
    }
}

int lab5_main(void)
{
    HAL_Init();
    SystemClock_Config();

    I2C_Init();

    // Gyroscope initialization
    int ctrlValue = 11;
    gyroscope(0x69, 1, &ctrlValue, 0, 0x20);
    
    int sensorData[4];
    
    while (1)
    {
        HAL_Delay(100);
        gyroscope(0x69, 4, sensorData, 1, 0x28);
     
        //int16_t xVal = (sensorData[1] << 8) | sensorData[0];
        //int16_t yVal = (sensorData[3] << 8) | sensorData[2];
        int16_t xRaw = (sensorData[1] << 8) | sensorData[0];
        int16_t yRaw = (sensorData[3] << 8) | sensorData[2];
        addToFilter(xRaw, yRaw);

        int16_t xVal = getAverage(xBuffer);
        int16_t yVal = getAverage(yBuffer);

        //stableSetLED(xVal, yVal);
        if (yVal >= 500) {
            if (xVal >= 500)
                setLED((xVal > yVal) ? 9 : 6);
            else if (xVal <= -500)
                setLED((abs(xVal) > yVal) ? 8 : 6);
        } else if (yVal <= -500) {
            if (xVal <= -500)
                setLED((abs(xVal) > abs(yVal)) ? 8 : 7);
            else if (xVal >= 500)
                setLED((xVal > abs(yVal)) ? 9 : 7);
        }
    }
}