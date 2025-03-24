#ifndef I2C_H
#define I2C_H

#include <stdint.h>

void I2C_Init(void);
void setLED(uint32_t ledPos);

int checkI2CStatusFlag(int flagBit);
void gyroscope(int addr, int count, int *buffer, int isRead, int regAddr);
#endif







