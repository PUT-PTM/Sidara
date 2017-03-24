#include "stm32f4xx_i2c.h"

#ifndef I2C_H_
#define I2C_H_

void initI2C(void);
void I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction);
void I2C_stop(I2C_TypeDef* I2Cx);
void I2C_write(I2C_TypeDef* I2Cx, uint8_t data);
uint8_t I2C_read(I2C_TypeDef* I2Cx, uint8_t ACK);

#endif
