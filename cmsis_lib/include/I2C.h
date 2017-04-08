#ifndef I2C_H_
#define I2C_H_

#include "stm32f4xx_i2c.h"

void I2C_init(void);

void I2C_start(void);
void I2C_adress_write(uint8_t adres);
void I2C_adress_read(uint8_t adres);
void I2C_write(uint8_t data);
uint8_t I2C_read(void);
void I2C_stop(void);
void I2C_write_register(uint8_t register_to_write, uint8_t value);
uint8_t I2C_read_register(uint8_t register_to_read);

#endif
