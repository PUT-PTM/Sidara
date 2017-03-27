#include "stm32f4xx_i2c.h"

#ifndef I2C_H_
#define I2C_H_

void I2C_init(void);

void I2C_start(void);
void I2C_adres_write(uint8_t adres);
void I2C_adres_read(uint8_t adres);
void I2C_write(uint8_t data);
uint8_t I2C_read(void);
void I2C_stop(void);
void I2C_wyslij(uint8_t rejestr, uint8_t wartosc);
uint8_t I2C_czytaj(uint8_t rejestr);

#endif
