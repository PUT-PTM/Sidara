#include "stm32f4xx_conf.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_i2c.h"
#include "I2C.h"

uint8_t var1 = 0;
uint8_t var2 = 0;


int main(void)
{
	SystemInit();
	initI2C();

	I2C_start(I2C1, 0x38, I2C_Direction_Transmitter);
	I2C_write(I2C1, 0x00);
	I2C_stop(I2C1);

	while (1)
	{
	    I2C_start(I2C1, 0x38, I2C_Direction_Receiver);
	    var1 = I2C_read(I2C1, ENABLE);
	    var2 = I2C_read(I2C1, DISABLE);
	    I2C_stop(I2C1);
	}
}
