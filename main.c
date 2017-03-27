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

	I2C_init();
	I2C_wyslij(0x80, 2);

	while (1)
	{
		var1 = I2C_czytaj(0x9D);

	}
}
