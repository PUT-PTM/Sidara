#include "stm32f4xx_conf.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_syscfg.h"
#include "misc.h"
#include "stm32f4xx_spi.h"
#include "diskio.h"


#ifndef spi
#define spi

void Konfiguracja_SPI();
void Konfiguracja_portow_SPI();
void Konfiguracja_struktury_SPI();

#endif
