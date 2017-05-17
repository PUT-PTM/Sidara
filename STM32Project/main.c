/* Basic libraries for STM */
#include "stm32f4xx_conf.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_syscfg.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_usart.h"
#include "misc.h"
#include "EXTI.h"
#include "GPIO.h"
#include "Timer.h"
#include "USART.h"


uint8_t PA1 = 0;

int gesture = 0;

/* for debugging */
int control_counter = 0;

char sign;


void USART3_IRQHandler(void)
{
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
		sign = readUSARTFromInterruption();
				if(sign == 'R')
					GPIO_ToggleBits(GPIOD, GPIO_Pin_14);
				else if(sign == 'L')
					GPIO_ToggleBits(GPIOD, GPIO_Pin_12);
				else if(sign == 'U')
					GPIO_ToggleBits(GPIOD, GPIO_Pin_13);
				else if(sign == 'D')
					GPIO_ToggleBits(GPIOD, GPIO_Pin_15);
	}
}



int i = 0;

int main(void)
{
	SystemInit();

	initGPIODiodes();
	initUSART();
	initUSARTInterruption();

	initTimer5For30msDelay();

	while (1)
	{

	}
}
