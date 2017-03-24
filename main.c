#include "stm32f4xx_conf.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_dac.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_usart.h"
#include "tm_stm32f4_lis302dl_lis3dsh.h"
#include "misc.h"
#include "stm32f4xx_exti.h"
#include "Timer.h"
#include "GPIO.h"
#include "PWM.h"
#include "EXTI.h"
#include "ADC.h"
#include "DAC.h"
#include "USART.h"
#include "SPI.h"
#include <math.h>

int dacSignal = 0;
float ADC_Result1 = 0;
float ADC_Res = 0;


//void TIM4_IRQHandler(void)
//{
//	if(TIM_GetITStatus(TIM4,TIM_IT_Update) != RESET)
//	{
//		dacSignal++;
//		sendDAC(dacSignal);
//
//		if(dacSignal == 0xFFF)
//			dacSignal = 0;
//
//		TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
//	}
//}

TM_LIS302DL_LIS3DSH_t Axes_Data;
const float divider = 16384.0;
int x = 0;
int y = 0;
int z = 0;
float rx = 0;
float ry = 0;
float rz = 0;

void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2,TIM_IT_Update) != RESET)
	{
		TM_LIS302DL_LIS3DSH_ReadAxes(&Axes_Data);

		x = Axes_Data.X;
		y = Axes_Data.Y;
		z = Axes_Data.Z;

		rx = (float)x/divider;
		ry = (float)y/divider;
		rz = (float)z/divider;

		TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
	}
}



int main(void)
{
	SystemInit();

//	initTimer4(19999, 8399);
//	initTimer4Interruption();
//	initADC1();
//	initDAC();
	initTimer2(19999, 8399);
	initTimer2Interruption();
	initAccelerometr();



	while(1)
	{
//		ADC_SoftwareStartConv(ADC1);
//		while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
//		ADC_Result1 = ADC_GetConversionValue(ADC1);
//		ADC_Res = nDigital_to_Voltage(ADC_Result1);


	}
}
