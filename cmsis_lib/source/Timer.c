#include "Timer.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_conf.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_syscfg.h"
#include "stm32f4xx_exti.h"
#include "misc.h"

void initTimer2(int period, int prescaler)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);

	TIM_TimeBaseInitTypeDef str;
	str.TIM_Period=period;
	str.TIM_Prescaler=prescaler;
	str.TIM_ClockDivision=TIM_CKD_DIV1;
	str.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2,&str);
	TIM_Cmd(TIM2, ENABLE);
}

void initTimer3(int period, int prescaler)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);

	TIM_TimeBaseInitTypeDef str;
	str.TIM_Period=period;
	str.TIM_Prescaler=prescaler;
	str.TIM_ClockDivision=TIM_CKD_DIV1;
	str.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3,&str);
	TIM_Cmd(TIM3, ENABLE);
}

void initTimer4(int period, int prescaler)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);

	TIM_TimeBaseInitTypeDef str;
	str.TIM_Period=period;
	str.TIM_Prescaler=prescaler;
	str.TIM_ClockDivision=TIM_CKD_DIV1;
	str.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4,&str);
	TIM_Cmd(TIM4, ENABLE);
}

void initTimer5(int period, int prescaler)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);

	TIM_TimeBaseInitTypeDef str;
	str.TIM_Period=period;
	str.TIM_Prescaler=prescaler;
	str.TIM_ClockDivision=TIM_CKD_DIV1;
	str.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM5,&str);
	TIM_Cmd(TIM5, ENABLE);
}

void initTimer7(int period, int prescaler)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,ENABLE);

	TIM_TimeBaseInitTypeDef str;
	str.TIM_Period=period;
	str.TIM_Prescaler=prescaler;
	str.TIM_ClockDivision=TIM_CKD_DIV1;
	str.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM7,&str);
	TIM_Cmd(TIM7, ENABLE);
}

void initTimer2Interruption()
{
	NVIC_InitTypeDef str;
	str.NVIC_IRQChannel = TIM2_IRQn;
	str.NVIC_IRQChannelPreemptionPriority = 0x00;
	str.NVIC_IRQChannelSubPriority = 0x00;
	str.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&str);
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE );
}

void initTimer3Interruption()
{
	NVIC_InitTypeDef str;
	str.NVIC_IRQChannel = TIM3_IRQn;
	str.NVIC_IRQChannelPreemptionPriority = 0x00;
	str.NVIC_IRQChannelSubPriority = 0x00;
	str.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&str);
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE );
}

void initTimer4Interruption()
{
	NVIC_InitTypeDef str;
	str.NVIC_IRQChannel = TIM4_IRQn;
	str.NVIC_IRQChannelPreemptionPriority = 0x00;
	str.NVIC_IRQChannelSubPriority = 0x00;
	str.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&str);
	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE );
}

void initTimer5Interruption()
{
	NVIC_InitTypeDef str;
	str.NVIC_IRQChannel = TIM5_IRQn;
	str.NVIC_IRQChannelPreemptionPriority = 0x00;
	str.NVIC_IRQChannelSubPriority = 0x00;
	str.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&str);
	TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
	TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE );
}

void initTimer7Interruption()
{
	NVIC_InitTypeDef str;
	str.NVIC_IRQChannel = TIM7_IRQn;
	str.NVIC_IRQChannelPreemptionPriority = 0x00;
	str.NVIC_IRQChannelSubPriority = 0x00;
	str.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&str);
	TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
	TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE );
}

void initTimer2_5HZ(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);

	TIM_TimeBaseInitTypeDef str;
	str.TIM_Period=8399;
	str.TIM_Prescaler=1999;
	str.TIM_ClockDivision=TIM_CKD_DIV1;
	str.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2,&str);
	TIM_Cmd(TIM2, ENABLE);
}

void initTimer2_2HZ(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);

	TIM_TimeBaseInitTypeDef str;
	str.TIM_Period=4999;
	str.TIM_Prescaler=8399;
	str.TIM_ClockDivision=TIM_CKD_DIV1;
	str.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2,&str);
	TIM_Cmd(TIM2, ENABLE);
}

void initTimer3_1HZ(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);

	TIM_TimeBaseInitTypeDef str;
	str.TIM_Period=8399;
	str.TIM_Prescaler=9999;
	str.TIM_ClockDivision=TIM_CKD_DIV1;
	str.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3,&str);
	TIM_Cmd(TIM3, ENABLE);
}

//void TIM2_IRQHandler(void)
//{
//	if(TIM_GetITStatus(TIM2,TIM_IT_Update) != RESET)
//	{
//		GPIO_ToggleBits(GPIOD,GPIO_Pin_12 | GPIO_Pin_14);
//
//		TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
//	}
//}
