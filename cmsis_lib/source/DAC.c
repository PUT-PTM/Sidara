#include "stm32f4xx_dac.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "DAC.h"
#include <math.h>

float alfa = 0; //k¹t <0;360>
float sinusek = 0;//wynik f-cji sinus <-1;1>
int digitAmplituda = 0;//zmienna w programie
int digital_napiecie_sinus = 0;//zmienna w programie
int ADC_Resultsinus = 0;//zmienna w programie

void initDAC(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA , ENABLE); // zegar dla portu GPIO z którego wykorzystany zostanie pin jako wyjœcie DAC (PA4)
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	DAC_InitTypeDef DAC_InitStructure;
	DAC_InitStructure.DAC_Trigger = DAC_Trigger_None;
	DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
	DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bit0;
	DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
	DAC_Init(DAC_Channel_1, &DAC_InitStructure);

	DAC_Cmd(DAC_Channel_1, ENABLE);
}

void sendDAC(int number)
{
	DAC_SetChannel1Data(DAC_Align_12b_R, number);
}

float nDigital_to_Voltage(int ADC_Result)
{//dziala
	float Vout = ((float)ADC_Result/(float)4095)*2.95;
	return Vout;
}

float getSine(float Amplituda, float deltaalfa)
{
	alfa +=deltaalfa;//k¹t zwiekszany od 0 do 360//alfa = 360/czestotliwosc timera //0.09 kiedys bylo
	if(alfa >= 360.0)
		alfa=0.0;

	sinusek = sin(alfa * 3.14/180.0);//wartosc f-cji sinus
	//2.95/4095=amplituda,ktora chcemy / Digital
	//Digital = Amplituda*4095/2.95//wazny wzor
	digitAmplituda = Amplituda*4095.0/2.95;// chyba cyfrowo amplituda
	digital_napiecie_sinus = (int)((sinusek+1.0)*digitAmplituda);// przesuwamy o jeden w gore by nie podawac liczb ujemnych
	DAC_SetChannel1Data(DAC_Align_12b_R, digital_napiecie_sinus);

	//DAC wlasnie wygenerowalo napiecie. Teraz trzeba odczytac je poprzez ADC
	ADC_SoftwareStartConv(ADC1);
	while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
	ADC_Resultsinus = ADC_GetConversionValue(ADC1);
	float Voutsinus = nDigital_to_Voltage(ADC_Resultsinus);
	return Voutsinus;
}
/*
 *	DAC_SetChannel1Data(DAC_Align_12b_R, 0xFFF);
 */

