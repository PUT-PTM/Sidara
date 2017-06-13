/* Basic libraries for STM */
#include "stm32f4xx_conf.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_syscfg.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_rng.h"
#include "misc.h"
#include "EXTI.h"
#include "GPIO.h"
#include "Timer.h"
#include "USART.h"
#include "stm32f4xx.h"
#include "delay.h"
#include "codec.h"
#include "List.h"
#include "ff.h"
#include <stdbool.h>
#include "tm_stm32f4_pcd8544.h"
#include "DMA.h"
#include "ADC.h"


uint8_t PA1 = 0;
int gesture = 0;
/* for debugging */
int control_counter = 0;
FRESULT fresult;
char sign;

int stopFlag = 0;

int i = 0;

FATFS fatfs;
FIL file;
u16 sample_buffer[2048];
volatile s8 num_of_switch=-1;
volatile u16 result_of_conversion=0;
volatile u8 diode_state=0;
volatile s8 change_song=0;
volatile u8 error_state=0;
volatile bool random_mode=0;
bool pause=0;
char song_time[5]={'0', '0', ':', '0', '0'};
bool half_second=0;

void Play()
{

	if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_7)==0)
	{
		play_wav("h.wav",fresult);
	}
	if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_8)==0)
	{
		play_wav("d.wav",fresult);
	}
	if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_6)==0)
	{
		play_wav("a.wav",fresult);
	}
	if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_5)==0)
	{
		play_wav("g.wav",fresult);
	}
	if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_0)==0)
	{
		play_wav("f.wav",fresult);
	}
	if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_3)==0)
	{
		play_wav("d1.wav",fresult);
	}
	if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_2)==0)
	{
		play_wav("c.wav",fresult);
	}

}

void USART3_IRQHandler(void)
{
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
		sign = readUSARTFromInterruption();

		if(sign == 'R'){
			stopFlag = 1;
			GPIO_ToggleBits(GPIOD, GPIO_Pin_14);
		}
		else if(sign == 'L'){
			stopFlag = 1;
			GPIO_ToggleBits(GPIOD, GPIO_Pin_12);
		}
		else if(sign == 'U'){
			stopFlag = 1;
			GPIO_ToggleBits(GPIOD, GPIO_Pin_13);
		}
		else if(sign == 'D'){
			stopFlag = 1;
			GPIO_ToggleBits(GPIOD, GPIO_Pin_15);
		}
	}
}

void TIM2_IRQHandler(void)
{

	if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
	{
		ADC_conversion();
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}

}
void TIM3_IRQHandler(void)
{

	if(TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
	{
		if(half_second==false)
		{
			half_second=true;
		}
		else
		{
			half_second=false;
		}
		if(half_second==0)
		{
			song_time[4]++;
			if(song_time[4]==':')
			{
				song_time[3]++;
				song_time[4]='0';
			}
			if(song_time[3]=='6')
			{
				song_time[1]++;
				song_time[3]='0';
			}
			if(song_time[1]==':')
			{
				song_time[0]++;
				song_time[1]='0';
			}
			if(song_time[0]==':')
			{
				song_time[0]='0';
			}
		}
		// wyzerowanie flagi wyzwolonego przerwania
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

	}
}


bool read_and_send(FRESULT fresult, int position, volatile ITStatus it_status, UINT read_bytes, uint32_t DMA_FLAG)
{
	it_status = RESET;
	while(it_status == RESET)
	{
		it_status = DMA_GetFlagStatus(DMA1_Stream5, DMA_FLAG);
	}
	fresult = f_read (&file,&sample_buffer[position],1024*2,&read_bytes);
	DMA_ClearFlag(DMA1_Stream5, DMA_FLAG);

	if(fresult != FR_OK)// jesli wyjeto karte w trakcie odtwarzania plikow
	{
		error_state=2;
		return 0;
	}
	if(read_bytes<1024*2||change_song!=0)
	{
		return 0;
	}
	return 1;
}
void play_wav(TCHAR* filename, FRESULT fresult)
{
	TIM_Cmd(TIM2, ENABLE);
	stopFlag =0;
	Codec_VolumeCtrl(result_of_conversion);
	UINT read_bytes;// uzyta w f_read
	fresult = f_open( &file, filename , FA_READ );
	if( fresult == FR_OK )
	{
		fresult=f_lseek(&file,44);// pominiecie 44 B naglowka pliku .wav
		volatile ITStatus it_status;// sprawdza flage DMA
		change_song=0;
		song_time[0]=song_time[1]=song_time[3]=song_time[4]='0';
		song_time[2]=':';
		half_second=0;
		TIM_Cmd(TIM3, ENABLE);
		while(stopFlag != 1)
		{
			if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_7)==1 &&
					GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_8)==1 &&
					GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_6)==1 &&
					GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_5)==1 &&
					GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_0)==1 &&
					GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_3)==1 &&
					GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_2)==1)
					{
				Codec_VolumeCtrl(0);
					break;
					}

			if (read_and_send(fresult,0, it_status, read_bytes, DMA_FLAG_HTIF5)==0)
			{
				break;
			}
			if (read_and_send(fresult, 1024, it_status, read_bytes, DMA_FLAG_TCIF5)==0)
			{
				break;
			}

		}
		Codec_VolumeCtrl(0);
		TIM_Cmd(TIM3, DISABLE);
		fresult = f_close(&file);
	}

}
bool isWAV(FILINFO fileInfo)
{
	int i=0;
	for (i=0;i<10;i++)
	{
		if(fileInfo.fname[i]=='.')
		{
			if(fileInfo.fname[i+1]=='W' && fileInfo.fname[i+2]=='A' && fileInfo.fname[i+3]=='V')
			{
				return 1;
			}
		}
	}
	return 0;
}


int main(void)
{
	SystemInit();

	initGPIODiodes();
	initUSART();
	initUSARTInterruption();


	delay_init( 80 );// wyslanie 80 impulsow zegarowych; do inicjalizacji SPI
	SPI_SD_Init();// inicjalizacja SPI pod SD

	// SysTick_CLK... >> taktowany z f = ok. 82MHz/8 = ok. 10MHz
	// Systick_Config >> generuje przerwanie co <10ms
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);// zegar 24-bitowy
	SysTick_Config(90000);

	DIR Dir;
	FILINFO fileInfo;

	struct List *first=0,*last=0,*pointer;

	disk_initialize(0);// inicjalizacja karty
	fresult = f_mount( &fatfs, 1,1 );// zarejestrowanie dysku logicznego w systemie

	fresult = f_opendir(&Dir, "\\");
	if(fresult != FR_OK)
	{
		return(fresult);
	}
	u32 number_of_songs=0;
	for(;;)
	{
		fresult = f_readdir(&Dir, &fileInfo);
		if(fresult != FR_OK)
		{
			return(fresult);
		}
		if(!fileInfo.fname[0])
		{
			break;
		}
	}

	Buttons_init();
	codec_init();
	codec_ctrl_init();
	I2S_Cmd(CODEC_I2S, ENABLE);
	MY_DMA_initM2P();
	ADC_init();

	for(;;)
	{
		if(stopFlag!=0)
		Play();
	}
}

void SysTick_Handler()
{
	disk_timerproc();
}
