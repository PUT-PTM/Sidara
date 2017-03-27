#include "stm32f4xx_conf.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_syscfg.h"
#include "misc.h"
#include "stm32f4xx_spi.h"
#include "SPI.h"
#include "ff.h"
#include "diskio.h"

int main(void)
{
	SystemInit();
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);


	Konfiguracja_SPI();









	// diody do testow

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	/* Configure PD12, PD13, PD14 and PD15 in output pushpull mode */
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin =
	GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);




	FRESULT fresult;
	 FIL plik;
	 WORD zapisanych_bajtow;
	 FATFS fatfs;



	 disk_initialize(0);// inicjalizacja karty
	 	fresult = f_mount( &fatfs, 1,1 );// zarejestrowanie dysku logicznego w systemie

	  // Tworzenie pliku
	  fresult = f_open (&plik,"plik.txt", FA_CREATE_ALWAYS);
	  fresult = f_close (&plik);

	  // Tworzenie katalogu
	  fresult = f_mkdir("katalog1");

	  // Zapis pliku
	 // fresult = f_open (&plik,"plik.txt", FA_WRITE);
	  //fresult = f_write(&plik, "zawartosc pliku", 15, &zapisanych_bajtow);
	  //fresult = f_close (&plik);

	  // Usuniecie pliku
	  //fresult = f_unlink("plik.txt");






















    while(1)
    {


    }
}











