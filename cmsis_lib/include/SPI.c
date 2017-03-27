#include "SPI.h"

void Konfiguracja_SPI(){

	Konfiguracja_portow_SPI();
	Konfiguracja_struktury_SPI();
}

void Konfiguracja_portow_SPI(){

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);

	GPIO_InitTypeDef port_B;
	port_B.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	port_B.GPIO_Mode = GPIO_Mode_AF;
	port_B.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &port_B);

	// SCK
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);
	// MISO
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);
	// MOSI
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);

	// CS >> TU MOZE SIE JEBNAC
	port_B.GPIO_Pin = GPIO_Pin_11;
	port_B.GPIO_Mode = GPIO_Mode_OUT;
	port_B.GPIO_OType = GPIO_OType_PP;
	port_B.GPIO_Speed = GPIO_Speed_100MHz;
	port_B.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &port_B);
}

void Konfiguracja_struktury_SPI(){

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

	SPI_InitTypeDef SPI;
	SPI.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI.SPI_DataSize = SPI_DataSize_8b;
	SPI.SPI_CPOL = SPI_CPOL_High;
	SPI.SPI_CPHA = SPI_CPHA_2Edge;
	SPI.SPI_NSS = SPI_NSS_Soft;
	SPI.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
	SPI.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI.SPI_CRCPolynomial = 7;
	SPI.SPI_Mode = SPI_Mode_Master;
	SPI_Init(SPI2, &SPI);

	SPI_Cmd(SPI2, ENABLE);
}
