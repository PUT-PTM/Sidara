#include "I2C.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_gpio.h"

#define LSM_ADRES 0x72

void I2C_init(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);

	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_I2C1);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_I2C1);
	GPIO_InitTypeDef gpio;
	GPIO_StructInit(&gpio);
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_OType = GPIO_OType_OD;
	gpio.GPIO_Speed =GPIO_Speed_50MHz;
	gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;

	GPIO_Init(GPIOB,&gpio);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);

	I2C_InitTypeDef i2c;
	I2C_StructInit(&i2c);

	i2c.I2C_Mode = I2C_Mode_I2C;
	//i2c.I2C_Ack = I2C_Ack_Enable;
	i2c.I2C_ClockSpeed = 100000;
	I2C_Init(I2C1,&i2c);
	I2C_Cmd(I2C1,ENABLE);
}

void I2C_start(void)
{
	I2C_GenerateSTART(I2C1,ENABLE);
	while(I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS);
}

void I2C_adress_write(uint8_t adres)
{
	I2C_Send7bitAddress(I2C1,adres,I2C_Direction_Transmitter);
	while(I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) != SUCCESS);
}

void I2C_adress_read(uint8_t adress)
{
	//I2C_AcknowledgeConfig(I2C1,ENABLE);
	I2C_Send7bitAddress(I2C1,adress,I2C_Direction_Receiver);
	while(I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)!= SUCCESS);
}

void I2C_write(uint8_t data)
{
	I2C_SendData(I2C1,data);
	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING) != SUCCESS);
}

uint8_t I2C_read(void)
{
	while(I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED) != SUCCESS);
	return I2C_ReceiveData(I2C1);
}

void I2C_stop(void)
{
	I2C_GenerateSTOP(I2C1,ENABLE);
}

void I2C_write_register(uint8_t register_to_write, uint8_t value)
{
	I2C_start();
	I2C_adress_write(LSM_ADRES);
	I2C_write(register_to_write);
	I2C_write(value);
	I2C_stop();
}

uint8_t I2C_read_register(uint8_t register_to_read)
{
	uint8_t v;
	I2C_start();

	I2C_adress_write(LSM_ADRES);

	I2C_write(register_to_read);
	I2C_start();
	I2C_AcknowledgeConfig(I2C1,ENABLE);
	I2C_adress_read(LSM_ADRES);
	v = I2C_read();
	I2C_AcknowledgeConfig(I2C1,DISABLE);
	I2C_stop();
	return v;
}

FlagStatus I2C_read_register_flag(uint8_t register_to_read, uint32_t I2C_FLAG)
{
	uint8_t v;
	I2C_start();

	I2C_adress_write(LSM_ADRES);

	I2C_write(register_to_read);
	I2C_start();
	I2C_AcknowledgeConfig(I2C1,ENABLE);
	I2C_adress_read(LSM_ADRES);

	v = I2C_GetFlagStatus(I2C1, I2C_FLAG);

	I2C_AcknowledgeConfig(I2C1,DISABLE);
	I2C_stop();
	return v;
}
