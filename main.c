/* Basic libraries for STM */
#include "stm32f4xx_conf.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_syscfg.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_tim.h"
#include "misc.h"

#include "I2C.h"
#include "SparkFun_APDS_9960.h"
#include "EXTI.h"
#include "GPIO.h"


/* All registers connected to gestures */
uint8_t addr80 = 0;
uint8_t addr81 = 0;
uint8_t addr83 = 0;
uint8_t addr84 = 0;
uint8_t addr85 = 0;
uint8_t addr86 = 0;
uint8_t addr87 = 0;
uint8_t addr89 = 0;
uint8_t addr8B = 0;
uint8_t addr8C = 0;
uint8_t addr8D = 0;
uint8_t addr8E = 0;
uint8_t addr8F = 0;

uint8_t addr90 = 0;
uint8_t addr92 = 0;
uint8_t addr93 = 0;
uint8_t addr94 = 0;
uint8_t addr95 = 0;
uint8_t addr96 = 0;
uint8_t addr97 = 0;
uint8_t addr98 = 0;
uint8_t addr99 = 0;
uint8_t addr9A = 0;
uint8_t addr9B = 0;
uint8_t addr9C = 0;
uint8_t addr9D = 0;
uint8_t addr9E = 0;
uint8_t addr9F = 0;

uint8_t addrA0 = 0;
uint8_t addrA1 = 0;
uint8_t addrA2 = 0;
uint8_t addrA3 = 0;
uint8_t addrA4 = 0;
uint8_t addrA5 = 0;
uint8_t addrA7 = 0;
uint8_t addrA9 = 0;
uint8_t addrA6 = 0;
uint8_t addrAA = 0;
uint8_t addrAB = 0;
uint8_t addrAE = 0;
uint8_t addrAF = 0;

uint8_t addrE4 = 0;
uint8_t addrE5 = 0;
uint8_t addrE6 = 0;
uint8_t addrE7 = 0;

uint8_t addrFC = 0;
uint8_t addrFD = 0;
uint8_t addrFE = 0;
uint8_t addrFF = 0;

uint8_t PA1 = 0;

bool gestureAvailable = 0;
int gesture = 0;

/* for debugging */
int control_counter = 0;

void readAllRegistersConnectedToGestures()
{
	addr80 = I2C_read_register(0x80);
	addr81 = I2C_read_register(0x81);
	addr83 = I2C_read_register(0x83);
	addr84 = I2C_read_register(0x84);
	addr85 = I2C_read_register(0x85);
	addr86 = I2C_read_register(0x86);
	addr87 = I2C_read_register(0x87);
	addr89 = I2C_read_register(0x89);
	addr8B = I2C_read_register(0x8B);
	addr8C = I2C_read_register(0x8C);
	addr8D = I2C_read_register(0x8D);
	addr8E = I2C_read_register(0x8E);
	addr8F = I2C_read_register(0x8F);

	addr90 = I2C_read_register(0x90);
	addr92 = I2C_read_register(0x92);
	addr93 = I2C_read_register(0x93);
	addr94 = I2C_read_register(0x94);
	addr95 = I2C_read_register(0x95);
	addr96 = I2C_read_register(0x96);
	addr97 = I2C_read_register(0x97);
	addr98 = I2C_read_register(0x98);
	addr99 = I2C_read_register(0x99);
	addr9A = I2C_read_register(0x9A);
	addr9B = I2C_read_register(0x9B);
	addr9C = I2C_read_register(0x9C);
	addr9D = I2C_read_register(0x9D);
	addr9E = I2C_read_register(0x9E);
	addr9F = I2C_read_register(0x9F);

	addrA0 = I2C_read_register(0xA0);
	addrA1 = I2C_read_register(0xA1);
	addrA2 = I2C_read_register(0xA2);
	addrA3 = I2C_read_register(0xA3);
	addrA4 = I2C_read_register(0xA4);
	addrA5 = I2C_read_register(0xA5);
	addrA7 = I2C_read_register(0xA7);
	addrA9 = I2C_read_register(0xA9);
	addrA6 = I2C_read_register(0xA6);
	addrAA = I2C_read_register(0xAA);
	addrAB = I2C_read_register(0xAB);
	addrAE = I2C_read_register(0xAE);
	addrAF = I2C_read_register(0xAF);

	addrE4 = I2C_read_register(0xE4);
	addrE5 = I2C_read_register(0xE5);
	addrE6 = I2C_read_register(0xE6);
	addrE7 = I2C_read_register(0xE7);

	addrFC = I2C_read_register(0xFC);
	addrFD = I2C_read_register(0xFD);
	addrFE = I2C_read_register(0xFE);
	addrFF = I2C_read_register(0xFF);

	PA1 = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1);
}


//void handleGesture()
//{
//    if ( isGestureAvailable() )
//    {
//    	switch ( readGesture() )
//    	{
//
//    	}
//    }
//}

void EXTI1_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line1) != RESET)
	{
//        handleGesture();

        control_counter++;
		EXTI_ClearITPendingBit(EXTI_Line1);
	}
}

void handleGesture()
{
	if ( isGestureAvailable() ) {
	    switch ( readGesture() ) {
//	      case DIR_UP:
//	        Serial.println("UP");
//	        break;
//	      case DIR_DOWN:
//	        Serial.println("DOWN");
//	        break;
	      case DIR_LEFT:
	    	  GPIO_ResetBits(GPIOD, GPIO_Pin_14);
	    	  GPIO_SetBits(GPIOD, GPIO_Pin_12);
	        break;
	      case DIR_RIGHT:
	    	GPIO_ResetBits(GPIOD, GPIO_Pin_12);
	        GPIO_SetBits(GPIOD, GPIO_Pin_14);
	        break;
//	      case DIR_NEAR:
//	        Serial.println("NEAR");
//	        break;
//	      case DIR_FAR:
//	        Serial.println("FAR");
//	        break;
	      default:
	    	  GPIO_SetBits(GPIOD, GPIO_Pin_15);
	    }
	  }
	}

int main(void)
{
	SystemInit();

	I2C_init();
	initGPIODiodes();

//	enableGestureSensor(TRUE);
//	EnableGestureSensor();
//	initTimer5For30msDelay();
//
//	initNVICForEXTI1();
//	ConfigureGestureSensorInterruptPin();
//	initEXTIForGPIOA1();


	//Configure ports and timer for delay
	initNVICForEXTI1();
	ConfigureGestureSensorInterruptPin();
	initEXTIForGPIOA1();

	init();

//	//Power on device
//	uint8_t receivedRegisterValue = I2C_read_register(0x80);
//	uint8_t valueToEnableDevice = 0x1;
//	I2C_write_register(0x80, receivedRegisterValue | valueToEnableDevice);
//
//	//Enable gestures
//	receivedRegisterValue = I2C_read_register(0x80);
//	uint8_t valueToEnableGestures = 0x64;
//	I2C_write_register(0x80, receivedRegisterValue | valueToEnableGestures);

	enableGestureSensor(TRUE);


	EnableGestureSensor();




	while (1)
	{
		readAllRegistersConnectedToGestures();

//		control_counter = I2C_read_register_flag(APDS9960_GFIFO_R, I2C_FLAG_RXNE);


	}
}
