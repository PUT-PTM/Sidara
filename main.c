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
uint8_t addrAB = 0;
uint8_t addrA0 = 0;
uint8_t addrA1 = 0;
uint8_t addrA2 = 0;
uint8_t addrA3 = 0;
uint8_t addr93 = 0;
uint8_t addr90 = 0;
uint8_t addrA4 = 0;
uint8_t addrA5 = 0;
uint8_t addrA7 = 0;
uint8_t addrA9 = 0;
uint8_t addrA6 = 0;
uint8_t addrAA = 0;
uint8_t addrAE = 0;
uint8_t addrAF = 0;
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
	addrAB = I2C_read_register(0xAB);
	addrA0 = I2C_read_register(0xA0);
	addrA1 = I2C_read_register(0xA1);
	addrA2 = I2C_read_register(0xA2);
	addrA3 = I2C_read_register(0xA3);
	addr93 = I2C_read_register(0x93);
	addr90 = I2C_read_register(0x90);
	addrA4 = I2C_read_register(0xA4);
	addrA5 = I2C_read_register(0xA5);
	addrA7 = I2C_read_register(0xA7);
	addrA9 = I2C_read_register(0xA9);
	addrA6 = I2C_read_register(0xA6);
	addrAA = I2C_read_register(0xAA);
	addrAE = I2C_read_register(0xAE);
	addrAF = I2C_read_register(0xAF);
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
        handleGesture();

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

	//Power on device
	uint8_t receivedRegisterValue = I2C_read_register(0x80);
	uint8_t valueToEnableDevice = 0x1;
	I2C_write_register(0x80, receivedRegisterValue | valueToEnableDevice);

	//Enable gestures
	receivedRegisterValue = I2C_read_register(0x80);
	uint8_t valueToEnableGestures = 0x64;
	I2C_write_register(0x80, receivedRegisterValue | valueToEnableGestures);

	enableGestureSensor(TRUE);


//	EnableGestureSensor();




	while (1)
	{
		readAllRegistersConnectedToGestures();

//		control_counter = I2C_read_register_flag(APDS9960_GFIFO_R, I2C_FLAG_RXNE);


	}
}
