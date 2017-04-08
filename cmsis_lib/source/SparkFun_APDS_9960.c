#include "SparkFun_APDS_9960.h"
#include "SPI.h"
#include "Timer.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_conf.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_syscfg.h"
#include "stm32f4xx_exti.h"
#include "misc.h"



void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2,TIM_IT_Update) != RESET)
	{
		GPIO_ToggleBits(GPIOD,GPIO_Pin_12 | GPIO_Pin_14);

		TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
		TIM_Cmd(TIM3, DISABLE);
	}
}

void EnableGestureSensor()
{
	//Power on device
	uint8_t receivedRegisterValue = I2C_read_register(0x80);
	uint8_t valueToEnableDevice = 0x1;
	I2C_write_register(0x80, receivedRegisterValue | valueToEnableDevice);

	//Enable gestures
	receivedRegisterValue = I2C_read_register(0x80);
	valueToEnableDevice = 0x64;
	I2C_write_register(0x80, receivedRegisterValue | valueToEnableDevice);

	//Enable gesture interruption
	receivedRegisterValue = I2C_read_register(0xAB);
	valueToEnableDevice = 0x2;
	I2C_write_register(0xAB, receivedRegisterValue | valueToEnableDevice);

	initTimer2For30msDelay();
}

void EnableGestureSensorInterrupt()
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_InitTypeDef  str;
	str.GPIO_Pin = GPIO_Pin_1;
	str.GPIO_Mode = GPIO_Mode_IN;
	str.GPIO_OType = GPIO_OType_OD;
	str.GPIO_Speed = GPIO_Speed_50MHz;
	str.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &str);
}

///**
// * @brief Processes a gesture event and returns best guessed gesture
// *
// * @return Number corresponding to gesture. -1 on error.
// */
//int ReadGesture()
//{
//    uint8_t fifo_level = 0;
//    uint8_t bytes_read = 0;
//    uint8_t fifo_data[128];
//    uint8_t gstatus;
//    int motion;
//    int i;
//
//    /* Make sure that power and gesture is on and data is valid */
//    if( !isGestureAvailable() || !(getMode() & 0b01000001) ) {
//        return DIR_NONE;
//    }
//
//    /* Keep looping as long as gesture data is valid */
//    while(1) {
//
//        /* Wait some time to collect next batch of FIFO data */
//        delay(FIFO_PAUSE_TIME);
//
//        /* Get the contents of the STATUS register. Is data still valid? */
//        if( !wireReadDataByte(APDS9960_GSTATUS, gstatus) ) {
//            return ERROR;
//        }
//
//        /* If we have valid data, read in FIFO */
//        if( (gstatus & APDS9960_GVALID) == APDS9960_GVALID ) {
//
//            /* Read the current FIFO level */
//            if( !wireReadDataByte(APDS9960_GFLVL, fifo_level) ) {
//                return ERROR;
//            }
//
//            /* If there's stuff in the FIFO, read it into our data block */
//            if( fifo_level > 0) {
//                bytes_read = wireReadDataBlock(  APDS9960_GFIFO_U,
//                                                (uint8_t*)fifo_data,
//                                                (fifo_level * 4) );
//                if( bytes_read == -1 ) {
//                    return ERROR;
//                }
//
//                /* If at least 1 set of data, sort the data into U/D/L/R */
//                if( bytes_read >= 4 ) {
//                    for( i = 0; i < bytes_read; i += 4 ) {
//                        gesture_data_.u_data[gesture_data_.index] = \
//                                                            fifo_data[i + 0];
//                        gesture_data_.d_data[gesture_data_.index] = \
//                                                            fifo_data[i + 1];
//                        gesture_data_.l_data[gesture_data_.index] = \
//                                                            fifo_data[i + 2];
//                        gesture_data_.r_data[gesture_data_.index] = \
//                                                            fifo_data[i + 3];
//                        gesture_data_.index++;
//                        gesture_data_.total_gestures++;
//                    }
//
//                    /* Filter and process gesture data. Decode near/far state */
//                    if( processGestureData() ) {
//                        if( decodeGesture() ) {
////                            //***TODO: U-Turn Gestures
//                        }
//                    }
//
//                    /* Reset data */
//                    gesture_data_.index = 0;
//                    gesture_data_.total_gestures = 0;
//                }
//            }
//        } else {
//
//            /* Determine best guessed gesture and clean up */
//            delay(FIFO_PAUSE_TIME);
//            decodeGesture();
//            motion = gesture_motion_;
//
//            resetGestureParameters();
//            return motion;
//        }
//    }
//}
//
///**
// * @brief Determines if there is a gesture available for reading
// *
// * @return True if gesture available. False otherwise.
// */
//bool isGestureAvailable()
//{
//    uint8_t val;
//
//    /* Read value from GSTATUS register */
//    if( !wireReadDataByte(APDS9960_GSTATUS, val) ) {
//        return ERROR;
//    }
//
//    /* Shift and mask out GVALID bit */
//    val &= APDS9960_GVALID;
//
//    /* Return true/false based on GVALID bit */
//    if( val == 1) {
//        return TRUE;
//    } else {
//        return FALSE;
//    }
//}
//
///**
// * @brief Reads a single byte from the I2C device and specified register
// *
// * @param[in] reg the register to read from
// * @param[out] the value returned from the register
// * @return True if successful read operation. False otherwise.
// */
//bool wireReadDataByte(uint8_t reg, uint8_t &val)
//{
//
//    /* Indicate which register we want to read from */
//    if (!wireWriteByte(reg)) {
//        return FALSE;
//    }
//
//    /* Read from register */
//    Wire.requestFrom(APDS9960_I2C_ADDR, 1);
//    while (Wire.available()) {
//        val = Wire.read();
//    }
//
//    return TRUE;
//}
//
///**
// * @brief Writes a single byte to the I2C device (no register)
// *
// * @param[in] val the 1-byte value to write to the I2C device
// * @return True if successful write operation. False otherwise.
// */
//bool wireWriteByte(uint8_t val)
//{
//    I2C_write_register(APDS9960_I2C_ADDR, val);
//    if( I2C_read_register(APDS9960_I2C_ADDR) != 0 ) {
//        return FALSE;
//    }
//
//    return TRUE;
//}
