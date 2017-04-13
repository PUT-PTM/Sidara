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


//void EnableGestureSensor()
//{
//	//Power on device
//	uint8_t receivedRegisterValue = I2C_read_register(0x80);
//	uint8_t valueToEnableDevice = 0x1;
//	I2C_write_register(0x80, receivedRegisterValue | valueToEnableDevice);
//
//	//Enable gestures
//	receivedRegisterValue = I2C_read_register(0x80);
//	valueToEnableDevice = 0x64;
//	I2C_write_register(0x80, receivedRegisterValue | valueToEnableDevice);
//
//	//Enable gesture interruption
//	receivedRegisterValue = I2C_read_register(0xAB);
//	valueToEnableDevice = 0x2;
//	I2C_write_register(0xAB, receivedRegisterValue | valueToEnableDevice);
//
//	initTimer2For30msDelay();
//}

void ConfigureGestureSensorInterruptPin()
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
//        delay30ms();
//
//        /* Get the contents of the STATUS register. Is data still valid? */
//        gstatus = I2C_read_register(APDS9960_GSTATUS);
//
//        /* If we have valid data, read in FIFO */
//        if( (gstatus & APDS9960_GVALID) == APDS9960_GVALID ) {
//
//            /* Read the current FIFO level */
//        	fifo_level = I2C_read_register(APDS9960_GFLVL);
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

/**
 * Rewritten
 */
uint8_t getMode()
{
    uint8_t enable_value = I2C_read_register(APDS9960_ENABLE);

    /* Read current ENABLE register */
    if( !enable_value ) {
        return ERROR;
    }

    return enable_value;
}

/**
 * @brief Determines if there is a gesture available for reading
 *	Rewritten
 * @return True if gesture available. False otherwise.
 */
bool isGestureAvailable()
{
    uint8_t val = I2C_read_register(APDS9960_GSTATUS);

    /* Read value from GSTATUS register */
    if( !val ) {
        return ERROR;
    }

    /* Shift and mask out GVALID bit */
    val &= APDS9960_GVALID;

    /* Return true/false based on GVALID bit */
    if( val == 1) {
        return TRUE;
    } else {
        return FALSE;
    }
}

/**
 * @brief Configures I2C communications and initializes registers to defaults
 *
 * @return True if initialized successfully. False otherwise.
 */
bool init()
{
    uint8_t id = I2C_read_register(APDS9960_ID);

    /* Read ID register and check against known values for APDS-9960 */
    if( !id ) {
        return FALSE;
    }
    if( !(id == APDS9960_ID_1 || id == APDS9960_ID_2) ) {
        return FALSE;
    }

    /* Set ENABLE register to 0 (disable all features) */
    if( !setMode(ALL, OFF) ) {
        return FALSE;
    }

    /* Set default values for ambient light and proximity registers */
    I2C_write_register(APDS9960_ATIME, DEFAULT_ATIME);
    I2C_write_register(APDS9960_WTIME, DEFAULT_WTIME);
    I2C_write_register(APDS9960_PPULSE, DEFAULT_PROX_PPULSE);
    I2C_write_register(APDS9960_POFFSET_UR, DEFAULT_POFFSET_UR);
    I2C_write_register(APDS9960_POFFSET_DL, DEFAULT_POFFSET_DL);
    I2C_write_register(APDS9960_CONFIG1, DEFAULT_CONFIG1);
    I2C_write_register(APDS9960_CONFIG1, DEFAULT_CONFIG1);


    /* Set default values for gesture sense registers */
    setGestureEnterThresh(DEFAULT_GPENTH);
    setGestureExitThresh(DEFAULT_GEXTH);
    I2C_write_register(APDS9960_GCONF1, DEFAULT_GCONF1);
    setGestureGain(DEFAULT_GGAIN);
    setGestureLEDDrive(DEFAULT_GLDRIVE);

    if( !setGestureWaitTime(DEFAULT_GWTIME) ) {
        return FALSE;
    }
    I2C_write_register(APDS9960_GOFFSET_U, DEFAULT_GOFFSET);
    I2C_write_register(APDS9960_GOFFSET_D, DEFAULT_GOFFSET);
    I2C_write_register(APDS9960_GOFFSET_L, DEFAULT_GOFFSET);
    I2C_write_register(APDS9960_GOFFSET_R, DEFAULT_GOFFSET);
    I2C_write_register(APDS9960_GPULSE, DEFAULT_GPULSE);
    I2C_write_register(APDS9960_GCONF3, DEFAULT_GCONF3);
    if( !setGestureIntEnable(DEFAULT_GIEN) ) {
        return FALSE;
    }

    return TRUE;
}

/**
 * @brief Enables or disables a feature in the APDS-9960
 * Revritten
 * @param[in] mode which feature to enable
 * @param[in] enable ON (1) or OFF (0)
 * @return True if operation success. False otherwise.
 */
bool setMode(uint8_t mode, uint8_t enable)
{
    uint8_t reg_val;

    /* Read current ENABLE register */
    reg_val = getMode();
    if( reg_val == ERROR ) {
        return FALSE;
    }

    /* Change bit(s) in ENABLE register */
    enable = enable & 0x01;
    if( mode >= 0 && mode <= 6 ) {
        if (enable) {
            reg_val |= (1 << mode);
        } else {
            reg_val &= ~(1 << mode);
        }
    } else if( mode == ALL ) {
        if (enable) {
            reg_val = 0x7F;
        } else {
            reg_val = 0x00;
        }
    }

    /* Write value back to ENABLE register */
    I2C_write_register(APDS9960_ENABLE, reg_val);

    return TRUE;
}

/**
 * @brief Sets the entry proximity threshold for gesture sensing
 * Revritten
 * @param[in] threshold proximity value needed to start gesture mode
 * @return True if operation successful. False otherwise.
 */
void setGestureEnterThresh(uint8_t threshold)
{
    I2C_write_register(APDS9960_GPENTH, threshold);
}

/**
 * @brief Sets the exit proximity threshold for gesture sensing
 * Revritten
 * @param[in] threshold proximity value needed to end gesture mode
 * @return True if operation successful. False otherwise.
 */
void setGestureExitThresh(uint8_t threshold)
{
    I2C_write_register(APDS9960_GEXTH, threshold);
}

/**
 * @brief Sets the gain of the photodiode during gesture mode
 * Revritten
 * Value    Gain
 *   0       1x
 *   1       2x
 *   2       4x
 *   3       8x
 *
 * @param[in] gain the value for the photodiode gain
 * @return True if operation successful. False otherwise.
 */

void setGestureGain(uint8_t gain)
{
    uint8_t val = I2C_read_register(APDS9960_GCONF2);

    /* Set bits in register to given value */
    gain &= 0b00000011;
    gain = gain << 5;
    val &= 0b10011111;
    val |= gain;

    /* Write register value back into GCONF2 register */
    I2C_write_register(APDS9960_GCONF2, val);
}

/**
 * @brief Gets the drive current of the LED during gesture mode
 * Revritten
 * Value    LED Current
 *   0        100 mA
 *   1         50 mA
 *   2         25 mA
 *   3         12.5 mA
 *
 * @return the LED drive current value. 0xFF on error.
 */
uint8_t getGestureLEDDrive()
{
    uint8_t val = I2C_read_register(APDS9960_GCONF2);

    /* Read value from GCONF2 register */
    if( !val ) {
        return ERROR;
    }

    /* Shift and mask out GLDRIVE bits */
    val = (val >> 3) & 0b00000011;

    return val;
}

/**
 * @brief Sets the LED drive current during gesture mode
 * Revritten
 * Value    LED Current
 *   0        100 mA
 *   1         50 mA
 *   2         25 mA
 *   3         12.5 mA
 *
 * @param[in] drive the value for the LED drive current
 * @return True if operation successful. False otherwise.
 */
void setGestureLEDDrive(uint8_t drive)
{
    uint8_t val = I2C_read_register(APDS9960_GCONF2, val);

    /* Set bits in register to given value */
    drive &= 0b00000011;
    drive = drive << 3;
    val &= 0b11100111;
    val |= drive;

    /* Write register value back into GCONF2 register */
    I2C_write_register(APDS9960_GCONF2, val);
}

/**
 * @brief Gets the time in low power mode between gesture detections
 * Revritten
 * Value    Wait time
 *   0          0 ms
 *   1          2.8 ms
 *   2          5.6 ms
 *   3          8.4 ms
 *   4         14.0 ms
 *   5         22.4 ms
 *   6         30.8 ms
 *   7         39.2 ms
 *
 * @return the current wait time between gestures. 0xFF on error.
 */
uint8_t getGestureWaitTime()
{
    uint8_t val = I2C_read_register(APDS9960_GCONF2);

    /* Read value from GCONF2 register */
    if( !val) {
        return ERROR;
    }

    /* Mask out GWTIME bits */
    val &= 0b00000111;

    return val;
}

/**
 * @brief Sets the time in low power mode between gesture detections
 * Revritten
 * Value    Wait time
 *   0          0 ms
 *   1          2.8 ms
 *   2          5.6 ms
 *   3          8.4 ms
 *   4         14.0 ms
 *   5         22.4 ms
 *   6         30.8 ms
 *   7         39.2 ms
 *
 * @param[in] the value for the wait time
 * @return True if operation successful. False otherwise.
 */
bool setGestureWaitTime(uint8_t time)
{
    uint8_t val = I2C_read_register(APDS9960_GCONF2);

    /* Read value from GCONF2 register */
    if( !val ) {
        return FALSE;
    }

    /* Set bits in register to given value */
    time &= 0b00000111;
    val &= 0b11111000;
    val |= time;

    /* Write register value back into GCONF2 register */
    I2C_write_register(APDS9960_GCONF2, val);

    return TRUE;
}

/**
 * @brief Turns gesture-related interrupts on or off
 * Revritten
 * @param[in] enable 1 to enable interrupts, 0 to turn them off
 * @return True if operation successful. False otherwise.
 */
bool setGestureIntEnable(uint8_t enable)
{
    uint8_t val = I2C_read_register(APDS9960_GCONF4);

    /* Read value from GCONF4 register */
    if( !val) {
        return FALSE;
    }

    /* Set bits in register to given value */
    enable &= 0b00000001;
    enable = enable << 1;
    val &= 0b11111101;
    val |= enable;

    /* Write register value back into GCONF4 register */
    I2C_write_register(APDS9960_GCONF4, val);

    return TRUE;
}

/**
 * @brief Starts the gesture recognition engine on the APDS-9960
 * Revritten
 * @param[in] interrupts true to enable hardware external interrupt on gesture
 * @return True if engine enabled correctly. False on error.
 */
bool enableGestureSensor(bool interrupts)
{

    /* Enable gesture mode
       Set ENABLE to 0 (power off)
       Set WTIME to 0xFF
       Set AUX to LED_BOOST_300
       Enable PON, WEN, PEN, GEN in ENABLE
    */
    resetGestureParameters();
    I2C_write_register(APDS9960_WTIME, 0xFF);
    I2C_write_register(APDS9960_PPULSE, DEFAULT_GESTURE_PPULSE);

    if( !setLEDBoost(LED_BOOST_300) ) {
        return FALSE;
    }
    if( interrupts ) {
        if( !setGestureIntEnable(1) ) {
            return FALSE;
        }
    } else {
        if( !setGestureIntEnable(0) ) {
            return FALSE;
        }
    }
    if( !setGestureMode(1) ) {
        return FALSE;
    }
    if( !enablePower() ){
        return FALSE;
    }
    if( !setMode(WAIT, 1) ) {
        return FALSE;
    }
    if( !setMode(PROXIMITY, 1) ) {
        return FALSE;
    }
    if( !setMode(GESTURE, 1) ) {
        return FALSE;
    }

    return TRUE;
}

/**
 * @brief Sets the LED current boost value
 * Revritten
 * Value  Boost Current
 *   0        100%
 *   1        150%
 *   2        200%
 *   3        300%
 *
 * @param[in] drive the value (0-3) for current boost (100-300%)
 * @return True if operation successful. False otherwise.
 */
bool setLEDBoost(uint8_t boost)
{
    uint8_t val = I2C_read_register(APDS9960_CONFIG2);

    /* Read value from CONFIG2 register */
    if( !val ) {
        return FALSE;
    }

    /* Set bits in register to given value */
    boost &= 0b00000011;
    boost = boost << 4;
    val &= 0b11001111;
    val |= boost;

    /* Write register value back into CONFIG2 register */
    I2C_write_register(APDS9960_CONFIG2, val);

    return TRUE;
}

/**
 * @brief Tells the state machine to either enter or exit gesture state machine
 * Revritten
 * @param[in] mode 1 to enter gesture state machine, 0 to exit.
 * @return True if operation successful. False otherwise.
 */
bool setGestureMode(uint8_t mode)
{
    uint8_t val = I2C_read_register(APDS9960_GCONF4);

    /* Read value from GCONF4 register */
    if( !val ) {
        return FALSE;
    }

    /* Set bits in register to given value */
    mode &= 0b00000001;
    val &= 0b11111110;
    val |= mode;

    /* Write register value back into GCONF4 register */
    I2C_write_register(APDS9960_GCONF4, val);

    return TRUE;
}

/**
 * Turn the APDS-9960 on
 * Revritten
 * @return True if operation successful. False otherwise.
 */
bool enablePower()
{
    if( !setMode(POWER, 1) ) {
        return FALSE;
    }

    return TRUE;
}

/**
 * Turn the APDS-9960 off
 * Revritten
 * @return True if operation successful. False otherwise.
 */
bool disablePower()
{
    if( !setMode(POWER, 0) ) {
        return FALSE;
    }

    return TRUE;
}

/**
 * @brief Resets all the parameters in the gesture data member
 * Revritten
 */
void resetGestureParameters()
{
    gesture_data_.index = 0;
    gesture_data_.total_gestures = 0;

    gesture_ud_delta_ = 0;
    gesture_lr_delta_ = 0;

    gesture_ud_count_ = 0;
    gesture_lr_count_ = 0;

    gesture_near_count_ = 0;
    gesture_far_count_ = 0;

    gesture_state_ = 0;
    gesture_motion_ = DIR_NONE;
}

///**
// * @brief Reads a block (array) of bytes from the I2C device and register
// * Revritten?
// * @param[in] reg the register to read from
// * @param[out] val pointer to the beginning of the data
// * @param[in] len number of bytes to read
// * @return Number of bytes read. -1 on read error.
// */
//int wireReadDataBlock(uint8_t reg, uint8_t *val, unsigned int len)
//{
//    unsigned char i = 0;
//
//    /* Indicate which register we want to read from */
//    if (!wireWriteByte(reg)) {
//        return -1;
//    }
//
//    /* Read block data */
//    Wire.requestFrom(APDS9960_I2C_ADDR, len);
//    while (Wire.available()) {
//        if (i >= len) {
//            return -1;
//        }
//        val[i] = Wire.read();
//        i++;
//    }
//
//    return i;
//}
