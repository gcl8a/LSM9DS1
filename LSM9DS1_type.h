/******************************************************************************
LSM9DS1_Types.h
SFE_LSM9DS1 Library - LSM9DS1 Types and Enumerations
Jim Lindblom @ SparkFun Electronics
Original Creation Date: April 21, 2015
https://github.com/sparkfun/LSM9DS1_Breakout

This file defines all types and enumerations used by the LSM9DS1 class.

Development environment specifics:
	IDE: Arduino 1.6.0
	Hardware Platform: Arduino Uno
	LSM9DS1 Breakout Version: 1.0

This code is beerware; if you see me (or any other SparkFun employee) at the
local, and you've found our code helpful, please buy us a round!

Distributed as-is; no warranty is given.
******************************************************************************/

#ifndef __LSM9DS1_TYPE_H__
#define __LSM9DS1_TYPE_H__

//#include "LSM9DS1_reg.h"

// The LSM9DS1 functions over both I2C or SPI. This library supports both.
// But the interface mode used must be sent to the LSM9DS1 constructor. Use
// one of these two as the first parameter of the constructor.
enum interface_mode
{
	IMU_MODE_SPI,
	IMU_MODE_I2C,
};

// accel_scale defines all possible FSR's of the accelerometer:
enum accel_scale
{
	A_SCALE_2G,	// 00:  2g
	A_SCALE_16G,// 01:  16g
	A_SCALE_4G,	// 10:  4g
	A_SCALE_8G	// 11:  8g
};

// gyro_scale defines the possible full-scale ranges of the gyroscope:
enum gyro_scale
{
	G_SCALE_245DPS,		// 00:  245 degrees per second
	G_SCALE_500DPS,		// 01:  500 dps
	G_SCALE_2000DPS,	// 11:  2000 dps
};

// mag_scale defines all possible FSR's of the magnetometer:
enum mag_scale
{
	M_SCALE_4GS, 	// 00:  4Gs
	M_SCALE_8GS,	// 01:  8Gs
	M_SCALE_12GS,	// 10:  12Gs
	M_SCALE_16GS,	// 11:  16Gs
};

// gyro_odr defines all possible data rate/bandwidth combos of the gyro:
enum gyro_odr
{
	//! TODO 
	G_ODR_PD,	// Power down (0)
	G_ODR_149,	// 14.9 Hz (1)
	G_ODR_595,	// 59.5 Hz (2)
	G_ODR_119,	// 119 Hz (3)
	G_ODR_238,	// 238 Hz (4)
	G_ODR_476,	// 476 Hz (5)
	G_ODR_952	// 952 Hz (6)
};
// accel_oder defines all possible output data rates of the accelerometer:
enum accel_odr
{
	XL_POWER_DOWN, 	// Power-down mode (0x0)
	XL_ODR_10,		// 10 Hz (0x1)
	XL_ODR_50,		// 50 Hz (0x02)
	XL_ODR_119,		// 119 Hz (0x3)
	XL_ODR_238,		// 238 Hz (0x4)
	XL_ODR_476,		// 476 Hz (0x5)
	XL_ODR_952		// 952 Hz (0x6)
};

// accel_abw defines all possible anti-aliasing filter rates of the accelerometer:
enum accel_abw
{
	A_ABW_408,		// 408 Hz (0x0)
	A_ABW_211,		// 211 Hz (0x1)
	A_ABW_105,		// 105 Hz (0x2)
	A_ABW_50,		//  50 Hz (0x3)
};


// mag_odr defines all possible output data rates of the magnetometer:
enum mag_odr
{
	M_ODR_0625,	// 0.625 Hz (0)
	M_ODR_125,	// 1.25 Hz (1)
	M_ODR_250,	// 2.5 Hz (2)
	M_ODR_5,	// 5 Hz (3)
	M_ODR_10,	// 10 Hz (4)
	M_ODR_20,	// 20 Hz (5)
	M_ODR_40,	// 40 Hz (6)
	M_ODR_80	// 80 Hz (7)
};

enum interrupt_select
{
    XG_INT1, // = INT1_CTRL,
    XG_INT2  // = INT2_CTRL
};

enum interrupt_generators
{
	INT_DRDY_XL = (1<<0),	 // Accelerometer data ready (INT1 & INT2)
	INT_DRDY_G = (1<<1),	 // Gyroscope data ready (INT1 & INT2)
	INT1_BOOT = (1<<2),	 // Boot status (INT1)
	INT2_DRDY_TEMP = (1<<2),// Temp data ready (INT2)
	INT_FTH = (1<<3),		 // FIFO threshold interrupt (INT1 & INT2)
	INT_OVR = (1<<4),		 // Overrun interrupt (INT1 & INT2)
	INT_FSS5 = (1<<5),		 // FSS5 interrupt (INT1 & INT2)
	INT_IG_XL = (1<<6),	 // Accel interrupt generator (INT1)
	INT1_IG_G = (1<<7),	 // Gyro interrupt enable (INT1)
	INT2_INACT = (1<<7),	 // Inactivity interrupt output (INT2)
};	

enum accel_interrupt_generator
{
	XLIE_XL = (1<<0),
	XHIE_XL = (1<<1),
	YLIE_XL = (1<<2),
	YHIE_XL = (1<<3),
	ZLIE_XL = (1<<4),
	ZHIE_XL = (1<<5),
	GEN_6D = (1<<6)
};

enum gyro_interrupt_generator
{
	XLIE_G = (1<<0),
	XHIE_G = (1<<1),
	YLIE_G = (1<<2),
	YHIE_G = (1<<3),
	ZLIE_G = (1<<4),
	ZHIE_G = (1<<5)
};

enum mag_interrupt_generator
{
	ZIEN = (1<<5),
	YIEN = (1<<6),
	XIEN = (1<<7)
};

enum h_lactive
{
	INT_ACTIVE_HIGH,
	INT_ACTIVE_LOW
};

enum pp_od
{
	INT_PUSH_PULL,
	INT_OPEN_DRAIN
};

enum fifoMode_type
{
	FIFO_OFF = 0,
	FIFO_THS = 1,
	FIFO_CONT_TRIGGER = 3,
	FIFO_OFF_TRIGGER = 4,
	FIFO_CONT = 6
};

struct deviceSettings
{
    uint8_t commInterface = IMU_MODE_I2C; //obsolete
    // [agAddress] sets the I2C address of the accelerometer/gyroscope.
    uint8_t agAddress = 0x6B;    // I2C address or SPI CS pin
    // [mAddress] sets the I2C address of the magnetometer.
    uint8_t mAddress = 0x1E;    // I2C address or SPI CS pin
};

struct gyroSettings
{
	// Gyroscope settings:
	uint8_t enabled = true;
    
    // [scale] sets the full-scale range of the gyroscope.
    // scale can be set to either 245, 500, or 2000
	uint16_t scale = 2000;	// Changed this to 16-bit
    
    // [sampleRate] sets the output data rate (ODR) of the gyro
    // sampleRate can be set between 1-6
    // 1 = 14.9    4 = 238
    // 2 = 59.5    5 = 476
    // 3 = 119     6 = 952
    uint8_t sampleRate = 1;  //should be much higher...
    
    // [bandwidth] can set the cutoff frequency of the gyro.
    // Allowed values: 0-3. Actual value of cutoff frequency
    // depends on the sample rate. (Datasheet section 7.12)
	uint8_t bandwidth = 0;
    
    // [lowPowerEnable] turns low-power mode on or off.
    uint8_t lowPowerEnable = false; // LP mode off
    // [HPFEnable] enables or disables the high-pass filter
    uint8_t HPFEnable = true; // HPF disabled
    // [HPFCutoff] sets the HPF cutoff frequency (if enabled)
    // Allowable values are 0-9. Value depends on ODR.
    // (Datasheet section 7.14)
    uint8_t HPFCutoff = 1; // HPF cutoff = 4Hz
    // [flipX], [flipY], and [flipZ] are booleans that can
    // automatically switch the positive/negative orientation
    // of the three gyro axes.
    
	uint8_t flipX = true;
	uint8_t flipY = false;
	uint8_t flipZ = false;
	uint8_t orientation = 0;
	uint8_t enableX = true;
	uint8_t enableY = true;
	uint8_t enableZ = true;
	uint8_t latchInterrupt = true;
};

struct accelSettings
{
	// Accelerometer settings:
    uint8_t enabled = true;

    // [scale] sets the full-scale range of the accelerometer.
    // accel scale can be 2, 4, 8, or 16
    uint8_t scale = 8;

    // [sampleRate] sets the output data rate (ODR) of the
    // accelerometer. ONLY APPLICABLE WHEN THE GYROSCOPE IS
    // DISABLED! Otherwise accel sample rate = gyro sample rate.
    // accel sample rate can be 1-6
    // 1 = 10 Hz    4 = 238 Hz
    // 2 = 50 Hz    5 = 476 Hz
    // 3 = 119 Hz   6 = 952 Hz
    uint8_t sampleRate = 1;
    
	// New accel stuff:
	uint8_t enableX = true;
	uint8_t enableY = true;
	uint8_t enableZ = true;
    
    // [bandwidth] sets the anti-aliasing filter bandwidth.
    // Accel cutoff freqeuncy can be any value between -1 - 3.
    // -1 = bandwidth determined by sample rate
    // 0 = 408 Hz   2 = 105 Hz
    // 1 = 211 Hz   3 = 50 Hz
    int8_t  bandwidth = 0;
    
    // [highResEnable] enables or disables high resolution
    // mode for the acclerometer.
    uint8_t highResEnable = false;
    
    // [highResBandwidth] sets the LP cutoff frequency of
    // the accelerometer if it's in high-res mode.
    // can be any value between 0-3
    // LP cutoff is set to a factor of sample rate
    // 0 = ODR/50    2 = ODR/9
    // 1 = ODR/100   3 = ODR/400
	uint8_t highResBandwidth = 0;
};

struct magSettings
{
	// Magnetometer settings:
    uint8_t enabled = true;
    
    // [scale] sets the full-scale range of the magnetometer
    // mag scale can be 4, 8, 12, or 16
    uint8_t scale = 12;
    
    // [sampleRate] sets the output data rate (ODR) of the
    // magnetometer.
    // mag data rate can be 0-7:
    // 0 = 0.625 Hz  4 = 10 Hz
    // 1 = 1.25 Hz   5 = 20 Hz
    // 2 = 2.5 Hz    6 = 40 Hz
    // 3 = 5 Hz      7 = 80 Hz
    uint8_t sampleRate = 4;

    // [tempCompensationEnable] enables or disables
    // temperature compensation of the magnetometer.
    uint8_t tempCompensationEnable = false;
	
    // [nPerformance] sets the x and y-axis performance of the
    // magnetometer to either:
    // 0 = Low power mode      2 = high performance
    // 1 = medium performance  3 = ultra-high performance
    uint8_t XYPerformance = 3;
	uint8_t ZPerformance = 3;
	
    // [lowPowerEnable] enables or disables low power mode in
    // the magnetometer.
    uint8_t lowPowerEnable = false;
	
    // [operatingMode] sets the operating mode of the
    // magnetometer. operatingMode can be 0-2:
    // 0 = continuous conversion
    // 1 = single-conversion
    // 2 = power down
    uint8_t operatingMode = 0;
};

struct temperatureSettings
{
    uint8_t enabled = true;
};

struct IMUSettings
{
	deviceSettings device;
	
	gyroSettings gyro;
	accelSettings accel;
	magSettings mag;
	
	temperatureSettings temp;
};

#endif
