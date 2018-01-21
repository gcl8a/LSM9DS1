/******************************************************************************
SFE_LSM9DS1.cpp
SFE_LSM9DS1 Library Source File
Jim Lindblom @ SparkFun Electronics
Original Creation Date: February 27, 2015
https://github.com/sparkfun/LSM9DS1_Breakout

This file implements all functions of the LSM9DS1 class. Functions here range
from higher level stuff, like reading/writing LSM9DS1 registers to low-level,
hardware reads and writes. Both SPI and I2C handler functions can be found
towards the bottom of this file.

Development environment specifics:
	IDE: Arduino 1.6
	Hardware Platform: Arduino Uno
	LSM9DS1 Breakout Version: 1.0

This code is beerware; if you see me (or any other SparkFun employee) at the
local, and you've found our code helpful, please buy us a round!

Distributed as-is; no warranty is given.
******************************************************************************/

#include "LSM9DS1.h"
#include "LSM9DS1_reg.h"

#include "Arduino.h"

#include <Wire.h> // Wire library is used for I2C

//using namespace LSM;

// Sensor Sensitivity Constants
// Values set according to the typical specifications provided in
// table 3 of the LSM9DS1 datasheet. (pg 12)
#define SENSITIVITY_ACCELEROMETER_2  0.000061  //g / LSB
#define SENSITIVITY_ACCELEROMETER_4  0.000122
#define SENSITIVITY_ACCELEROMETER_8  0.000244
#define SENSITIVITY_ACCELEROMETER_16 0.000732

#define SENSITIVITY_GYROSCOPE_245    0.00875    //dps / LSB
#define SENSITIVITY_GYROSCOPE_500    0.0175
#define SENSITIVITY_GYROSCOPE_2000   0.07

#define SENSITIVITY_MAGNETOMETER_4   0.00014
#define SENSITIVITY_MAGNETOMETER_8   0.00029
#define SENSITIVITY_MAGNETOMETER_12  0.00043
#define SENSITIVITY_MAGNETOMETER_16  0.00058

LSM9DS1::LSM9DS1(void) {}

uint16_t LSM9DS1::Init(const IMUSettings& s)
{
    settings = s;

    Wire.begin();
    
    //! Todo: don't use _xgAddress or _mAddress, duplicating memory
	_xgAddress = settings.device.agAddress;
	_mAddress = settings.device.mAddress;
			
	// To verify communication, we can read from the WHO_AM_I register of
	// each device. Store those in a variable so we can return them.
	uint8_t mTest = I2CreadByte(_mAddress, WHO_AM_I_M);		// Read the gyro WHO_AM_I
	uint8_t xgTest = I2CreadByte(_xgAddress, WHO_AM_I_XG);	// Read the accel/mag WHO_AM_I
	uint16_t whoAmICombined = (xgTest << 8) | mTest;
	
	if (whoAmICombined != ((WHO_AM_I_AG_RSP << 8) | WHO_AM_I_M_RSP))
		return 0;
	
    // Gyro initialization stuff:
	initGyro();	// This will "turn on" the gyro. Setting up interrupts, etc.
	
	// Accelerometer initialization stuff:
	initAccel(); // "Turn on" all axes of the accel. Set up interrupts, etc.
	
	// Magnetometer initialization stuff:
	initMag(); // "Turn on" all axes of the mag. Set up interrupts, etc.

	// Once everything is initialized, return the WHO_AM_I registers we read:
	return whoAmICombined;
}

void LSM9DS1::initGyro(void)
{
	uint8_t tempRegValue = 0;
	
	// CTRL_REG1_G (Default value: 0x00)
	// [ODR_G2][ODR_G1][ODR_G0][FS_G1][FS_G0][0][BW_G1][BW_G0]
	// ODR_G[2:0] - Output data rate selection
	// FS_G[1:0] - Gyroscope full-scale selection
	// BW_G[1:0] - Gyroscope bandwidth selection
	
	// To disable gyro, set sample rate bits to 0. We'll only set sample
	// rate if the gyro is enabled.
	if (settings.gyro.enabled)
	{
		tempRegValue = (settings.gyro.sampleRate & 0x07) << 5;
	}
    
    switch (settings.gyro.scale)
    {
        case 500:
            tempRegValue |= (0x1 << 3);
            gCond.scale = SENSITIVITY_GYROSCOPE_500 * M_PI / 180.0;
            break;
        case 2000:
            tempRegValue |= (0x3 << 3);
            gCond.scale = SENSITIVITY_GYROSCOPE_2000 * M_PI / 180.0;
            break;
        default: // Otherwise we'll set it to 245 dps (0x0 << 4)
            settings.gyro.scale = 245;
            gCond.scale = SENSITIVITY_GYROSCOPE_245 * M_PI / 180.0;
            break;
    }
    
	tempRegValue |= (settings.gyro.bandwidth & 0x3);
	I2CwriteByte(_xgAddress, CTRL_REG1_G, tempRegValue);
    
	// CTRL_REG2_G (Default value: 0x00)
	// [0][0][0][0][INT_SEL1][INT_SEL0][OUT_SEL1][OUT_SEL0]
	// INT_SEL[1:0] - INT selection configuration
	// OUT_SEL[1:0] - Out selection configuration
	I2CwriteByte(_xgAddress, CTRL_REG2_G, 0x00);
	
	// CTRL_REG3_G (Default value: 0x00)
	// [LP_mode][HP_EN][0][0][HPCF3_G][HPCF2_G][HPCF1_G][HPCF0_G]
	// LP_mode - Low-power mode enable (0: disabled, 1: enabled)
	// HP_EN - HPF enable (0:disabled, 1: enabled)
	// HPCF_G[3:0] - HPF cutoff frequency
	tempRegValue = settings.gyro.lowPowerEnable ? (1<<7) : 0;
	if (settings.gyro.HPFEnable)
	{
		tempRegValue |= (1<<6) | (settings.gyro.HPFCutoff & 0x0F);
	}
    
	I2CwriteByte(_xgAddress, CTRL_REG3_G, tempRegValue);
	
	// CTRL_REG4 (Default value: 0x38)
	// [0][0][Zen_G][Yen_G][Xen_G][0][LIR_XL1][4D_XL1]
	// Zen_G - Z-axis output enable (0:disable, 1:enable)
	// Yen_G - Y-axis output enable (0:disable, 1:enable)
	// Xen_G - X-axis output enable (0:disable, 1:enable)
	// LIR_XL1 - Latched interrupt (0:not latched, 1:latched)
	// 4D_XL1 - 4D option on interrupt (0:6D used, 1:4D used)
	tempRegValue = 0;
	if (settings.gyro.enableZ) tempRegValue |= (1<<5);
	if (settings.gyro.enableY) tempRegValue |= (1<<4);
	if (settings.gyro.enableX) tempRegValue |= (1<<3);
	if (settings.gyro.latchInterrupt) tempRegValue |= (1<<1);
	I2CwriteByte(_xgAddress, CTRL_REG4, tempRegValue);

	// ORIENT_CFG_G (Default value: 0x00)
	// [0][0][SignX_G][SignY_G][SignZ_G][Orient_2][Orient_1][Orient_0]
	// SignX_G - Pitch axis (X) angular rate sign (0: positive, 1: negative)
	// Orient [2:0] - Directional user orientation selection
	tempRegValue = 0;
	if (settings.gyro.flipX) tempRegValue |= (1<<5);
	if (settings.gyro.flipY) tempRegValue |= (1<<4);
	if (settings.gyro.flipZ) tempRegValue |= (1<<3);
	I2CwriteByte(_xgAddress, ORIENT_CFG_G, tempRegValue);
}

void LSM9DS1::initAccel(void)
{
	uint8_t tempRegValue = 0;
	
	//	CTRL_REG5_XL (0x1F) (Default value: 0x38)
	//	[DEC_1][DEC_0][Zen_XL][Yen_XL][Zen_XL][0][0][0]
	//	DEC[0:1] - Decimation of accel data on OUT REG and FIFO.
	//		00: None, 01: 2 samples, 10: 4 samples 11: 8 samples
	//	Zen_XL - Z-axis output enabled
	//	Yen_XL - Y-axis output enabled
	//	Xen_XL - X-axis output enabled
    //tempRegValue |= 0xC0;
	if (settings.accel.enableZ) tempRegValue |= (1<<5);
	if (settings.accel.enableY) tempRegValue |= (1<<4);
	if (settings.accel.enableX) tempRegValue |= (1<<3);
	
	I2CwriteByte(_xgAddress, CTRL_REG5_XL, tempRegValue);
	
	// CTRL_REG6_XL (0x20) (Default value: 0x00)
	// [ODR_XL2][ODR_XL1][ODR_XL0][FS1_XL][FS0_XL][BW_SCAL_ODR][BW_XL1][BW_XL0]
	// ODR_XL[2:0] - Output data rate & power mode selection
	// FS_XL[1:0] - Full-scale selection
	// BW_SCAL_ODR - Bandwidth selection
	// BW_XL[1:0] - Anti-aliasing filter bandwidth selection
	tempRegValue = 0;
	// To disable the accel, set the sampleRate bits to 0.
	if (settings.accel.enabled)
	{
		tempRegValue |= (settings.accel.sampleRate & 0x07) << 5;
	}
    switch (settings.accel.scale)
    {
        case 4:
            tempRegValue |= (0x2 << 3);
            aCond.scale = SENSITIVITY_ACCELEROMETER_4;
            break;
        case 8:
            tempRegValue |= (0x3 << 3);
            aCond.scale = SENSITIVITY_ACCELEROMETER_8;
            break;
        case 16:
            tempRegValue |= (0x1 << 3);
            aCond.scale = SENSITIVITY_ACCELEROMETER_16;
            break;
        default: // Otherwise it'll be set to 2g (0x0 << 3)
            settings.accel.scale = 2; //in case it's none of these
            aCond.scale = SENSITIVITY_ACCELEROMETER_2;
            break;
    }
	if (settings.accel.bandwidth >= 0)
	{
		tempRegValue |= (1<<2); // Set BW_SCAL_ODR
		tempRegValue |= (settings.accel.bandwidth & 0x03);
	}
	I2CwriteByte(_xgAddress, CTRL_REG6_XL, tempRegValue);
	
	// CTRL_REG7_XL (0x21) (Default value: 0x00)
	// [HR][DCF1][DCF0][0][0][FDS][0][HPIS1]
	// HR - High resolution mode (0: disable, 1: enable)
	// DCF[1:0] - Digital filter cutoff frequency
	// FDS - Filtered data selection
	// HPIS1 - HPF enabled for interrupt function
	tempRegValue = 0;
	if (settings.accel.highResEnable)
	{
		tempRegValue |= (1<<7); // Set HR bit
		tempRegValue |= (settings.accel.highResBandwidth & 0x3) << 5;
	}
	I2CwriteByte(_xgAddress, CTRL_REG7_XL, tempRegValue);
}

uint8_t LSM9DS1::IsAvailableAccelAndGyro(void)
{
    uint8_t status = I2CreadByte(_xgAddress, STATUS_REG_1);
    return (status & 0x03) == 0x03;
}

//void LSM9DS1::magOffset(uint8_t axis, int16_t offset)
//{
//    if (axis > 2)
//        return;
//    uint8_t msb, lsb;
//    msb = (offset & 0xFF00) >> 8;
//    lsb = offset & 0x00FF;
//    I2CwriteByte(_mAddress, OFFSET_X_REG_L_M + (2 * axis), lsb);
//    I2CwriteByte(_mAddress, OFFSET_X_REG_H_M + (2 * axis), msb);
//}

void LSM9DS1::initMag(void)
{
	uint8_t tempRegValue = 0;
	
	// CTRL_REG1_M (Default value: 0x10)
	// [TEMP_COMP][OM1][OM0][DO2][DO1][DO0][0][ST]
	// TEMP_COMP - Temperature compensation
	// OM[1:0] - X & Y axes op mode selection
	//	00:low-power, 01:medium performance
	//	10: high performance, 11:ultra-high performance
	// DO[2:0] - Output data rate selection
	// ST - Self-test enable
	if (settings.mag.tempCompensationEnable) tempRegValue |= (1<<7);
	tempRegValue |= (settings.mag.XYPerformance & 0x3) << 5;
	tempRegValue |= (settings.mag.sampleRate & 0x7) << 2;
	I2CwriteByte(_mAddress, CTRL_REG1_M, tempRegValue);
	
	// CTRL_REG2_M (Default value 0x00)
	// [0][FS1][FS0][0][REBOOT][SOFT_RST][0][0]
	// FS[1:0] - Full-scale configuration
	// REBOOT - Reboot memory content (0:normal, 1:reboot)
	// SOFT_RST - Reset config and user registers (0:default, 1:reset)
	tempRegValue = 0;
	switch (settings.mag.scale)
	{
        case 8:
            tempRegValue |= (0x1 << 5);
            mCond.scale = SENSITIVITY_MAGNETOMETER_8;
            break;
        case 12:
            tempRegValue |= (0x2 << 5);
            mCond.scale = SENSITIVITY_MAGNETOMETER_12;
            break;
        case 16:
            tempRegValue |= (0x3 << 5);
            mCond.scale = SENSITIVITY_MAGNETOMETER_16;
            break;
        default:
            settings.mag.scale = 4;
            mCond.scale = SENSITIVITY_MAGNETOMETER_4;
            break;
	// Otherwise we'll default to 4 gauss (00)
	}
	I2CwriteByte(_mAddress, CTRL_REG2_M, tempRegValue); // +/-4Gauss
	
	// CTRL_REG3_M (Default value: 0x03)
	// [I2C_DISABLE][0][LP][0][0][SIM][MD1][MD0]
	// I2C_DISABLE - Disable I2C interace (0:enable, 1:disable)
	// LP - Low-power mode cofiguration (1:enable)
	// SIM - SPI mode selection (0:write-only, 1:read/write enable)
	// MD[1:0] - Operating mode
	//	00:continuous conversion, 01:single-conversion,
	//  10,11: Power-down
	tempRegValue = 0;
	if (settings.mag.lowPowerEnable) tempRegValue |= (1<<5);
	tempRegValue |= (settings.mag.operatingMode & 0x3);
	I2CwriteByte(_mAddress, CTRL_REG3_M, tempRegValue); // Continuous conversion mode
	
	// CTRL_REG4_M (Default value: 0x00)
	// [0][0][0][0][OMZ1][OMZ0][BLE][0]
	// OMZ[1:0] - Z-axis operative mode selection
	//	00:low-power mode, 01:medium performance
	//	10:high performance, 10:ultra-high performance
	// BLE - Big/little endian data
	tempRegValue = 0;
	tempRegValue = (settings.mag.ZPerformance & 0x3) << 2;
	I2CwriteByte(_mAddress, CTRL_REG4_M, tempRegValue);
	
	// CTRL_REG5_M (Default value: 0x00)
	// [0][BDU][0][0][0][0][0][0]
	// BDU - Block data update for magnetic data
	//	0:continuous, 1:not updated until MSB/LSB are read
	tempRegValue = 0;
	I2CwriteByte(_mAddress, CTRL_REG5_M, tempRegValue);
}


uint8_t LSM9DS1::IsAvailableTemperature(void)
{
	uint8_t status = I2CreadByte(_xgAddress, STATUS_REG_1);
	return (status & 0x04);
}

bool LSM9DS1::ReadAccel(void)
{
	uint8_t temp[6]; // We'll read six bytes from the accelerometer into temp
	if ( I2CreadBytes(_xgAddress, OUT_X_L_XL, temp, 6) == 6 ) // Read 6 bytes, beginning at OUT_X_L_XL
	{
		aReadings[0] = -((uint16_t)temp[1] << 8) | temp[0]; // x-axis times (-1) to make right handed
		aReadings[1] = ((uint16_t)temp[3] << 8) | temp[2];
		aReadings[2] = ((uint16_t)temp[5] << 8) | temp[4];
        
        return true;
    }
    
    else return false;
}

uint8_t LSM9DS1::IsAvailableMagnetometer(void)
{
    uint8_t status = I2CreadByte(_mAddress, STATUS_REG_M);
    return (status & 0x08);
}

bool LSM9DS1::ReadMag(void)
{
	uint8_t temp[6]; // We'll read six bytes from the mag into temp
	if ( I2CreadBytes(_mAddress, OUT_X_L_M, temp, 6) == 6) // Read 6 bytes, beginning at OUT_X_L_M
	{
		mReadings[0] = ((uint16_t)temp[1] << 8) | temp[0]; // Store x-axis values into mx
		mReadings[1] = ((uint16_t)temp[3] << 8) | temp[2]; // Store y-axis values into my
		mReadings[2] = ((uint16_t)temp[5] << 8) | temp[4]; // Store z-axis values into mz
        
        return true;
	}
    
    else return false;
}

bool LSM9DS1::ReadTemp(void)
{
	uint8_t temp[2]; // We'll read two bytes from the temperature sensor into temp	
	if ( I2CreadBytes(_xgAddress, OUT_TEMP_L, temp, 2) == 2 ) // Read 2 bytes, beginning at OUT_TEMP_L
	{
		temperature = ((int16_t)temp[1] << 8) | temp[0];
        
        return true;
	}
    
    return false;
}

bool LSM9DS1::ReadGyro(void)
{
	uint8_t temp[6]; // We'll read six bytes from the gyro into temp
	if ( I2CreadBytes(_xgAddress, OUT_X_L_G, temp, 6) == 6) // Read 6 bytes, beginning at OUT_X_L_G
	{
		gReadings[0] = ((uint16_t)temp[1] << 8) | temp[0]; // Store x-axis values into gx
		gReadings[1] = ((uint16_t)temp[3] << 8) | temp[2]; // Store y-axis values into gy
		gReadings[2] = ((uint16_t)temp[5] << 8) | temp[4]; // Store z-axis values into gz
        
        return true;
	}

    else return false;
}

float32vector LSM9DS1::CalcGyro(void)
{
    return gCond.CalcValues(gReadings);
}

float32vector LSM9DS1::CalcAccel(void)
{
    return aCond.CalcValues(aReadings);
}

float32vector LSM9DS1::CalcMag(void)
{
    return mCond.CalcValues(mReadings);
}


void LSM9DS1::configInt(interrupt_select interrupt, uint8_t generator,
	                     h_lactive activeLow, pp_od pushPull)
{
	// Write to INT1_CTRL or INT2_CTRL. [interupt] should already be one of
	// those two values.
	// [generator] should be an OR'd list of values from the interrupt_generators enum
    uint8_t interruptRegister = (interrupt == XG_INT1) ? INT1_CTRL : INT2_CTRL;
    
	I2CwriteByte(_xgAddress, interruptRegister, generator);
	
	// Configure CTRL_REG8
	uint8_t temp;
	temp = I2CreadByte(_xgAddress, CTRL_REG8);
	
	if (activeLow) temp |= (1<<5);
	else temp &= ~(1<<5);
	
	if (pushPull) temp &= ~(1<<4);
	else temp |= (1<<4);
	
	I2CwriteByte(_xgAddress, CTRL_REG8, temp);
}

void LSM9DS1::configInactivity(uint8_t duration, uint8_t threshold, bool sleepOn)
{
	uint8_t temp = 0;
	
	temp = threshold & 0x7F;
	if (sleepOn) temp |= (1<<7);
	I2CwriteByte(_xgAddress, ACT_THS, temp);
	
	I2CwriteByte(_xgAddress, ACT_DUR, duration);
}

uint8_t LSM9DS1::getInactivity(void)
{
	uint8_t temp = I2CreadByte(_xgAddress, STATUS_REG_0);
	temp &= (0x10);
	return temp;
}

void LSM9DS1::configAccelInt(uint8_t generator, bool andInterrupts)
{
	// Use variables from accel_interrupt_generator, OR'd together to create
	// the [generator]value.
	uint8_t temp = generator;
	if (andInterrupts) temp |= 0x80;
	I2CwriteByte(_xgAddress, INT_GEN_CFG_XL, temp);
}

void LSM9DS1::configAccelThs(uint8_t threshold, lsm9ds1_axis axis, uint8_t duration, bool wait)
{
	// Write threshold value to INT_GEN_THS_?_XL.
	// axis will be 0, 1, or 2 (x, y, z respectively)
	I2CwriteByte(_xgAddress, INT_GEN_THS_X_XL + axis, threshold);
	
	// Write duration and wait to INT_GEN_DUR_XL
	uint8_t temp;
	temp = (duration & 0x7F);
	if (wait) temp |= 0x80;
	I2CwriteByte(_xgAddress, INT_GEN_DUR_XL, temp);
}

uint8_t LSM9DS1::getAccelIntSrc(void)
{
	uint8_t intSrc = I2CreadByte(_xgAddress, INT_GEN_SRC_XL);
	
	// Check if the IA_XL (interrupt active) bit is set
	if (intSrc & (1<<6))
	{
		return (intSrc & 0x3F);
	}
	
	return 0;
}

void LSM9DS1::configGyroInt(uint8_t generator, bool aoi, bool latch)
{
	// Use variables from accel_interrupt_generator, OR'd together to create
	// the [generator]value.
	uint8_t temp = generator;
	if (aoi) temp |= 0x80;
	if (latch) temp |= 0x40;
	I2CwriteByte(_xgAddress, INT_GEN_CFG_G, temp);
}

void LSM9DS1::configGyroThs(int16_t threshold, lsm9ds1_axis axis, uint8_t duration, bool wait)
{
	uint8_t buffer[2];
	buffer[0] = (threshold & 0x7F00) >> 8;
	buffer[1] = (threshold & 0x00FF);
	// Write threshold value to INT_GEN_THS_?H_G and  INT_GEN_THS_?L_G.
	// axis will be 0, 1, or 2 (x, y, z respectively)
	I2CwriteByte(_xgAddress, INT_GEN_THS_XH_G + (axis * 2), buffer[0]);
	I2CwriteByte(_xgAddress, INT_GEN_THS_XH_G + 1 + (axis * 2), buffer[1]);
	
	// Write duration and wait to INT_GEN_DUR_XL
	uint8_t temp;
	temp = (duration & 0x7F);
	if (wait) temp |= 0x80;
	I2CwriteByte(_xgAddress, INT_GEN_DUR_G, temp);
}

uint8_t LSM9DS1::getGyroIntSrc(void)
{
	uint8_t intSrc = I2CreadByte(_xgAddress, INT_GEN_SRC_G);
	
	// Check if the IA_G (interrupt active) bit is set
	if (intSrc & (1<<6))
	{
		return (intSrc & 0x3F);
	}
	
	return 0;
}

void LSM9DS1::configMagInt(uint8_t generator, h_lactive activeLow, bool latch)
{
	// Mask out non-generator bits (0-4)
	uint8_t config = (generator & 0xE0);	
	// IEA bit is 0 for active-low, 1 for active-high.
	if (activeLow == INT_ACTIVE_HIGH) config |= (1<<2);
	// IEL bit is 0 for latched, 1 for not-latched
	if (!latch) config |= (1<<1);
	// As long as we have at least 1 generator, enable the interrupt
	if (generator != 0) config |= (1<<0);
	
	I2CwriteByte(_mAddress, INT_CFG_M, config);
}

void LSM9DS1::configMagThs(uint16_t threshold)
{
	// Write high eight bits of [threshold] to INT_THS_H_M
	I2CwriteByte(_mAddress, INT_THS_H_M, uint8_t((threshold & 0x7F00) >> 8));
	// Write low eight bits of [threshold] to INT_THS_L_M
	I2CwriteByte(_mAddress, INT_THS_L_M, uint8_t(threshold & 0x00FF));
}

uint8_t LSM9DS1::getMagIntSrc(void)
{
	uint8_t intSrc = I2CreadByte(_mAddress, INT_SRC_M);
	
	// Check if the INT (interrupt active) bit is set
	if (intSrc & (1<<0))
	{
		return (intSrc & 0xFE);
	}
	
	return 0;
}

void LSM9DS1::sleepGyro(bool enable)
{
	uint8_t temp = I2CreadByte(_xgAddress, CTRL_REG9);
	if (enable) temp |= (1<<6);
	else temp &= ~(1<<6);
	I2CwriteByte(_xgAddress, CTRL_REG9, temp);
}

void LSM9DS1::InitFIFO(uint8_t threshold)
{
    enableFIFO(true);
    setFIFO(FIFO_CONT, 16 - 1);
    configInt(XG_INT1, INT_FTH, INT_ACTIVE_LOW, INT_PUSH_PULL);
}

void LSM9DS1::enableFIFO(bool enable)
{
	uint8_t temp = I2CreadByte(_xgAddress, CTRL_REG9);
	if (enable) temp |= (1<<1);
	else temp &= ~(1<<1);
	I2CwriteByte(_xgAddress, CTRL_REG9, temp);
}

void LSM9DS1::setFIFO(fifoMode_type fifoMode, uint8_t fifoThs)
{
	// Limit threshold - 0x1F (31) is the maximum. If more than that was asked
	// limit it to the maximum.
	uint8_t threshold = fifoThs <= 0x1F ? fifoThs : 0x1F;
	I2CwriteByte(_xgAddress, FIFO_CTRL, ((fifoMode & 0x7) << 5) | (threshold & 0x1F));
}

uint8_t LSM9DS1::getFIFOSamples(void)
{
	return (I2CreadByte(_xgAddress, FIFO_SRC) & 0x3F);
}

// Wire.h read and write protocols
void LSM9DS1::I2CwriteByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
	Wire.beginTransmission(address);  // Initialize the Tx buffer
	Wire.write(subAddress);           // Put slave register address in Tx buffer
	Wire.write(data);                 // Put data in Tx buffer
	Wire.endTransmission();           // Send the Tx buffer
}

uint8_t LSM9DS1::I2CreadByte(uint8_t address, uint8_t subAddress)
{
	uint8_t data; // `data` will store the register data	
	
	Wire.beginTransmission(address);         // Initialize the Tx buffer
	Wire.write(subAddress);	                 // Put slave register address in Tx buffer
	Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
	Wire.requestFrom(address, (uint8_t) 1);  // Read one byte from slave register address 
	
	data = Wire.read();                      // Fill Rx buffer with result
	return data;                             // Return data read from slave register
}

uint8_t LSM9DS1::I2CreadBytes(uint8_t address, uint8_t subAddress, uint8_t * dest, uint8_t count)
{
	byte retVal;
	Wire.beginTransmission(address);      // Initialize the Tx buffer
	// Next send the register to be read. OR with 0x80 to indicate multi-read.
	Wire.write(subAddress | 0x80);        // Put slave register address in Tx buffer
	retVal = Wire.endTransmission(false); // Send Tx buffer, send a restart to keep connection alive
	if (retVal != 0) // endTransmission should return 0 on success
		return 0;
	
	retVal = Wire.requestFrom(address, count);  // Read bytes from slave register address 
	if (retVal != count)
		return 0;
	
	for (int i=0; i<count;)
		dest[i++] = Wire.read();
	
	return count;
}

uint8_t LSM9DS1::ReadAccelFIFO(uint8_t address, uint8_t count, bool inklGyro)
{
    //pseudo burst mode...
    
    if(count > aFIFO.CountRows()) return -1;
    if(count > gFIFO.CountRows()) return -1;
    
    uint8_t bytesPerReading = inklGyro ? 12 : 6;

    for(uint8_t i = 0; i < count; i++)
    {
        //a little slower to read them this way, but I haven't had time yet to dig deeper into Wire
        //probably adds 20 - 30% overhead
        //right now, takes ~25ms to read 16 sets of gyro and accelerometer, which is way too slow
        //if I want to run at top speed (~1kHz), which needs to be way less than 1ms/read
        //with high speed I2C (400kHz clock), you get ~25us / byte (inkl. some fluff for overhead),
        //which is 300us for each 12-byte dataset. Looks like Arduino is defaulting to standard 100kHz.
        
        //OPTIONS:
        //use high-speed (quick test shows it works just fine and reduces time to ~7ms/16 sets)
        //just poll accelerometer; just poll z-xl if all I want is jitter
        //slow the heck down
        //instead of trying to process every reading, just count the big ones
        
        Wire.beginTransmission(address);      // Initialize the Tx buffer
        Wire.write(OUT_X_L_G);// | 0x80);        // start at the temperature register
        uint8_t retCount = Wire.endTransmission(false); // Send Tx buffer, send a restart to keep connection alive
        if (retCount != 0) // endTransmission should return 0 on success
            return 0;

        retCount = Wire.requestFrom(address, bytesPerReading, true);
        if(retCount != bytesPerReading) return 0;
        if(inklGyro)
        {
            for(int j = 0; j < 3; j++)
            {
                uint8_t lsb = Wire.read();
                uint8_t hsb = Wire.read();

                int16_t reading = (hsb << 8) | lsb; //is this legit?
                gFIFO[i][j] = reading;
            }
        }

        for(int j = 0; j < 3; j++)
        {
            uint8_t lsb = Wire.read();
            uint8_t hsb = Wire.read();

            int16_t reading = (hsb << 8) | lsb; //is this legit?
            aFIFO[i][j] = reading;
        }
    }
    
    return count;
}



void LSM9DS1::ProcessReadings(void)
{
    currReading.timestamp = millis(); //not really true
    ReadGyro();
    float32vector gyroValues = CalcGyro();
    ahrs.UpdateGyro(gyroValues[0], gyroValues[1], gyroValues[2]);

    currReading.timestamp = millis(); //not really true
    ReadAccel();
    float32vector accelValues = CalcAccel();
    ahrs.CorrectAccel(accelValues[0], accelValues[1], accelValues[2]);
    
    //return status & 0x03;
}

