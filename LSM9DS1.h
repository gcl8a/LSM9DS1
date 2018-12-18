/******************************************************************************
SFE_LSM9DS1.h
SFE_LSM9DS1 Library Header File
Jim Lindblom @ SparkFun Electronics
Original Creation Date: February 27, 2015
https://github.com/sparkfun/LSM9DS1_Breakout

This file prototypes the LSM9DS1 class, implemented in SFE_LSM9DS1.cpp. In
addition, it defines every register in the LSM9DS1 (both the Gyro and Accel/
Magnetometer registers).

Development environment specifics:
	IDE: Arduino 1.6.0
	Hardware Platform: Arduino Uno
	LSM9DS1 Breakout Version: 1.0

This code is beerware; if you see me (or any other SparkFun employee) at the
local, and you've found our code helpful, please buy us a round!

Distributed as-is; no warranty is given.
******************************************************************************/
#ifndef __LSM9DS1_H__
#define __LSM9DS1_H__

#include "Arduino.h"

#include "LSM9DS1_type.h"

#include <vector.h>
#include <matrix.h>

#define float32vector TVector<float>
#define float32matrix TMatrix<float>

#include <MahonyAHRS.h>

#define int16vector TVector<int16_t>
#define int16matrix TMatrix<int16_t>

//#define LSM9DS1_AG_ADDR(sa0)    ((sa0) == 0 ? 0x6A : 0x6B)
//#define LSM9DS1_M_ADDR(sa1)        ((sa1) == 0 ? 0x1C : 0x1E)

struct AHRSDatum
{
    uint32_t timestamp = 0;
    int16_t roll = 0, pitch = 0, yaw = 0; //1000ths of radians
    
    String MakeDataString(void)
    {
        char dataStr[100];
        
//        sprintf(dataStr, "%lu,%i,%i,%i",    //mrad
//                timestamp % 1000,
//                roll,
//                pitch,
//                yaw);
        sprintf(dataStr, "%lu,%2.2f,%2.2f,%2.2f",
                timestamp,
                roll / 1000.0,
                pitch / 1000.0,
                yaw / 1000.0);

        return String(dataStr);
    }
};

enum lsm9ds1_axis {
    X_AXIS,
    Y_AXIS,
    Z_AXIS,
    ALL_AXIS
};

class IMU
{
protected:
    MahonyAHRS ahrs;
    AHRSDatum currReading;
    
public:
    IMU(void) {}
    
    //virtual bool CheckInput(AHRSDatum* datum) = 0;

    AHRSDatum CalcRPY(void)
    {
        float32vector rpy = ahrs.RPY();

        currReading.roll = rpy[0] * 1000; //stored as mrad
        currReading.pitch = rpy[1] * 1000;
        currReading.yaw = rpy[2] * 1000;
        
        return currReading;
    }
    
    String MakeDataString(void) {return currReading.MakeDataString();}
};

//for each reading, we condition the raw readings with:
//conditioned = condMatrix * [reading * scale - bias],
//multiplying by condMatrix is contigent on condition factor being true
struct SensorConditioner
{
    float scale = 0;

    //n.b., bias and condMatrix are independent of scale
    float32vector bias;
    float32matrix condMatrix;
    
    float32vector CalcValues(const int16vector& raw, bool condition = false) //not sure how to default the conditioner...
    {
        float32vector values(3);
        for(int i = 0; i < 3; i++) values[i] = raw[i];
        
        values = values * scale - bias;
        if(condition) values = condMatrix * values;
        return values;
    }
    
    SensorConditioner(void)
    {
        bias = float32vector(3);
        condMatrix = float32matrix::Eye(3);
    }
    
    float32vector Serialize(void)
    {
        float32vector data(12);
        data[0] = bias[0];
        data[1] = bias[1];
        data[2] = bias[2];
        
        data[3] = condMatrix[0][0];
        data[4] = condMatrix[0][1];
        data[5] = condMatrix[0][2];

        data[6] = condMatrix[0][0];
        data[7] = condMatrix[1][1];
        data[8] = condMatrix[1][2];

        data[9]  = condMatrix[2][0];
        data[10] = condMatrix[2][1];
        data[11] = condMatrix[2][2];
        
        return data;
    }
    
    void Deserialize(const float32vector& data)
    {
        bias[0] = data[0];
        bias[1] = data[1];
        bias[2] = data[2];

        condMatrix[0][0] = data[3];
        condMatrix[0][1] = data[4];
        condMatrix[0][2] = data[5];
        
        condMatrix[1][0] = data[6];
        condMatrix[1][1] = data[7];
        condMatrix[1][2] = data[8];

        condMatrix[2][0] = data[9];
        condMatrix[2][1] = data[10];
        condMatrix[2][2] = data[11];
    }
};

class LSM9DS1 : public IMU
{
protected:
	IMUSettings settings;
    
    int16vector gReadings = int16vector(3);
    int16vector aReadings = int16vector(3);
    int16vector mReadings = int16vector(3);
    
    int16matrix gFIFO = int16matrix(32, 3); //32 is max -- make smarter later
    int16matrix aFIFO = int16matrix(32, 3);

    int16_t temperature; //chip temperature

    SensorConditioner aCond;
    SensorConditioner gCond;
    SensorConditioner mCond;
    
public:
	LSM9DS1(void);
    uint16_t Init(const IMUSettings&);
    
    void ProcessReadings(void);

    void CalibrateAccelerometerAndGyroBasic(uint16_t samples);
    int CalibrateMagnetometer(uint16_t nSamples);
    
    void LoadCalibrationDataAccel(const float32vector&);
    void LoadCalibrationDataGyro(const float32vector&);
    void LoadCalibrationDataMag(const float32vector&);

    float32vector OffloadCalibrationDataAccel(void);
    float32vector OffloadCalibrationDataGyro(void);
    float32vector OffloadCalibrationDataMag(void);
    //void calibrateMag(bool loadIn = true);
	//void magOffset(uint8_t axis, int16_t offset);
	
    uint8_t IsAvailableAccelAndGyro(void);
    bool ReadGyro(void);
    bool ReadAccel(void);
	
    uint8_t IsAvailableMagnetometer(void);
	bool ReadMag(void);
	
    uint8_t IsAvailableTemperature(void);
    bool ReadTemp(void);
	
	float32vector CalcGyro(void);
	float32vector CalcAccel(void);
	float32vector CalcMag(void);
		
	// setGyroODR() -- Set the output data rate and bandwidth of the gyroscope
	// Input:
	//	- gRate = The desired output rate and cutoff frequency of the gyro.
	void setGyroODR(uint8_t gRate);
	
	// setAccelODR() -- Set the output data rate of the accelerometer
	// Input:
	//	- aRate = The desired output rate of the accel.
	void setAccelODR(uint8_t aRate); 	
	
	// setMagODR() -- Set the output data rate of the magnetometer
	// Input:
	//	- mRate = The desired output rate of the mag.
	void setMagODR(uint8_t mRate);
	
	// configInactivity() -- Configure inactivity interrupt parameters
	// Input:
	//	- duration = Inactivity duration - actual value depends on gyro ODR
	//	- threshold = Activity Threshold
	//	- sleepOn = Gyroscope operating mode during inactivity.
	//	  true: gyroscope in sleep mode
	//	  false: gyroscope in power-down
	void configInactivity(uint8_t duration, uint8_t threshold, bool sleepOn);
	
	// configAccelInt() -- Configure Accelerometer Interrupt Generator
	// Input:
	//	- generator = Interrupt axis/high-low events
	//	  Any OR'd combination of ZHIE_XL, ZLIE_XL, YHIE_XL, YLIE_XL, XHIE_XL, XLIE_XL
	//	- andInterrupts = AND/OR combination of interrupt events
	//	  true: AND combination
	//	  false: OR combination
	void configAccelInt(uint8_t generator, bool andInterrupts = false);
	
	// configAccelThs() -- Configure the threshold of an accelereomter axis
	// Input:
	//	- threshold = Interrupt threshold. Possible values: 0-255.
	//	  Multiply by 128 to get the actual raw accel value.
	//	- axis = Axis to be configured. Either X_AXIS, Y_AXIS, or Z_AXIS
	//	- duration = Duration value must be above or below threshold to trigger interrupt
	//	- wait = Wait function on duration counter
	//	  true: Wait for duration samples before exiting interrupt
	//	  false: Wait function off
	void configAccelThs(uint8_t threshold, lsm9ds1_axis axis, uint8_t duration = 0, bool wait = 0);
	
	// configGyroInt() -- Configure Gyroscope Interrupt Generator
	// Input:
	//	- generator = Interrupt axis/high-low events
	//	  Any OR'd combination of ZHIE_G, ZLIE_G, YHIE_G, YLIE_G, XHIE_G, XLIE_G
	//	- aoi = AND/OR combination of interrupt events
	//	  true: AND combination
	//	  false: OR combination
	//	- latch: latch gyroscope interrupt request.
	void configGyroInt(uint8_t generator, bool aoi, bool latch);
	
	// configGyroThs() -- Configure the threshold of a gyroscope axis
	// Input:
	//	- threshold = Interrupt threshold. Possible values: 0-0x7FF.
	//	  Value is equivalent to raw gyroscope value.
	//	- axis = Axis to be configured. Either X_AXIS, Y_AXIS, or Z_AXIS
	//	- duration = Duration value must be above or below threshold to trigger interrupt
	//	- wait = Wait function on duration counter
	//	  true: Wait for duration samples before exiting interrupt
	//	  false: Wait function off
	void configGyroThs(int16_t threshold, lsm9ds1_axis axis, uint8_t duration, bool wait);
	
	// configInt() -- Configure INT1 or INT2 (Gyro and Accel Interrupts only)
	// Input:
	//	- interrupt = Select INT1 or INT2
	//	  Possible values: XG_INT1 or XG_INT2
	//	- generator = Or'd combination of interrupt generators.
	//	  Possible values: INT_DRDY_XL, INT_DRDY_G, INT1_BOOT (INT1 only), INT2_DRDY_TEMP (INT2 only)
	//	  INT_FTH, INT_OVR, INT_FSS5, INT_IG_XL (INT1 only), INT1_IG_G (INT1 only), INT2_INACT (INT2 only)
	//	- activeLow = Interrupt active configuration
	//	  Can be either INT_ACTIVE_HIGH or INT_ACTIVE_LOW
	//	- pushPull =  Push-pull or open drain interrupt configuration
	//	  Can be either INT_PUSH_PULL or INT_OPEN_DRAIN
	void configInt(interrupt_select interupt, uint8_t generator,
				   h_lactive activeLow = INT_ACTIVE_LOW, pp_od pushPull = INT_PUSH_PULL);
				   
	// configMagInt() -- Configure Magnetometer Interrupt Generator
	// Input:
	//	- generator = Interrupt axis/high-low events
	//	  Any OR'd combination of ZIEN, YIEN, XIEN
	//	- activeLow = Interrupt active configuration
	//	  Can be either INT_ACTIVE_HIGH or INT_ACTIVE_LOW
	//	- latch: latch gyroscope interrupt request.
	void configMagInt(uint8_t generator, h_lactive activeLow, bool latch = true);
	
	// configMagThs() -- Configure the threshold of a gyroscope axis
	// Input:
	//	- threshold = Interrupt threshold. Possible values: 0-0x7FF.
	//	  Value is equivalent to raw magnetometer value.
	void configMagThs(uint16_t threshold);
	
	// getGyroIntSrc() -- Get contents of Gyroscope interrupt source register
	uint8_t getGyroIntSrc();
	
	// getGyroIntSrc() -- Get contents of accelerometer interrupt source register
	uint8_t getAccelIntSrc();
	
	// getGyroIntSrc() -- Get contents of magnetometer interrupt source register
	uint8_t getMagIntSrc();
	
	// getGyroIntSrc() -- Get status of inactivity interrupt
	uint8_t getInactivity();
	
	// sleepGyro() -- Sleep or wake the gyroscope
	// Input:
	//	- enable: True = sleep gyro. False = wake gyro.
	void sleepGyro(bool enable = true);
	
    void InitFIFO(uint8_t threshold);

	// enableFIFO() - Enable or disable the FIFO
	// Input:
	//	- enable: true = enable, false = disable.
	void enableFIFO(bool enable = true);
	
	// setFIFO() - Configure FIFO mode and Threshold
	// Input:
	//	- fifoMode: Set FIFO mode to off, FIFO (stop when full), continuous, bypass
	//	  Possible inputs: FIFO_OFF, FIFO_THS, FIFO_CONT_TRIGGER, FIFO_OFF_TRIGGER, FIFO_CONT
	//	- fifoThs: FIFO threshold level setting
	//	  Any value from 0-0x1F is acceptable.
	void setFIFO(fifoMode_type fifoMode, uint8_t fifoThs);
	
	// getFIFOSamples() - Get number of FIFO samples
	uint8_t getFIFOSamples();
		

public:
	// x_mAddress and gAddress store the I2C address or SPI chip select pin
	// for each sensor.
	uint8_t _mAddress, _xgAddress;
	
	// gRes, aRes, and mRes store the current resolution for each sensor. 
	// Units of these values would be DPS (or g's or Gs's) per ADC tick.
	// This value is calculated as (sensor scale) / (2^15).
	//float gRes, aRes, mRes;
	
	// _autoCalc keeps track of whether we're automatically subtracting off
	// accelerometer and gyroscope bias calculated in calibrate().
	//bool _autoCalc;
	
	// init() -- Sets up gyro, accel, and mag settings to default.
	// - interface - Sets the interface mode (IMU_MODE_I2C or IMU_MODE_SPI)
	// - xgAddr - Sets either the I2C address of the accel/gyro or SPI chip 
	//   select pin connected to the CS_XG pin.
	// - mAddr - Sets either the I2C address of the magnetometer or SPI chip 
	//   select pin connected to the CS_M pin.
	//void init(interface_mode interface, uint8_t xgAddr, uint8_t mAddr);
	
	// initGyro() -- Sets up the gyroscope to begin reading.
	// This function steps through all five gyroscope control registers.
	// Upon exit, the following parameters will be set:
	//	- CTRL_REG1_G = 0x0F: Normal operation mode, all axes enabled. 
	//		95 Hz ODR, 12.5 Hz cutoff frequency.
	//	- CTRL_REG2_G = 0x00: HPF set to normal mode, cutoff frequency
	//		set to 7.2 Hz (depends on ODR).
	//	- CTRL_REG3_G = 0x88: Interrupt enabled on INT_G (set to push-pull and
	//		active high). Data-ready output enabled on DRDY_G.
	//	- CTRL_REG4_G = 0x00: Continuous update mode. Data LSB stored in lower
	//		address. Scale set to 245 DPS. SPI mode set to 4-wire.
	//	- CTRL_REG5_G = 0x00: FIFO disabled. HPF disabled.
	void initGyro();
	
	// initAccel() -- Sets up the accelerometer to begin reading.
	// This function steps through all accelerometer related control registers.
	// Upon exit these registers will be set as:
	//	- CTRL_REG0_XM = 0x00: FIFO disabled. HPF bypassed. Normal mode.
	//	- CTRL_REG1_XM = 0x57: 100 Hz data rate. Continuous update.
	//		all axes enabled.
	//	- CTRL_REG2_XM = 0x00:  2g scale. 773 Hz anti-alias filter BW.
	//	- CTRL_REG3_XM = 0x04: Accel data ready signal on INT1_XM pin.
	void initAccel();
	
	// initMag() -- Sets up the magnetometer to begin reading.
	// This function steps through all magnetometer-related control registers.
	// Upon exit these registers will be set as:
	//	- CTRL_REG4_XM = 0x04: Mag data ready signal on INT2_XM pin.
	//	- CTRL_REG5_XM = 0x14: 100 Hz update rate. Low resolution. Interrupt
	//		requests don't latch. Temperature sensor disabled.
	//	- CTRL_REG6_XM = 0x00:  2 Gs scale.
	//	- CTRL_REG7_XM = 0x00: Continuous conversion mode. Normal HPF mode.
	//	- INT_CTRL_REG_M = 0x09: Interrupt active-high. Enable interrupts.
	void initMag();
		
	///////////////////
	// I2C Functions //
	///////////////////
	// initI2C() -- Initialize the I2C hardware.
	// This function will setup all I2C pins and related hardware.
	void initI2C();
	
	void I2CwriteByte(uint8_t address, uint8_t subAddress, uint8_t data);
	uint8_t I2CreadByte(uint8_t address, uint8_t subAddress);
    uint8_t I2CreadBytes(uint8_t address, uint8_t subAddress, uint8_t * dest, uint8_t count);
    uint8_t ReadAccelFIFO(uint8_t address, uint8_t count, bool inklGyro = true);
    
    uint8_t ReadRegister(uint8_t subAddr) { return I2CreadByte(_xgAddress, subAddr);}
    uint8_t ReadFIFO(void)
    {
        uint8_t c = ReadAccelFIFO(_xgAddress, 16);
        return c;
    }
};
    
//}

#endif // SFE_LSM9DS1_H //
