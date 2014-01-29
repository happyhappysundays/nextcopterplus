//***********************************************************
//* gyros.c
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include "compiledefs.h"
#include <avr/io.h>
#include <stdbool.h>
#include <util/delay.h>
#include "io_cfg.h"
#include "adc.h"
#include <avr/pgmspace.h>
#include "i2c.h"
#include "MPU6050.h"
#include "main.h"

//************************************************************
// Defines
//************************************************************

#define GYROS_STABLE 1
#define SECOND_TIMER 19531			// Unit of timing for seconds
#define GYROFS2000DEG 0x18			// 2000 deg/s fullscale
#define GYROFS500DEG 0x08			// 500 deg/s fullscale
#define GYROFS250DEG 0x00			// 250 deg/s fullscale

//************************************************************
// Prototypes
//************************************************************

void ReadGyros(void);
void CalibrateGyrosFast(void);
void CalibrateGyrosSlow(void);
void get_raw_gyros(void);

//************************************************************
// Code
//************************************************************

int16_t gyroADC[3];						// Holds Gyro ADCs

// Polarity handling - same for both KK2.0 and KK2.1
const int8_t Gyro_Pol[5][3] PROGMEM = // ROLL, PITCH, YAW * 5 orientations
{
	{1,1,1},		// Forward
	{1,1,1},		// Vertical
	{1,-1,-1},		// Upside down
	{-1,-1,1},		// Aft
	{1,-1,1},		// Sideways
};

void ReadGyros(void)					// Conventional orientation
{
	uint8_t i;

	get_raw_gyros();					// Updates gyroADC[]

	for (i=0;i<3;i++)					// For all axis
	{
		// Remove offsets from gyro outputs
		gyroADC[i] -= Config.gyroZero[i];

		// Change polarity as per orientation mode
		gyroADC[i] *= (int8_t)pgm_read_byte(&Gyro_Pol[Config.Orientation][i]);
	}
}

void CalibrateGyrosFast(void)
{
	uint8_t i;

	Config.gyroZero[ROLL] 	= 0;						
	Config.gyroZero[PITCH]	= 0;	
	Config.gyroZero[YAW] 	= 0;

	for (i=0;i<32;i++)					// Calculate average over 32 reads
	{
		get_raw_gyros();				// Updates gyroADC[]

		Config.gyroZero[ROLL] 	+= gyroADC[ROLL];						
		Config.gyroZero[PITCH] 	+= gyroADC[PITCH];	
		Config.gyroZero[YAW] 	+= gyroADC[YAW];

		_delay_ms(10);					// Get a better gyro average over time
	}

	Config.gyroZero[ROLL] 	= (Config.gyroZero[ROLL] >> 5);	//Divide by 32				
	Config.gyroZero[PITCH] 	= (Config.gyroZero[PITCH] >> 5);
	Config.gyroZero[YAW] 	= (Config.gyroZero[YAW]	>> 5);
}

void CalibrateGyrosSlow(void)
{
	uint8_t axis;
	uint8_t Gyro_seconds = 0;
	uint8_t Gyro_TCNT2 = 0;
	uint16_t Gyro_timeout = 0;
	bool	Gyros_Stable = false;
	float 	GyroSmooth[NUMBEROFAXIS];

	// Force recalculation
	for (axis = 0; axis < NUMBEROFAXIS; axis++) 
	{
		GyroSmooth[axis] = 0;
	}

	// Wait until gyros stable. Timeout after 5 seconds
	while (!Gyros_Stable && (Gyro_seconds <= 5))
	{
		// Update status timeout
		Gyro_timeout += (uint8_t) (TCNT2 - Gyro_TCNT2);
		Gyro_TCNT2 = TCNT2;

		// Count elapsed seconds
		if (Gyro_timeout > SECOND_TIMER)
		{
			Gyro_seconds++;
			Gyro_timeout = 0;
		}

		get_raw_gyros();

		// Calculate very long rolling average
		for (axis = 0; axis < NUMBEROFAXIS; axis++) 
		{
			GyroSmooth[axis] = ((GyroSmooth[axis] * (float)999) + (float)(gyroADC[axis])) / (float)1000;
			Config.gyroZero[axis] = (int16_t)GyroSmooth[axis];
		}

		// Check for movement
		ReadGyros();

		if ((gyroADC[ROLL] > GYROS_STABLE) || (gyroADC[ROLL] < -GYROS_STABLE) ||
			(gyroADC[PITCH] > GYROS_STABLE) || (gyroADC[PITCH] < -GYROS_STABLE) ||
			(gyroADC[YAW] > GYROS_STABLE) || (gyroADC[YAW] < -GYROS_STABLE))
		{
			Gyros_Stable = false;
		}
		else
		{
			Gyros_Stable = true;
		}
	}
	
	// Still no stability after 5 seconds
	if (!Gyros_Stable)
	{
		General_error |= (1 << SENSOR_ERROR); 	// Set sensor error bit
	}
}

void get_raw_gyros(void)
{
// Get data from MPU6050 for KK2.1
#ifdef KK21
	uint8_t Gyros[6];
	int16_t temp1, temp2;
	int16_t RawADC[3];

	// For KK2.1 boards, use the i2c data from the MPU6050
	// Check gyro array axis order and change in io_cfg.h
	readI2CbyteArray(MPU60X0_DEFAULT_ADDRESS,MPU60X0_RA_GYRO_XOUT_H,(uint8_t *)Gyros,6);

	// Reassemble data into gyroADC array and down-sample to reduce resolution and noise
	temp1 = Gyros[0] << 8;
	temp2 = Gyros[1];
	RawADC[PITCH] = (temp1 + temp2) >> 6;

	temp1 = Gyros[2] << 8;
	temp2 = Gyros[3];
	RawADC[ROLL] = (temp1 + temp2) >> 6;

	temp1 = Gyros[4] << 8;
	temp2 = Gyros[5];
	RawADC[YAW] = (temp1 + temp2) >> 6;

	// Reorient the data as per the board orientation
	gyroADC[ROLL] 	= RawADC[(int8_t)pgm_read_byte(&Gyro_RPY_Order[Config.Orientation][ROLL])];
	gyroADC[PITCH] 	= RawADC[(int8_t)pgm_read_byte(&Gyro_RPY_Order[Config.Orientation][PITCH])];
	gyroADC[YAW]	= RawADC[(int8_t)pgm_read_byte(&Gyro_RPY_Order[Config.Orientation][YAW])];

#else
	read_adc(AIN_Y_GYRO);				// Read roll gyro ADC1 (Roll)
	gyroADC[ROLL] = ADCW;

	read_adc(AIN_X_GYRO);				// Read pitch gyro ADC4 (Pitch)
	gyroADC[PITCH] = ADCW;

	read_adc(AIN_Z_GYRO);				// Read yaw gyro ADC2 (Yaw)
	gyroADC[YAW] = ADCW;
#endif
}


#ifdef KK21
void init_i2c_gyros(void)
{
	// First, configure the MPU6050
	writeI2Cbyte(MPU60X0_DEFAULT_ADDRESS, MPU60X0_RA_PWR_MGMT_1, 0x01); // Gyro X clock, awake

	// Other regs cannot be written until the MPU6050 is out of sleep mode
//	writeI2Cbyte(MPU60X0_DEFAULT_ADDRESS, MPU60X0_RA_SMPLRT_DIV, 0x02);	// Sample rate divder 1kHz / (2+1) = 333Hz
	writeI2Cbyte(MPU60X0_DEFAULT_ADDRESS, MPU60X0_RA_CONFIG, Config.MPU6050_LPF); 	// 0x06 = 5Hz, (5)10Hz, (4)20Hz, (3)42Hz, (2)98Hz, (1)188Hz LPF
	
	// Now configure gyros
	writeI2Cbyte(MPU60X0_DEFAULT_ADDRESS, MPU60X0_RA_GYRO_CONFIG, GYROFS500DEG); // 500 deg/sec
}
#endif
