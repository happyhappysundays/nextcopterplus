//***********************************************************
//* gyros.c
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include "compiledefs.h"
#include <string.h>
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
// Prototypes
//************************************************************

void ReadGyros(void);
void CalibrateGyrosFast(void);
void CalibrateGyrosSlow(void);
void get_raw_gyros(void);

//************************************************************
// Defines
//************************************************************

#define GYROS_STABLE 1
#define SECOND_TIMER 19531			// Unit of timing for seconds
#define GYROFS2000DEG 0x18			// 2000 deg/s fullscale
#define GYROFS500DEG 0x08			// 500 deg/s fullscale
#define GYROFS250DEG 0x00			// 250 deg/s fullscale

//***********************************************************
// ROLL, PITCH, YAW mapping for alternate orientation modes
//***********************************************************

// This is the order to return data in ROLL, PITCH, YAW order
const int8_t Gyro_RPY_Order[NUMBEROFORIENTS][NUMBEROFAXIS] PROGMEM = 
{
// 	 ROLL, PITCH, YAW
	{ROLL, PITCH, YAW}, // Forward (Normal)
	{PITCH, YAW, ROLL}, // Vertical
	{ROLL, PITCH, YAW},	// Upside down
	{ROLL, PITCH, YAW},	// Aft
	{PITCH, ROLL, YAW},	// Sideways
	{YAW, PITCH, ROLL}, // Rear/bottom (PitchUp)
};

// These are the polarities to return them to the default
const int8_t Gyro_Pol[NUMBEROFORIENTS][NUMBEROFAXIS] PROGMEM = 
{
// 	ROLL, PITCH, YAW
	{1,1,1},		// Forward (Normal)
	{1,1,1},		// Vertical
	{1,-1,-1},		// Upside down
	{-1,-1,1},		// Aft
	{1,-1,1},		// Sideways
	{-1,1,1},		// Rear/bottom (PitchUp)
};

//************************************************************
// Code
//************************************************************

int16_t gyroADC[NUMBEROFAXIS];			// Holds Gyro ADCs

void ReadGyros(void)					// Conventional orientation
{
	uint8_t i;

	get_raw_gyros();					// Updates gyroADC[]

	for (i=0; i<NUMBEROFAXIS; i++)	
	{
		// Remove offsets from gyro outputs
		gyroADC[i] -= Config.gyroZero[i];

		// Change polarity
		gyroADC[i] *= (int8_t)pgm_read_byte(&Gyro_Pol[Config.Orientation][i]);
	}
}

void get_raw_gyros(void)
{
	int16_t RawADC[NUMBEROFAXIS];
	uint8_t i;

// Get data from MPU6050 for KK2.1
#ifdef KK21
	uint8_t Gyros[6];
	int16_t temp1, temp2;

	// For KK2.1 boards, use the i2c data from the MPU6050
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

#else
	read_adc(AIN_Y_GYRO);				// Read roll gyro ADC1 (Roll)
	RawADC[ROLL] = ADCW;

	read_adc(AIN_X_GYRO);				// Read pitch gyro ADC4 (Pitch)
	RawADC[PITCH] = ADCW;

	read_adc(AIN_Z_GYRO);				// Read yaw gyro ADC2 (Yaw)
	RawADC[YAW] = ADCW;
#endif

	// Reorient the data as per the board orientation	
	for (i=0; i<NUMBEROFAXIS; i++)
	{
		// Rearrange the sensors
		gyroADC[i] 	= RawADC[(int8_t)pgm_read_byte(&Gyro_RPY_Order[Config.Orientation][i])];
	}
}

//***************************************************************
// Calibration routines
//***************************************************************

void CalibrateGyrosFast(void)
{
	uint8_t i;

	// Clear gyro zeros
	memset(&Config.gyroZero[ROLL],0,(sizeof(int16_t) * NUMBEROFAXIS));

	for (i=0; i<32; i++)				// Calculate average over 32 reads
	{
		get_raw_gyros();				// Updates gyroADC[]

		Config.gyroZero[ROLL] 	+= gyroADC[ROLL];						
		Config.gyroZero[PITCH] 	+= gyroADC[PITCH];	
		Config.gyroZero[YAW] 	+= gyroADC[YAW];

		_delay_ms(10);					// Get a better gyro average over time
	}

	for (i=0; i<NUMBEROFAXIS; i++)		// Average readings for all axis
	{
		Config.gyroZero[i] 	= (Config.gyroZero[i] >> 5);	// Divide by 32	
	}
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
// Optimise starting point for each board
#ifdef KK21
		GyroSmooth[axis] = 0;
#else
		GyroSmooth[axis] = 500;
#endif
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
}

#ifdef KK21
//***************************************************************
// Set up the MPU6050 (Gyro)
//***************************************************************

void init_i2c_gyros(void)
{
	// First, configure the MPU6050
	writeI2Cbyte(MPU60X0_DEFAULT_ADDRESS, MPU60X0_RA_PWR_MGMT_1, 0x01); 			// Gyro X clock, awake

	// Other regs cannot be written until the MPU6050 is out of sleep mode
	writeI2Cbyte(MPU60X0_DEFAULT_ADDRESS, MPU60X0_RA_CONFIG, MPU60X0_DLPF_BW_5);	// 0x06 = 5Hz, (5)10Hz, (4)20Hz, (3)42Hz, (2)98Hz, (1)188Hz LPF
	
	// Now configure gyros
	writeI2Cbyte(MPU60X0_DEFAULT_ADDRESS, MPU60X0_RA_GYRO_CONFIG, GYROFS500DEG);	// 500 deg/sec
}
#endif
