//***********************************************************
//* gyros.c
//* 
//* Read and calibrate the gyroscopes.
//* Manage the different board orientations.
//* gyroADC[] holds the raw values
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
#include "imu.h"
#include "eeprom.h"
#include "mixer.h"
#include "rc.h"

//************************************************************
// Prototypes
//************************************************************

void ReadGyros(void);
void CalibrateGyrosFast(void);
bool CalibrateGyrosSlow(void);
void get_raw_gyros(void);

//************************************************************
// Defines
//************************************************************

#define CAL_TIMEOUT	5				// Calibration timeout
#define GYRODIV	4					// Divide by 16 for 2000 deg/s
#define CAL_STABLE_TIME 200			// Calibration stable timeout
#define GYROS_STABLE 1				// Minimum gyro error
#define SECOND_TIMER 19531			// Unit of timing for seconds
#define GYROFS2000DEG 0x18			// 2000 deg/s full scale
#define GYROFS500DEG 0x08			// 500 deg/s full scale
#define GYROFS250DEG 0x00			// 250 deg/s full scale

//***********************************************************
// ROLL, PITCH, YAW mapping for alternate orientation modes
//***********************************************************

// This is the order to return data in ROLL, PITCH, YAW order
const int8_t Gyro_RPY_Order[NUMBEROFORIENTS][NUMBEROFAXIS] PROGMEM = 
{
	//	 Model referenced
	// 	 ROLL, PITCH, YAW
	{ROLL, PITCH, YAW}, // Up/Back (Normal)
	{PITCH, ROLL, YAW}, // Up/Left
	{ROLL, PITCH, YAW}, // Up/Front (Aft)
	{PITCH, ROLL, YAW}, // Up/Right (Sideways)
	
	{YAW, PITCH, ROLL}, // Back/Down (PitchUp)
	{YAW, ROLL, PITCH}, // Back/Left
	{YAW, PITCH, ROLL}, // Back/Up
	{YAW, ROLL, PITCH}, // Back/Right
	
	{ROLL, PITCH, YAW}, // Down/Back (Upside down)
	{PITCH, ROLL, YAW}, // Down/Right
	{ROLL, PITCH, YAW}, // Down/Front
	{PITCH, ROLL, YAW}, // Down/Left
	
	{YAW, PITCH, ROLL}, // Front/Down
	{YAW, ROLL, PITCH}, // Front/Right
	{YAW, PITCH, ROLL}, // Front/Up
	{YAW, ROLL, PITCH}, // Front/Left
	
	{PITCH, YAW, ROLL}, // Left/Down
	{ROLL, YAW, PITCH}, // Left/Front
	{PITCH, YAW, ROLL}, // Left/Up
	{ROLL, YAW, PITCH}, // Left/Back
	
	{PITCH, YAW, ROLL}, // Right/Down (Vertical)
	{ROLL, YAW, PITCH}, // Right/Back
	{PITCH, YAW, ROLL}, // Right/Up
	{ROLL, YAW, PITCH}, // Right/Front
};

// These are the polarities to return them to the default
const int8_t Gyro_Pol[NUMBEROFORIENTS][NUMBEROFAXIS] PROGMEM = 
{
	//	 Model referenced
	// 	 ROLL, PITCH, YAW
	{1,1,1},	// Up/Back (Normal)
	{-1,1,1},	// Up/Left
	{-1,-1,1},	// Up/Front (Aft)
	{1,-1,1},	// Up/Right (Sideways)
	
	{-1,1,1},	// Back/Down (PitchUp)
	{-1,1,-1},	// Back/Left
	{-1,-1,-1}, // Back/Up
	{-1,-1,1},	// Back/Right
	
	{1,-1,-1},	// Down/Back (Upside down)
	{-1,- 1,-1},// Down/Right
	{-1,1,-1},	// Down/Front
	{1,1,-1},	// Down/Left
	
	{1,-1,1},	// Front/Down
	{1,-1,-1},	// Front/Right
	{1,1,-1},	// Front/Up
	{1,1,1},	// Front/Left
	
	{-1,-1,1},	// Left/Down
	{-1,-1,-1}, // Left/Front
	{1,-1,-1},	// Left/Up
	{1,-1,1},	// Left/Back
	
	{1,1,1},	// Right/Down (Vertical)
	{1,1,-1},	// Right/Back
	{-1,1,-1},	// Right/Up
	{-1,1,1},	// Right/Front
};

//************************************************************
// Code
//************************************************************

int16_t gyroADC_raw[NUMBEROFAXIS];		// Holds raw Gyro data
int16_t gyroADC[NUMBEROFAXIS];			// Holds Gyro data - always in RPY order (Combined - for rest of system)
int16_t gyroADCalt[NUMBEROFAXIS];		// Holds Gyro data - always in RPY order (Combined - for IMU)
int16_t gyroADC_P1[NUMBEROFAXIS];		// Holds temp Gyro data - always in RPY order (P1)
int16_t gyroADC_P2[NUMBEROFAXIS];		// Holds temp Gyro data - always in RPY order (P2)

//***************************************************************
// Fill gyroADC[] and gyroADCalt[] with RPY data appropriate to
// the board orientation and users' P1 referencing setting
//
// Remove offsets and change polarity
// Ensure the the correct zero is removed when working across board orientations.
// {ROLL, PITCH, YAW}, // Up/Back (Normal)
// {YAW, PITCH, ROLL}, // Back/Down (PitchUp) (Roll comes from Yaw gyro and needs Yaw's zero, Yaw comes from Roll gyro and needs Roll's zero)
//
// Note that when MODEL or normal VTOL, gyroADC_P1 = gyroADC_P2.
// Only when EARTH-oriented TS is gyroADC_P1 unique.
//
//***************************************************************

void ReadGyros(void)					// Conventional orientation
{
	uint8_t i;
	int16_t temp1, temp2, temp3;

	get_raw_gyros();					// Updates gyroADC_P1[] and gyroADC_P2[]

	for (i = 0; i < NUMBEROFAXIS; i++)	
	{
		// Only need to do this if the orientations differ
		if (Config.P1_Reference != NO_ORIENT)
		{
			// P1 alternate (original) orientation. Swap zeros so that they match.
			temp1 = (gyroADC_P1[i] - Config.gyroZero_P1[i]) * (int8_t)pgm_read_byte(&Gyro_Pol[Config.Orientation_P1][i]);

			// P2 orientation
			temp2 = (gyroADC_P2[i] - Config.gyroZero_P2[i]) * (int8_t)pgm_read_byte(&Gyro_Pol[Config.Orientation_P2][i]);

			// Merge the two gyros per transition percentage
			temp3 = scale32(temp1, (100 - transition)) + scale32(temp2, transition); // Sum the two values

			// Gyro alt is always per orientation
			gyroADCalt[i] = temp3;

			// If the P1 reference is MODEL, always use the same gyros as P2
			if (Config.P1_Reference == MODEL)
			{
				// Use P2 orientation
				gyroADC[i] = temp2;	
			}
			
			// Otherwise use the merged orientation (EARTH reference).
			else
			{
				// Use merged orientation
				gyroADC[i] = temp3;	
			}
		}
		// Single-orientation models
		else
		{
			// Change polarity using P2 orientation by default
			gyroADC[i] = (gyroADC_P2[i] - Config.gyroZero_P2[i]) * (int8_t)pgm_read_byte(&Gyro_Pol[Config.Orientation_P2][i]);	
				
			// Copy to alternate set of gyro values
			gyroADCalt[i] = gyroADC[i];
		}
	}
}

//***************************************************************
// Fill gyroADC_P1[] and gyroADC_P2[] with RPY data appropriate to 
// the board orientation.
// Nice as it would be, we cannot remove zeros here as this is the
// routine used by the zeroing calibration routine. Chicken | Egg.
// We also cannot merge P1 and P2 here as each have their own zeros.
//***************************************************************

void get_raw_gyros(void)
{
	uint8_t i;
	uint8_t Gyros[6];

	// Get the i2c data from the MPU6050
	readI2CbyteArray(MPU60X0_DEFAULT_ADDRESS,MPU60X0_RA_GYRO_XOUT_H,(uint8_t *)Gyros,6);

	// Reassemble data into gyroADC array and down-sample to reduce resolution and noise
	gyroADC_raw[PITCH] = (Gyros[0] << 8) + Gyros[1];
	gyroADC_raw[ROLL] = (Gyros[2] << 8) + Gyros[3];
	gyroADC_raw[YAW] = (Gyros[4] << 8) + Gyros[5];

	// Reorient the data as per the board orientation	
	for (i = 0; i < NUMBEROFAXIS; i++)
	{
		// Rearrange the sensors for both orientations
		gyroADC_P1[i] = gyroADC_raw[(int8_t)pgm_read_byte(&Gyro_RPY_Order[Config.Orientation_P1][i])] >> GYRODIV;
		gyroADC_P2[i] = gyroADC_raw[(int8_t)pgm_read_byte(&Gyro_RPY_Order[Config.Orientation_P2][i])] >> GYRODIV;
	}
}

//***************************************************************
// Calibration routines
//***************************************************************

void CalibrateGyrosFast(void)
{
	uint8_t i;
	
	// Work out which orientation we are calibrating.
	// Only need to do this if the orientations differ.
	// Just do P2 if orientations the same.
	// Will not save new calibration when different and not firmly in P1 or p2.
	if ((transition > 95) || (Config.P1_Reference == NO_ORIENT))
	{
		// Clear gyro zeros for the orientation that we are calibrating
		memset(&Config.gyroZero_P2[ROLL],0,(sizeof(int16_t) * NUMBEROFAXIS));

		// Calculate average over 32 reads
		for (i = 0; i < 32; i++)
		{
			get_raw_gyros();				// Updates gyroADC_P1/P2[] with the correct orientation-based RPY

			Config.gyroZero_P2[ROLL] 	+= gyroADC_P2[ROLL];
			Config.gyroZero_P2[PITCH] 	+= gyroADC_P2[PITCH];
			Config.gyroZero_P2[YAW] 	+= gyroADC_P2[YAW];
		}

		// Average readings for all axis
		for (i = 0; i < NUMBEROFAXIS; i++)
		{
			Config.gyroZero_P2[i] 	= (Config.gyroZero_P2[i] >> 5);
		}
	}
	// P1
	else if (transition <= 5)
	{
		// Clear gyro zeros for the orientation that we are calibrating
		memset(&Config.gyroZero_P1[ROLL],0,(sizeof(int16_t) * NUMBEROFAXIS));

		// Calculate average over 32 reads
		for (i = 0; i < 32; i++)
		{
			get_raw_gyros();				// Updates gyroADC_P1/P2[] with the correct orientation-based RPY

			Config.gyroZero_P1[ROLL] 	+= gyroADC_P1[ROLL];
			Config.gyroZero_P1[PITCH] 	+= gyroADC_P1[PITCH];
			Config.gyroZero_P1[YAW] 	+= gyroADC_P1[YAW];
		}

		// Average readings for all axis
		for (i = 0; i < NUMBEROFAXIS; i++)
		{
			Config.gyroZero_P1[i] 	= (Config.gyroZero_P1[i] >> 5);	// Divide by 32
		}
	}
	
	Save_Config_to_EEPROM();
}

bool CalibrateGyrosSlow(void)
{
	float 		GyroSmooth[NUMBEROFAXIS];
	int16_t		GyroOld[NUMBEROFAXIS] = {0,0,0};
	uint16_t	Stable_counter = 0;	
	uint16_t	Gyro_timeout = 0;
	uint8_t		axis;
	uint8_t		Gyro_seconds = 0;
	uint8_t		Gyro_TCNT2 = 0;
	bool		Gyros_Stable = false;

	// Populate Config.gyroZero[] with ballpark figures
	// This makes slow calibrate setting much more quickly
	CalibrateGyrosFast();	
	
	// Optimise starting point for each board
	for (axis = 0; axis < NUMBEROFAXIS; axis++)
	{
		// Work out which orientation we are calibrating
		// Only need to do this if the orientations differ
		if ((transition > 95) || (Config.P1_Reference == NO_ORIENT))
		{
			GyroSmooth[axis] = Config.gyroZero_P2[axis];
		}
		else
		{
			GyroSmooth[axis] = Config.gyroZero_P1[axis];	
		}		
	}
	
	// Wait until gyros stable. Timeout after CAL_TIMEOUT seconds
	while (!Gyros_Stable && ((Gyro_seconds <= CAL_TIMEOUT)))
	{
		// Update status timeout
		Gyro_timeout += (uint8_t)(TCNT2 - Gyro_TCNT2);
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
			
			// See if changing
			if (GyroOld[axis] != (int16_t)GyroSmooth[axis])
			{
				Gyros_Stable = false;
				Stable_counter = 0;
			}
		
			// Save old reading
			GyroOld[axis] = (int16_t)GyroSmooth[axis];
		}
		
		// Increment stable counter to measure how long we are still
		Stable_counter++;
		
		// If stable for 5 seconds, do a quick calibrate
		if (Stable_counter > CAL_STABLE_TIME)
		{
			Gyros_Stable = true;	
			CalibrateGyrosFast();		
		}
		
		_delay_ms(1);

		// Otherwise the original saved values are used
	}
	
	// Return success or failure
	return(Gyros_Stable);
}

//***************************************************************
// Set up the MPU6050 (Gyro)
//***************************************************************

void init_i2c_gyros(void)
{
	// First, configure the MPU6050
	writeI2Cbyte(MPU60X0_DEFAULT_ADDRESS, MPU60X0_RA_PWR_MGMT_1, 0x01); 			// Gyro X clock, awake
	
	// Make INT pin open-drain so that we can connect it straight to the MPU
	writeI2Cbyte(MPU60X0_DEFAULT_ADDRESS, MPU60X0_RA_INT_PIN_CFG, 0x40);			// INT output is open-drain
	
	// MPU6050's internal LPF. Values are 0x06 = 5Hz, (5)10Hz, (4)21Hz, (3)44Hz, (2)94Hz, (1)184Hz LPF, (0)260Hz
	// Software's values are 0 to 6 = 5Hz to 260Hz, so numbering is reversed here.
	writeI2Cbyte(MPU60X0_DEFAULT_ADDRESS, MPU60X0_RA_CONFIG, (6 - Config.MPU6050_LPF));
	
	// Now configure gyros
	writeI2Cbyte(MPU60X0_DEFAULT_ADDRESS, MPU60X0_RA_GYRO_CONFIG, GYROFS2000DEG);	// 2000 deg/sec
}
