//***********************************************************
//* imu.c
//*
//* Based partly on the IMU from the open-sourced MultiWii project
//*
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include "compiledefs.h"
#include <avr/io.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include "io_cfg.h"
#include "acc.h"
#include "gyros.h"
#include "main.h"
#include <avr/pgmspace.h> 
#include "menu_ext.h"

//************************************************************
// Prototypes
//************************************************************

void getEstimatedAttitude(uint16_t period);
void UpdateIMUvalues(void);

//************************************************************
// 	Defines
//************************************************************

/* Set the Low Pass Filter factor for ACC */
// Time constant T = 1/ (2*PI*f)
// factor (filter coefficient) = T / (T + dt) where dt is the loop period or 1 / Looprate(Hz) = 2.5e-3(s)
// 1Hz = 64, 5Hz = 13, 100Hz = 1.6,  Infinite = 1
// Increasing ACC_LPF_FACTOR would reduce ACC noise, but would increase ACC lag time
// Set to zero if you do not want filter at all, otherwise 8 is a typical number

/* Set the Gyro Weight for Gyro/Acc complementary filter */
// Increasing GYR_CMPF_FACTOR would reduce and delay Acc influence on the output of the filter*/
// 300 is the default for aeroplane mode, and 800 is good for camstab

/* Gyro scaling factor determination */
// IDG650 has 440 deg/sec on the 4.5x outputs (KK2 default) and 2000 deg/sec on the XOUT outputs
// The output we use has 2.27mV/deg/s. So for +/-440 deg/s that is 440 * 2.27e-3 = +/-1.0V
// On the MPU, Vref is 2.45V for full-scale 1024 bits. +/-1.0V translates to 1/2.45 = 0.408 * 1024 = +/-418 digits.
// So the acc has a span of 2 x 418 = 836 (+/-418). These are the highest values that we have to deal with.
//
// OpenAero2's natural tick timer is 2.5MHz or 400ns, so intervals are measured in these units.
// A max value of 418 represents 440 deg/s. At a tick level that max is 440/2500000 = 1.672e-4 deg/tick.
// There are 6250 ticks per 400Hz loop cycle - the interval that will be used in practice.
// So +/-418 will indicate (1.672e-4 * 6250) +/-1.045 deg/interval at the 440 deg/s maximum. (0.0182 rad/interval)
//
// So to work out the GYRO_SCALE we need to work out how to turn this gyro data into radians/interval or degrees/interval.
// deltaGyroAngle = gyroADC[ROLL] * deltaTime;
// The above code does the conversion. gyroADC[ROLL] has the values +/-418 and deltaTime is the interval.
// We want deltaGyroAngle to be 1.045 degree (0.0182r) with 418 for gyroADC[ROLL].
// This makes deltaTime 0.0182r / 418 = 4.354e-5 or 1.045 / 418 = 2.5e-3 for degrees.
// 
// Now deltaTime = (CurrentTime - PreviousTime) * GYRO_SCALE, so GYRO_SCALE = 4.354e-5 / 6250, or 2.5e-3 / 6250
// GYRO_SCALE = 4.354e-5 / 6250 = 6.966e-9 for radians or 2.5e-3 / 6250 = 4.0e-7 for degrees
// 
// While the above gives accurate output in degrees or radias, I chose to increase GYRO_SCALE until its
// arbitary scale matches the raw output of the accelerometers. This gives us the best resolution, but
// the output is not usable in degrees without rescaling.

//#define GYRO_SCALE	0.000000007f 	// for conversion to radians
//#define GYRO_SCALE	0.0000004f 		// for conversion to degrees
#ifdef KK21
#define GYRO_SCALE	0.0000008f 			// for conversion to the same scale as the accs
#else
#define GYRO_SCALE	0.0000010f 			// for KK2.0
#endif

/* Scaling factor for acc to read in Gs */
#define acc_1G		125					// Z-axis full scale (+/- 1G) is 249 so I guess 1G is half of that
// MW 1.9 values
#define acc_1_4G_SQ 30625				// (1.4 * acc_1G) * (1.4 * acc_1G)
#define acc_0_6G_SQ 5625				// (0.6 * acc_1G) * (0.6 * acc_1G)
// MW2.3 values
#define acc_1_15G_SQ 20664				// (1.15 * acc_1G) * (1.15 * acc_1G)
#define acc_0_85G_SQ 11289				// (0.85 * acc_1G) * (0.85 * acc_1G)

// AVRGCC defines M_PI as this
// #define M_PI		3.14159265358979323846
#define CONV_DEGREES (float)(18000.0f / M_PI)	// For 0.01 deg/bit accuracy
#define ONE_EIGHTY 18000
#define NINETY 9000
#define DELTA_FACTOR 50							// deltaGyroAngle is 50 times smaller than the angle value
#define DELTA_180 (ONE_EIGHTY/DELTA_FACTOR)		// 180 degrees in the same scale as deltaGyroAngle
#define DELTA_90 (NINETY/DELTA_FACTOR)			// 90 degrees in the same scale as deltaGyroAngle

//#define CONV_DEGREES (float)(180.0f / M_PI)	// For 1 deg/bit accuracy
//#define ONE_EIGHTY 180 						// For 0.01 deg/bit accuracy

// Notes:
// Pitch should have a range of +/-90 degrees. 
// After you pitch past vertical (90 degrees) your roll and yaw value should swing 180 degrees. 
// A pitch value of 100 degrees is measured as a pitch of 80 degrees and inverted flight (roll = 180 degrees). 
// Another example is a pitch of 180 degrees (upside down). This is measured as a level pitch (0 degrees) and a roll of 180 degrees. 
//
//************************************************************
// Code
//************************************************************

float	GYR_CMPF_FACTOR;
float	INV_GYR_CMPF_FACTOR;
float 	accSmooth[NUMBEROFAXIS];
int16_t	angle[2]; 			// Attitude

void getEstimatedAttitude(uint16_t period)
{
	static float deltaGyroAngle[NUMBEROFAXIS] = {0.0f,0.0f,0.0f};
	float 		deltaTime, tempf;
	int16_t		roll_sq, pitch_sq, yaw_sq;
	uint8_t		axis;
	uint16_t	AccMag = 0;

	//************************************************************
	// Set up IMU
	//************************************************************
	
	// Reset IMU 
	if (Config.Main_flags & (1 << FirstTimeIMU))
	{
		deltaTime = 0.0f;
		Config.Main_flags &= ~(1 << FirstTimeIMU);
		
		// Reset accumulating variables
		for (axis = 0; axis < NUMBEROFAXIS; axis++) 
		{	
			accSmooth[axis] = 0;
			deltaGyroAngle[axis] = 0;
		}
	}
	// Scale gyro signal to angle
	else
	{
		tempf = (float)period;
		deltaTime = tempf * GYRO_SCALE;	
	}

	//************************************************************
	// Process sensor data
	//************************************************************

	// Smooth Acc signals and estimate angle from gyro
	for (axis = 0; axis < NUMBEROFAXIS; axis++) 
	{
		// Acc LPF
		if (Config.Acc_LPF > 1)
		{
			// Acc LPF
			accSmooth[axis] = ((accSmooth[axis] * (float)(Config.Acc_LPF - 1)) - (float)(accADC[axis])) / Config.Acc_LPF;
		}
		else
		{
			// Use raw accADC[axis] as source for acc values
			accSmooth[axis] =  accADC[axis];
		}

		// Estimate angle via gyros
		// Flip sign when inverted (>90 right or <-90 left, aka top 180 degrees)
		// This keeps the CF happy as the gyros and accs are always the same sign regardless of orientation
		if (accADC[YAW] < 0)
		{
			deltaGyroAngle[axis] -= (float)gyroADC[axis] * deltaTime;
		}
		else
		{
			deltaGyroAngle[axis] += (float)gyroADC[axis] * deltaTime;
		}
	}

	//************************************************************
	// Calculate acceleration magnitude
	//************************************************************
	
	// This works perfectly as long as ACC_Z is calibrated to have =/- values (+/-125)
	roll_sq = (accADC[ROLL] * accADC[ROLL]);
	pitch_sq = (accADC[PITCH] * accADC[PITCH]) ;
	yaw_sq = (accADC[YAW] * accADC[YAW]);

	AccMag = (uint16_t)(roll_sq + pitch_sq + yaw_sq);

	//************************************************************
	// Apply complementary filter (Gyro drift correction)
	//************************************************************
	// If accel magnitude >1.4G or <0.6G => we neutralize the effect of accelerometers in the angle estimation.
	// To do that, we just skip the filter temporarily and use the old angle estimation code.
	// Note that this equation is a cheat to save doing a square root on the right-hand side.
	// The equation is really just mag^2 = x^2 + y^2 + z^2.
	// 125^2 or 15625 represents 1G (standing still). 
	//	MW2.3 values have been updated to +/- 0.15G (up from +/-0.4G)

	// Region of true CF-based operation (gyros + accs) - While under normal G.
	if ((AccMag > acc_0_85G_SQ) && (AccMag < acc_1_15G_SQ))
	{ 
		// Complementary filter
		deltaGyroAngle[ROLL] = ((deltaGyroAngle[ROLL] * GYR_CMPF_FACTOR) - accSmooth[ROLL]) * INV_GYR_CMPF_FACTOR;
		deltaGyroAngle[PITCH] = ((deltaGyroAngle[PITCH] * GYR_CMPF_FACTOR) - accSmooth[PITCH]) * INV_GYR_CMPF_FACTOR;

		// Calculate the roll and pitch angles properly then convert to degrees x 100
		tempf = atan(deltaGyroAngle[PITCH] / (float)sqrt(roll_sq + yaw_sq));
		angle[PITCH]  = (int16_t)(tempf * CONV_DEGREES);

		tempf = atan(deltaGyroAngle[ROLL]  / (float)sqrt(pitch_sq + yaw_sq));
		angle[ROLL]  = (int16_t)(tempf * CONV_DEGREES);
	}

	// Use simple IMU when under unusual acceleration
	// deltaGyroAngle[] is 50 times smaller than angle[]
	// So we need to compensate here to make them equal
	// Note that the above adjustments when inverted are not needed for gyro-based angles
	else
	{
		angle[ROLL] = (int16_t)(deltaGyroAngle[ROLL] * DELTA_FACTOR);
		angle[PITCH] = (int16_t)(deltaGyroAngle[PITCH] * DELTA_FACTOR);
	}

	// The following code changes the (CW - 0,90,0,-90) and (CCW - 0,-90,0,90,0) deg to 0-90-180 deg
	// It will snap between 180 and -180 deg when 100% inverted
	if (accADC[YAW] < 0)
	{
		// Roll
		if (accADC[ROLL] > 0)	// Right hemisphere
		{
			angle[ROLL] = (ONE_EIGHTY - angle[ROLL]);
		}
		else					// Left hemisphere
		{
			angle[ROLL] = (-ONE_EIGHTY - angle[ROLL]);
		}
	}
}

void UpdateIMUvalues(void)
{
	// Recalculate CF factors
	GYR_CMPF_FACTOR = (int16_t)Config.CF_factor * 10;
	INV_GYR_CMPF_FACTOR = (1.0f / (GYR_CMPF_FACTOR + 1.0f));
}

