//***********************************************************
//* imu.c
//*
//* Based partly on the IMU from the open-sourced MultiWii project
//*
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include <avr/io.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include "..\inc\io_cfg.h"
#include "..\inc\acc.h"
#include "..\inc\gyros.h"
#include "..\inc\main.h"

//************************************************************
// Prototypes
//************************************************************

void getEstimatedAttitude(void);
void UpdateIMUvalues(void);

int16_t _atan2(float y, float x);


//************************************************************
// 	Defines
//************************************************************

/* Set the Low Pass Filter factor for ACC */
// Time constant T = 1/ (2*PI*f)
// factor = T / (T + dt) where dt is the loop period or 1 / Looprate 2.5e-3
// 1Hz = 64, 5Hz = 13, 100Hz = 1.6,  Infinite = 1
// Increasing ACC_LPF_FACTOR would reduce ACC noise, but would increase ACC lag time
// Set to zero if you do not want filter at all, otherwise 8 is a typical number

//#define ACC_LPF_FACTOR 8 		// Hard code the number for now

/* Set the Gyro Weight for Gyro/Acc complementary filter */
// Increasing GYR_CMPF_FACTOR would reduce and delay Acc influence on the output of the filter*/
// 300 is the default for aeroplane mode, and 800 is good for camstab

//#define GYR_CMPF_FACTOR 800.0f	// Hard code the number for now
//#define INV_GYR_CMPF_FACTOR (1.0f / (GYR_CMPF_FACTOR + 1.0f))

// Gyro scaling factor determination
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
// While the above gives accurate output in degrees or radias, I choose to increase GYRO_SCALE until its
// arbitary scale matches the raw output of the accelerometers. This gives us the best resolution, but
// the output is not usable in degrees without rescaling.

//#define GYRO_SCALE	0.000000007f 	// for conversion to radians
//#define GYRO_SCALE	0.0000004f 		// for conversion to degrees
#define GYRO_SCALE	0.000001f 			// for conversion to the same scale as the accs

// Scaling factor for acc to read in Gs
//#define acc_1G 125 						// Z-axis full scale (+/- 1G) is 249 so I guess 1G is half of that
//#define acc_1G 125 
//#define acc_Z1G 196 

//************************************************************
// Code
//************************************************************

int8_t	ACC_LPF_FACTOR;		// User-set Acc low-pass filter
float	GYR_CMPF_FACTOR;
float	INV_GYR_CMPF_FACTOR;
bool 	FirstTimeIMU;

int16_t	angle[2]; 			// Attitude

// Debug
int16_t	AccMag;

void getEstimatedAttitude(void)
{
	static float deltaGyroAngle[3] = {0.0f,0.0f,0.0f};
	static uint32_t PreviousTime = 0;
	static float accSmooth[3];

	uint8_t		axis;
	//int16_t		AccMag = 0;
	uint32_t 	CurrentTime;
	float 		deltaTime;

	int16_t		temp;

	// Get global timestamp
	// The first calculation has no PreviousTime to measure from, so zero and move on.
	if (FirstTimeIMU)
	{
		deltaTime = 0.0f;
		FirstTimeIMU = false;
		PreviousTime = ticker_32;
	}
	else
	{
		CurrentTime = ticker_32;
		deltaTime = (CurrentTime - PreviousTime);
		deltaTime = deltaTime * GYRO_SCALE;
		PreviousTime = CurrentTime;
	}

	AccMag = 0; // Debug

	// Initialization
	for (axis = 0; axis < 3; axis++) 
	{
		if (ACC_LPF_FACTOR > 0)
		{
			// LPF for ACC values
			accSmooth[axis] = ((accSmooth[axis] * (ACC_LPF_FACTOR - 1)) + accADC[axis]) / ACC_LPF_FACTOR;

			// Precalc to speed up
			temp = (int16_t)((accSmooth[axis] * 10) / (int16_t)Config.AccMax[axis]);

			// Check for any unusual acceleration		
			AccMag += temp * temp;
			
			//if (axis == 2) AccMag -= acc_Z1G; // Offset for 1G at neutral for Z
		}
		else
		{
			// Precalc to speed up
			temp = ((accADC[axis] * 10) / (int16_t)Config.AccMax[axis]);

			// Check for any unusual acceleration	
			AccMag += temp * temp;

			//if (axis == 2) AccMag -= acc_Z1G; // Offset for 1G at neutral for Z

			// Use raw accADC[axis] as source for acc values
			accSmooth[axis] =  accADC[axis];
		}

		// Estimate angle via gyros
		deltaGyroAngle[axis] += (float)gyroADC[axis] * deltaTime;
	}

	// Apply complimentary filter (Gyro drift correction)
	// If accel magnitude >1.4G or <0.6G => we neutralize the effect of accelerometers in the angle estimation.
	// To do that, we just skip the filter temporarily
	//
	// Note that this equation is a cheat to save doing a square root on the right-hand side.
	// (SQR)36 is 6 (0.6G) and (SQR)196 is 14 (1.4G). The accADC numbers on the right have already been multiplied by 10
	// so the equation is really just mag^2 = x^2 + y^2 + z^2.

	//if (!((36 > AccMag) || (AccMag > 196))) 

	if (!((-196 > AccMag) || (AccMag > 196)))
	{
		for (axis = 0; axis < 2; axis++)
		{
			deltaGyroAngle[axis] = ((deltaGyroAngle[axis] * GYR_CMPF_FACTOR) - accSmooth[axis]) * INV_GYR_CMPF_FACTOR;
		}
	}
/*
	// Calculate roll/pitch angles
	if (AccMag > 0) // Model is upside down
	{
		// Left side
		if (angle[ROLL] < 0)
		{
			angle[ROLL]	= -Config.AccMin[ROLL] - (Config.AccMin[ROLL] + (int16_t)deltaGyroAngle[ROLL]);
		}
		// Right side
		else
		{
			angle[ROLL]	= Config.AccMax[ROLL] + (Config.AccMax[ROLL] - (int16_t)deltaGyroAngle[ROLL]);
		}
	}
	else	// Model is rightside up
	{
		angle[ROLL] = (int16_t)deltaGyroAngle[ROLL];
	}
*/
	angle[ROLL] = (int16_t)deltaGyroAngle[ROLL];
	angle[PITCH] = (int16_t)deltaGyroAngle[PITCH];

/*
Pitch should have a range of +/-90 degrees. 
After you pitch past vertical (90 degrees) your roll and yaw value should swing 180 degrees. 
A pitch value of 100 degrees is measured as a pitch of 80 degrees and inverted flight (roll = 180 degrees). 
Another example is a pitch of 180 degrees (upside down). This is measured as a level pitch (0 degrees) and a roll of 180 degrees. 
*/

	//angle[ROLL] 	= _atan2((float)accADC[PITCH], (float)accADC[YAW]);
	//angle[PITCH]	= _atan2((float)accADC[ROLL], (float)accADC[YAW]);

	//angle[ROLL] 	= _atan2((float)accADC[YAW], (float)accADC[PITCH]);
	//angle[PITCH]	= _atan2((float)accADC[YAW], (float)accADC[ROLL]);
}

void UpdateIMUvalues(void)
{
	ACC_LPF_FACTOR = Config.Acc_LPF;
	GYR_CMPF_FACTOR = Config.CF_factor * 10;
	INV_GYR_CMPF_FACTOR = (1.0f / (GYR_CMPF_FACTOR + 1.0f));
}
/*
#define fp_is_neg(val) ((((unsigned char *)&val)[0] & 0x80) != 0)

int16_t _atan2(float y, float x)
{
    float z = y / x;
    int16_t zi = abs(((int16_t) (z * 100)));
    int8_t y_neg = fp_is_neg(y);

    if (zi < 100) 
	{
		if (zi > 10)
		    z = z / (1.0f + 0.28f * z * z);
		if (fp_is_neg(x)) 
		{
		    if (y_neg)
			z -= M_PI;
		    else
			z += M_PI;
		}
    } 
	else 
	{
		z = (M_PI / 2.0f) - z / (z * z + 0.28f);
		if (y_neg)
	    	z -= M_PI;
    }

    //z *= (180.0f / M_PI * 10);
	z *= (180.0f / M_PI);

    return z;
}
*/

/*
Pitch Angle: arctan[A_X / sqrt(A_Y^2 + A_Z^2)]
Roll Angle: arctan[A_Y / sqrt(A_X^2 + A_Z^2)]


  angle[ROLL]  =  _atan2(EstG.V.X , sqrt(EstG.V.Y*EstG.V.Y + EstG.V.Z * EstG.V.Z)) ;
  angle[PITCH] =  _atan2(EstG.V.Y , sqrt(EstG.V.X*EstG.V.X + EstG.V.Z * EstG.V.Z)) ;

  
            float g = sqrtf(EstG.V.X * EstG.V.X + EstG.V.Y * EstG.V.Y + EstG.V.Z * EstG.V.Z);
            angle[ROLL] = _atan2f(EstG.V.X, EstG.V.Z);
            angle[PITCH] = asinf(EstG.V.Y / -g) * (180.0f / M_PI * 10.0f);


//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}


*/
