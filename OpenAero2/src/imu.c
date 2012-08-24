//***********************************************************
//* imu.c
//***********************************************************
//* Simplified IMU based on "Complementary Filter"
//* Inspired by http://starlino.com/imu_guide.html
//*
//* The following ideas was used in this project:
//* 1) Rotation matrix: http://en.wikipedia.org/wiki/Rotation_matrix
//* 2) Small-angle approximation: http://en.wikipedia.org/wiki/Small-angle_approximation
//* 3) C. Hastings approximation for atan2()
//* 4) Optimization tricks: http://www.hackersdelight.org/
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
int16_t _atan2(float y, float x);

//************************************************************
// Defines
//************************************************************

//******advanced users settings *******************
/* Set the Low Pass Filter factor for ACC */
/* Increasing this value would reduce ACC noise, but would increase ACC lag time */
/* Comment this out if you do not want filter at all.*/
/* Default WMC value: 8*/
#define ACC_LPF_FACTOR 8

/* Set the Gyro Weight for Gyro/Acc complementary filter */
/* Increasing this value would reduce and delay Acc influence on the output of the filter*/
/* Default WMC value: 300*/
#define GYR_CMPF_FACTOR 310.0f

//****** end of advanced users settings *************

// _atan2 define
#define fp_is_neg(val) ((((uint8_t*)&val)[3] & 0x80) != 0)

#define INV_GYR_CMPF_FACTOR (1.0f / (GYR_CMPF_FACTOR+ 1.0f))
#define GYRO_SCALE (1.0f/200e6f)

// Small angle approximation
#define ssin(val) (val)
#define scos(val) 1.0f

// Scaling factor for acc
#define acc_1G 125 			// Debug - Z-axis full scale (+/- 1G) is 249 so I guess 1G is half of that

//************************************************************
// Code
//************************************************************

int16_t	angle[2]; 			// Attitude
int16_t	accSmooth[3]; 		// Debug

void getEstimatedAttitude(void)
{
	static t_fp_vector GEstG;
	static uint16_t PreviousTime;

	GEstG.A[0] = 0;				// Debug - only need this once
	GEstG.A[1] = 0;
	GEstG.A[2] = 200;

	t_fp_vector EstG = GEstG;
	uint8_t axis;
	int16_t	AccMag = 0;
	uint16_t CurrentTime;
	float deltaGyroAngle;
	float deltaTime;

	CurrentTime= micros();
	deltaTime = (CurrentTime - PreviousTime) * GYRO_SCALE;
	PreviousTime = CurrentTime;

	// Initialization
	for (axis = 0; axis < 3; axis++) 
	{
	#if defined(ACC_LPF_FACTOR)
		// LPF for ACC values
		accSmooth[axis] = (accSmooth[axis] * (ACC_LPF_FACTOR - 1) + accADC[axis]) / ACC_LPF_FACTOR;
		AccMag += (accSmooth[axis] * 10 / acc_1G) * (accSmooth[axis] * 10 / acc_1G);

		// Use accSmooth[axis] as source for acc values
		#define ACC_VALUE accSmooth[axis]

	#else
		accSmooth[axis] = accADC[axis];
		AccMag = AccMag + ((accADC[axis] * 10) / acc_1G) * ((accADC[axis] * 10) / acc_1G);

		// Use accADC[axis] as source for acc values
		#define ACC_VALUE accADC[axis]

	#endif
	}

	// Rotate Estimated vector(s), ROLL
	deltaGyroAngle = gyroADC[ROLL] * deltaTime;
	EstG.V.Z = scos(deltaGyroAngle) * EstG.V.Z - ssin(deltaGyroAngle) * EstG.V.X;
	EstG.V.X = ssin(deltaGyroAngle) * EstG.V.Z + scos(deltaGyroAngle) * EstG.V.X;

	// Rotate Estimated vector(s), PITCH
	deltaGyroAngle = gyroADC[PITCH] * deltaTime;
	EstG.V.Y = scos(deltaGyroAngle) * EstG.V.Y + ssin(deltaGyroAngle) * EstG.V.Z;
	EstG.V.Z = -ssin(deltaGyroAngle) * EstG.V.Y + scos(deltaGyroAngle) * EstG.V.Z;

	// Rotate Estimated vector(s), YAW
	deltaGyroAngle = gyroADC[YAW] * deltaTime;
	EstG.V.X = scos(deltaGyroAngle) * EstG.V.X - ssin(deltaGyroAngle) * EstG.V.Y;
	EstG.V.Y = ssin(deltaGyroAngle) * EstG.V.X + scos(deltaGyroAngle) * EstG.V.Y;

	// Apply complimentary filter (Gyro drift correction)
	// If accel magnitude >1.4G or <0.6G => we neutralize the effect of accelerometers in the angle estimation.
	// To do that, we just skip filter, as EstV already rotated by Gyro
	if (!((36 > AccMag) || (AccMag > 196))) 
	{
		// Note that EstG.A[ROLL] is EstG.V.X, EstG.A[PITCH] is EstG.V.Y and EstG.A[YAW] is EstG.V.Z
		for (axis = 0; axis < 3; axis++)
		EstG.A[axis] = (EstG.A[axis] * GYR_CMPF_FACTOR + ACC_VALUE) * INV_GYR_CMPF_FACTOR;
	}

	// Attitude of the estimated vector
	angle[ROLL]=_atan2(EstG.V.X, EstG.V.Z);
	angle[PITCH] =_atan2(EstG.V.Y, EstG.V.Z);
	GEstG = EstG;
}

// Fast ATAN calculation
int16_t _atan2(float y, float x)
{
	float z = y / x;
	int16_t zi = abs((int16_t)(z * 100)); 
	int8_t y_neg = fp_is_neg(y);

	if (zi < 100)
	{
		if (zi > 10) 
		{
 		z = z / (1.0f + (0.28f * z * z));
		}

 		if (fp_is_neg(x)) 
		{
 			if (y_neg) 
			{
				z = (z - M_PI);
			}
		}
		else
		{
			z = z + M_PI;
		}
	} 
	else 
	{
		z = (M_PI / 2.0f) - z / ((z * z) + 0.28f);
		if (y_neg)
		{
			z -= M_PI;
		}
	}

	z *= ((180.0f / M_PI) * 10); 

	return z;
}
