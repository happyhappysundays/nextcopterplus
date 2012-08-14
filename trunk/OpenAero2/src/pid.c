//***********************************************************
//* pid.c
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include <avr/io.h>
#include <stdbool.h>
#include <util/delay.h>
#include "..\inc\io_cfg.h"
#include "..\inc\gyros.h"
#include "..\inc\main.h"
#include "..\inc\init.h"
#include "..\inc\acc.h"

//************************************************************
// Defines
//************************************************************

// PID constants
#define ITERM_LIMIT_RP 250			// Max I-term sum for Roll/Pitch axis in normal modes
#define ITERM_LIMIT_YAW 500			// Max I-term sum for Yaw axis (Heading hold)
#define ITERM_LIMIT_LEVEL 250		// Max I-term sum for Roll/Pitch axis in AUTOLEVEL mode

//************************************************************
// Prototypes
//************************************************************

void Calculate_PID(void);

//************************************************************
// Code
//************************************************************

// PID globals
int16_t PID_Gyros[3];
int16_t PID_ACCs[3];
int32_t	IntegralaPitch;			// PID I-terms (acc.) for each axis
int32_t	IntegralaRoll;

void Calculate_PID(void)
{
	static int16_t currentError[4];			// Used with lastError to keep track of D-Terms in PID calculations
	static	int16_t lastError[4];
	int16_t DifferentialGyro;				// Holds difference between last two errors (angular acceleration)
	int32_t PID_gyro_temp;
	int32_t PID_acc_temp;
	int32_t PID_I_temp;

	// Calculate roll PID
	PID_gyro_temp = gyroADC[ROLL];

	// D-term
	currentError[ROLL] = PID_gyro_temp;								
	DifferentialGyro = currentError[ROLL] - lastError[ROLL];
	lastError[ROLL] = currentError[ROLL];

	if (AutoLevel) // Autolevel mode (Use averaged accelerometer to calculate attitude)
	{
		// Gyro PD terms
		PID_gyro_temp = PID_gyro_temp * Config.G_level.P_mult;		// Multiply P-term (Max gain of 256)
		PID_gyro_temp = PID_gyro_temp * 3;							// Multiply by 3, so max effective gain is 768
		DifferentialGyro *= Config.G_level.D_mult;					// Multiply D-term by up to 256

		// Acc PI terms
		PID_acc_temp = AvgRoll * Config.A_level.P_mult;				// P-term of accelerometer (Max gain of 256)
		IntegralaRoll += AvgRoll;									// Acc I-term
		if (IntegralaRoll > ITERM_LIMIT_LEVEL)  					// Anti wind-up limit check
		{
			IntegralaRoll = ITERM_LIMIT_LEVEL;
		}
		else if (IntegralaRoll < -ITERM_LIMIT_LEVEL)
		{
			IntegralaRoll = -ITERM_LIMIT_LEVEL;
		}
		PID_I_temp = IntegralaRoll * Config.A_level.I_mult;			// Multiply I-term (Max gain of 256)
		PID_I_temp = PID_I_temp >> 3;								// Divide by 8, so max effective gain is 16

		// Sum Gyro P and D terms + Acc P and I terms
		PID_Gyros[ROLL] = (PID_gyro_temp + DifferentialGyro) >> 6;	// Sum Gyro P and D terms and rescale
		PID_ACCs[ROLL] 	= (PID_acc_temp + PID_I_temp) >> 6;			// Sum  Acc P and I terms and rescale

	}
	else // Normal mode (Just use raw gyro errors to guess at attitude)
	{
		// Gyro PD terms
		PID_gyro_temp = PID_gyro_temp * Config.Roll.P_mult;			// Multiply P-term (Max gain of 256)
		DifferentialGyro *= Config.Roll.D_mult;						// Multiply D-term by up to 256
		PID_gyro_temp = PID_gyro_temp * 3;							// Multiply by 3, so max effective gain is 768

		// Sum	
		PID_Gyros[ROLL] = (PID_gyro_temp + DifferentialGyro) >> 6;	// Sum Gyro P and D terms and rescale
	}

	// Calculate pitch PID
	PID_gyro_temp = gyroADC[PITCH];

	currentError[PITCH] = PID_gyro_temp;							// D-term
	DifferentialGyro = currentError[PITCH] - lastError[PITCH];
	lastError[PITCH] = currentError[PITCH];

	if (AutoLevel) // Autolevel mode (Use averaged accelerometer to calculate attitude)
	{
		// Gyro PD terms
		PID_gyro_temp = PID_gyro_temp * Config.G_level.P_mult;		// Multiply P-term (Max gain of 256)
		PID_gyro_temp = PID_gyro_temp * 3;							// Multiply by 3, so max effective gain is 768
		DifferentialGyro *= Config.G_level.D_mult;					// Multiply D-term by up to 256

		// Acc PI terms
		PID_acc_temp = AvgPitch * Config.A_level.P_mult;			// P-term of accelerometer (Max gain of 256)
		IntegralaPitch += AvgPitch;									// Acc I-term
		if (IntegralaPitch > ITERM_LIMIT_LEVEL)   					// Anti wind-up limit check
		{
			IntegralaPitch = ITERM_LIMIT_LEVEL;
		}
		else if (IntegralaPitch < -ITERM_LIMIT_LEVEL) 
		{
			IntegralaPitch = -ITERM_LIMIT_LEVEL;
		}
		PID_I_temp = IntegralaPitch * Config.A_level.I_mult;		// Multiply I-term (Max gain of 256)
		PID_I_temp = PID_I_temp >> 3;								// Divide by 8, so max effective gain is 16

		// Sum Gyro P and I terms + Acc P and I terms
		PID_Gyros[PITCH] = (PID_gyro_temp + DifferentialGyro) >> 6;	// Sum Gyro P and D terms and rescale
		PID_ACCs[PITCH] = (PID_acc_temp + PID_I_temp) >> 6;			// Sum  Acc P and I terms and rescale

	}
	else // Normal mode (Just use raw gyro errors to guess at attitude)
	{
		// Gyro PD terms
		PID_gyro_temp = PID_gyro_temp * Config.Pitch.P_mult;		// Multiply P-term (Max gain of 256)
		DifferentialGyro *= Config.Pitch.D_mult;					// Multiply D-term by up to 256
		PID_gyro_temp = PID_gyro_temp * 3;							// Multiply by 3, so max effective gain is 768

		// Sum
		PID_Gyros[PITCH] = (PID_gyro_temp + DifferentialGyro) >> 6;	// Sum Gyro P and D terms and rescale
	}

	// Calculate yaw PID
	PID_gyro_temp = gyroADC[YAW];
	
	currentError[YAW] = PID_gyro_temp;								// D-term
	DifferentialGyro = currentError[YAW] - lastError[YAW];
	lastError[YAW] = currentError[YAW];

	PID_gyro_temp *= Config.Yaw.P_mult;								// Multiply P-term (Max gain of 256)
	DifferentialGyro *= Config.Yaw.D_mult;							// Multiply D-term by up to 256
	PID_gyro_temp = PID_gyro_temp * 3;

	// Sum
	PID_Gyros[YAW] = (PID_gyro_temp + DifferentialGyro) >> 6;		// Sum Gyro P and D terms and rescale
}
