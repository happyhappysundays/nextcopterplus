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
#define ITERM_LIMIT_RP 1000			// Max I-term sum for Roll/Pitch axis in STABILITY mode
#define ITERM_LIMIT_YAW 2000		// Max I-term sum for Yaw axis (Heading hold)
#define ITERM_LIMIT_LEVEL 250		// Max I-term sum for Roll/Pitch axis in AUTOLEVEL mode

#define GYRO_DEADBAND	5			// Region where no gyro input is added to I-term

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
int32_t	IntegralaPitch;						// PID I-terms (acc.) for each axis
int32_t	IntegralaRoll;
int32_t	IntegralgPitch;						// PID I-terms (gyro) for each axis
int32_t	IntegralgRoll;
int32_t	IntegralYaw;

void Calculate_PID(void)
{
	static int16_t currentError[4];			// Used with lastError to keep track of D-Terms in PID calculations
	static	int16_t lastError[4];
	int16_t DifferentialGyro;				// Holds difference between last two errors (angular acceleration)
	int32_t PID_gyro_temp;
	int32_t PID_acc_temp;
	int32_t PID_Acc_I_temp;
	int32_t PID_Gyro_I_temp;

	//************************************************************
	// Calculate roll PID
	//************************************************************
	// Roll P-term
	PID_gyro_temp = gyroADC[ROLL];

	// Roll I-term - Reduce Gyro drift noise into the I-term
	if ((gyroADC[ROLL] > GYRO_DEADBAND) && (gyroADC[ROLL] < -GYRO_DEADBAND)) 
	{
		IntegralgRoll += gyroADC[ROLL]; 
	}

	// Anti wind-up limits
	if (IntegralgRoll > ITERM_LIMIT_RP)
	{
		IntegralgRoll = ITERM_LIMIT_RP;
	}
	else if (IntegralgRoll < -ITERM_LIMIT_RP) 
	{
		IntegralgRoll = -ITERM_LIMIT_RP;
	}

	// D-term
	currentError[ROLL] = PID_gyro_temp;								
	DifferentialGyro = currentError[ROLL] - lastError[ROLL];
	lastError[ROLL] = currentError[ROLL];

	if (AutoLevel) // Autolevel mode (Use averaged accelerometer to calculate attitude)
	{
		// Gyro PID terms
		PID_gyro_temp = PID_gyro_temp * Config.G_level.P_mult;		// Multiply P-term (Max gain of 127)
		PID_gyro_temp = PID_gyro_temp * 3;							// Multiply by 3, so max effective gain is 768

		PID_Gyro_I_temp = IntegralgRoll * Config.G_level.I_mult;	// Multiply I-term (Max gain of 127)
		PID_Gyro_I_temp = PID_Gyro_I_temp >> 3;						// Divide by 8

		DifferentialGyro *= Config.G_level.D_mult;					// Multiply D-term by up to 127
		DifferentialGyro = DifferentialGyro << 4;					// Multiply by 16

		// Acc PI terms
		PID_acc_temp = AvgRoll * Config.A_level.P_mult;				// P-term of accelerometer (Max gain of 127)

		IntegralaRoll += AvgRoll;									// Acc I-term
		if (IntegralaRoll > ITERM_LIMIT_LEVEL)  					// Anti wind-up limit check
		{
			IntegralaRoll = ITERM_LIMIT_LEVEL;
		}
		else if (IntegralaRoll < -ITERM_LIMIT_LEVEL)
		{
			IntegralaRoll = -ITERM_LIMIT_LEVEL;
		}

		PID_Acc_I_temp = IntegralaRoll * Config.A_level.I_mult;		// Multiply I-term (Max gain of 127)
		PID_Acc_I_temp = PID_Acc_I_temp >> 3;						// Divide by 8, so max effective gain is 16

		// Sum Gyro P, I and D terms + Acc P and I terms
		PID_Gyros[ROLL] = (PID_gyro_temp + PID_Gyro_I_temp + DifferentialGyro) >> 6;
		PID_ACCs[ROLL] 	= (PID_acc_temp + PID_Acc_I_temp) >> 2;		// Accs need much less scaling

	}
	else // Normal mode (Just use raw gyro errors to guess at attitude)
	{
		// Gyro PID terms
		PID_gyro_temp = PID_gyro_temp * Config.Roll.P_mult;			// Multiply P-term (Max gain of 127)
		PID_gyro_temp = PID_gyro_temp * 3;							// Multiply by 3

		PID_Gyro_I_temp = IntegralgRoll * Config.Roll.I_mult;		// Multiply I-term (Max gain of 127)
		PID_Gyro_I_temp = PID_Gyro_I_temp >> 3;						// Divide by 8

		DifferentialGyro *= Config.Roll.D_mult;						// Multiply D-term by up to 127
		DifferentialGyro = DifferentialGyro << 4;					// Multiply by 16

		// Sum Gyro P and D terms and rescale	
		PID_Gyros[ROLL] = (PID_gyro_temp + PID_Gyro_I_temp + DifferentialGyro) >> 6;
	}

	//************************************************************
	// Calculate pitch PID
	//************************************************************
	// Pitch P-term
	PID_gyro_temp = gyroADC[PITCH];

	// Pitch I-term - Reduce Gyro drift noise into the I-term
	if ((gyroADC[PITCH] > GYRO_DEADBAND) && (gyroADC[PITCH] < -GYRO_DEADBAND)) 
	{
		IntegralgPitch += gyroADC[PITCH]; 
	}

	// Anti wind-up limits
	if (IntegralgPitch > ITERM_LIMIT_RP) 
	{
		IntegralgPitch = ITERM_LIMIT_RP;
	}
	else if (IntegralgPitch < -ITERM_LIMIT_RP)
	{
		IntegralgPitch = -ITERM_LIMIT_RP;
	}

	// D-term
	currentError[PITCH] = PID_gyro_temp;
	DifferentialGyro = currentError[PITCH] - lastError[PITCH];
	lastError[PITCH] = currentError[PITCH];

	if (AutoLevel) // Autolevel mode (Use averaged accelerometer to calculate attitude)
	{
		// Gyro PID terms
		PID_gyro_temp = PID_gyro_temp * Config.G_level.P_mult;		// Multiply P-term (Max gain of 127)
		PID_gyro_temp = PID_gyro_temp * 3;							// Multiply by 3, so max effective gain is 768

		PID_Gyro_I_temp = IntegralgPitch * Config.G_level.I_mult;	// Multiply I-term (Max gain of 127)
		PID_Gyro_I_temp = PID_Gyro_I_temp >> 3;						// Divide by 8

		DifferentialGyro *= Config.G_level.D_mult;					// Multiply D-term by up to 127
		DifferentialGyro = DifferentialGyro << 4;					// Multiply by 16

		// Acc PI terms
		PID_acc_temp = AvgPitch * Config.A_level.P_mult;			// P-term of accelerometer (Max gain of 127)

		IntegralaPitch += AvgPitch;									// Acc I-term
		if (IntegralaPitch > ITERM_LIMIT_LEVEL)  					// Anti wind-up limit check
		{
			IntegralaPitch = ITERM_LIMIT_LEVEL;
		}
		else if (IntegralaPitch < -ITERM_LIMIT_LEVEL)
		{
			IntegralaPitch = -ITERM_LIMIT_LEVEL;
		}

		PID_Acc_I_temp = IntegralaPitch * Config.A_level.I_mult;	// Multiply I-term (Max gain of 127)
		PID_Acc_I_temp = PID_Acc_I_temp >> 3;						// Divide by 8, so max effective gain is 16

		// Sum Gyro P, I and D terms + Acc P and I terms
		PID_Gyros[PITCH] = (PID_gyro_temp + PID_Gyro_I_temp + DifferentialGyro) >> 6;	
		PID_ACCs[PITCH] = (PID_acc_temp + PID_Acc_I_temp) >> 2;		// Accs need much less scaling

	}
	else // Normal mode (Just use raw gyro errors to guess at attitude)
	{
		// Gyro PID terms
		PID_gyro_temp = PID_gyro_temp * Config.Pitch.P_mult;		// Multiply P-term (Max gain of 127)
		PID_gyro_temp = PID_gyro_temp * 3;							// Multiply by 3

		PID_Gyro_I_temp = IntegralgPitch * Config.Pitch.I_mult;		// Multiply I-term (Max gain of 127)
		PID_Gyro_I_temp = PID_Gyro_I_temp >> 3;						// Divide by 8

		DifferentialGyro *= Config.Pitch.D_mult;					// Multiply D-term by up to 127
		DifferentialGyro = DifferentialGyro << 4;					// Multiply by 16

		// Sum Gyro P and D terms and rescale	
		PID_Gyros[PITCH] = (PID_gyro_temp + PID_Gyro_I_temp + DifferentialGyro) >> 6;
	}

	//************************************************************
	// Calculate yaw PID
	//************************************************************
	// Yaw P-term
	PID_gyro_temp = gyroADC[YAW];

	// Pitch I-term - Reduce Gyro drift noise into the I-term
	if ((gyroADC[YAW] > GYRO_DEADBAND) && (gyroADC[YAW] < -GYRO_DEADBAND)) 
	{
		IntegralYaw += gyroADC[YAW]; 
	}

	// Anti wind-up limits
	if (IntegralYaw > ITERM_LIMIT_YAW) 
	{
		IntegralYaw = ITERM_LIMIT_YAW;
	}
	else if (IntegralYaw < -ITERM_LIMIT_YAW) 
	{
		IntegralYaw = -ITERM_LIMIT_YAW;	
	}

	// D-term
	currentError[YAW] = PID_gyro_temp;	
	DifferentialGyro = currentError[YAW] - lastError[YAW];
	lastError[YAW] = currentError[YAW];

	// Gyro PID terms
	PID_gyro_temp = PID_gyro_temp * Config.Yaw.P_mult;				// Multiply P-term (Max gain of 127)
	PID_gyro_temp = PID_gyro_temp * 3;								// Multiply by 3

	PID_Gyro_I_temp = IntegralYaw * Config.Yaw.I_mult;				// Multiply IntegralYaw by up to 127
	PID_Gyro_I_temp = PID_Gyro_I_temp >> 3;							// Divide by 8

	DifferentialGyro = DifferentialGyro * Config.Yaw.D_mult;		// Multiply D-term by up to 127
	DifferentialGyro = DifferentialGyro << 4;						// Multiply by 16

	// Sum Gyro P, I and D terms and rescale
	PID_Gyros[YAW] = (-PID_gyro_temp - PID_Gyro_I_temp - DifferentialGyro) >> 6;		
}
