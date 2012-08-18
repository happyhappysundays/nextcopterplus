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

#define MAX_I_SPAN 160000			// Servo range of 2500 * 64 to limit maximum influence of I-term

// PID constants
#define ITERM_LIMIT_RP 40000		// Max I-term sum for Roll/Pitch axis in STABILITY mode
#define ITERM_LIMIT_YAW 80000		// Max I-term sum for Yaw axis (Heading hold) (MAX_I_SPAN * 2 * 32 / 127)
#define ITERM_LIMIT_LEVEL 1200		// Max I-term sum for Roll/Pitch axis in AUTOLEVEL mode (5000 * 4 * 8 / 127)

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
int32_t	IntegralGyro[3];					// PID I-terms (gyro) for each axis

void Calculate_PID(void)
{
	static int16_t currentError[4];			// Used with lastError to keep track of D-Terms in PID calculations
	static	int16_t lastError[4];
	int16_t DifferentialGyro[3];			// Holds difference between last two errors (angular acceleration)
	int32_t PID_gyro_temp;
	int32_t PID_acc_temp;
	int32_t PID_Gyro_I_temp;
	int8_t	axis;
	int32_t	I_limts[3] = {ITERM_LIMIT_RP, ITERM_LIMIT_RP, ITERM_LIMIT_YAW};

	//************************************************************
	// Increment and limit I-terms, pre-calculate D-terms
	//************************************************************
	
	for (axis = 0; axis < YAW; axis ++)
	{
		// Reduce Gyro drift noise into the I-terms
		if ((gyroADC[axis] > GYRO_DEADBAND) || (gyroADC[axis] < -GYRO_DEADBAND)) 
		{
			IntegralGyro[axis] += gyroADC[axis]; 
		}

		// Anti wind-up limits
		if (IntegralGyro[axis] > I_limts[axis])
		{
			IntegralGyro[axis] = I_limts[axis];
		}
		else if (IntegralGyro[axis] < -I_limts[axis]) 
		{
			IntegralGyro[axis] = -I_limts[axis];
		}

		// D-terms
		currentError[axis] = gyroADC[axis];								
		DifferentialGyro[axis] = currentError[axis] - lastError[axis];
		lastError[axis] = currentError[axis];
	}

	//************************************************************
	// Calculate roll PID
	//************************************************************
	// Roll P-term
	PID_gyro_temp = gyroADC[ROLL];

	// Gyro PID terms
	PID_gyro_temp = PID_gyro_temp * Config.Roll.P_mult;			// Multiply P-term (Max gain of 127)
	PID_gyro_temp = PID_gyro_temp * 3;							// Multiply by 3

	PID_Gyro_I_temp = IntegralGyro[ROLL] * Config.Roll.I_mult;	// Multiply I-term (Max gain of 127)
	PID_Gyro_I_temp = PID_Gyro_I_temp >> 5;						// Divide by 8

	DifferentialGyro[ROLL] *= Config.Roll.D_mult;				// Multiply D-term by up to 127
	DifferentialGyro[ROLL] = DifferentialGyro[ROLL] << 4;		// Multiply by 16

	if (AutoLevel) // Autolevel mode (Use averaged accelerometer to calculate attitude)
	{
		// Acc P terms
		PID_acc_temp = AvgRoll * Config.A_level.P_mult;			// P-term of accelerometer (Max gain of 127)
		PID_ACCs[ROLL] = PID_acc_temp >> 2;						// Accs need much less scaling
	}

	// I-term limits
	if (PID_Gyro_I_temp > MAX_I_SPAN) 
	{
		PID_Gyro_I_temp = MAX_I_SPAN;
	}
	else if (PID_Gyro_I_temp < -MAX_I_SPAN) 
	{
		PID_Gyro_I_temp = -MAX_I_SPAN;	
	}

	// Sum Gyro P and D terms and rescale	
	PID_Gyros[ROLL] = (PID_gyro_temp + PID_Gyro_I_temp + DifferentialGyro[ROLL]) >> 6;

	//************************************************************
	// Calculate pitch PID
	//************************************************************
	// Pitch P-term
	PID_gyro_temp = gyroADC[PITCH];

	// Gyro PID terms
	PID_gyro_temp = PID_gyro_temp * Config.Pitch.P_mult;		// Multiply P-term (Max gain of 127)
	PID_gyro_temp = PID_gyro_temp * 3;							// Multiply by 3

	PID_Gyro_I_temp = IntegralGyro[PITCH] * Config.Pitch.I_mult;// Multiply I-term (Max gain of 127)
	PID_Gyro_I_temp = PID_Gyro_I_temp >> 5;						// Divide by 8

	DifferentialGyro[PITCH] *= Config.Pitch.D_mult;				// Multiply D-term by up to 127
	DifferentialGyro[PITCH] = DifferentialGyro[PITCH] << 4;		// Multiply by 16

	if (AutoLevel) // Autolevel mode (Use averaged accelerometer to calculate attitude)
	{
		// Acc P terms
		PID_acc_temp = AvgPitch * Config.A_level.P_mult;		// P-term of accelerometer (Max gain of 127)
		PID_ACCs[PITCH] = PID_acc_temp >> 2;					// Accs need much less scaling
	}

	// I-term limits
	if (PID_Gyro_I_temp > MAX_I_SPAN) 
	{
		PID_Gyro_I_temp = MAX_I_SPAN;
	}
	else if (PID_Gyro_I_temp < -MAX_I_SPAN) 
	{
		PID_Gyro_I_temp = -MAX_I_SPAN;	
	}

	// Sum Gyro P and D terms and rescale	
	PID_Gyros[PITCH] = (PID_gyro_temp + PID_Gyro_I_temp + DifferentialGyro[PITCH]) >> 6;

	//************************************************************
	// Calculate yaw PID
	//************************************************************
	// Yaw P-term
	PID_gyro_temp = gyroADC[YAW];

	// Gyro PID terms
	PID_gyro_temp = PID_gyro_temp * Config.Yaw.P_mult;				// Multiply P-term (Max gain of 127)
	PID_gyro_temp = PID_gyro_temp * 3;								// Multiply by 3

	PID_Gyro_I_temp = IntegralGyro[YAW] * Config.Yaw.I_mult;		// Multiply IntegralYaw by up to 127
	PID_Gyro_I_temp = PID_Gyro_I_temp >> 5;							// Divide by 32

	DifferentialGyro[YAW] = DifferentialGyro[YAW] * Config.Yaw.D_mult;// Multiply D-term by up to 127
	DifferentialGyro[YAW] = DifferentialGyro[YAW] << 4;				// Multiply by 16

	// I-term limits
	if (PID_Gyro_I_temp > MAX_I_SPAN) 
	{
		PID_Gyro_I_temp = MAX_I_SPAN;
	}
	else if (PID_Gyro_I_temp < -MAX_I_SPAN) 
	{
		PID_Gyro_I_temp = -MAX_I_SPAN;	
	}

	// Sum Gyro P, I and D terms and rescale
	PID_Gyros[YAW] = (-PID_gyro_temp - PID_Gyro_I_temp - DifferentialGyro[YAW]) >> 6;		
}
