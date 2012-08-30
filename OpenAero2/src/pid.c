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
#include "..\inc\imu.h"

//************************************************************
// Defines
//************************************************************

#define MAX_I_SPAN 80000			// Servo range of 1250 * 64 to limit maximum influence of I-term
#define GYRO_DEADBAND	5			// Region where no gyro input is added to I-term

//************************************************************
// Notes
//************************************************************
//
// Servo output range is 2500 to 5000, centered on 3750.
// RC and PID values are added to this then rescaled at the the output stage to 1000 to 2000.
// As such, the maximum usable value that the PID section can output is +/-1250.
// So working backwards, prior to rescaling (/64) the max values are +/-80,000.
// Prior to this, PID_Gyro_I_temp has been divided by 32 so the values are now +/- 2,560,000
// however the I-term gain can be up to 127 which means the values are now limited to +/-20,157 for full scale authority.
// For reference, a constant gyro value of 50 would go full scale in about 1 second at max gain of 127 if incremented at 400Hz.
// This seems about right for heading hold usage.

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

	//************************************************************
	// Increment and limit I-terms, pre-calculate D-terms
	//************************************************************

	for (axis = 0; axis <= YAW; axis ++)
	{
		// Reduce Gyro drift noise into the I-terms
		if ((gyroADC[axis] > GYRO_DEADBAND) || (gyroADC[axis] < -GYRO_DEADBAND)) 
		{
			IntegralGyro[axis] += gyroADC[axis]; 
		}

		// Anti wind-up limits
		if (IntegralGyro[axis] > Config.Raw_I_Limits[axis])
		{
			IntegralGyro[axis] = Config.Raw_I_Limits[axis];
		}
		else if (IntegralGyro[axis] < -Config.Raw_I_Limits[axis]) 
		{
			IntegralGyro[axis] = -Config.Raw_I_Limits[axis];
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
	PID_Gyro_I_temp = PID_Gyro_I_temp >> 5;						// Divide by 32

	DifferentialGyro[ROLL] *= Config.Roll.D_mult;				// Multiply D-term by up to 127
	DifferentialGyro[ROLL] = DifferentialGyro[ROLL] << 4;		// Multiply by 16

	if (AutoLevel) // Autolevel mode (Use averaged accelerometer to calculate attitude)
	{
		// Acc P terms
		PID_acc_temp = angle[ROLL] * Config.A_Roll_P_mult;			// P-term of accelerometer (Max gain of 127)
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
	PID_Gyro_I_temp = PID_Gyro_I_temp >> 5;						// Divide by 32

	DifferentialGyro[PITCH] *= Config.Pitch.D_mult;				// Multiply D-term by up to 127
	DifferentialGyro[PITCH] = DifferentialGyro[PITCH] << 4;		// Multiply by 16

	if (AutoLevel) // Autolevel mode (Use averaged accelerometer to calculate attitude)
	{
		// Acc P terms
		PID_acc_temp = angle[PITCH] * Config.A_Pitch_P_mult;		// P-term of accelerometer (Max gain of 127)
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
