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
#include "..\inc\rc.h"
#include "..\inc\mixer.h"
#include "..\inc\isr.h"

//************************************************************
// Defines
//************************************************************

#define GYRO_DEADBAND	5			// Region where no gyro input is added to I-term

//************************************************************
// Notes
//************************************************************
//
// Servo output range is 2500 to 5000, centered on 3750.
// RC and PID values are added to this then rescaled at the the output stage to 1000 to 2000.
// As such, the maximum usable value that the PID section can output is +/-1250.
// So working backwards, prior to rescaling (/64) the max values are +/-80,000.
// Prior to this, PID_Gyro_I_actual has been divided by 32 so the values are now +/- 2,560,000
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
int32_t	IntegralGyro[3];					// PID I-terms (gyro) for each axis

void Calculate_PID(void)
{
	static 	int32_t currentError[4];		// Used with lastError to keep track of D-Terms in PID calculations
	static	int32_t lastError[4];

	int32_t DifferentialGyro;				// Holds difference between last two errors (angular acceleration)
	int32_t PID_gyro_temp;
	int32_t PID_acc_temp;
	int32_t PID_Gyro_I_temp = 0;			// Temporary i-terms bound to max throw
	int32_t PID_Gyro_I_actual = 0;			// Actual unbound i-terms
	int8_t	axis;
	int32_t temp32 = 0;						// Needed for 32-bit dynamic gain calculations
	int32_t mult32 = 0;
	int32_t RC_Input_32;
	int32_t PID_Gyros_32;

	// Cross-ref for actual RCinput elements
	// I have no idea why pitch has to be reversed here...
	int16_t	RCinputsAxis[3] = {RCinputs[AILERON], -RCinputs[ELEVATOR], RCinputs[RUDDER]}; 

	// Initialise arrays with gain values. Cludgy fix to reduce code space
	int8_t 	P_gain[3] = {Config.FlightMode[Config.Flight].Roll.P_mult, Config.FlightMode[Config.Flight].Pitch.P_mult, Config.FlightMode[Config.Flight].Yaw.P_mult};
	int8_t 	I_gain[3] = {Config.FlightMode[Config.Flight].Roll.I_mult, Config.FlightMode[Config.Flight].Pitch.I_mult, Config.FlightMode[Config.Flight].Yaw.I_mult};
	int8_t 	D_gain[3] = {Config.FlightMode[Config.Flight].Roll.D_mult, Config.FlightMode[Config.Flight].Pitch.D_mult, Config.FlightMode[Config.Flight].Yaw.D_mult};
	int8_t 	L_gain[2] = {Config.FlightMode[Config.Flight].A_Roll_P_mult, Config.FlightMode[Config.Flight].A_Pitch_P_mult};

	int16_t	roll_actual = 0;
	int16_t temp16 = 0;


	//************************************************************
	// Set up dynamic gain variable once per loop
	// N.B. Config.DynGainDiv = 2500 / Config.DynGain;
	//************************************************************

	// Channel controlling the dynamic gain
	temp16 = RxChannel[Config.DynGainSrc] - 2500; // 0-1250-2500 range

	// Scale 0 - 2500 down to 0 - Config.DynGain (%)
	temp16 = temp16 / Config.DynGainDiv;

	//************************************************************
	// Un-mix ailerons from flaperons as required
	//************************************************************

	// If in AEROPLANE mixer mode and flaperons set up
	if ((Config.FlapChan != NOCHAN) && (Config.MixMode == AEROPLANE))
	{
		// Recreate actual roll signal from flaperons
		roll_actual = RCinputs[AILERON] + RCinputs[Config.FlapChan];
		RCinputsAxis[ROLL] = roll_actual >> 1;
	}

	// In in Flying Wing mode RCinputsAxis comes directly from RCinputs
	// Pitch has to be reversed (again)
	else if (Config.MixMode == FWING)
	{
		RCinputsAxis[ROLL] = RCinputs[AILERON];
		RCinputsAxis[PITCH] = -RCinputs[ELEVATOR];
	}

	// Otherwise roll is just roll...
	else
	{
		RCinputsAxis[ROLL] = RCinputs[AILERON];
	}

	//************************************************************
	// PID loop
	//************************************************************

	for (axis = 0; axis <= YAW; axis ++)
	{
		//************************************************************
		// Filter and calculate gyro error
		//************************************************************

		// Reduce Gyro drift noise before adding into I-term
		if ((gyroADC[axis] > -GYRO_DEADBAND) && (gyroADC[axis] < GYRO_DEADBAND)) 
		{
			gyroADC[axis] = 0;
		}

		// Calculate gyro error from gyro and stick data
		currentError[axis] = gyroADC[axis] - (RCinputsAxis[axis] >> 2);

		//************************************************************
		// Increment and limit gyro I-terms, handle heading hold nicely
		//************************************************************

		// Calculate I-term from gyro and stick data (scaled down by Config.Stick_Lock_rate)
		IntegralGyro[axis] += (gyroADC[axis] - (RCinputsAxis[axis] >> Config.Stick_Lock_rate));

		// Reset the I-terms when you need to adjust the I-term with RC
		// Note that the I-term is not constrained when no RC input is present.
		if (RCinputsAxis[axis] != 0)
		{
			if (IntegralGyro[axis] > Config.Raw_I_Constrain[axis])
			{
				IntegralGyro[axis] = Config.Raw_I_Constrain[axis];
			}
			if (IntegralGyro[axis] < -Config.Raw_I_Constrain[axis])
			{
				IntegralGyro[axis] = -Config.Raw_I_Constrain[axis];
			}
		}

		//************************************************************
		// Handle auto-centering of I-terms in Camstab autocenter mode
		// If no significant gyro input and IntegralGyro[axis] is non-zero, 
		// pull it back slowly.
		//************************************************************

		if ((Config.AutoCenter == ON) && (Config.CamStab == ON))
		{
			if (IntegralGyro[axis] > 0)
			{
				IntegralGyro[axis] --;
			}
			else if (IntegralGyro[axis] < 0)
			{	
				IntegralGyro[axis] ++;
			}
		}		

		//************************************************************
		// Calculate PID gains
		//************************************************************

		// Gyro P-term
		PID_gyro_temp = currentError[axis] * P_gain[axis];			// Multiply P-term (Max gain of 127)
		PID_gyro_temp = PID_gyro_temp * (int32_t)3;					// Multiply by 3

		// Gyro I-term
		PID_Gyro_I_actual = IntegralGyro[axis] * I_gain[axis];		// Multiply I-term (Max gain of 127)
		PID_Gyro_I_actual = PID_Gyro_I_actual >> 5;					// Divide by 32

		// Gyro D-term
		DifferentialGyro = (int16_t)(currentError[axis] - lastError[axis]);
		lastError[axis] = currentError[axis];
		DifferentialGyro *= D_gain[axis];							// Multiply D-term by up to 127
		DifferentialGyro = DifferentialGyro << 4;					// Multiply by 16

		//************************************************************
		// I-term output limits
		//************************************************************

		// Maximum 125% limit is full servo throw 
		if (PID_Gyro_I_actual > Config.Raw_I_Limits[axis]) 
		{
			PID_Gyro_I_temp = Config.Raw_I_Limits[axis];
		}
		else if (PID_Gyro_I_actual < -Config.Raw_I_Limits[axis]) 
		{
			PID_Gyro_I_temp = -Config.Raw_I_Limits[axis];	
		}
		else
		{
			PID_Gyro_I_temp = PID_Gyro_I_actual;
		}

		//************************************************************
		// Sum Gyro P and D terms and rescale
		//************************************************************
	
		PID_Gyros[axis] = (int16_t)((PID_gyro_temp + PID_Gyro_I_temp + DifferentialGyro) >> 6);

		//************************************************************
		// Modify gains dynamically as required
		// Do this by mixing between raw RC(no PID) and PID
		// PID gains are not changed but the effect is the same
		//************************************************************

		// If dynamic gain set up 
		if (Config.DynGainSrc != NOCHAN)
		{
			temp32 = 0;
			mult32 = 0;

			// Promote to 32 bits and multiply
			temp32 = PID_Gyros[axis];
			mult32 = 100 - temp16;				// Max (100%) - Current setting (0 to Config.DynGain)
			PID_Gyros_32 = temp32 * mult32;		// Scale to Config.DynGain (100% down to 0%)

			// Promote to 32 bits and multiply
			temp32 = RCinputsAxis[axis];
			mult32 = temp16;
			RC_Input_32 = temp32 * mult32;		// Scale to current setting (0% up to 100%)

			// Combine and normalise the PID vs. RC
			temp32 = ((PID_Gyros_32 - RC_Input_32) / (int32_t)Config.DynGain);

			// Cast back to native size
			PID_Gyros[axis] = (int16_t)temp32;
		}

		//************************************************************
		// Calculate acc error from angle data (roll and pitch only)
		//************************************************************

		// Autolevel mode (Use IMU to calculate attitude) for roll and pitch only
		if ((Flight_flags & (1 << AutoLevel)) && (axis < YAW)) 
		{
			PID_acc_temp = angle[axis];

			PID_acc_temp *= L_gain[axis];							// P-term of accelerometer (Max gain of 127)
			PID_ACCs[axis] = (int16_t)(PID_acc_temp >> 2);			// Accs need much less scaling
		}
		else
		{
			PID_ACCs[axis] = 0;										// Ensure these are zeroed when autolevel OFF
		}

	} // PID loop
}
