//***********************************************************
//* pid.c
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include "compiledefs.h"
#include <avr/pgmspace.h> 
#include <avr/io.h>
#include <stdbool.h>
#include <util/delay.h>
#include <stdlib.h>
#include "io_cfg.h"
#include "gyros.h"
#include "main.h"
#include "init.h"
#include "acc.h"
#include "imu.h"
#include "rc.h"
#include "mixer.h"
#include "isr.h"

//************************************************************
// Defines
//************************************************************

#define PID_SCALE 6					// Empirical amount to reduce the PID values by to make them most useful
#define STANDARDLOOP 3571.0			// T1 counts of 700Hz cycle time (2500000/700)

//************************************************************
// Notes
//************************************************************
//
// Servo output range is 2500 to 5000, centered on 3750.
// RC and PID values are added to this then rescaled at the the output stage to 1000 to 2000.
// As such, the maximum usable value that the PID section can output is +/-1250.
// So working backwards, prior to rescaling (/64) the max values are +/-80,000.
// Prior to this, PID_Gyro_I_actual has been divided by 32 so the values are now +/- 2,560,000
// However the I-term gain can be up to 127 which means the values are now limited to +/-20,157 for full scale authority.
// For reference, a constant gyro value of 50 would go full scale in about 1 second at max gain of 127 if incremented at 400Hz.
// This seems about right for heading hold usage.
//
// On the KK2.1 the gyros are configured to read +/-2000 deg/sec at full scale, or 16.4 deg/sec for each LSB value.  
// I divide that by 16 to give 0.976 deg/sec for each digit the gyros show. So "50" is about 48.8 degrees per second.
// 360 deg/sec would give a reading of 368 on the sensor calibration screen. Full stick is about 1000 or so. 
// So with no division of stick value by "Axis rate", full stick would equate to (1000/368 * 360) = 978 deg/sec. 
// With axis rate set to 2, the stick amount is quartered (250) or 244 deg/sec. A value of 3 would result in 122 deg/sec. 
//
// Stick rates: /64 (15.25), /32 (30.5), /16 (61*), /8 (122), /4 (244)
		
//************************************************************
// Prototypes
//************************************************************

void Sensor_PID(uint32_t period);
void Calculate_PID(void);

//************************************************************
// Code
//************************************************************

// PID globals
int16_t PID_Gyros[NUMBEROFAXIS];
int16_t PID_ACCs[NUMBEROFAXIS];
int32_t	IntegralGyro[NUMBEROFAXIS];			// PID I-terms (gyro) for each axis
int32_t PID_AvgGyro[NUMBEROFAXIS];					// Averaged gyro data

int16_t DynGain = 0;

// Run each loop to average gyro data
void Sensor_PID(uint32_t period)
{
	float tempf2 = 0;
	float factor = 0;						// Interval in seconds since the last loop
	int8_t	axis = 0;	
	int16_t	stick = 0;
	int32_t P1_temp = 0;
	int16_t	roll_actual = 0;
	
	// Cross-reference table for actual RCinput elements
	// Note that axes are reversed here with respect to their gyros
	// So why is AILERON different? Well on the KK hardware the sensors are arranged such that
	// RIGHT roll = +ve gyro, UP pitch = +ve gyro and LEFT yaw = +ve gyro.
	// However the way we have organised stick polarity, RIGHT roll and yaw are +ve, and DOWN elevator is too.
	// When combining with the gyro signals, the sticks have to be in the opposite polarity as the gyros.
	// As described above, pitch and yaw are already opposed, but roll needs to be reversed.

	int16_t	RCinputsAxis[NUMBEROFAXIS] = {-RCinputs[AILERON], RCinputs[ELEVATOR], RCinputs[RUDDER]};
	
	//************************************************************
	// Set up dynamic gain variable once per loop
	// N.B. Config.DynGainDiv = 2500 / Config.DynGain;
	//************************************************************

	// Channel controlling the dynamic gain
	DynGain = RxChannel[Config.DynGainSrc] - 2500; // 0-1250-2500 range

	// Scale 0 - 2500 down to 0 - Config.DynGain (%)
	DynGain = DynGain / Config.DynGainDiv;

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
	// Pitch has to be reversed
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

	// Zero RC inputs in pure camstab mode
	if ((Config.MixMode == CAMSTAB) && (Config.CamStab == ON))
	{
		RCinputsAxis[ROLL] = 0;
		RCinputsAxis[PITCH] = 0;
		RCinputsAxis[YAW] = 0;
	}

	//************************************************************
	// Per-cycle PID handling
	//************************************************************

	for (axis = 0; axis <= YAW; axis ++)
	{
		//************************************************************
		// Increment and limit gyro I-terms, handle heading hold nicely
		//************************************************************

		// Work out stick rate divider. 0 is slowest, 4 is fastest.
		// /64 (15.25), /32 (30.5), /16 (61*), /8 (122), /4 (244)
		stick = RCinputsAxis[axis] >> (4 - Config.Stick_Lock_rate - 2);

		//************************************************************
		// Magically correlate the I-term value with the loop rate.
		// This keeps the I-term and stick input constant over varying 
		// loop rates 
		//************************************************************
		
		P1_temp = gyroADC[axis] + stick;
		
		// Work out multiplication factor compared to standard loop time
		tempf2 = period;							// Promote int32_t to float
		factor = period/STANDARDLOOP;
		
		// Adjust gyro and stick values based on factor		
		tempf2 = P1_temp;							// Promote int32_t to float
		tempf2 = tempf2 * factor;
		P1_temp = (int32_t)tempf2;					// Demote to int32_t
		
		// Calculate I-term from gyro and stick data 
		// These may look similar, but they are constrained quite differently.
		IntegralGyro[axis] += P1_temp;

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
		// Sum gyro readings for P-terms for later averaging
		//************************************************************

		PID_AvgGyro[axis] += gyroADC[axis];
		
	} // for (axis = 0; axis <= YAW; axis ++)
}

void Calculate_PID(void)
{
	static	int16_t lastError[NUMBEROFAXIS];
	int32_t DifferentialGyro;				// Holds difference between last two errors (angular acceleration)
	int32_t PID_gyro_temp;
	int32_t PID_acc_temp;
	int32_t PID_Gyro_I_temp = 0;			// Temporary i-terms bound to max throw
	int32_t PID_Gyro_I_actual = 0;			// Actual unbound i-terms
	int8_t	axis;
	int32_t temp32 = 0;						// Needed for 32-bit dynamic gain calculations
	int32_t mult32 = 0;
	int32_t PID_Gyros_32;

	// Initialise arrays with gain values. Cludgy fix to reduce code space
	int8_t 	P_gain[NUMBEROFAXIS] = {Config.FlightMode[Config.Flight].Roll.P_mult, Config.FlightMode[Config.Flight].Pitch.P_mult, Config.FlightMode[Config.Flight].Yaw.P_mult};
	int8_t 	I_gain[NUMBEROFAXIS] = {Config.FlightMode[Config.Flight].Roll.I_mult, Config.FlightMode[Config.Flight].Pitch.I_mult, Config.FlightMode[Config.Flight].Yaw.I_mult};
	int8_t 	D_gain[NUMBEROFAXIS] = {Config.FlightMode[Config.Flight].Roll.D_mult, Config.FlightMode[Config.Flight].Pitch.D_mult, Config.FlightMode[Config.Flight].Yaw.D_mult};
	int8_t 	L_gain[NUMBEROFAXIS - 1] = {Config.FlightMode[Config.Flight].A_Roll_P_mult, Config.FlightMode[Config.Flight].A_Pitch_P_mult};

	// Only for roll and pitch acc trim
	int16_t	L_trim[2] =	{Config.Rolltrim[Config.Flight], Config.Pitchtrim[Config.Flight]};

	int16_t DynGain = 0;

	//************************************************************
	// PID loop
	//************************************************************

	for (axis = 0; axis <= YAW; axis ++)
	{
		//************************************************************
		// Get average gyro readings for P-terms
		//************************************************************

		gyroADC[axis] = (int16_t)(PID_AvgGyro[axis] / LoopCount);
		PID_AvgGyro[axis] = 0;					// Reset average		
		
		//************************************************************
		// Add in gyro Yaw trim
		//************************************************************

		if (axis == YAW)
		{
			PID_gyro_temp = (int32_t)(Config.FlightMode[Config.Flight].Yaw_trim << PID_SCALE);
		}
		// Reset PID_gyro variables to that data does not accumulate cross-axis
		else
		{
			PID_gyro_temp = 0;
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
		PID_gyro_temp += gyroADC[axis] * P_gain[axis];				// Multiply P-term (Max gain of 127)
		PID_gyro_temp = PID_gyro_temp * (int32_t)3;					// Multiply by 3

		// Gyro I-term
		PID_Gyro_I_actual = IntegralGyro[axis] * I_gain[axis];		// Multiply I-term (Max gain of 127)
		PID_Gyro_I_actual = PID_Gyro_I_actual >> 5;					// Divide by 32

		// Gyro D-term
		DifferentialGyro = (int16_t)(gyroADC[axis] - lastError[axis]);
		lastError[axis] = gyroADC[axis];
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
	
		PID_Gyros[axis] = (int16_t)((PID_gyro_temp + PID_Gyro_I_temp + DifferentialGyro) >> PID_SCALE);

		//************************************************************
		// Modify gains dynamically as required
		// Do this by mixing between (no PID) and PID
		// PID gains are not changed but the effect is the same
		//************************************************************

		// If dynamic gain set up 
		if (Config.DynGainSrc != NOCHAN)
		{
			temp32 = 0;
			mult32 = 0;

			// Promote to 32 bits and multiply
			temp32 = PID_Gyros[axis];
			mult32 = DynGain;					// Max (100%) - Current setting (0 to Config.DynGain)
			PID_Gyros_32 = temp32 * mult32;		// Scale to Config.DynGain (100% down to 0%)

			// Normalise the PID
			temp32 = (PID_Gyros_32 / (int32_t)Config.DynGain);

			// Cast back to native size
			PID_Gyros[axis] = (int16_t)temp32;
		}

		//************************************************************
		// Calculate acc error from angle data (roll and pitch only)
		//************************************************************

		// Autolevel mode (Use IMU to calculate attitude) for roll and pitch only
		if ((Flight_flags & (1 << AutoLevel)) && (axis < YAW)) 
		{
			PID_acc_temp = angle[axis] - L_trim[axis];				// Offset angle with trim
			PID_acc_temp *= L_gain[axis];							// P-term of accelerometer (Max gain of 127)
			PID_ACCs[axis] = (int16_t)(PID_acc_temp >> 8);			// Accs need much less scaling
		}
		else
		{
			PID_ACCs[axis] = 0;										// Ensure these are zeroed when autolevel OFF
		}
	} // PID loop

	// Offset Autolevel trims in failsafe mode
	if ((Config.FailsafeType == ADVANCED) && (Flight_flags & (1 << Failsafe)) && (Config.CamStab == OFF))
	{
		PID_ACCs[ROLL] += (Config.FailsafeAileron * 10);
		PID_ACCs[PITCH] += (Config.FailsafeElevator * 10);
	}

}
