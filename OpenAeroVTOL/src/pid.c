//***********************************************************
//* pid.c
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include <avr/io.h>
#include <stdbool.h>
#include <util/delay.h>
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
// However the I-term gain can be up to 127 which means the values are now limited to +/-20,157 for full scale authority.
// For reference, a constant gyro value of 50 would go full scale in about 1 second at max gain of 127 if incremented at 400Hz.
// This seems about right for heading hold usage.
//
// On the KK2.1 the gyros are configured to read +/-2000 deg/sec at full scale, or 16.4 deg/sec for each LSB value.  
// I divide that by 128 to give 0.128 deg/sec for each digit the gyros show. So "50" is only 6.4 degrees per second.
// 360 deg/sec would give a reading of 2809 on the sensor calibration screen. Full stick is about 1250 or so. 
// So with no division of stick value by "Axis rate", full stick would equate to (1250/2809 * 360) = 160 deg/sec. 
// With axis rate set to 2, the stick amount is quartered (312.5) or 40 deg/sec. A value of 3 would result in 20 deg/sec. 
//
// 90 to 720 deg/sec is a good range so to achieve this we need to both divide and multiply the stick.
// Stick * 2 would give 320 deg/sec, *4 = 640 deg/sec
//
// Therefore, usable operations on the stick throw are *4 (640), *2 (320), *1 (160), /2 (80) 
//

//************************************************************
// Prototypes
//************************************************************

void Calculate_PID(void);

//************************************************************
// Code
//************************************************************

// PID globals for each [Profile] and [axis]
int16_t PID_Gyros[FLIGHT_MODES][NUMBEROFAXIS];
int16_t PID_ACCs[FLIGHT_MODES][NUMBEROFAXIS];
int32_t	IntegralGyro[FLIGHT_MODES][NUMBEROFAXIS];	// PID I-terms (gyro) for each axis

void Calculate_PID(void)
{
	static	int16_t	old_z = 0; 				// Historical Z-axis acc value

	int32_t PID_gyro_temp1;					// P1
	int32_t PID_gyro_temp2;					// P2
	int32_t PID_acc_temp1 = 0;				// P1
	int32_t PID_acc_temp2 = 0;				// P2
	int32_t PID_Gyro_I_actual1 = 0;			// Actual unbound i-terms P1
	int32_t PID_Gyro_I_actual2 = 0;			// P2
	int32_t temp32 = 0;						// Needed for 32-bit dynamic gain calculations
	int32_t mult32 = 0;
	int32_t PID_Gyros_32;
	int8_t	axis;
	int16_t	stick;

	// Cross-ref for actual RCinput elements
	int16_t	RCinputsAxis[NUMBEROFAXIS] = {RCinputs[AILERON], RCinputs[ELEVATOR], RCinputs[RUDDER]}; 

	// Initialise arrays with gain values.
	int8_t 	P_gain[FLIGHT_MODES][NUMBEROFAXIS] = 
		{
			{Config.FlightMode[P1].Roll.P_mult, Config.FlightMode[P1].Pitch.P_mult, Config.FlightMode[P1].Yaw.P_mult},
		 	{Config.FlightMode[P2].Roll.P_mult, Config.FlightMode[P2].Pitch.P_mult, Config.FlightMode[P2].Yaw.P_mult}
		};

	int8_t 	I_gain[FLIGHT_MODES][NUMBEROFAXIS] = 
		{
			{Config.FlightMode[P1].Roll.I_mult, Config.FlightMode[P1].Pitch.I_mult, Config.FlightMode[P1].Yaw.I_mult},
			{Config.FlightMode[P2].Roll.I_mult, Config.FlightMode[P2].Pitch.I_mult, Config.FlightMode[P2].Yaw.I_mult}
		};

	int8_t 	L_gain[FLIGHT_MODES][NUMBEROFAXIS] = 
		{
			{Config.FlightMode[P1].A_Roll_P_mult, Config.FlightMode[P1].A_Pitch_P_mult, Config.FlightMode[P1].A_Zed_P_mult},
			{Config.FlightMode[P2].A_Roll_P_mult, Config.FlightMode[P2].A_Pitch_P_mult, Config.FlightMode[P2].A_Zed_P_mult}
		};

	int16_t DynamicScale = 0;

	//************************************************************
	// Set up dynamic gain variable once per loop if requested
	// N.B. Config.DynGainDiv = 2500 / Config.DynGain;
	//************************************************************

	if (Config.DynGainSrc != NOCHAN)
	{
		// Channel controlling the dynamic gain
		DynamicScale = RxChannel[Config.DynGainSrc] - 2500; // 0-1250-2500 range

		// Scale 0 - 2500 down to 0 - Config.DynGain (%)
		DynamicScale = DynamicScale / Config.DynGainDiv;
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

		//************************************************************
		// Increment and limit gyro I-terms, handle heading hold nicely
		//************************************************************

		// Work out stick rate divider/multiplier
		// /16 (10), /8 (20), /4 (40), /2 (80), *1 (160), *2 (320), *4 (640)
		if (Config.Stick_Lock_rate <= 4)
		{
			stick = RCinputsAxis[axis] >> (4 - Config.Stick_Lock_rate);
		}
		else
		{
			stick = RCinputsAxis[axis] << (Config.Stick_Lock_rate - 4);
		}

		// Calculate I-term from gyro and stick data 
		IntegralGyro[P1][axis] += (gyroADC[axis] - stick);
		IntegralGyro[P2][axis] += (gyroADC[axis] - stick);

		// Reset the I-terms when you need to adjust the I-term with RC
		// Note that the I-term is not constrained when no RC input is present.
		if (RCinputsAxis[axis] != 0)
		{
			if (IntegralGyro[P1][axis] > Config.Raw_I_Constrain[P1][axis])
			{
				IntegralGyro[P1][axis] = Config.Raw_I_Constrain[P1][axis];
			}
			if (IntegralGyro[P1][axis] < -Config.Raw_I_Constrain[P1][axis])
			{
				IntegralGyro[P1][axis] = -Config.Raw_I_Constrain[P1][axis];
			}

			if (IntegralGyro[P2][axis] > Config.Raw_I_Constrain[P2][axis])
			{
				IntegralGyro[P2][axis] = Config.Raw_I_Constrain[P2][axis];
			}
			if (IntegralGyro[P2][axis] < -Config.Raw_I_Constrain[P2][axis])
			{
				IntegralGyro[P2][axis] = -Config.Raw_I_Constrain[P2][axis];
			}
		}

		//************************************************************
		// Calculate PID gains
		//************************************************************

		// Gyro P-term													// Profile P1
		PID_gyro_temp1 = gyroADC[axis] * P_gain[P1][axis];				// Multiply P-term (Max gain of 127)
		PID_gyro_temp1 = PID_gyro_temp1 * (int32_t)3;					// Multiply by 3

		// Gyro I-term
		PID_Gyro_I_actual1 = IntegralGyro[P1][axis] * I_gain[P1][axis];	// Multiply I-term (Max gain of 127)
		PID_Gyro_I_actual1 = PID_Gyro_I_actual1 >> 5;					// Divide by 32

		// Gyro P-term
		PID_gyro_temp2 = gyroADC[axis] * P_gain[P2][axis];				// Profile P2
		PID_gyro_temp2 = PID_gyro_temp2 * (int32_t)3;

		// Gyro I-term
		PID_Gyro_I_actual2 = IntegralGyro[P2][axis] * I_gain[P2][axis];
		PID_Gyro_I_actual2 = PID_Gyro_I_actual2 >> 5;

		//************************************************************
		// I-term output limits
		//************************************************************

		// P1 limits
		if (PID_Gyro_I_actual1 > Config.Raw_I_Limits[P1][axis]) 
		{
			PID_Gyro_I_actual1 = Config.Raw_I_Limits[P1][axis];
		}
		else if (PID_Gyro_I_actual1 < -Config.Raw_I_Limits[P1][axis]) 
		{
			PID_Gyro_I_actual1 = -Config.Raw_I_Limits[P1][axis];	
		}
		else
		{
			PID_Gyro_I_actual1 = PID_Gyro_I_actual1;
		}

		// P2 limits
		if (PID_Gyro_I_actual2 > Config.Raw_I_Limits[P2][axis]) 
		{
			PID_Gyro_I_actual2 = Config.Raw_I_Limits[P2][axis];
		}
		else if (PID_Gyro_I_actual2 < -Config.Raw_I_Limits[P2][axis]) 
		{
			PID_Gyro_I_actual2 = -Config.Raw_I_Limits[P2][axis];	
		}
		else
		{
			PID_Gyro_I_actual2 = PID_Gyro_I_actual2;
		}

		//************************************************************
		// Sum Gyro P and I terms and rescale
		//************************************************************

		PID_Gyros[P1][axis] = (int16_t)((PID_gyro_temp1 + PID_Gyro_I_actual1) >> 6);
		PID_Gyros[P2][axis] = (int16_t)((PID_gyro_temp2 + PID_Gyro_I_actual2) >> 6);

		//************************************************************
		// Modify gains dynamically as required.
		// Do this by scaling PID based on the current percentage of 
		// the user-set maximum Dynamic Gain.
		// PID gains themselves are not changed but the effect is the same
		//************************************************************

		// If dynamic gain set up 
		if (Config.DynGainSrc != NOCHAN)
		{
			temp32 = 0;
			mult32 = 0;

			// Promote to 32 bits and multiply
			temp32 = PID_Gyros[P1][axis];
			mult32 = DynamicScale;				// Max (100%). temp16 = 0 to Config.DynGain (Max. 100)
			PID_Gyros_32 = temp32 * mult32;		// Scale to Config.DynGain (100% down to 0%) = 0x up to 100x

			// Scale back to 0% to 100% gyro 
			// If Config.DynGain is 50(%) the gyro gain is reduced by 50%
			temp32 = PID_Gyros_32 / 100;

			// Cast back to native size
			PID_Gyros[P1][axis] = (int16_t)temp32;

			// Promote to 32 bits and multiply
			temp32 = PID_Gyros[P2][axis];
			mult32 = DynamicScale;				// Max (100%). temp16 = 0 to Config.DynGain (Max. 100)
			PID_Gyros_32 = temp32 * mult32;		// Scale to Config.DynGain (100% down to 0%) = 0x up to 100x

			// Scale back to 0% to 100% gyro 
			// If Config.DynGain is 50(%) the gyro gain is reduced by 50%
			temp32 = PID_Gyros_32 / 100;

			// Cast back to native size
			PID_Gyros[P2][axis] = (int16_t)temp32;
		}

		//************************************************************
		// Calculate acc error from angle data (roll and pitch only)
		//************************************************************

		// Autolevel mode (Use IMU to calculate attitude) for roll and pitch only
		if (axis < YAW)
		{
			PID_acc_temp1 = angle[axis];
			PID_acc_temp2 = PID_acc_temp1;

			PID_acc_temp1 *= L_gain[P1][axis];							// P-term of accelerometer (Max gain of 127)
			PID_ACCs[P1][axis] = (int16_t)(PID_acc_temp1 >> 2);			// Accs need much less scaling

			PID_acc_temp2 *= L_gain[P2][axis];							// Same for P2
			PID_ACCs[P2][axis] = (int16_t)(PID_acc_temp2 >> 2);	
		}

	} // PID loop

	//************************************************************
	// Calculate a delta-Z value 
	//************************************************************

	PID_acc_temp1 = old_z - accADC[YAW];
	PID_acc_temp2 = PID_acc_temp1;


	// P1
	PID_acc_temp1 *= L_gain[P1][YAW];		// Multiply P-term (Max gain of 127)

	if (PID_acc_temp1 > MAX_ZGAIN)			// Limit to a sensible value
	{
		PID_acc_temp1 = MAX_ZGAIN;
	}

	// P2
	PID_acc_temp2 *= L_gain[P2][YAW];		// Multiply P-term (Max gain of 127)

	if (PID_acc_temp2 > MAX_ZGAIN)			// Limit to a sensible value
	{
		PID_acc_temp2 = MAX_ZGAIN;
	}

	// Update values
	PID_ACCs[P1][YAW] = (int16_t)PID_acc_temp1;	
	PID_ACCs[P2][YAW] = (int16_t)PID_acc_temp2;	

	// Save current acc Z value
	old_z = accADC[YAW];
}
