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
	static	int16_t lastError[NUMBEROFAXIS];// Used to keep track of D-Terms in PID calculations

	int32_t PID_gyro_temp1 = 0;				// P1
	int32_t PID_gyro_temp2 = 0;				// P2
	int32_t PID_acc_temp1 = 0;				// P1
	int32_t PID_acc_temp2 = 0;				// P2
	int32_t PID_Gyro_I_actual1 = 0;			// Actual unbound i-terms P1
	int32_t PID_Gyro_I_actual2 = 0;			// P2

	int16_t DifferentialGyro1 = 0;			// Holds difference between last two errors (angular acceleration)
	int16_t DifferentialGyro2 = 0;
	int16_t Differential = 0;
	int16_t	stick = 0;
	int8_t i = 0;
	int8_t	axis = 0;

	// Cross-ref for actual RCinput elements
	// Note that axes are reversed here with respect to their gyros
	int16_t	RCinputsAxis[NUMBEROFAXIS] = {-RCinputs[AILERON], -RCinputs[ELEVATOR], -RCinputs[RUDDER]}; 

	// Initialise arrays with gain values.
	int8_t 	P_gain[FLIGHT_MODES][NUMBEROFAXIS] = 
		{
			{Config.FlightMode[P1].Roll_P_mult, Config.FlightMode[P1].Pitch_P_mult, Config.FlightMode[P1].Yaw_P_mult},
		 	{Config.FlightMode[P2].Roll_P_mult, Config.FlightMode[P2].Pitch_P_mult, Config.FlightMode[P2].Yaw_P_mult}
		};

	int8_t 	I_gain[FLIGHT_MODES][NUMBEROFAXIS] = 
		{
			{Config.FlightMode[P1].Roll_I_mult, Config.FlightMode[P1].Pitch_I_mult, Config.FlightMode[P1].Yaw_I_mult},
			{Config.FlightMode[P2].Roll_I_mult, Config.FlightMode[P2].Pitch_I_mult, Config.FlightMode[P2].Yaw_I_mult}
		};

	int8_t 	D_gain[FLIGHT_MODES][NUMBEROFAXIS] =  
		{
			{Config.FlightMode[P1].Roll_D_mult, Config.FlightMode[P1].Pitch_D_mult, Config.FlightMode[P1].Yaw_D_mult},
			{Config.FlightMode[P2].Roll_D_mult, Config.FlightMode[P2].Pitch_D_mult, Config.FlightMode[P2].Yaw_D_mult}
		};

	int8_t 	L_gain[FLIGHT_MODES][NUMBEROFAXIS] = 
		{
			{Config.FlightMode[P1].A_Roll_P_mult, Config.FlightMode[P1].A_Pitch_P_mult, Config.FlightMode[P1].A_Zed_P_mult},
			{Config.FlightMode[P2].A_Roll_P_mult, Config.FlightMode[P2].A_Pitch_P_mult, Config.FlightMode[P2].A_Zed_P_mult}
		};

	// Only for roll and pitch acc trim
	int16_t	L_trim[FLIGHT_MODES][2] =
		{
			{Config.Rolltrim[P1], Config.Pitchtrim[P1]},
			{Config.Rolltrim[P2], Config.Pitchtrim[P2]}
		};

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

		// Work out stick rate divider. 0 is fastest, 4 is slowest.
		// /64 (15), /32 (30), /16 (60*), /8 (120), /4 (240)
		stick = RCinputsAxis[axis] >> (Config.Stick_Lock_rate + 2);

		// Calculate I-term from gyro and stick data 
		// These may look identical, but they are constrained quite differently.
		IntegralGyro[P1][axis] += (gyroADC[axis] - stick);
		IntegralGyro[P2][axis] += (gyroADC[axis] - stick);

		// Limit the I-terms when you need to adjust the I-term with RC
		// Note that the I-term is not constrained when no RC input is present.
		if (RCinputsAxis[axis] != 0)
		{
			for (i = P1; i <= P2; i++)
			{
				if (IntegralGyro[i][axis] > Config.Raw_I_Constrain[i][axis])
				{
					IntegralGyro[i][axis] = Config.Raw_I_Constrain[i][axis];
				}
				if (IntegralGyro[i][axis] < -Config.Raw_I_Constrain[i][axis])
				{
					IntegralGyro[i][axis] = -Config.Raw_I_Constrain[i][axis];
				}
			}
		}

		//************************************************************
		// Add in gyro Yaw trim
		//************************************************************

		if (axis == YAW)
		{
			PID_gyro_temp1 = (int32_t)(Config.FlightMode[P1].Yaw_trim << 6);
			PID_gyro_temp2 = (int32_t)(Config.FlightMode[P2].Yaw_trim << 6);
		}
		// Reset PID_gyro variables to that data does not accumulate cross-axis
		else
		{
			PID_gyro_temp1 = 0;
			PID_gyro_temp2 = 0;
		}

		//************************************************************
		// Calculate PID gains
		//************************************************************

		// Gyro P-term													// Profile P1
		PID_gyro_temp1 += gyroADC[axis] * P_gain[P1][axis];				// Multiply P-term (Max gain of 127)
		PID_gyro_temp1 = PID_gyro_temp1 * (int32_t)3;					// Multiply by 3

		// Gyro I-term
		PID_Gyro_I_actual1 = IntegralGyro[P1][axis] * I_gain[P1][axis];	// Multiply I-term (Max gain of 127)
		PID_Gyro_I_actual1 = PID_Gyro_I_actual1 >> 5;					// Divide by 32

		// Gyro D-term
		Differential = gyroADC[axis] - lastError[axis];
		lastError[axis] = gyroADC[axis];
		DifferentialGyro1 = Differential * D_gain[P1][axis];			// Multiply D-term by up to 127
		DifferentialGyro1 = DifferentialGyro1 << 4;						// Multiply by 16

		// Gyro P-term
		PID_gyro_temp2 += gyroADC[axis] * P_gain[P2][axis];				// Profile P2
		PID_gyro_temp2 = PID_gyro_temp2 * (int32_t)3;

		// Gyro I-term
		PID_Gyro_I_actual2 = IntegralGyro[P2][axis] * I_gain[P2][axis];
		PID_Gyro_I_actual2 = PID_Gyro_I_actual2 >> 5;

		// Gyro D-term
		DifferentialGyro2 = Differential * D_gain[P2][axis];			// Multiply D-term by up to 127
		DifferentialGyro2 = DifferentialGyro2 << 4;						// Multiply by 16

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
		// Sum Gyro P, I and D terms and rescale
		//************************************************************

		PID_Gyros[P1][axis] = (int16_t)((PID_gyro_temp1 + PID_Gyro_I_actual1 + DifferentialGyro1) >> 6);
		PID_Gyros[P2][axis] = (int16_t)((PID_gyro_temp2 + PID_Gyro_I_actual2 + DifferentialGyro2) >> 6);

		//************************************************************
		// Calculate error from angle data and trim (roll and pitch only)
		//************************************************************

		if (axis < YAW)
		{
			PID_acc_temp1 = angle[axis] - L_trim[P1][axis];				// Offset angle with trim
			PID_acc_temp2 = angle[axis] - L_trim[P2][axis];

			PID_acc_temp1 *= L_gain[P1][axis];							// P-term of accelerometer (Max gain of 127)
			PID_ACCs[P1][axis] = (int16_t)(PID_acc_temp1 >> 8);			// Reduce and convert to integer

			PID_acc_temp2 *= L_gain[P2][axis];							// Same for P2
			PID_ACCs[P2][axis] = (int16_t)(PID_acc_temp2 >> 8);	
		}

	} // PID loop

	//************************************************************
	// Calculate an Acc-Z value 
	//************************************************************

	PID_acc_temp1 = -accVert;				// Get and copy Z-acc value. Negate to oppose G
	PID_acc_temp2 = PID_acc_temp1;

	PID_acc_temp1 *= L_gain[P1][YAW];		// Multiply P-term (Max gain of 127)
	PID_acc_temp2 *= L_gain[P2][YAW];		// Multiply P-term (Max gain of 127)

	PID_acc_temp1 = PID_acc_temp1 >> 4;		// Moderate Z-acc to reasonable values
	PID_acc_temp2 = PID_acc_temp2 >> 4;	

	if (PID_acc_temp1 > MAX_ZGAIN)			// Limit to +/-MAX_ZGAIN
	{
		PID_acc_temp1 = MAX_ZGAIN;
	}
	if (PID_acc_temp1 < -MAX_ZGAIN)
	{
		PID_acc_temp1 = -MAX_ZGAIN;
	}

	if (PID_acc_temp2 > MAX_ZGAIN)
	{
		PID_acc_temp2 = MAX_ZGAIN;
	}
	if (PID_acc_temp2 < -MAX_ZGAIN)
	{
		PID_acc_temp2 = -MAX_ZGAIN;
	}

	PID_ACCs[P1][YAW] = (int16_t)PID_acc_temp1; // Copy to global values
	PID_ACCs[P2][YAW] = (int16_t)PID_acc_temp2;	
}
