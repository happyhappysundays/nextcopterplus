//***********************************************************
//* mixer.c
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include <string.h>
#include <avr/io.h>
#include <stdbool.h>
#include <util/delay.h>
#include "io_cfg.h"
#include "rc.h"
#include "isr.h"
#include "servos.h"
#include "pid.h"
#include "main.h"
#include <avr/pgmspace.h> 
#include "mixer.h"
#include "init.h"
#include "imu.h"

//************************************************************
// Prototypes
//************************************************************

void ProcessMixer(void);
void UpdateLimits(void);
void get_preset_mix (const channel_t*);
int16_t scale32(int16_t value16, int16_t multiplier16);
int16_t scale_percent(int8_t value);
int16_t scale_percent_nooffset(int8_t value);

//************************************************************
// Defines
//************************************************************

#define EXT_SOURCE 8	// Offset for indexing sensor sources

//************************************************************
// Mix tables (both RC inputs and servo/ESC outputs)
//************************************************************

// Aeroplane mixer defaults
const channel_t AEROPLANE_MIX[MAX_OUTPUTS] PROGMEM = 
{
	// Value, 
	// source_a, source_a_vol, source_b,src_vol,roll_gyro,pitch_gyro,yaw_gyro,roll_acc,pitch_acc
	// source_b,source_b_volume,source_c,source_c_volume,source_d,source_d_volume

	{0,THROTTLE,100,NOCHAN,0,OFF,OFF,OFF,OFF,OFF,NOMIX,0,NOMIX,0,},	// ServoOut1 (Throttle)
	{0,AILERON,100,NOCHAN,0,OFF,OFF,OFF,OFF,OFF,NOMIX,0,NOMIX,0,},	// ServoOut2 (Left aileron)
	{0,ELEVATOR,100,NOCHAN,0,OFF,OFF,OFF,OFF,OFF,NOMIX,0,NOMIX,0,},	// ServoOut3 (Elevator)
	{0,RUDDER,100,NOCHAN,0,OFF,OFF,OFF,OFF,OFF,NOMIX,0,NOMIX,0,},	// ServoOut4 (Rudder)
	{0,GEAR,100,NOCHAN,0,OFF,OFF,OFF,OFF,OFF,NOMIX,0,NOMIX,0,},		// ServoOut5 
	{0,AUX1,100,NOCHAN,0,OFF,OFF,OFF,OFF,OFF,NOMIX,0,NOMIX,0,},		// ServoOut6
	{0,AUX2,100,NOCHAN,0,OFF,OFF,OFF,OFF,OFF,NOMIX,0,NOMIX,0,},		// ServoOut7
	{0,AUX3,100,NOCHAN,0,OFF,OFF,OFF,OFF,OFF,NOMIX,0,NOMIX,0,},		// ServoOut8
}; 

const channel_t FLYING_WING_MIX[MAX_OUTPUTS] PROGMEM = 
{
	// Value, Motor_marker
	// source_a, source_a_vol, source_b,src_vol, roll_gyro,pitch_gyro,yaw_gyro,roll_acc,pitch_acc
	// source_b,source_b_volume,source_c,source_c_volume,source_d,source_d_volume

	{0,THROTTLE,100,NOCHAN,0,OFF,OFF,OFF,OFF,OFF,NOMIX,0,NOMIX,0,},	// ServoOut1 (Throttle)
	{0,AILERON,50,ELEVATOR,50,OFF,OFF,OFF,OFF,OFF,NOMIX,0,NOMIX,0,},// ServoOut2 (Left elevon)
	{0,AILERON,-50,ELEVATOR,50,OFF,OFF,OFF,OFF,OFF,NOMIX,0,NOMIX,0,},// ServoOut3 (Right elevon)
	{0,RUDDER,100,NOCHAN,0,OFF,OFF,OFF,OFF,OFF,NOMIX,0,NOMIX,0,},	// ServoOut4 (Rudder)
	{0,GEAR,0,NOCHAN,0,OFF,OFF,OFF,OFF,OFF,NOMIX,0,NOMIX,0,},		// ServoOut5 
	{0,AUX1,0,NOCHAN,0,OFF,OFF,OFF,OFF,OFF,NOMIX,0,NOMIX,0,},		// ServoOut6
	{0,AUX2,0,NOCHAN,0,OFF,OFF,OFF,OFF,OFF,NOMIX,0,NOMIX,0,},		// ServoOut7
	{0,AUX3,0,NOCHAN,0,OFF,OFF,OFF,OFF,OFF,NOMIX,0,NOMIX,0,},		// ServoOut8
}; 

//************************************************************
// Code
//************************************************************

void ProcessMixer(void)
{
	static	int16_t flap = 0;
	static	int16_t slowFlaps = 0;
	static	uint8_t flapskip;
	static	int16_t roll = 0;

	uint8_t i;
	int8_t	speed;
	int16_t temp = 0;
	int16_t temp1 = 0;
	int16_t temp2 = 0;
	int16_t solution = 0;
	bool	TwoAilerons = false;
	
	// Copy the sensor data to an array for easy indexing - acc data is from accSmooth, increased to reasonable rates
	temp1 = (int16_t)accSmooth[ROLL] << 3;
	temp2 = (int16_t)accSmooth[PITCH] << 3;
	int16_t	SensorData[7] = {PID_Gyros[ROLL], PID_Gyros[PITCH], PID_Gyros[YAW], temp1, temp2, PID_ACCs[ROLL], PID_ACCs[PITCH]};

	//************************************************************
	// Un-mix flaps from flaperons as required
	//************************************************************ 

	if (Config.FlapChan != NOCHAN)
	{
		// Update flap only if ailerons are within measurable positions
		if ((RCinputs[AILERON] > -1200) && 
			(RCinputs[AILERON] < 1200) &&
			(RCinputs[Config.FlapChan] > -1200) && 
			(RCinputs[Config.FlapChan] < 1200))
		{
			flap = RCinputs[AILERON] - RCinputs[Config.FlapChan]; 	
			flap = flap >> 1; 	
		}
	}
	else
	{
		flap = 0;
	}

	//************************************************************
	// Un-mix ailerons from flaperons as required in all modes
	//************************************************************

	// If in AEROPLANE mixer mode and flaperons set up
	if ((Config.FlapChan != NOCHAN) && (Config.MixMode == AEROPLANE))
	{
		// Remove flap signal from flaperons, leaving ailerons only
		roll = RCinputs[AILERON] + RCinputs[Config.FlapChan];

		// Otherwise throw is 50% of both signals
		RCinputs[AILERON] = roll >> 1;
		
		// Copy to second aileron channel
		RCinputs[Config.FlapChan] = RCinputs[AILERON];
	}

	//************************************************************
	// Main mix loop - sensors, RC inputs and other channels
	//************************************************************

	for (i = 0; i < MAX_OUTPUTS; i++)
	{
		//************************************************************
		// Zero each channel value to start
		//************************************************************

		solution = 0;
		
		//************************************************************
		// Mix in gyros
		//************************************************************ 

		// Use PID gyro values
		if (Flight_flags & (1 << Stability))
		{
			switch (Config.Channel[i].roll_gyro)
			{
				case ON:
					solution = solution - PID_Gyros[ROLL];
					break;
				case REV:
					solution = solution + PID_Gyros[ROLL];
					break;	
				default:
					break;
			}
			switch (Config.Channel[i].pitch_gyro)
			{
				case ON:
					solution = solution + PID_Gyros[PITCH];
					break;
				case REV:
					solution = solution - PID_Gyros[PITCH];
					break;	
				default:
					break;
			}
			switch (Config.Channel[i].yaw_gyro)
			{
				case ON:
					solution = solution + PID_Gyros[YAW];
					break;
				case REV:
					solution = solution - PID_Gyros[YAW];
					break;	
				default:
					break;
			}
		} // Stability		

		//************************************************************
		// Mix in accelerometers
		//************************************************************ 

		// Add PID acc values including trim
		if (Flight_flags & (1 << AutoLevel))
		{
			switch (Config.Channel[i].roll_acc)
			{
				case ON:
					solution = solution - PID_ACCs[ROLL];
					break;
				case REV:
					solution = solution + PID_ACCs[ROLL];
					break;	
				default:
					break;
			}

			switch (Config.Channel[i].pitch_acc)
			{
				case ON:
					solution = solution + PID_ACCs[PITCH];
					break;
				case REV:
					solution = solution - PID_ACCs[PITCH];
					break;	
				default:
					break;
			}
		} // Autolevel

		//************************************************************
		// Process RC mixing and source volume calculation
		//************************************************************		
		
		// Skip Source A if no RC mixing required for this channel
		if (Config.Channel[i].source_a_volume != 0)
		{		
			temp = scale32(RCinputs[Config.Channel[i].source_a], Config.Channel[i].source_a_volume);
			solution += temp;
		}

		// Skip Source B if no RC mixing required for this channel
		if (Config.Channel[i].source_b_volume != 0)
		{
			temp = scale32(RCinputs[Config.Channel[i].source_b], Config.Channel[i].source_b_volume);
			solution += temp;
		}

		//************************************************************
		// Process universal mixers
		//************************************************************ 

		if ((Config.Channel[i].output_b_volume != 0) && (Config.Channel[i].output_b != NOMIX)) // Mix in first extra source
		{
			// Is the source a sensor?
			if (Config.Channel[i].output_b > (MAX_RC_CHANNELS - 1))
			{
				temp2 = SensorData[Config.Channel[i].output_b - EXT_SOURCE];
			}
			// Is the source an RC input?
			else
			{
				temp2 = RCinputs[Config.Channel[i].output_b];
			}

			temp2 = scale32(temp2, Config.Channel[i].output_b_volume);
			solution = solution + temp2;
		}
		
		if ((Config.Channel[i].output_c_volume != 0) && (Config.Channel[i].output_c != NOMIX)) // Mix in second extra source
		{
			// Is the source a sensor?
			if (Config.Channel[i].output_c > (MAX_RC_CHANNELS - 1))
			{
				temp2 = SensorData[Config.Channel[i].output_c - EXT_SOURCE];
			}
			// Is the source an RC input?
			else
			{
				temp2 = RCinputs[Config.Channel[i].output_c];
			}

			temp2 = scale32(temp2, Config.Channel[i].output_c_volume);
			solution = solution + temp2;
		}

		// Save solution for this channel. "solution" contains the current cycle's data.
		// Up to this point, Config.Channel[i].value contains data from the last cycle.
		Config.Channel[i].value = solution;
		
	} //for (i = 0; i < MAX_OUTPUTS; i++)

	//************************************************************
	// Process differential if set up and two ailerons used
	//************************************************************

	if ((Config.FlapChan != NOCHAN) && (Config.Differential != 0))
	{
		// Search through outputs for aileron channels
		for (i = 0; i < MAX_OUTPUTS; i++)
		{
			// Get current channel value
			temp = Config.Channel[i].value;

			// If some kind of aileron channel
			if ((Config.Channel[i].source_a == AILERON) || (Config.Channel[i].source_a == Config.FlapChan))
			{
				// For the second aileron (RHS)
				if (TwoAilerons)			
				{
					// Limit negative-going values
					if (temp < 0)
					{
						temp = scale32(temp, (100 - Config.Differential));
						Config.Channel[i].value = temp;
					}
				}

				// For the first aileron (LHS) 
				// Limit positive-going values
				else if (temp > 0)			
				{
					temp = scale32(temp, (100 - Config.Differential));
					Config.Channel[i].value = temp;
					TwoAilerons = true; // Found an aileron
				}

				// Else was the normal side of the first aileron
				else
				{
					TwoAilerons = true; // Found an aileron
				}
			}

		}
		// Reset after all outputs done
		TwoAilerons = false;
	}

	//************************************************************
	// Re-mix flaps from flaperons as required
	//************************************************************ 

	// The flap part of the signal has been removed so we have to reinsert it here.
	if ((Config.FlapChan != NOCHAN) && (Config.MixMode == AEROPLANE))
	{
		// If flapspeed is set to anything other than zero (normal)
		if (Config.flapspeed) 
		{
			// Do flap speed control
			if (((slowFlaps - flap) >= 1) || ((slowFlaps - flap) <= -1))	// Difference larger than one step, so ok
			{
				speed = 15;					// Need to manipulate speed as target approaches									
			}
			else
			{
				speed = 1;					// Otherwise this will oscillate
			}

			if ((slowFlaps < flap) && (flapskip == Config.flapspeed))
			{
				slowFlaps += speed;
			} 
			else if ((slowFlaps > flap) && (flapskip == Config.flapspeed)) 
			{
				slowFlaps -= speed;
			}
			
		} 
		// No speed control requested so copy flaps
		else
		{
		 	slowFlaps = flap;
		}

		flapskip++;
		if (flapskip > Config.flapspeed) flapskip = 0;

		for (i = 0; i < MAX_OUTPUTS; i++)
		{
			// Get solution
			temp = Config.Channel[i].value;

			// Restore flaps
			if (Config.Channel[i].source_a == AILERON)
			{
				temp += slowFlaps;
			}
			if (Config.Channel[i].source_a == Config.FlapChan)
			{
				temp -= slowFlaps;
			}

			// Update channel data solution
			Config.Channel[i].value = temp;
		} // Flaps
	}

	//************************************************************
	// Add offset value
	//************************************************************ 

	for (i = 0; i < MAX_OUTPUTS; i++)
	{
		Config.Channel[i].value += Config.Limits[i].trim;
	}

	//************************************************************
	// Handle Failsafe condition
	//************************************************************ 

	if (Flight_flags & (1 << FailsafeFlag))
	{
		// Simple failsafe. Replace outputs with user-set values
		if (Config.FailsafeType == SIMPLE) 
		{
			for (i = 0; i < MAX_OUTPUTS; i++)
			{
				// Note that "value" is still centered on zero
				Config.Channel[i].value = Config.Limits[i].failsafe;
			}
		}

		// Advanced failsafe. Autolevel ON, use failsafe trims to adjust autolevel.
		// Set any throttle or rudder channels to preset values
		// Pitch and Roll values trim the pitch/roll autolevel
		else if (Config.FailsafeType == ADVANCED) 
		{
			for (i = 0; i < MAX_OUTPUTS; i++)
			{
				// Override throttle
				if ((Config.Channel[i].source_a == THROTTLE) || (Config.Channel[i].source_b == THROTTLE))
				{
					// Convert throttle setting to servo value
					Config.Channel[i].value = scale_percent_nooffset(Config.FailsafeThrottle);				
				}

				// Override rudder						
				if (Config.Channel[i].source_a == RUDDER)
				{
					Config.Channel[i].value = scale_percent_nooffset(Config.FailsafeRudder);	
				}
			}
		}
	} // Failsafe
}

// Get preset mix from Program memory
void get_preset_mix(const channel_t* preset)
{
	// Clear all channels first
	memset(&Config.Channel[0].value,0,(sizeof(channel_t) * MAX_OUTPUTS));
	memcpy_P(&Config.Channel[0].value,&preset[0].value,(sizeof(channel_t) * MAX_OUTPUTS));
}

// Update actual limits value with that from the mix setting percentages
// This is only done at start-up and whenever the values are changed
void UpdateLimits(void)
{
	uint8_t i;
	int8_t limits[NUMBEROFAXIS] = {Config.FlightMode[Config.Flight].Roll_limit, Config.FlightMode[Config.Flight].Pitch_limit, Config.FlightMode[Config.Flight].Yaw_limit};
	int32_t temp32, gain32;
	int8_t gains[NUMBEROFAXIS] = {Config.FlightMode[Config.Flight].Roll.I_mult, Config.FlightMode[Config.Flight].Pitch.I_mult, Config.FlightMode[Config.Flight].Yaw.I_mult};

	// Update LVA trigger
	// Vbat is measured in units of 10mV, so PowerTriggerActual of 1270 equates to 12.7V
	switch (Config.PowerTrigger)
	{
		case 0:
			Config.PowerTriggerActual = 0;			// Off
			break;
		case 1:
			Config.PowerTriggerActual = 320; 		// 3.2V
			break;
		case 2:
			Config.PowerTriggerActual = 330; 		// 3.3V
			break;
		case 3:
			Config.PowerTriggerActual = 340;		// 3.4V
			break;
		case 4:
			Config.PowerTriggerActual = 350; 		// 3.5V
			break;
		case 5:
			Config.PowerTriggerActual = 360; 		// 3.6V
			break;
		case 6:
			Config.PowerTriggerActual = 370; 		// 3.7V
			break;
		case 7:
			Config.PowerTriggerActual = 380; 		// 3.8V
			break;
		case 8:
			Config.PowerTriggerActual = 390; 		// 3.9V
			break;
		default:
			Config.PowerTriggerActual = 0;			// Off
			break;
	}
	
	// Determine cell count and use to multiply trigger
	if (SystemVoltage >= 2150)										// 6S - 21.5V or at least 3.58V per cell
	{
		Config.PowerTriggerActual *= 6;
	}
	else if ((SystemVoltage >= 1730) && (SystemVoltage < 2150))		// 5S 17.3V to 21.5V or 4.32V(4S) to 3.58V(6S) per cell
	{
		Config.PowerTriggerActual *= 5;
	}
	else if ((SystemVoltage >= 1300) && (SystemVoltage < 1730))		// 4S 13.0V to 17.3V or 4.33V(3S) to 3.46V(5S) per cell
	{
		Config.PowerTriggerActual *= 4;
	}
	else if ((SystemVoltage >= 900) && (SystemVoltage < 1300))		// 3S 9.0V to 13.0V or 4.5V(2S) to 3.25V(4S) per cell
	{
		Config.PowerTriggerActual *= 3;
	}
	else if (SystemVoltage < 900)									// 2S Under 9.0V or 3.0V(3S) per cell
	{
		Config.PowerTriggerActual *= 2;
	}

	// Update I_term limits
	for (i = 0; i < NUMBEROFAXIS; i++)
	{
		temp32 	= limits[i]; 						// Promote

		// I-term output (throw). Convert from % to actual count
		// A value of 80,000 results in +/- 1250 or full throw at the output stage
		// This is because the maximum signal value is +/-1250 after division by 64. 1250 * 64 = 80,000
		Config.Raw_I_Limits[i] = temp32 * (int32_t)640;	// 80,000 / 125% = 640

		// I-term source limits. These have to be different due to the I-term gain setting
		// I-term = (gyro * gain) / 32, so the gyro count for a particular gain and limit is
		// Gyro = (I-term * 32) / gain :)

		if (gains[i] != 0)
		{
			gain32 = gains[i];						// Promote gain value
			Config.Raw_I_Constrain[i] = (Config.Raw_I_Limits[i] << 5) / gain32;
		}
		else
		{
			Config.Raw_I_Constrain[i] = 0;
		}
	}

	// Update travel limits
	for (i = 0; i < MAX_OUTPUTS; i++)
	{
		Config.Limits[i].minimum = scale_percent(Config.min_travel[i]);			// Limits are checked post conversion to system values, so need to be 2500~5000
		Config.Limits[i].maximum = scale_percent(Config.max_travel[i]);
		Config.Limits[i].failsafe = scale_percent_nooffset(Config.Failsafe[i]);	// Config.Failsafe and .trim are used prior to conversion, so need to be +/-1250
		Config.Limits[i].trim = scale_percent_nooffset(Config.Offset[i]);
	}

	// Update dynamic gain divisor
	if (Config.DynGain > 0)
	{
		Config.DynGainDiv = 2500 / Config.DynGain;
	}
	else
	{
		Config.DynGainDiv = 2500;
	}

	// Update RC deadband amount
	 Config.DeadbandLimit = (Config.Deadband * 12); // 0 to 5% scaled to 0 to 60

	// Update Hands-free trigger based on deadband setting
	Config.HandsFreetrigger = Config.DeadbandLimit;
	
	// Adjust trim to match 0.01 degree resolution
	// A value of 127 multiplied by 10 = 1270 which in 1/100ths of a degree equates to 12.7 degrees
	for (i = 0; i <= FLIGHT_MODES; i++)
	{
		Config.Rolltrim[i] = Config.FlightMode[i].AccRollZeroTrim * 10;
		Config.Pitchtrim[i] = Config.FlightMode[i].AccPitchZeroTrim * 10;
	}
}

// 32 bit multiply/scale for broken GCC
// Returns immediately if multiplier is 100, 0 or -100
int16_t scale32(int16_t value16, int16_t multiplier16)
{
	int32_t temp32 = 0;
	int32_t mult32 = 0;

	// No change if 100% (no scaling)
	if (multiplier16 == 100)
	{
		return value16;
	}

	// Reverse if -100%
	else if (multiplier16 == -100)
	{
		return -value16;
	}

	// Zero if 0%
	else if (multiplier16 == 0)
	{
		return 0;
	}

	// Only do the scaling if necessary
	else
	{
		// GCC is broken bad regarding multiplying 32 bit numbers, hence all this crap...
		mult32 = multiplier16;
		temp32 = value16;
		temp32 = temp32 * mult32;

		// Divide by 100 and round to get scaled value
		temp32 = (temp32 + (int32_t)50) / (int32_t)100; // Constants need to be cast up to 32 bits
		value16 = (int16_t)temp32;
	}

	return value16;
}

// Scale percentages to position
int16_t scale_percent(int8_t value)
{
	int16_t temp16_1, temp16_2;

	temp16_1 = value; // Promote
	temp16_2 = ((temp16_1 * (int16_t)10) + 3750);

	return temp16_2;
}


// Scale percentages to relative position
int16_t scale_percent_nooffset(int8_t value)
{
	int16_t temp16_1, temp16_2;

	temp16_1 = value; // Promote
	temp16_2 = (temp16_1 * (int16_t)10);

	return temp16_2;
}
