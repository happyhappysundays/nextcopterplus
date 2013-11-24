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
#include "..\inc\io_cfg.h"
#include "..\inc\rc.h"
#include "..\inc\isr.h"
#include "..\inc\servos.h"
#include "..\inc\pid.h"
#include "..\inc\main.h"
#include <avr/pgmspace.h> 
#include "..\inc\mixer.h"

//************************************************************
// Prototypes
//************************************************************

void ProcessMixer(void);
void UpdateServos(void);
void UpdateLimits(void);
void get_preset_mix (const channel_t*);
int16_t scale32(int16_t value16, int16_t multiplier16);
int16_t scale_percent(int8_t value);
int16_t scale_percent_nooffset(int8_t value);

//************************************************************

void ProcessMixer(void)
{
	uint8_t i;
	uint8_t j;
	int16_t P1_solution = 0;
	int16_t P2_solution = 0;

	int16_t temp1 = 0;
	int16_t temp2 = 0;
	int16_t	temp3 = 0;
	int16_t	transition = 0;
	int16_t	Step1 = 0;
	int16_t	Step2 = 0;
	int16_t	rolltrim = 0;
	int16_t	pitchtrim = 0;

	//************************************************************
	// Main mix loop
	//************************************************************

	for (i = 0; i < MAX_OUTPUTS; i++)
	{
		//************************************************************
		// Zero each channel value to start
		//************************************************************

		P1_solution = 0;
		P2_solution = 0;

		//************************************************************
		// Mix in gyros
		//************************************************************ 

		// P1
		if ((Transition_state == TRANS_0) || (TRANSITIONING))
		{
			if ((Config.Channel[i].P1_sensors & (1 << RollGyro)) != 0) // P1
			{
				if ((Config.Channel[i].P1_RevFlags & (1 << RollRev)) != 0)
				{
					P1_solution = P1_solution + PID_Gyros[P1][ROLL];
				}
				else
				{
					P1_solution = P1_solution - PID_Gyros[P1][ROLL];
				}
			}
			if ((Config.Channel[i].P1_sensors & (1 << PitchGyro)) != 0)
			{
				if ((Config.Channel[i].P1_RevFlags & (1 << PitchRev)) != 0)
				{
					P1_solution = P1_solution + PID_Gyros[P1][PITCH];
				}
				else
				{
					P1_solution = P1_solution - PID_Gyros[P1][PITCH];
				};
			}
			if ((Config.Channel[i].P1_sensors & (1 << YawGyro)) != 0)
			{
				if ((Config.Channel[i].P1_RevFlags & (1 << YawRev)) != 0)
				{
					P1_solution = P1_solution + PID_Gyros[P1][YAW];
				}
				else
				{
					P1_solution = P1_solution - PID_Gyros[P1][YAW];
				};
			}
		}

		// P2
		if ((Transition_state == TRANS_1) || (TRANSITIONING))
		{
			if ((Config.Channel[i].P2_sensors & (1 << RollGyro)) != 0) // P2
			{
				if ((Config.Channel[i].P2_RevFlags & (1 << RollRev)) != 0)
				{
					P2_solution = P2_solution + PID_Gyros[P2][ROLL];
				}
				else
				{
					P2_solution = P2_solution - PID_Gyros[P2][ROLL];
				}
			}
			if ((Config.Channel[i].P2_sensors & (1 << PitchGyro)) != 0)
			{
				if ((Config.Channel[i].P2_RevFlags & (1 << PitchRev)) != 0)
				{
					P2_solution = P2_solution + PID_Gyros[P2][PITCH];
				}
				else
				{
					P2_solution = P2_solution - PID_Gyros[P2][PITCH];
				};
			}
			if ((Config.Channel[i].P2_sensors & (1 << YawGyro)) != 0)
			{
				if ((Config.Channel[i].P2_RevFlags & (1 << YawRev)) != 0)
				{
					P2_solution = P2_solution + PID_Gyros[P2][YAW];
				}
				else
				{
					P2_solution = P2_solution - PID_Gyros[P2][YAW];
				};
			}
		}

		//************************************************************
		// Autolevel trims
		//************************************************************
	
		rolltrim = (Config.FlightMode[2].AccRollZeroTrim << 2); // Roll trim
		pitchtrim = (Config.FlightMode[2].AccPitchZeroTrim << 2);// Pitch trim

		//************************************************************
		// Mix in accelerometers
		//************************************************************ 
		// P1
		if ((Transition_state == TRANS_0) || (TRANSITIONING))
		{
			if ((Config.Channel[i].P1_sensors & (1 << RollAcc)) != 0)
			{
				P1_solution += rolltrim;
			
				if ((Config.Channel[i].P1_RevFlags & (1 << AccRollRev)) != 0)
				{
					P1_solution = P1_solution + PID_ACCs[P1][ROLL];
				}
				else
				{
					P1_solution = P1_solution - PID_ACCs[P1][ROLL];
				}
			}

			if ((Config.Channel[i].P1_sensors & (1 << PitchAcc)) != 0)
			{
				P1_solution += pitchtrim;
			
				if ((Config.Channel[i].P1_RevFlags & (1 << AccPitchRev)) != 0)
				{
					P1_solution = P1_solution + PID_ACCs[P1][PITCH];
				}
				else
				{
					P1_solution = P1_solution - PID_ACCs[P1][PITCH];
				}
			}

			if ((Config.Channel[i].P1_sensors & (1 << ZDeltaAcc)) != 0)
			{
				if ((Config.Channel[i].P1_RevFlags & (1 << AccZRev)) != 0)
				{
					P1_solution = P1_solution + PID_ACCs[P1][YAW];
				}
				else
				{
					P1_solution = P1_solution - PID_ACCs[P1][YAW];
				}
			}
		}

		// P2
		if ((Transition_state == TRANS_1) || (TRANSITIONING))
		{
			if ((Config.Channel[i].P2_sensors & (1 << RollAcc)) != 0)
			{
				P2_solution += rolltrim;
			
				if ((Config.Channel[i].P2_RevFlags & (1 << AccRollRev)) != 0)
				{
					P2_solution = P2_solution + PID_ACCs[P2][ROLL];
				}
				else
				{
					P2_solution = P2_solution - PID_ACCs[P2][ROLL];
				}
			}

			if ((Config.Channel[i].P2_sensors & (1 << PitchAcc)) != 0)
			{
				P2_solution += pitchtrim;
			
				if ((Config.Channel[i].P2_RevFlags & (1 << AccPitchRev)) != 0)
				{
					P2_solution = P2_solution + PID_ACCs[P2][PITCH];
				}
				else
				{
					P2_solution = P2_solution - PID_ACCs[P2][PITCH];
				}
			}

			if ((Config.Channel[i].P2_sensors & (1 << ZDeltaAcc)) != 0)
			{
				if ((Config.Channel[i].P2_RevFlags & (1 << AccZRev)) != 0)
				{
					P2_solution = P2_solution + PID_ACCs[P2][YAW];
				}
				else
				{
					P2_solution = P2_solution - PID_ACCs[P2][YAW];
				}
			}
		}

		//************************************************************
		// Process mixers
		//************************************************************ 

		// Mix in other outputs here (P1)
		if ((Transition_state == TRANS_0) || (TRANSITIONING))
		{
			if ((Config.Channel[i].P1_source_a_volume !=0) && (Config.Channel[i].P1_source_a != NOMIX)) // Mix in first extra source
			{
				// Is the source an RC input?
				if (Config.Channel[i].P1_source_a > (MAX_OUTPUTS - 1))
				{
					// Yes, calculate RC channel number from source number and return RC value
					temp2 = RCinputs[Config.Channel[i].P1_source_a - MAX_OUTPUTS];
				}
				else
				{
					// No, just use the selected output's old data
					temp2 = Config.Channel[Config.Channel[i].P1_source_a].P1_value;
				}

				temp2 = scale32(temp2, Config.Channel[i].P1_source_a_volume);
				P1_solution = P1_solution + temp2;
			}
			if ((Config.Channel[i].P1_source_b_volume !=0) && (Config.Channel[i].P1_source_b != NOMIX)) // Mix in second extra source
			{
				if (Config.Channel[i].P1_source_b > (MAX_OUTPUTS - 1))
				{
					temp2 = RCinputs[Config.Channel[i].P1_source_b - MAX_OUTPUTS];
				}
				else
				{
					temp2 = Config.Channel[Config.Channel[i].P1_source_b].P1_value;
				}

				temp2 = scale32(temp2, Config.Channel[i].P1_source_b_volume);
				P1_solution = P1_solution + temp2;
			}
			if ((Config.Channel[i].P1_source_c_volume !=0) && (Config.Channel[i].P1_source_c != NOMIX)) // Mix in third extra source
			{
				if (Config.Channel[i].P1_source_c > (MAX_OUTPUTS - 1))
				{
					temp2 = RCinputs[Config.Channel[i].P1_source_c - MAX_OUTPUTS];
				}
				else
				{
					temp2 = Config.Channel[Config.Channel[i].P1_source_c].P1_value;
				}

				temp2 = scale32(temp2, Config.Channel[i].P1_source_c_volume);
				P1_solution = P1_solution + temp2;
			}
			if ((Config.Channel[i].P1_source_d_volume !=0) && (Config.Channel[i].P1_source_d != NOMIX)) // Mix in fourth extra source
			{
				if (Config.Channel[i].P1_source_d > (MAX_OUTPUTS - 1))
				{
					temp2 = RCinputs[Config.Channel[i].P1_source_d - MAX_OUTPUTS];
				}
				else
				{
					temp2 = Config.Channel[Config.Channel[i].P1_source_d].P1_value;
				}

				temp2 = scale32(temp2, Config.Channel[i].P1_source_d_volume);
				P1_solution = P1_solution + temp2;
			}
		}

		// Mix in other outputs here (P2)
		if ((Transition_state == TRANS_1) || (TRANSITIONING))	
		{
			if ((Config.Channel[i].P2_source_a_volume !=0) && (Config.Channel[i].P2_source_a != NOMIX)) // Mix in first extra source
			{
				if (Config.Channel[i].P2_source_a > (MAX_OUTPUTS - 1))
				{
					temp2 = RCinputs[Config.Channel[i].P2_source_a - MAX_OUTPUTS];
				}
				else
				{
					temp2 = Config.Channel[Config.Channel[i].P2_source_a].P2_value;
				}

				temp2 = scale32(temp2, Config.Channel[i].P2_source_a_volume);
				P2_solution = P2_solution + temp2;
			}
			if ((Config.Channel[i].P2_source_b_volume !=0) && (Config.Channel[i].P2_source_b != NOMIX)) // Mix in second extra source
			{
				if (Config.Channel[i].P2_source_b > (MAX_OUTPUTS - 1))
				{
					temp2 = RCinputs[Config.Channel[i].P2_source_b - MAX_OUTPUTS];
				}
				else
				{
					temp2 = Config.Channel[Config.Channel[i].P2_source_b].P2_value;
				}

				temp2 = scale32(temp2, Config.Channel[i].P2_source_b_volume);
				P2_solution = P2_solution + temp2;
			}
			if ((Config.Channel[i].P2_source_c_volume !=0) && (Config.Channel[i].P2_source_c != NOMIX)) // Mix in third extra source
			{
				if (Config.Channel[i].P2_source_c > (MAX_OUTPUTS - 1))
				{
					temp2 = RCinputs[Config.Channel[i].P2_source_c - MAX_OUTPUTS];
				}
				else
				{
					temp2 = Config.Channel[Config.Channel[i].P2_source_c].P2_value;
				}

				temp2 = scale32(temp2, Config.Channel[i].P2_source_c_volume);
				P2_solution = P2_solution + temp2;
			}
			if ((Config.Channel[i].P2_source_d_volume !=0) && (Config.Channel[i].P2_source_d != NOMIX)) // Mix in fourth extra source
			{
				if (Config.Channel[i].P2_source_d > (MAX_OUTPUTS - 1))
				{
					temp2 = RCinputs[Config.Channel[i].P2_source_d - MAX_OUTPUTS];
				}
				else
				{
					temp2 = Config.Channel[Config.Channel[i].P2_source_d].P2_value;
				}

				temp2 = scale32(temp2, Config.Channel[i].P2_source_d_volume);
				P2_solution = P2_solution + temp2;
			}
		}
				
		// Save solution for this channel. Note that this contains cross-mixed data from the *last* cycle
		Config.Channel[i].P1_value = P1_solution;
		Config.Channel[i].P2_value = P2_solution;

	} // Mixer loop: for (i = 0; i < MAX_OUTPUTS; i++)

	//************************************************************
	// Mixer transition code
	//************************************************************ 
/*
	switch (Transition_state)
	{
		case 	TRANSITIONING:
			// Convert number to percentage (0 to 100%)
*/			if (Config.TransitionSpeed == 0) 
			{
			// RCinput range is 2500 for 1ms (+/-1250). Ran measured the +/-1000 to equate to +/-125% on his radio
			// That would make +/-100% equate to +/-800 counts for RCinput.
			// transition_value_16 is the RCinput / 16 so can range +/-50 for +/-800
				//transition  = (transition_value_16 >> 4); // 

				// transition_value_16 is the RCinput / 16 so can range +/-62 for +/-1000
				// Trim that value down to +/-50 for just over +/-1000
				transition  = (transition_value_16 >> 1); // 62/2 = 31+
				transition += (transition_value_16 >> 3); // 62/4 = 15+
				transition += (transition_value_16 >> 3); // 62/16 = 3 = 49

				// Limit extent of transition value
				if (transition < -50) transition = -50;
				if (transition > 50) transition = 50;
				transition += 50;
			}
			else 
			{
				transition = transition_counter;
			}

			for (i = 0; i < MAX_OUTPUTS; i++)
			{
				// Get source channel value
				temp1 = Config.Channel[i].P1_value;
				temp1 = scale32(temp1, (100 - transition));

				// Get destination channel value
				temp2 = Config.Channel[i].P2_value;
				temp2 = scale32(temp2, transition);

				// Sum the mixers
				temp1 = temp1 + temp2;

				// Save transitioned solution into P1
				Config.Channel[i].P1_value = temp1;
			} 
/*			break;

		case TRANS_0: // Do nothing as P1 values are correct
			break;

		case TRANS_1: // Do nothing as P2 values are correct
			break;

		default:
			break;
	}
*/
	//************************************************************
	// Per-channel 3-point offset needs to be post transition loop 
	// as it is non-linear
	//************************************************************ 

	for (i = 0; i < MAX_OUTPUTS; i++)
	{
		// Work out distance to cover over stage 1 (P1 to P1.n)
		temp1 = Config.Channel[i].P1n_offset - Config.Channel[i].P1_offset;
		temp1 = temp1 << 7; // Multiply by 128 so divide gives reasonable setp values

		// Divide distance into steps
		temp2 = Config.Channel[i].P1n_position; 
		Step1 = temp1 / temp2;
		
		// Work out distance to cover over stage 2 (P1.n to P2)
		temp2 = Config.Channel[i].P2_offset - Config.Channel[i].P1n_offset;
		temp2 = temp2 << 7;

		// Divide distance into steps
		temp1 = (100 - Config.Channel[i].P1n_position); 
		Step2 = temp2 / temp1; 	

		// Set start (P1) point
		temp3 = Config.Channel[i].P1_offset; // Promote to 16bits
		temp3 = temp3 << 7;

		// Count up transition steps of the appropriate step size
		for (j = 0; j < transition; j++)
		{
			// If in stage 1 use Step1 size
			if (j < Config.Channel[i].P1n_position)
			{
				temp3 += Step1;
			}
			// If in stage 2 use Step2 size
			else
			{
				temp3 += Step2;
			}
		}

		// Reformat directly into a system-compatible value (+/-1250)
		P1_solution = (temp3 >> 4) + (temp3 >> 5); 							// Divide by 128 then * 12 lol

		// Add offset to channel value
		Config.Channel[i].P1_value += P1_solution;
	}

} // ProcessMixer()

// Update actual limits value with that from the mix setting percentages
// This is only done at start-up and whenever the values are changed
void UpdateLimits(void)
{
	uint8_t i,j;
	int32_t temp32, gain32;

	int8_t temp8[FLIGHT_MODES][NUMBEROFAXIS] = 
		{
			{Config.FlightMode[P1].Roll_limit, Config.FlightMode[P1].Pitch_limit, Config.FlightMode[P1].Yaw_limit},
			{Config.FlightMode[P2].Roll_limit, Config.FlightMode[P2].Pitch_limit, Config.FlightMode[P2].Yaw_limit}
		};

	int8_t gains[FLIGHT_MODES][NUMBEROFAXIS] = 
		{
			{Config.FlightMode[P1].Roll.I_mult, Config.FlightMode[P1].Pitch.I_mult, Config.FlightMode[P1].Yaw.I_mult},
			{Config.FlightMode[P1].Roll.I_mult, Config.FlightMode[P1].Pitch.I_mult, Config.FlightMode[P1].Yaw.I_mult}
		};

	// Update triggers
	Config.PowerTriggerActual = Config.PowerTrigger * 10;

	// Update I_term input constraints for all profiles
	for (j = 0; j < FLIGHT_MODES; j++)
	{
		for (i = 0; i < NUMBEROFAXIS; i++)
		{
			temp32 	= temp8[j][i]; 						// Promote

			// I-term output (throw)
			// A value of 80,000 results in +/- 1250 or full throw at the output stage when set to 125%
			Config.Raw_I_Limits[j][i] = temp32 * (int32_t)640;	// 125% * 640 = 80,000

			// I-term source limits. These have to be different due to the I-term gain setting
			// For a gain of 32 and 125%, Constrain = 80,000
			// For a gain of 100 and 125%, Constrain = 32,768
			// For a gain of 10 and 25%, Constrain = 51,200
			// For a gain of 1 and 100%, Constrain = 2,048,000
			// For a gain of 127 and 125%, Constrain = 20,157
			if (gains[j][i] != 0)
			{
				gain32 = (int32_t)gains[j][i];
				gain32 = gain32 << 7;				// Multiply divisor by 128
				Config.Raw_I_Constrain[j][i] = Config.Raw_I_Limits[j][i] / (gain32 / (int32_t)32);
				Config.Raw_I_Constrain[j][i] = Config.Raw_I_Constrain[j][i] << 7; // Restore by multiplying total by 128
			}
			else 
			{
				Config.Raw_I_Constrain[j][i] = 0;
			}
		}
	}

	// Update travel limits
	for (i = 0; i < MAX_OUTPUTS; i++)
	{
		Config.Limits[i].minimum = scale_percent(Config.min_travel[i]);
		Config.Limits[i].maximum = scale_percent(Config.max_travel[i]);
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
}

// Update servos from the mixer Config.Channel[i].value data, add offsets and enforce travel limits
void UpdateServos(void)
{
	uint8_t i;
	int16_t temp1 = 0; // Output value

	for (i = 0; i < MAX_OUTPUTS; i++)
	{
		// Servo reverse and trim for the eight physical outputs
		temp1 = Config.Channel[i].P1_value;

		// Reverse this channel for the eight physical outputs
		if ((i <= MAX_OUTPUTS) && (Config.Servo_reverse[i] == ON))
		{	
			temp1 = -temp1;
		}

		// Add offset value to restore to system compatible value
//		temp1 += Config.Limits[i].trim;
		temp1 += 3750;

		// Enforce min, max travel limits
		if (temp1 > Config.Limits[i].maximum)
		{
			ServoOut[i] = Config.Limits[i].maximum;
		}

		else if (temp1 < Config.Limits[i].minimum)
		{
			ServoOut[i] = Config.Limits[i].minimum;
		}
		// Transfer value to servo
		else
		{
			ServoOut[i] = temp1;
		}
	}
}

// 32 bit multiply/scale for broken GCC
// Returns immediately if multiplier is 100
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
		value16 = -value16;	
	}

	// Zero if 0%
	else if (multiplier16 == 0)
	{
		value16 = 0;	
	}

	// Only do the scaling if necessary
	else
	{
		// GCC broken bad regarding multiplying 32 bit numbers, hence all this crap...
		mult32 = multiplier16;
		temp32 = value16;
		temp32 = temp32 * mult32;

		// Divide by 100 to get scaled value
		temp32 = temp32 / (int32_t)100; // I shit you not...
		value16 = (int16_t) temp32;
	}

	return value16;
}

// Scale percentages to position
int16_t scale_percent(int8_t value)
{
	int16_t temp16_1, temp16_2;

	temp16_1 = value; // Promote
	temp16_2 = ((temp16_1 * (int16_t)12) + 3750);

	return temp16_2;
}


// Scale percentages to relative position
int16_t scale_percent_nooffset(int8_t value)
{
	int16_t temp16_1, temp16_2;

	temp16_1 = value; // Promote
	temp16_2 = (temp16_1 * (int16_t)12);

	return temp16_2;
}
