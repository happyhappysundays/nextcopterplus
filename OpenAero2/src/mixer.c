//***********************************************************
//* mixer.c
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

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

//************************************************************
// Prototypes
//************************************************************

void ProcessMixer(void);
void UpdateServos(void);
void UpdateLimits(void);
void get_preset_mix (channel_t*);

int16_t scale32(int16_t value16, int16_t multiplier16);
int16_t scale_percent(int8_t value);

//************************************************************
// Mix tables (both RC inputs and servo/ESC outputs)
//************************************************************

// Aeroplane mixer defaults
channel_t AEROPLANE_MIX[MAX_OUTPUTS] PROGMEM = 
{
	// Rudder -= Yaw; (normal)
	// Aileron -= Roll; (normal)
	// Elevator -= Pitch; (normal)

	// (IN) source_a,src_vol,source_b,src_vol,roll_gyro,gyro_pol,pitch_gyro,pol,yaw_gyro,pol,roll_acc,pol,pitch_acc,pol
	// (OUT) source_a,source_a_volume,source_b,source_b_volume,source_c,source_c_volume,source_d,source_d_volume

	{0,THROTTLE,100,NOCHAN,0,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,CH1,100,CH2,0,CH3,0,CH4,0,-100,100,0,0},// ServoOut1 (Throttle)
	{0,NOCHAN  ,100,NOCHAN,0,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,0,0,0,0,0,0,0,0,-100,100,0,0}, 		// ServoOut2
	{0,NOCHAN  ,100,NOCHAN,0,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,0,0,0,0,0,0,0,0,-100,100,0,0}, 		// ServoOut3
	{0,NOCHAN  ,100,NOCHAN,0,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,0,0,0,0,0,0,0,0,-100,100,0,0},  	// ServoOut4
	{0,ELEVATOR,100,NOCHAN,0,OFF,NORMAL,ON, NORMAL,OFF,NORMAL,OFF,NORMAL,ON, NORMAL,CH5,100,0,0,0,0,0,0,-100,100,0,0}, 	// ServoOut5 (Elevator)
	{0,AILERON ,100,NOCHAN,0,ON, NORMAL,OFF,NORMAL,OFF,NORMAL,ON, NORMAL,OFF,NORMAL,CH6,100,0,0,0,0,0,0,-100,100,0,0},	// ServoOut6 (Left aileron)
	{0,NOCHAN  ,100,NOCHAN,0,ON, NORMAL,OFF,NORMAL,OFF,NORMAL,ON, NORMAL,OFF,NORMAL,CH7,100,0,0,0,0,0,0,-100,100,0,0}, 	// ServoOut7 (Right aileron)
	{0,RUDDER  ,100,NOCHAN,0,OFF,NORMAL,OFF,NORMAL,ON, NORMAL,OFF,NORMAL,OFF,NORMAL,CH8,100,0,0,0,0,0,0,-100,100,0,0},  // ServoOut8 (Rudder)
}; 

channel_t FLYING_WING_MIX[MAX_OUTPUTS] PROGMEM = 
{
	// Rudder -= Yaw; (normal)
	// LAileron += Roll; (reversed)
	// LElevator -= Pitch; (normal)
	// RAileron += Roll;(reversed)
	// RElevator += Pitch;(reversed)
	
	{0,THROTTLE,100,NOCHAN,0,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,CH1,100,0,0,0,0,0,0,-100,100,0,0}, 	// ServoOut1
	{0,NOCHAN,  100,NOCHAN,0,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,CH1,0,0,0,0,0,0,0,-100,100,0,0}, 	// ServoOut2
	{0,NOCHAN,  100,NOCHAN,0,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,CH1,0,0,0,0,0,0,0,-100,100,0,0}, 	// ServoOut3
	{0,NOCHAN,  100,NOCHAN,0,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,CH1,0,0,0,0,0,0,0,-100,100,0,0},  	// ServoOut4
	{0,AILERON, 100,NOCHAN,0,ON, REVERSED,ON,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,CH5,0,0,0,0,0,0,0,-100,100,0,0}, 	// ServoOut5
	{0,ELEVATOR,100,NOCHAN,0,ON, REVERSED,ON,NORMAL,OFF,NORMAL,ON,REVERSED,ON,NORMAL,CH6,0,0,0,0,0,0,0,-100,100,0,0},	// ServoOut6 (left elevon)
	{0,AILERON, 100,NOCHAN,0,ON, REVERSED,ON,REVERSED,OFF,NORMAL,ON,REVERSED,ON,REVERSED,CH7,0,0,0,0,0,0,0,-100,100,0,0},// ServoOut7 (right elevon)
	{0,RUDDER,  100,NOCHAN,0,OFF,NORMAL,OFF,NORMAL,ON, NORMAL,OFF,NORMAL,OFF,NORMAL,CH8,0,0,0,0,0,0,0,-100,100,0,0}, 	// ServoOut8
}; 
/*
channel_t CAM_STAB[MAX_OUTPUTS] PROGMEM = 
{
 	// For presets, use
	// M2 Pitch (Tilt) + Pitch gyro;
 	// M3 Yaw	(Pan) + Yaw;
 	// M4 Roll (Roll - only for 3-axis gimbals) + Roll gyro;

 	// For controlled axis, use
	// M6 Pitch (Tilt) + Pitch gyro;
 	// M7 Yaw	(Pan) + Yaw;
 	// M8 Roll (Roll - only for 3-axis gimbals) + Roll gyro;
		
	{0,NOCHAN, 100,NOCHAN,0,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,CH1,100,0,0,0,0,0,0,-100,100,0,0},// ServoOut1
	{0,NOCHAN, 100,NOCHAN,0,OFF,NORMAL,ON, NORMAL,OFF,NORMAL,OFF,NORMAL,ON, NORMAL,CH1,0,0,0,0,0,0,0,-100,100,0,0}, 	// ServoOut2 (Tilt axis)
	{0,NOCHAN, 100,NOCHAN,0,OFF,NORMAL,OFF,NORMAL,ON, NORMAL,OFF,NORMAL,OFF,NORMAL,CH1,0,0,0,0,0,0,0,-100,100,0,0}, 	// ServoOut3 (Pan axis)
	{0,NOCHAN, 100,NOCHAN,0,ON, NORMAL,OFF,NORMAL,OFF,NORMAL,ON, NORMAL,OFF,NORMAL,CH1,0,0,0,0,0,0,0,-100,100,0,0},  // ServoOut4 (Roll axis)
	{0,NOCHAN, 100,NOCHAN,0,ON, REVERSED,ON,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,CH5,0,0,0,0,0,0,0,-100,100,0,0}, // ServoOut5
	{0,ELEVATOR,100,NOCHAN,0,OFF,NORMAL,ON,NORMAL,OFF,NORMAL,OFF,NORMAL,ON, NORMAL,CH6,0,0,0,0,0,0,0,-100,100,0,0}, 	// ServoOut6 (Tilt axis)
	{0,RUDDER, 100,NOCHAN,0,OFF,NORMAL,OFF,NORMAL,ON,NORMAL, OFF,NORMAL,OFF,NORMAL,CH7,0,0,0,0,0,0,0,-100,100,0,0}, 	// ServoOut7 (Pan axis)
	{0,AILERON,100,NOCHAN,0,ON, NORMAL,OFF,NORMAL,OFF,NORMAL,ON, NORMAL,OFF,NORMAL,CH8,0,0,0,0,0,0,0,-100,100,0,0},  // ServoOut8 (Roll axis)
};
*/
//************************************************************
// Get preset mix from Program memory
void get_preset_mix(channel_t* preset)
{
	memcpy_P(&Config.Channel[0].value,&preset[0].value,(sizeof(channel_t) * MAX_OUTPUTS));
}

void ProcessMixer(void)
{
	uint8_t i, outputs;
	int16_t temp = 0;
	int16_t temp2 = 0;
	int16_t flap = 0;

	// Quick fudge to allow easy look-up of which channels require expo
	uint8_t expos[MAX_OUTPUTS] = {0,0,0,0,Config.ElevatorExpo,Config.AileronExpo,Config.AileronExpo,Config.RudderExpo};

	//************************************************************
	// Limit output mixing as needed to save processing power
	//************************************************************

	if (Config.CamStab == ON)
	{
		outputs = MIN_OUTPUTS;
	}
	else
	{
		outputs = MAX_OUTPUTS;
	}

	//************************************************************
	// Zero all channel values to start
	//************************************************************

	for (i = 0; i < outputs; i++)
	{
		Config.Channel[i].value = 0;
	}

	//************************************************************
	// Un-mix flaps from flaperons as required
	//************************************************************ 

	if (Config.FlapChan != NOCHAN)
	{
		// Aileron common movement (Flap x 2). Roll cancelled
		flap = RCinputs[AILERON] - RCinputs[Config.FlapChan]; 	
		flap = flap >> 1; 
	}
	else
	{
		flap = 0;
	}

	//************************************************************
	// Process differential for dual-aileron setups 
	//************************************************************

	if (Config.Differential != 0) // Skip if zero
	{
		// If flaperons set up 
		if (Config.FlapChan != NOCHAN)
		{
			// Recreate actual roll signal from flaperons
			temp  = RCinputs[AILERON] + RCinputs[Config.FlapChan];
			temp  = temp >> 1;
		}
		// If flaperons not set up
		else
		{
			temp  = RCinputs[AILERON];
		}

		// Copy to other flaperon
		temp2 = temp;

		// Apply differential
		if (temp > 0)			// For one side only
		{
			temp = scale32(temp, (100 - Config.Differential));
		}

		if (temp2 < 0)			// For one side only
		{
			temp2 = scale32(temp2, (100 - Config.Differential));
		}

		// Add flap back into aileron
		RCinputs[AILERON] = temp + flap;
		
		// If flaperons set up 
		if (Config.FlapChan != NOCHAN)
		{
			// Add flap back into aileron
			RCinputs[Config.FlapChan] = temp2 - flap;
		}
	}

	//************************************************************
	// Process RC mixing, expo and source volume calculation
	//************************************************************

	for (i = 0; i < outputs; i++)
	{
		// Get requested volume of Source A
		temp = RCinputs[Config.Channel[i].source_a];
		temp = scale32(temp, Config.Channel[i].source_a_volume);

		// Apply any expo as set by user
		temp = get_expo_value(temp, expos[i]);

		// Skip Source B if no RC mixing required for this channel
		if (Config.Channel[i].source_b_volume != 0)
		{
			temp2 = RCinputs[Config.Channel[i].source_b];
			temp2 = scale32(temp2, Config.Channel[i].source_b_volume);
			temp2 = get_expo_value(temp2, expos[i]);

			// Sum the mixers
			temp = temp + temp2;
		}

		// Save solution for now
		Config.Channel[i].value = temp;
	}

	//************************************************************
	// Mix in gyros and acc
	//************************************************************ 

	// Stabilised modes
	if (Stability || AutoLevel)
	{
		// Process sensor mixers
		for (i = 0; i < outputs; i++)
		{
			// Discard RC part of fly-by-wire or 3D channels as these are replaced by that from the PID loop
			if (
				// For fly-by-wire flight mode or when in 3D Heading hold mode
				(((Config.FlightMode == FLYBYWIRE) || (Config.AutoCenter == FIXED))
			     &&
				 // ...and for the RC channels
			     (
					(Config.Channel[i].source_a == AILERON)	||
					(Config.Channel[i].source_a == Config.FlapChan) ||
					(Config.Channel[i].source_a == ELEVATOR)||

					// ...except rudder which is a special case as AutoLevel will only replace values for Roll/Pitch
					((Config.Channel[i].source_a == RUDDER) && !AutoLevel)
				 )
				)
			   )
			{
				// Clear solution
				temp = 0;
			}

			else
			{
				// Otherwise use the RC value as a base
				temp = Config.Channel[i].value;
			}

			// Use PID gyro values
			if (Stability)
			{
				if (Config.Channel[i].roll_gyro == ON)
				{
						
					if (Config.Channel[i].roll_gyro_polarity == REVERSED)
					{
						temp = temp - PID_Gyros[ROLL];
					}
					else
					{
						temp = temp + PID_Gyros[ROLL];
					}
				}

				if (Config.Channel[i].pitch_gyro == ON)
				{
					if (Config.Channel[i].pitch_gyro_polarity == REVERSED)
					{
						temp = temp - PID_Gyros[PITCH];
					}
					else
					{
						temp = temp + PID_Gyros[PITCH];
					}
				}

				if (Config.Channel[i].yaw_gyro == ON)
				{
					if (Config.Channel[i].yaw_gyro_polarity == REVERSED)
					{
						temp = temp - PID_Gyros[YAW];
					}
					else
					{
						temp = temp + PID_Gyros[YAW];
					}
				}
			} // Stability

			// Add PID acc values including trim
			if (AutoLevel)
			{
				if (Config.Channel[i].roll_acc == ON)
				{
					// Add in Roll trim
					temp += Config.AccRollZeroTrim;

					if (Config.Channel[i].roll_acc_polarity == REVERSED)
					{
						temp = temp - PID_ACCs[ROLL];
					}
					else
					{
						temp = temp + PID_ACCs[ROLL];
					}
				}

				if (Config.Channel[i].pitch_acc == ON)
				{
					// Add in Pitch trim
					temp += Config.AccPitchZeroTrim;

					if (Config.Channel[i].pitch_acc_polarity == REVERSED)
					{
						temp = temp - PID_ACCs[PITCH];
					}
					else
					{
						temp = temp + PID_ACCs[PITCH];
					}
				}
			} // Autolevel

			// Update channel data solution
			Config.Channel[i].value = temp;

			//************************************************************
			// Re-mix flaps from flaperons as required
			//************************************************************ 

			// If set up for flaperons restore for fly-by-wire mode
			if  ((Config.FlapChan != NOCHAN) && (Config.FlightMode == FLYBYWIRE))
			{
				if (Config.Channel[i].source_a == AILERON)
				{
					Config.Channel[i].value += flap;
				}
				if (Config.Channel[i].source_a == Config.FlapChan)
				{
					Config.Channel[i].value -= flap;
				}
			}

		} // Process sensor mixers
	} // Stabilised modes
	

		
	//************************************************************
	// Process output mixers
	//************************************************************ 

	for (i = 0; i < outputs; i++)
	{
		// Get primary value (normally its own channel)
		temp = Config.Channel[Config.Channel[i].output_a].value;
		
		if (Config.Channel[i].output_a_volume !=0) // Mix in primary output volume
		{	
			temp = scale32(temp, Config.Channel[i].output_a_volume);
		}
		if (Config.Channel[i].output_b_volume !=0) // Mix in first extra output
		{
			temp2 = Config.Channel[Config.Channel[i].output_b].value;
			temp2 = scale32(temp2, Config.Channel[i].output_b_volume);
			temp = temp + temp2;
		}
		if (Config.Channel[i].output_c_volume !=0) // Mix in second extra output
		{
			temp2 = Config.Channel[Config.Channel[i].output_c].value;
			temp2 = scale32(temp2, Config.Channel[i].output_c_volume);
			temp = temp + temp2;
		}
		if (Config.Channel[i].output_d_volume !=0) // Mix in third extra output
		{
			temp2 = Config.Channel[Config.Channel[i].output_d].value;
			temp2 = scale32(temp2, Config.Channel[i].output_d_volume);
			temp = temp + temp2;
		}

		Config.Channel[i].value = temp;
	}

	//************************************************************
	// Add offset value to restore to system compatible value
	//************************************************************ 

	for (i = 0; i < outputs; i++)
	{
		Config.Channel[i].value += Config.Limits[i].trim;
	}
}

// Update actual limits value with that from the mix setting percentages
// This is only done at start-up and whenever the values are changed
void UpdateLimits(void)
{
	uint8_t i;
	int8_t temp8;
	int32_t temp32, gain32;
	int8_t gains[3] = {Config.Roll.I_mult, Config.Pitch.I_mult, Config.Yaw.I_mult};

	// Update triggers
	Config.HandsFreetrigger = Config.Autolimit * 5;
	Config.Stabtrigger = scale_percent(Config.Stablimit);
	Config.Autotrigger = scale_percent(Config.Autolimit);
	Config.Launchtrigger = scale_percent(Config.LaunchThrPos);

	// Update I_term limits
	for (i = 0; i < 3; i++)
	{
		temp8 	= Config.I_Limits[i];			// 0 to 125%
		temp32 	= temp8; 						// Promote

		// I-term output (throw)
		// A value of 80,000 results in +/- 1250 or full throw at the output stage when set to 125%
		Config.Raw_I_Limits[i] = temp32 * (int32_t)640;	// 125% * 640 = 80,000

		// I-term source limits. These have to be different due to the I-term gain setting
		// For a gain of 32 and 125%, Constrain = 80,000
		// For a gain of 100 and 125%, Constrain = 32,768
		// For a gain of 10 and 25%, Constrain = 51,200
		// For a gain of 1 and 100%, Constrain = 2,048,000
		// For a gain of 127 and 125%, Constrain = 20,157
		if (gains[i] != 0)
		{
			gain32 = (int32_t)gains[i];
			gain32 = gain32 << 7;				// Multiply divisor by 128
			Config.Raw_I_Constrain[i] = Config.Raw_I_Limits[i] / (gain32 / (int32_t)32);
			Config.Raw_I_Constrain[i] = Config.Raw_I_Constrain[i] << 7; // Restore by multiplying total by 128
		}
		else 
		{
			Config.Raw_I_Constrain[i] = 0;
		}
	}

	// Update travel limits
	for (i = 0; i < MAX_OUTPUTS; i++)
	{
		Config.Limits[i].minimum = scale_percent(Config.Channel[i].min_travel);
		Config.Limits[i].maximum = scale_percent(Config.Channel[i].max_travel);
		Config.Limits[i].failsafe = scale_percent(Config.Channel[i].Failsafe);
		Config.Limits[i].trim = scale_percent(Config.Channel[i].Offset);
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
}

// Update servos from the mixer Config.Channel[i].value data and enforce travel limits
void UpdateServos(void)
{
	uint8_t i;

	for (i = 0; i < MAX_OUTPUTS; i++)
	{
	
		if (Config.Channel[i].value < Config.Limits[i].minimum) 
		{
			ServoOut[i] = Config.Limits[i].minimum;
		}
		else if (Config.Channel[i].value > Config.Limits[i].maximum)
		{
			ServoOut[i] = Config.Limits[i].maximum;
		}
		else
		{
			ServoOut[i] = Config.Channel[i].value;
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
