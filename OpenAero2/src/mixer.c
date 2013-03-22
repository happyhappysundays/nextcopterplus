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
#include "..\inc\mixer.h"

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
	// Elevator += Pitch; (normal)

	// Value, Servo_reverse, Offset, min_travel, max_travel, Failsafe,
	// source_a, source_a_vol, source_b,src_vol,source_mix, roll_gyro,gyro_pol,pitch_gyro,pol,yaw_gyro,pol,roll_acc,pol,pitch_acc,pol
	// source_b,source_b_volume,source_c,source_c_volume,source_d,source_d_volume

	{0,NORMAL,0,-100,100,-100,THROTTLE,100,NOCHAN,0,ON ,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,UNUSED,0,UNUSED,0,UNUSED,0},	// ServoOut1 (Throttle)
	{0,NORMAL,0,-100,100,   0,NOCHAN,100,NOCHAN,0,OFF,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,UNUSED,0,UNUSED,0,UNUSED,0},	// ServoOut2
	{0,NORMAL,0,-100,100,   0,NOCHAN,100,NOCHAN,0,OFF,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,UNUSED,0,UNUSED,0,UNUSED,0},	// ServoOut3
	{0,NORMAL,0,-100,100,   0,NOCHAN,100,NOCHAN,0,OFF,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,UNUSED,0,UNUSED,0,UNUSED,0},	// ServoOut4
	{0,NORMAL,0,-100,100, 0,ELEVATOR,100,NOCHAN,0,OFF,OFF,NORMAL,ON, NORMAL,OFF,NORMAL,OFF,NORMAL,ON, NORMAL,UNUSED,0,UNUSED,0,UNUSED,0},	// ServoOut5 (Elevator)
	{0,NORMAL,0,-100,100, 0,AILERON,100,NOCHAN,0,OFF,ON, NORMAL,OFF,NORMAL,OFF,NORMAL,ON, NORMAL,OFF,NORMAL,UNUSED,0,UNUSED,0,UNUSED,0},	// ServoOut6 (Left aileron)
	{0,NORMAL,0,-100,100,   0,NOCHAN,100,NOCHAN,0,OFF,ON, NORMAL,OFF,NORMAL,OFF,NORMAL,ON, NORMAL,OFF,NORMAL,UNUSED,0,UNUSED,0,UNUSED,0},	// ServoOut7 (Right aileron)
	{0,NORMAL,0,-100,100,   0,RUDDER,100,NOCHAN,0,OFF,OFF,NORMAL,OFF,NORMAL,ON, NORMAL,OFF,NORMAL,OFF,NORMAL,UNUSED,0,UNUSED,0,UNUSED,0},	// ServoOut8 (Rudder)

}; 

channel_t FLYING_WING_MIX[MAX_OUTPUTS] PROGMEM = 
{
	// Rudder -= Yaw (normal)
	// L.Elevon + Roll (reversed) - Pitch (normal)
	// R.Elevon + Roll (reversed) + Pitch (reversed)

	{0,NORMAL,0,-100,100,-100,THROTTLE,100,NOCHAN,0,ON ,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,UNUSED,0,UNUSED,0,UNUSED,0},	// ServoOut1 (Throttle)
	{0,NORMAL,0,-100,100,   0,NOCHAN,100,NOCHAN,0,OFF,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,UNUSED,0,UNUSED,0,UNUSED,0},	// ServoOut2
	{0,NORMAL,0,-100,100,   0,NOCHAN,100,NOCHAN,0,OFF,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,UNUSED,0,UNUSED,0,UNUSED,0},	// ServoOut3
	{0,NORMAL,0,-100,100,   0,NOCHAN,100,NOCHAN,0,OFF,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,UNUSED,0,UNUSED,0,UNUSED,0},	// ServoOut4
	{0,NORMAL,0,-100,100, 	0,NOCHAN,100,NOCHAN,0,OFF,OFF,NORMAL,ON, NORMAL,OFF,NORMAL,OFF,NORMAL,ON, NORMAL,UNUSED,0,UNUSED,0,UNUSED,0},	// ServoOut5
	{0,NORMAL,0,-100,100, 	0,ELEVATOR,-50,AILERON,-50,OFF,ON, REVERSED,ON,REVERSED,OFF,NORMAL,ON,REVERSED,ON,REVERSED,UNUSED,0,UNUSED,0,UNUSED,0},// ServoOut6 (Left elevon)
	{0,NORMAL,0,-100,100,   0,AILERON,50,ELEVATOR,-50,OFF,ON,NORMAL,ON,REVERSED,OFF,NORMAL,ON,NORMAL,ON,REVERSED,UNUSED,0,UNUSED,0,UNUSED,0},// ServoOut7 (Left elevon)
	{0,NORMAL,0,-100,100,   0,RUDDER,100,NOCHAN,0,OFF,OFF,NORMAL,OFF,NORMAL,ON, NORMAL,OFF,NORMAL,OFF,NORMAL,UNUSED,0,UNUSED,0,UNUSED,0},	// ServoOut8 (Rudder)
}; 

channel_t CAM_STAB[MAX_OUTPUTS] PROGMEM = 
{
 	// For non-controlled, use
	// M2 Pitch (Tilt) + Pitch gyro;
 	// M3 Yaw	(Pan) + Yaw;
 	// M4 Roll (Roll - only for 3-axis gimbals) + Roll gyro;

 	// For controlled axis, use
	// M6 Pitch (Tilt) + Pitch gyro;
 	// M7 Yaw	(Pan) + Yaw;
 	// M8 Roll (Roll - only for 3-axis gimbals) + Roll gyro;
	{0,NORMAL,0,-100,100,0,NOCHAN,100,NOCHAN,0,OFF,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,UNUSED,0,UNUSED,0,UNUSED,0},	// ServoOut1 (Throttle)
	{0,NORMAL,0,-100,100,0,NOCHAN,100,NOCHAN,0,OFF,OFF,NORMAL,ON, NORMAL,OFF,NORMAL,OFF,NORMAL,ON, NORMAL,UNUSED,0,UNUSED,0,UNUSED,0},	// ServoOut2
	{0,NORMAL,0,-100,100,0,NOCHAN,100,NOCHAN,0,OFF,OFF,NORMAL,OFF,NORMAL,ON, NORMAL,OFF,NORMAL,OFF,NORMAL,UNUSED,0,UNUSED,0,UNUSED,0},	// ServoOut3
	{0,NORMAL,0,-100,100,0,NOCHAN,100,NOCHAN,0,OFF,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,ON, NORMAL,OFF,NORMAL,UNUSED,0,UNUSED,0,UNUSED,0},	// ServoOut4
	{0,NORMAL,0,-100,100,0,NOCHAN,100,NOCHAN,0,OFF,OFF,NORMAL,ON, NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,UNUSED,0,UNUSED,0,UNUSED,0},	// ServoOut5 (Elevator)
	{0,NORMAL,0,-100,100,0,ELEVATOR,100,NOCHAN,0,OFF,OFF,NORMAL,ON,NORMAL,OFF,NORMAL,OFF,NORMAL,ON, NORMAL,UNUSED,0,UNUSED,0,UNUSED,0},	// ServoOut6 (Left aileron)
	{0,NORMAL,0,-100,100,0,RUDDER,100,NOCHAN,0,OFF,OFF,NORMAL,OFF,NORMAL,ON, NORMAL,OFF,NORMAL,OFF,NORMAL,UNUSED,0,UNUSED,0,UNUSED,0},	// ServoOut7 (Right aileron)
	{0,NORMAL,0,-100,100,0,AILERON,100,NOCHAN,0,OFF,ON,NORMAL,OFF,NORMAL,OFF,NORMAL,ON, NORMAL,OFF,NORMAL,UNUSED,0,UNUSED,0,UNUSED,0},	// ServoOut8 (Rudder)
};

//************************************************************

void ProcessMixer(void)
{
	uint8_t i, outputs;
	int16_t temp = 0;
	int16_t temp2 = 0;
	static	int16_t flap = 0;
	static	int16_t roll = 0;
	int16_t	pitch_trim = 0;
	int16_t	roll_trim = 0;
	bool	TwoAilerons = false;
	bool	FlapLock = false;

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
		// Update flap only if ailerons are within measureable positions
		if ((RCinputs[AILERON] > -1200) && 
			(RCinputs[AILERON] < 1200) &&
			(RCinputs[Config.FlapChan] > -1200) && 
			(RCinputs[Config.FlapChan] < 1200))
		{
			flap = RCinputs[AILERON] - RCinputs[Config.FlapChan]; 	
			flap = flap >> 1; 	
			FlapLock = false;
		}
		else
		{
			FlapLock = true;
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
	// Process RC mixing and source volume calculation
	//************************************************************

	for (i = 0; i < outputs; i++)
	{
		// Get requested volume of Source A
		temp = RCinputs[Config.Channel[i].source_a];
		temp = scale32(temp, Config.Channel[i].source_a_volume);

		// Skip Source B if no RC mixing required for this channel
		if (Config.Channel[i].source_b_volume != 0)
		{
			temp2 = RCinputs[Config.Channel[i].source_b];
			temp2 = scale32(temp2, Config.Channel[i].source_b_volume);

			// Sum the mixers
			temp = temp + temp2;
		}

		// Save solution for now
		Config.Channel[i].value = temp;
	}

	//************************************************************
	// Mix in gyros
	//************************************************************ 

	// Use PID gyro values
	if (Stability)
	{
		for (i = 0; i < outputs; i++)
		{
			// Clear RC source if not needed in mix with sensors
			if (Config.Channel[i].source_mix == ON)
			{
				temp = Config.Channel[i].value;
			}
			else
			{
				temp = 0;
			}

			// Mix in gyros
			if (Config.Channel[i].roll_gyro == ON)
			{
				if (Config.Channel[i].roll_gyro_polarity == REVERSED)
				{
					temp = temp + PID_Gyros[ROLL];
				}
				else
				{
					temp = temp - PID_Gyros[ROLL];
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
					temp = temp + PID_Gyros[YAW];
				}
				else
				{
					temp = temp - PID_Gyros[YAW];
				}
			}

			// Save solution for now
			Config.Channel[i].value = temp;
		}
	} // Stability

	//************************************************************
	// Mix in accelerometers
	//************************************************************ 

	// Add PID acc values including trim
	if (AutoLevel)
	{
		// Offset Autolevel trims in failsafe mode
		if ((Config.FailsafeType == 1) && Failsafe && (Config.CamStab == OFF))
		{
			roll_trim += Config.FailsafeAileron;
			roll_trim = roll_trim << 2;
			pitch_trim += Config.FailsafeElevator;
			pitch_trim = pitch_trim << 2;
		}

		// Add autolevel trims * 4		
		roll_trim += (Config.AccRollZeroTrim << 2);
		pitch_trim += (Config.AccPitchZeroTrim << 2);

		// Mix in accelerometers
		for (i = 0; i < outputs; i++)
		{
			// Get solution
			temp = Config.Channel[i].value;

			if (Config.Channel[i].roll_acc == ON)
			{
				// Add in Roll trim
				temp += roll_trim;

				if (Config.Channel[i].roll_acc_polarity == REVERSED)
				{
					temp = temp + PID_ACCs[ROLL];
				}
				else
				{
					temp = temp - PID_ACCs[ROLL];
				}
			}

			if (Config.Channel[i].pitch_acc == ON)
			{
				// Add in Pitch trim
				temp += pitch_trim;

				if (Config.Channel[i].pitch_acc_polarity == REVERSED)
				{
					temp = temp - PID_ACCs[PITCH];
				}
				else
				{
					temp = temp + PID_ACCs[PITCH];
				}
			}

			// Save solution for now
			Config.Channel[i].value = temp;
		}
	} // Autolevel

	//************************************************************
	// Process differential if set up and two ailerons used
	//************************************************************

	if ((Config.FlapChan != NOCHAN) && (Config.Differential != 0))
	{
		// Search through outputs for aileron channels
		for (i = 0; i < outputs; i++)
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
		for (i = 0; i < outputs; i++)
		{
			// Get solution
			temp = Config.Channel[i].value;

			// Restore flaps
			if (Config.Channel[i].source_a == AILERON)
			{
				temp += flap;
			}
			if (Config.Channel[i].source_a == Config.FlapChan)
			{
				temp -= flap;
			}

			// Update channel data solution
			Config.Channel[i].value = temp;
		} // Flaps
	}
		
	//************************************************************
	// Process output mixers
	//************************************************************ 

	for (i = 0; i < outputs; i++)
	{

		// Get primary value
		temp = Config.Channel[i].value;
		
		if (Config.Channel[i].Servo_reverse == ON) // Reverse this channel's primary
		{	
			temp = -temp;
		}

		// Mix in other outputs here
		if ((Config.Channel[i].output_b_volume !=0) && (Config.Channel[i].output_b != UNUSED)) // Mix in first extra output
		{
			temp2 = Config.Channel[Config.Channel[i].output_b].value;
			temp2 = scale32(temp2, Config.Channel[i].output_b_volume);
			temp = temp + temp2;
		}
		if ((Config.Channel[i].output_c_volume !=0) && (Config.Channel[i].output_c != UNUSED)) // Mix in second extra output
		{
			temp2 = Config.Channel[Config.Channel[i].output_c].value;
			temp2 = scale32(temp2, Config.Channel[i].output_c_volume);
			temp = temp + temp2;
		}
		if ((Config.Channel[i].output_d_volume !=0) && (Config.Channel[i].output_d != UNUSED)) // Mix in third extra output
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

	//************************************************************
	// Handle Failsafe condition
	//************************************************************ 

	if (Failsafe && (Config.CamStab == OFF))
	{
		// Simple failsafe. Replace outputs with user-set values
		if (Config.FailsafeType == 0) 
		{
			for (i = 0; i < MAX_OUTPUTS; i++)
			{
				Config.Channel[i].value = Config.Limits[i].failsafe;
			}
		}

		// Advanced failsafe. Autolevel ON, use failsafe trims to adjust autolevel.
		if (Config.FailsafeType == 1)
		{
			for (i = 0; i < MAX_OUTPUTS; i++)
			{
				// Over-ride throttle if in CPPM mode
				if ((Config.Channel[i].source_a == THROTTLE) && (Config.RxMode == CPPM_MODE))
				{
					// Convert throttle setting to servo value
					Config.Channel[i].value = scale_percent(Config.FailsafeThrottle);				
				}

				// Tweak rudder channel						
				if (Config.Channel[i].source_a == RUDDER)
				{
					temp = Config.FailsafeRudder;
					temp = temp << 4;
					Config.Channel[i].value += temp;
				}
			}
		}

	} // Failsafe
}


// Get preset mix from Program memory
void get_preset_mix(channel_t* preset)
{
	memcpy_P(&Config.Channel[0].value,&preset[0].value,(sizeof(channel_t) * MAX_OUTPUTS));
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
		if (Config.Channel[i].value > Config.Limits[i].maximum)
		{
			ServoOut[i] = Config.Limits[i].maximum;
		}

		else if (Config.Channel[i].value < Config.Limits[i].minimum)
		{
			ServoOut[i] = Config.Limits[i].minimum;
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
