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

//************************************************************
// Mix tables (both RC inputs and servo/ESC outputs)
//************************************************************

// Aeroplane mixer defaults
channel_t AEROPLANE_MIX[MAX_OUTPUTS] PROGMEM = 
{
	// Rudder -= Yaw; (normal)
	// Aileron -= Roll; (normal)
	// Elevator -= Pitch; (normal)

	// (IN) Value,source_a,src_vol,source_b,src_vol,roll_gyro,gyro_pol,pitch_gyro,pol,yaw_gyro,pol,roll_acc,pol,pitch_acc,pol
	// (OUT) source_a,source_a_volume,source_b,source_b_volume,source_c,source_c_volume,source_d,source_d_volume,min,max,failsafe

	{0,THROTTLE,100,NOCHAN,0,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,CH1,100,0,0,0,0,0,0,-100,100,0}, 	// ServoOut1 (Throttle)
	{0,PRESET1,100,NOCHAN,0,OFF,NORMAL,ON,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,0,0,0,0,0,0,0,0,-100,100,0}, 		// ServoOut2
	{0,PRESET2,100,NOCHAN,0,OFF,NORMAL,OFF,NORMAL,ON,NORMAL,OFF,NORMAL,OFF,NORMAL,0,0,0,0,0,0,0,0,-100,100,0}, 		// ServoOut3
	{0,PRESET3,100,NOCHAN,0,ON,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,0,0,0,0,0,0,0,0,-100,100,0},  	// ServoOut4
	{0,ELEVATOR,100,NOCHAN,0,OFF,NORMAL,ON,NORMAL,OFF,NORMAL,OFF,NORMAL,ON,NORMAL,4,100,0,0,0,0,0,0,-100,100,0}, 	// ServoOut5 (Elevator)
	{0,AILERON,100,NOCHAN,0,ON,NORMAL,OFF,NORMAL,OFF,NORMAL,ON,NORMAL,OFF,NORMAL,5,100,0,0,0,0,0,0,-100,100,0},	 	// ServoOut6 (Left aileron)
	{0,THROTTLE,100,NOCHAN,0,ON,NORMAL,OFF,NORMAL,OFF,NORMAL,ON,NORMAL,OFF,NORMAL,6,100,0,0,0,0,0,0,-100,100,0}, 	// ServoOut7 (Right aileron)
	{0,RUDDER,100,NOCHAN,0,OFF,NORMAL,OFF,NORMAL,ON,REVERSED,OFF,NORMAL,OFF,NORMAL,7,100,0,0,0,0,0,0,-100,100,0}, 	// ServoOut8 (Rudder)
}; 

channel_t FLYING_WING_MIX[MAX_OUTPUTS] PROGMEM = 
{
	// Rudder -= Yaw; (normal)
	// LAileron += Roll; (reversed)
	// LElevator -= Pitch; (normal)
	// RAileron += Roll;(reversed)
	// RElevator += Pitch;(reversed)
	
	{0,THROTTLE,100,NOCHAN,0,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,0,100,0,0,0,0,0,0,-100,100,0}, 	// ServoOut1
	{0,PRESET1,100,NOCHAN,0,OFF,NORMAL,ON,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,0,0,0,0,0,0,0,0,-100,100,0}, 	// ServoOut2
	{0,PRESET2,100,NOCHAN,00,OFF,NORMAL,OFF,NORMAL,ON,NORMAL,OFF,NORMAL,OFF,NORMAL,0,0,0,0,0,0,0,0,-100,100,0}, 	// ServoOut3
	{0,PRESET3,100,NOCHAN,0,ON,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,0,0,0,0,0,0,0,0,-100,100,0},  	// ServoOut4
	{0,AILERON,100,NOCHAN,0,ON,REVERSED,ON,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,4,0,0,0,0,0,0,0,-100,100,0}, 	// ServoOut5
	{0,ELEVATOR,100,NOCHAN,0,ON,REVERSED,ON,NORMAL,OFF,NORMAL,ON,REVERSED,ON,NORMAL,5,0,0,0,0,0,0,0,-100,100,0},	// ServoOut6 (left elevon)
	{0,AILERON,100,NOCHAN,0,ON,REVERSED,ON,REVERSED,OFF,NORMAL,ON,REVERSED,ON,REVERSED,6,0,0,0,0,0,0,0,-100,100,0},// ServoOut7 (right elevon)
	{0,RUDDER,100,NOCHAN,0,OFF,NORMAL,OFF,NORMAL,ON,NORMAL,OFF,NORMAL,OFF,NORMAL,7,0,0,0,0,0,0,0,-100,100,0}, 	// ServoOut8
}; 

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
		
	{0,NOCHAN,100,NOCHAN,0,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,0,100,0,0,0,0,0,0,-100,100,0}, 	// ServoOut1
	{0,PRESET1,100,NOCHAN,0,OFF,NORMAL,ON,NORMAL,OFF,NORMAL,OFF,NORMAL,ON,NORMAL,0,0,0,0,0,0,0,0,-100,100,0}, 	// ServoOut2 (Tilt axis)
	{0,PRESET2,100,NOCHAN,0,OFF,NORMAL,OFF,NORMAL,ON,NORMAL,OFF,NORMAL,OFF,NORMAL,0,0,0,0,0,0,0,0,-100,100,0}, 	// ServoOut3 (Pan axis)
	{0,PRESET3,100,NOCHAN,0,ON,NORMAL,OFF,NORMAL,OFF,NORMAL,ON,NORMAL,OFF,NORMAL,0,0,0,0,0,0,0,0,-100,100,0},  	// ServoOut4 (Roll axis)
	{0,NOCHAN,100,NOCHAN,0,ON,REVERSED,ON,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,4,0,0,0,0,0,0,0,-100,100,0}, 	// ServoOut5
	{0,ELEVATOR,100,NOCHAN,0,OFF,NORMAL,ON,NORMAL,OFF,NORMAL,OFF,NORMAL,ON,NORMAL,5,0,0,0,0,0,0,0,-100,100,0}, 	// ServoOut6 (Tilt axis)
	{0,RUDDER,100,NOCHAN,0,OFF,NORMAL,OFF,NORMAL,ON,NORMAL,OFF,NORMAL,OFF,NORMAL,6,0,0,0,0,0,0,0,-100,100,0}, 	// ServoOut7 (Pan axis)
	{0,AILERON,100,NOCHAN,0,ON,NORMAL,OFF,NORMAL,OFF,NORMAL,ON,NORMAL,OFF,NORMAL,7,0,0,0,0,0,0,0,-100,100,0},  	// ServoOut8 (Roll axis)
}; 

// 120 degree Swashplate mixer defaults
channel_t SWASH120_MIX[MAX_OUTPUTS] PROGMEM = 
{
	// Aileron = aileron * 60%
	// Elevator = elevator * 60%
	// Collective = collective * 60%
	// CYC0 = Collective - elevator
	// CYC1 = Collective + elevator/2 + aileron
	// CYC2 = Collective + elevator/2 - aileron

	// (IN) Value,source_a,src_vol,source_b,src_vol,roll_gyro,gyro_pol,pitch_gyro,pol,yaw_gyro,pol,roll_acc,pol,pitch_acc,pol
	// (OUT) source_a,source_a_volume,source_b,source_b_volume,source_c,source_c_volume,source_d,source_d_volume,min,max,failsafe

	{0,THROTTLE,100,NOCHAN,0,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,CH1,100,0,0,0,0,0,0,-100,100,0}, 		// ServoOut1 (Collective)
	{0,AILERON ,100,NOCHAN,0,ON, NORMAL,OFF,NORMAL,OFF,NORMAL,ON, NORMAL,OFF,NORMAL,CH2,0,0,0,0,0,0,0,-100,100,0}, 			// ServoOut2 (Stabilised Aileron)
	{0,ELEVATOR,100,NOCHAN,0,OFF,NORMAL,ON, NORMAL,OFF,NORMAL,OFF,NORMAL,ON, NORMAL,CH3,0,0,0,0,0,0,0,-100,100,0}, 			// ServoOut3 (Stabilised Elevator)
	{0,NOCHAN  ,100,NOCHAN,0,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,CH4,0,0,0,0,0,0,0,-100,100,0},  		// ServoOut4 
	{0,RUDDER  ,100,NOCHAN,0,OFF,NORMAL,OFF,NORMAL,ON, NORMAL,OFF,NORMAL,OFF,NORMAL,CH5,100,0,0,0,0,0,0,-100,100,0}, 		// ServoOut5 (Yaw)
	{0,THROTTLE,60, NOCHAN,0,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,CH6,100,CH3,-100,0,0,0,0,-100,100,0},	// ServoOut6 (CYC0)
	{0,THROTTLE,60, NOCHAN,0,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,CH7,100,CH3,30,CH2,100,0,0,-100,100,0},	// ServoOut7 (CYC1)
	{0,THROTTLE,60, NOCHAN,0,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,CH8,100,CH3,30,CH2,-100,0,0,-100,100,0},// ServoOut8 (CYC2)
}; 

//************************************************************
// Get preset mix from Program memory
void get_preset_mix(channel_t* preset)
{
	memcpy_P(&Config.Channel[0].value,&preset[0].value,(sizeof(channel_t) * MAX_OUTPUTS));
}

void ProcessMixer(void)
{
	uint8_t i, outputs;
	int16_t temp, temp2 = 0;
	// Quick fudge to allow easy look-up of which channels require expo
	uint8_t expos[] = {0,0,0,0,Config.ElevatorExpo,Config.AileronExpo,Config.AileronExpo,Config.RudderExpo,0,0,0,0,0,0,0,0};

	// Limit output mixing as needed to save processing power
	if (Config.CamStab == ON)
	{
		outputs = MIN_OUTPUTS;
	}
	else
	{
		outputs = MAX_OUTPUTS;
	}

	// Process RC mixing, expo and source volume calculation
	for (i = 0; i < outputs; i++)
	{
		// Source A
		temp = RxChannel[Config.Channel[i].source_a] - Config.RxChannelZeroOffset[Config.Channel[i].source_a];
		temp = scale32(temp, Config.Channel[i].source_a_volume);
		temp = get_expo_value(temp, expos[i]);

		// Skip Source B if no RC mixing required for this channel
		if (Config.Channel[i].source_b_volume != 0)
		{
			temp2 = RxChannel[Config.Channel[i].source_b] - Config.RxChannelZeroOffset[Config.Channel[i].source_b];
			temp2 = scale32(temp2, Config.Channel[i].source_b_volume);
			temp2 = get_expo_value(temp2, expos[i]);

			// Sum the mixers
			temp = temp + temp2;
		}

		// Save solution for now
		Config.Channel[i].value = temp;
	}

	// Process sensor mixers
	for (i = 0; i < outputs; i++)
	{
		// Get source
		temp = Config.Channel[i].value;

		// Post-PID gyro input
		if (Stability)
		{
			if (Config.Channel[i].roll_gyro == ON)
			{
				if (Config.Channel[i].roll_gyro_polarity == REVERSED)
				{
					temp -=	PID_Gyros[ROLL];
				}
				else
				{
					temp +=	PID_Gyros[ROLL];
				}
			}

			if (Config.Channel[i].pitch_gyro == ON)
			{
				if (Config.Channel[i].pitch_gyro_polarity == REVERSED)
				{
					temp -=	PID_Gyros[PITCH];
				}
				else
				{
					temp +=	PID_Gyros[PITCH];
				}
			}

			if (Config.Channel[i].yaw_gyro == ON)
			{
				if (Config.Channel[i].yaw_gyro_polarity == REVERSED)
				{
					temp -=	PID_Gyros[YAW];
				}
				else
				{
					temp +=	PID_Gyros[YAW];
				}
			}
		}


		// Post-PID acc input
		if (AutoLevel)
		{
			if (Config.Channel[i].roll_acc == ON)
			{
				if (Config.Channel[i].roll_acc_polarity == REVERSED)
				{
					temp -=	PID_ACCs[ROLL];
				}
				else
				{
					temp +=	PID_ACCs[ROLL];
				}
			}
			if (Config.Channel[i].pitch_acc == ON)
			{
				if (Config.Channel[i].pitch_acc_polarity == REVERSED)
				{
					temp -=	PID_ACCs[PITCH];
				}
				else
				{
					temp +=	PID_ACCs[PITCH];
				}
			}
		}

		// Update channel data solution
		Config.Channel[i].value = temp + Config.RxChannelZeroOffset[Config.Channel[i].output_a];
	}

	// Process output mixers
	for (i = 0; i < outputs; i++)
	{
		// Get primary value (normally its own channel)
		temp = Config.Channel[Config.Channel[i].output_a].value - Config.RxChannelZeroOffset[Config.Channel[i].output_a];
		
		if (Config.Channel[i].output_a_volume !=0) // Mix in primary output volume
		{	
			temp = scale32(temp, Config.Channel[i].output_a_volume);
		}
		if (Config.Channel[i].output_b_volume !=0) // Mix in first extra output
		{
			temp2 = Config.Channel[Config.Channel[i].output_b].value - Config.RxChannelZeroOffset[Config.Channel[i].output_b];
			temp2 = scale32(temp2, Config.Channel[i].output_b_volume);
			temp = temp + temp2;
		}
		if (Config.Channel[i].output_c_volume !=0) // Mix in second extra output
		{
			temp2 = Config.Channel[Config.Channel[i].output_c].value - Config.RxChannelZeroOffset[Config.Channel[i].output_c];
			temp2 = scale32(temp2, Config.Channel[i].output_c_volume);
			temp = temp + temp2;
		}
		if (Config.Channel[i].output_d_volume !=0) // Mix in third extra output
		{
			temp2 = Config.Channel[Config.Channel[i].output_d].value - Config.RxChannelZeroOffset[Config.Channel[i].output_d];
			temp2 = scale32(temp2, Config.Channel[i].output_d_volume);
			temp = temp + temp2;
		}

		// Add zero offset of Source A back in to restore to system compatible value
		temp = temp + Config.RxChannelZeroOffset[Config.Channel[i].output_a];

		// Update channel data solution
		Config.Channel[i].value = temp;
	}

}

// Update actual limits value with that from the mix setting percentages
// This is only done at start-up and whenever the values are changed
void UpdateLimits(void)
{
	uint8_t i;
	int16_t min, max, failsafe;
	int16_t temp;

	// Update triggers
	temp = Config.Stablimit;
	temp = ((temp * 12) + 3750);
	Config.Stabtrigger = temp;

	temp = Config.Autolimit;
	temp = ((temp * 12) + 3750);
	Config.Autotrigger = temp;

	temp = Config.LaunchThrPos;
	temp = ((temp * 12) + 3750);
	Config.Launchtrigger = temp;

	// Update I-term limits
	for (i = 0; i < 3; i++)
	{
		temp = Config.I_Limits[i]; 				// 0 to 125%
		Config.Raw_I_Limits[i] = temp * 160;	// Multiply by 160 (max is 160 x 125 = 20000)
	}

	// Update travel limits
	for (i = 0; i < MAX_OUTPUTS; i++)
	{
		min = Config.Channel[i].min_travel;
		min = ((min * 12) + 3750);
		Config.Limits[i].minimum = min;

		max = Config.Channel[i].max_travel;
		max = ((max * 12) + 3750);
		Config.Limits[i].maximum = max;

		failsafe = Config.Channel[i].Failsafe;
		failsafe = ((failsafe * 12) + 3750);
		Config.Limits[i].failsafe = failsafe;
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
