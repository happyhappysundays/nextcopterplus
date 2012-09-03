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
// Code
//************************************************************

// Aeroplane mixer defaults
channel_t AEROPLANE_MIX[MAX_OUTPUTS] PROGMEM = 
{
	// Rudder -= Yaw; (normal)
	// Aileron -= Roll; (normal)
	// Elevator -= Pitch; (normal)

	//Value,source,src_pol,src_vol,roll_gyro,gyro_pol,pitch_gyro,pol,yaw_gyro,pol,roll_acc,pol,pitch_acc,pol,min,max,fs

	{0,THROTTLE,NORMAL,100,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,-100,100,0}, 	// ServoOut1 (Throttle)
	{0,PRESET1,NORMAL,100,OFF,NORMAL,ON,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,-100,100,0}, 	// ServoOut2
	{0,PRESET2,NORMAL,100,OFF,NORMAL,OFF,NORMAL,ON,NORMAL,OFF,NORMAL,OFF,NORMAL,-100,100,0}, 	// ServoOut3
	{0,PRESET3,NORMAL,100,ON,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,-100,100,0},  	// ServoOut4
	{0,ELEVATOR,NORMAL,100,OFF,NORMAL,ON,NORMAL,OFF,NORMAL,OFF,NORMAL,ON,NORMAL,-100,100,0}, 	// ServoOut5 (Elevator)
	{0,AILERON,NORMAL,100,ON,NORMAL,OFF,NORMAL,OFF,NORMAL,ON,NORMAL,OFF,NORMAL,-100,100,0},	 	// ServoOut6 (Left aileron)
	{0,THROTTLE,NORMAL,100,ON,NORMAL,OFF,NORMAL,OFF,NORMAL,ON,NORMAL,OFF,NORMAL,-100,100,0}, 	// ServoOut7 (Right aileron)
	{0,RUDDER,NORMAL,100,OFF,NORMAL,OFF,NORMAL,ON,REVERSED,OFF,NORMAL,OFF,NORMAL,-100,100,0}, 	// ServoOut8 (Rudder)
}; 

channel_t FLYING_WING_MIX[MAX_OUTPUTS] PROGMEM = 
{
	// Rudder -= Yaw; (normal)
	// LAileron += Roll; (reversed)
	// LElevator -= Pitch; (normal)
	// RAileron += Roll;(reversed)
	// RElevator += Pitch;(reversed)
	
	{0,THROTTLE,NORMAL,100,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,-100,100,0}, 	// ServoOut1
	{0,PRESET1,NORMAL,100,OFF,NORMAL,ON,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,-100,100,0}, 	// ServoOut2
	{0,PRESET2,NORMAL,100,OFF,NORMAL,OFF,NORMAL,ON,NORMAL,OFF,NORMAL,OFF,NORMAL,-100,100,0}, 	// ServoOut3
	{0,PRESET3,NORMAL,100,ON,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,-100,100,0},  	// ServoOut4
	{0,AILERON,NORMAL,100,ON,REVERSED,ON,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,-100,100,0}, 	// ServoOut5
	{0,ELEVATOR,NORMAL,100,ON,REVERSED,ON,NORMAL,OFF,NORMAL,ON,REVERSED,ON,NORMAL,-100,100,0},	// ServoOut6 (left elevon)
	{0,AILERON,NORMAL,100,ON,REVERSED,ON,REVERSED,OFF,NORMAL,ON,REVERSED,ON,REVERSED,-100,100,0},// ServoOut7 (right elevon)
	{0,RUDDER,NORMAL,100,OFF,NORMAL,OFF,NORMAL,ON,NORMAL,OFF,NORMAL,OFF,NORMAL,-100,100,0}, 	// ServoOut8
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
		
	{0,NOCHAN,NORMAL,100,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,-100,100,0}, 	// ServoOut1
	{0,PRESET1,NORMAL,100,OFF,NORMAL,ON,NORMAL,OFF,NORMAL,OFF,NORMAL,ON,NORMAL,-100,100,0}, 	// ServoOut2 (Tilt axis)
	{0,PRESET2,NORMAL,100,OFF,NORMAL,OFF,NORMAL,ON,NORMAL,OFF,NORMAL,OFF,NORMAL,-100,100,0}, 	// ServoOut3 (Pan axis)
	{0,PRESET3,NORMAL,100,ON,NORMAL,OFF,NORMAL,OFF,NORMAL,ON,NORMAL,OFF,NORMAL,-100,100,0},  	// ServoOut4 (Roll axis)
	{0,NOCHAN,NORMAL,100,ON,REVERSED,ON,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,-100,100,0}, 	// ServoOut5
	{0,ELEVATOR,NORMAL,100,OFF,NORMAL,ON,NORMAL,OFF,NORMAL,OFF,NORMAL,ON,NORMAL,-100,100,0}, 	// ServoOut6 (Tilt axis)
	{0,RUDDER,NORMAL,100,OFF,NORMAL,OFF,NORMAL,ON,NORMAL,OFF,NORMAL,OFF,NORMAL,-100,100,0}, 	// ServoOut7 (Pan axis)
	{0,AILERON,NORMAL,100,ON,NORMAL,OFF,NORMAL,OFF,NORMAL,ON,NORMAL,OFF,NORMAL,-100,100,0},  	// ServoOut8 (Roll axis)
}; 

//************************************************************
// Get preset mix from Program memory
void get_preset_mix(channel_t* preset)
{
	memcpy_P(&Config.Channel[0].value,&preset[0].value,(sizeof(channel_t) * MAX_OUTPUTS));
}

//************************************************************
// ProcessMixer Concept:
//
// Servo output = 	if Config.RCMix == ON
//						Update Config.Mix_value
//					RxChannel[source] x source_polarity x source_volume 
//					if not NO_GYRO
//					  + Gyros[gyro] x gyro_polarity
//					if not NO_ACC
//					  + Accs[acc] x acc_polarity
//
//************************************************************

void ProcessMixer(void)
{
	uint8_t i, outputs;
	int16_t temp, temp2 = 0;
	// Quick fudge to allow easy look-up of which channels require expo
	uint8_t expos[] = {0,0,0,0,Config.ElevatorExpo,Config.AileronExpo,Config.AileronExpo,Config.RudderExpo,0,0,0,0,0,0,0,0};

	// Process RC mixer if enabled
	if ((Config.RCMix == ON) && (Config.CamStab == OFF))
	{

		//for (i = 0; i < NUM_MIXERS; i++)
		for (i = 0; i < 2; i++)
		{
			// Source A
			temp = scale32((RxChannel[Config.mixer_data[i].source_a] - Config.RxChannelZeroOffset[Config.mixer_data[i].source_a]), Config.mixer_data[i].source_a_volume);

			// Source B
			temp2 = scale32((RxChannel[Config.mixer_data[i].source_b] - Config.RxChannelZeroOffset[Config.mixer_data[i].source_b]), Config.mixer_data[i].source_b_volume);

			// Sum the mixers
			temp = temp + temp2;

			// Add zero offset of Source A back in to restore to system compatible value
			Config.Mix_value[i] = temp + Config.RxChannelZeroOffset[Config.mixer_data[i].source_a];
		}
	}

	// Limit output mixing as needed to save processing power
	if (Config.CamStab == ON)
	{
		outputs = MIN_OUTPUTS;
	}
	else
	{
		outputs = MAX_OUTPUTS;
	}

	// Process output mixers
	for (i = 0; i < outputs; i++)
	{
		// RC source inputs and reversing, and expo
		if (Config.Channel[i].source_polarity == REVERSED)
		{
			temp = (Config.RxChannelZeroOffset[Config.Channel[i].source] - RxChannel[Config.Channel[i].source]);
			temp = get_expo_value(temp, expos[i]);
		}
		else
		{
			temp = RxChannel[Config.Channel[i].source] - Config.RxChannelZeroOffset[Config.Channel[i].source];
			temp = get_expo_value(temp, expos[i]);
		}
		
		// Multiply RC with source volume
		temp = scale32(temp, Config.Channel[i].source_volume);

		// Add zero offset back in to restore to system compatible value
		temp = temp + Config.RxChannelZeroOffset[Config.Channel[i].source];

	
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

		// Update channel data
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

	// Only do the scaling if necessary
	if (multiplier16 != 100)
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
