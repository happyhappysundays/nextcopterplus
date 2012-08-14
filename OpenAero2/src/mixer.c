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
	{0,FLAP,NORMAL,100,ON,NORMAL,OFF,NORMAL,OFF,NORMAL,ON,NORMAL,OFF,NORMAL,-100,100,0}, 		// ServoOut7 (Right aileron)
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
	{0,FLAP,NORMAL,100,ON,REVERSED,ON,REVERSED,OFF,NORMAL,ON,REVERSED,ON,REVERSED,-100,100,0},	// ServoOut7 (right elevon)
	{0,RUDDER,NORMAL,100,OFF,NORMAL,OFF,NORMAL,ON,NORMAL,OFF,NORMAL,OFF,NORMAL,-100,100,0}, 	// ServoOut8
}; 

channel_t MANUAL_MIX[MAX_OUTPUTS] PROGMEM = 
{
	{0,THROTTLE,NORMAL,100,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,-100,100,0}, 	// ServoOut1
	{0,GEAR,NORMAL,100,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,-100,100,0}, 		// ServoOut2
	{0,AUX1,NORMAL,100,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,-100,100,0}, 		// ServoOut3
	{0,AUX2,NORMAL,100,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,-100,100,0},  	// ServoOut4
	{0,ELEVATOR,NORMAL,100,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,-100,100,0}, 	// ServoOut5
	{0,AILERON,NORMAL,100,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,-100,100,0}, 	// ServoOut6
	{0,FLAP,NORMAL,100,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,-100,100,0}, 		// ServoOut7
	{0,RUDDER,NORMAL,100,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,-100,100,0}, 	// ServoOut8
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
	uint8_t i;
	int16_t temp, temp2 = 0;
	// Quick fudge to allow easy look-up of which channels require expo
	uint8_t expos[] = {0,0,0,0,Config.ElevatorExpo,Config.AileronExpo,Config.AileronExpo,Config.RudderExpo,0,0,0,0,0,0,0,0};

	// Process RC mixer if enabled
	if (Config.RCMix == ON)
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

	// Process output mixers
	for (i = 0; i < MAX_OUTPUTS; i++)
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
		if (Stability || AutoLevel)
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
