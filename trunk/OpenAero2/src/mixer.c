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

void SetMixer(void);
void ProcessMixer(void);
void UpdateServos(void);
void SetServoPositions(void);
void get_preset_mix (channel_t*);

//************************************************************
// Code
//************************************************************

//************************************************************

// Aeroplane mixer defaults
channel_t AEROPLANE_MIX[MAX_OUTPUTS] PROGMEM = 
{
	// Rudder -= Yaw; (normal)
	// Aileron -= Roll; (normal)
	// Elevator -= Pitch; (normal)

	//Value,source,src_pol,src_vol,roll_gyro,gyro_pol,pitch_gyro,pol,yaw_gyro,pol,roll_acc,pol,pitch_acc,pol,min,max,fs

	{0,THROTTLE,NORMAL,100,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,2250,5250,3750}, 	// ServoOut1 (Throttle)
	{0,PRESET1,NORMAL,100,OFF,NORMAL,ON,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,2250,5250,3750}, 	// ServoOut2
	{0,PRESET2,NORMAL,100,OFF,NORMAL,OFF,NORMAL,ON,NORMAL,OFF,NORMAL,OFF,NORMAL,2250,5250,3750}, 	// ServoOut3
	{0,PRESET3,NORMAL,100,ON,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,2250,5250,3750},  	// ServoOut4
	{0,ELEVATOR,NORMAL,100,OFF,NORMAL,ON,NORMAL,OFF,NORMAL,OFF,NORMAL,ON,NORMAL,2250,5250,3750}, 	// ServoOut5 (Elevator)
	{0,AILERON,NORMAL,100,ON,NORMAL,OFF,NORMAL,OFF,NORMAL,ON,NORMAL,OFF,NORMAL,2250,5250,3750},	 	// ServoOut6 (Left aileron)
	{0,FLAP,NORMAL,100,ON,NORMAL,OFF,NORMAL,OFF,NORMAL,ON,NORMAL,OFF,NORMAL,2250,5250,3750}, 		// ServoOut7 (Right aileron)
	{0,RUDDER,NORMAL,100,OFF,NORMAL,OFF,NORMAL,ON,REVERSED,OFF,NORMAL,OFF,NORMAL,2250,5250,3750}, 	// ServoOut8 (Rudder)
}; 

channel_t FLYING_WING_MIX[MAX_OUTPUTS] PROGMEM = 
{
	// Rudder -= Yaw; (normal)
	// LAileron += Roll; (reversed)
	// LElevator -= Pitch; (normal)
	// RAileron += Roll;(reversed)
	// RElevator += Pitch;(reversed)
	
	{0,THROTTLE,NORMAL,100,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,2250,5250,3750}, 	// ServoOut1
	{0,PRESET1,NORMAL,100,OFF,NORMAL,ON,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,2250,5250,3750}, 	// ServoOut2
	{0,PRESET2,NORMAL,100,OFF,NORMAL,OFF,NORMAL,ON,NORMAL,OFF,NORMAL,OFF,NORMAL,2250,5250,3750}, 	// ServoOut3
	{0,PRESET3,NORMAL,100,ON,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,2250,5250,3750},  	// ServoOut4
	{0,AILERON,NORMAL,100,ON,REVERSED,ON,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,2250,5250,3750}, 	// ServoOut5
	{0,ELEVATOR,NORMAL,100,ON,REVERSED,ON,NORMAL,OFF,NORMAL,ON,REVERSED,ON,NORMAL,2250,5250,3750},	// ServoOut6 (left elevon)
	{0,FLAP,NORMAL,100,ON,REVERSED,ON,REVERSED,OFF,NORMAL,ON,REVERSED,ON,REVERSED,2250,5250,3750},	// ServoOut7 (right elevon)
	{0,RUDDER,NORMAL,100,OFF,NORMAL,OFF,NORMAL,ON,NORMAL,OFF,NORMAL,OFF,NORMAL,2250,5250,3750}, 	// ServoOut8
}; 

channel_t MANUAL_MIX[MAX_OUTPUTS] PROGMEM = 
{
	{0,THROTTLE,NORMAL,100,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,2250,5250,3750}, 	// ServoOut1
	{0,GEAR,NORMAL,100,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,2250,5250,3750}, 		// ServoOut2
	{0,AUX1,NORMAL,100,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,2250,5250,3750}, 		// ServoOut3
	{0,AUX2,NORMAL,100,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,2250,5250,3750},  	// ServoOut4
	{0,ELEVATOR,NORMAL,100,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,2250,5250,3750}, 	// ServoOut5
	{0,AILERON,NORMAL,100,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,2250,5250,3750}, 	// ServoOut6
	{0,FLAP,NORMAL,100,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,2250,5250,3750}, 		// ServoOut7
	{0,RUDDER,NORMAL,100,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,2250,5250,3750}, 	// ServoOut8
}; 

//************************************************************
// Get preset mix from Program memory
void get_preset_mix(channel_t* preset)
{
	uint8_t i;
	for (i = 0; i < MAX_OUTPUTS; i++)
	{
		Config.Channel[i].value = pgm_read_word(&preset[i].value);
		Config.Channel[i].source = pgm_read_byte(&preset[i].source);
		Config.Channel[i].source_polarity = pgm_read_byte(&preset[i].source_polarity);
		Config.Channel[i].source_volume = pgm_read_byte(&preset[i].source_volume);
		Config.Channel[i].roll_gyro = pgm_read_byte(&preset[i].roll_gyro);
		Config.Channel[i].roll_gyro_polarity = pgm_read_byte(&preset[i].roll_gyro_polarity);
		Config.Channel[i].pitch_gyro = pgm_read_byte(&preset[i].pitch_gyro);
		Config.Channel[i].pitch_gyro_polarity = pgm_read_byte(&preset[i].pitch_gyro_polarity);
		Config.Channel[i].yaw_gyro = pgm_read_byte(&preset[i].yaw_gyro);
		Config.Channel[i].yaw_gyro_polarity = pgm_read_byte(&preset[i].yaw_gyro_polarity);
		Config.Channel[i].roll_acc = pgm_read_byte(&preset[i].roll_acc);
		Config.Channel[i].roll_acc_polarity = pgm_read_byte(&preset[i].roll_acc_polarity);
		Config.Channel[i].pitch_acc = pgm_read_byte(&preset[i].pitch_acc);
		Config.Channel[i].pitch_acc_polarity = pgm_read_byte(&preset[i].pitch_acc_polarity);
		Config.Channel[i].min_travel = pgm_read_word(&preset[i].min_travel);
		Config.Channel[i].max_travel = pgm_read_word(&preset[i].max_travel);
		Config.Channel[i].Failsafe = pgm_read_word(&preset[i].Failsafe);
	}
}

//************************************************************
// SetMixer Concept:
//
// Copy preset data into Config.Channel[x] structures
//
//************************************************************

void SetMixer(void)
{

}

//************************************************************
// ProcessMixer Concept:
//
// Servo output = 	if source = NOCHAN
//						use fixed offset (TBD)
//					else 
//						RxChannel[source] x source_polarity x source_volume 
//					if not NO_GYRO
//					  + Gyros[gyro] x gyro_polarity
//					if not NO_ACC
//					  + Accs[acc] x acc_polarity
//
//************************************************************


void ProcessMixer(void)
{
	uint8_t i;
	int16_t temp = 0;
	int32_t temp32 = 0;
	int32_t mult32 = 0;
	uint8_t expos[] = {0,0,0,0,Config.ElevatorExpo,Config.AileronExpo,Config.AileronExpo,Config.RudderExpo,0};

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
		// GCC broken bad regarding multiplying 32 bit numbers, hence all this...
		mult32 = Config.Channel[i].source_volume;
		temp32 = temp;					// Yes, really...
		temp32 = temp32 * mult32;

		// Divide by 100 to get scaled value
		temp32 = temp32 / (int32_t)100; // I shit you not...
		temp = (int16_t) temp32;

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

// Update servos from the mixer Config.Channel[i].value data and enforce travel limits
void UpdateServos(void)
{
	uint8_t i;

	for (i = 0; i < MAX_OUTPUTS; i++)
	{
		if (Config.Channel[i].value < Config.Channel[i].min_travel) 
		{
			ServoOut[i] = Config.Channel[i].min_travel;
		}
		else if (Config.Channel[i].value > Config.Channel[i].max_travel)
		{
			ServoOut[i] = Config.Channel[i].max_travel;
		}
		else
		{
			ServoOut[i] = Config.Channel[i].value;
		}
	}
}

// Set servo position from raw RC data
void SetServoPositions(void)	
{
	uint8_t i;

	for (i = 0; i < MAX_OUTPUTS; i++)
	{
		// Remap servo outputs to the preset channel order 
		ServoOut[i] = RxChannel[i];
	}
}
