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
	// Rudder -= Yaw;
	// Aileron -= Roll;
	// Elevator -= Pitch;

	//Value,source,src_pol,src_vol,roll_gyro,gyro_pol,pitch_gyro,pol,yaw_gyro,pol,acc,pol,min,max,fs

	{0,THROTTLE,NORMAL,1,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,NO_ACC,NORMAL,2250,5250,3500}, 	// ServoOut1 (Throttle)
	{0,NOCHAN,NORMAL,1,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,NO_ACC,NORMAL,2250,5250,3500}, 		// ServoOut2
	{0,AUX1,NORMAL,1,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,NO_ACC,NORMAL,2250,5250,3500}, 		// ServoOut3
	{0,AUX2,NORMAL,1,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,NO_ACC,NORMAL,2250,5250,3500},  		// ServoOut4
	{0,ELEVATOR,NORMAL,1,OFF,NORMAL,ON,NORMAL,OFF,NORMAL,Y,NORMAL,2250,5250,3500}, 			// ServoOut5 (Elevator)
	{0,AILERON,NORMAL,1,ON,NORMAL,OFF,NORMAL,OFF,NORMAL,X,NORMAL,2250,5250,3500},	 		// ServoOut6 (Left aileron)
	{0,FLAP,NORMAL,1,ON,G_REVERSED,OFF,NORMAL,OFF,NORMAL,X,NORMAL,2250,5250,3500}, 			// ServoOut7 (Right aileron)
	{0,RUDDER,NORMAL,1,OFF,NORMAL,OFF,NORMAL,ON,NORMAL,NO_ACC,NORMAL,2250,5250,3500}, 		// ServoOut8 (Rudder)
}; 

channel_t FLYING_WING_MIX[MAX_OUTPUTS] PROGMEM = 
{
	// Rudder -= Yaw;
	// Aileron += Roll;
	// Elevator -= Pitch;
	// Aileron += Roll;
	// Elevator += Pitch;
	
	{0,THROTTLE,NORMAL,1,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,NO_ACC,NORMAL,2250,5250,3500}, 	// ServoOut1
	{0,NOCHAN,NORMAL,1,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,NO_ACC,NORMAL,2250,5250,3500}, 		// ServoOut2
	{0,AUX1,NORMAL,1,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,NO_ACC,NORMAL,2250,5250,3500}, 		// ServoOut3
	{0,AUX2,NORMAL,1,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,NO_ACC,NORMAL,2250,5250,3500},  		// ServoOut4
	{0,AILERON,NORMAL,1,ON,G_REVERSED,ON,NORMAL,OFF,NORMAL,NO_ACC,NORMAL,2250,5250,3500}, 	// ServoOut5
	{0,ELEVATOR,NORMAL,1,ON,G_REVERSED,ON,G_REVERSED,OFF,NORMAL,NO_ACC,NORMAL,2250,5250,3500},	// ServoOut6
	{0,FLAP,NORMAL,1,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,NO_ACC,NORMAL,2250,5250,3500}, 		// ServoOut7
	{0,RUDDER,NORMAL,1,OFF,NORMAL,OFF,NORMAL,ON,NORMAL,NO_ACC,NORMAL,2250,5250,3500}, 		// ServoOut8
}; 

channel_t MANUAL_MIX[MAX_OUTPUTS] PROGMEM = 
{
	{0,THROTTLE,NORMAL,1,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,NO_ACC,NORMAL,2250,5250,3500}, 	// ServoOut1
	{0,GEAR,NORMAL,1,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,NO_ACC,NORMAL,2250,5250,3500}, 		// ServoOut2
	{0,AUX1,NORMAL,1,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,NO_ACC,NORMAL,2250,5250,3500}, 		// ServoOut3
	{0,AUX2,NORMAL,1,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,NO_ACC,NORMAL,2250,5250,3500},  		// ServoOut4
	{0,ELEVATOR,NORMAL,1,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,NO_ACC,NORMAL,2250,5250,3500}, 	// ServoOut5
	{0,AILERON,NORMAL,1,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,NO_ACC,NORMAL,2250,5250,3500}, 	// ServoOut6
	{0,FLAP,NORMAL,1,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,NO_ACC,NORMAL,2250,5250,3500}, 		// ServoOut7
	{0,RUDDER,NORMAL,1,OFF,NORMAL,OFF,NORMAL,OFF,NORMAL,NO_ACC,NORMAL,2250,5250,3500}, 		// ServoOut8
}; 

//************************************************************
// Get preset mix from Program memory
void get_preset_mix(channel_t* preset)
{
	uint8_t i;
	for (i = 0; i < MAX_OUTPUTS; i++)
	{
		Config.Channel[i].value = pgm_read_word(&preset[i].value);
		Config.Channel[i].source = pgm_read_word(&preset[i].source);
		Config.Channel[i].source_polarity = pgm_read_word(&preset[i].source_polarity);
		Config.Channel[i].source_volume = pgm_read_word(&preset[i].source_volume);
		Config.Channel[i].roll_gyro = pgm_read_word(&preset[i].roll_gyro);
		Config.Channel[i].roll_gyro_polarity = pgm_read_word(&preset[i].roll_gyro_polarity);
		Config.Channel[i].pitch_gyro = pgm_read_word(&preset[i].pitch_gyro);
		Config.Channel[i].pitch_gyro_polarity = pgm_read_word(&preset[i].pitch_gyro_polarity);
		Config.Channel[i].yaw_gyro = pgm_read_word(&preset[i].yaw_gyro);
		Config.Channel[i].yaw_gyro_polarity = pgm_read_word(&preset[i].yaw_gyro_polarity);
		Config.Channel[i].acc = pgm_read_word(&preset[i].acc);
		Config.Channel[i].acc_polarity = pgm_read_word(&preset[i].acc_polarity);
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
//						use fixed offset
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
	uint16_t temp;

	for (i = 0; i < MAX_OUTPUTS; i++)
	{

		// RC source inputs and reversing
		if (Config.Channel[i].source_polarity == REVERSED)
		{
			temp  =	(Config.RxChannelZeroOffset[Config.Channel[i].source] - (RxChannel[Config.Channel[i].source] - Config.RxChannelZeroOffset[Config.Channel[i].source])) * Config.Channel[i].source_volume;
		}
		else
		{
			temp  =	RxChannel[Config.Channel[i].source] * Config.Channel[i].source_volume;
		}
		// Debug - will have to end up with source_volume / 100 to get realistic values (0% to 200%)
		// Defaults to 1 for now.

		// Exponential
		//temp = get_expo_value(temp, Config.AileronExpo);
		//temp = get_expo_value(temp, Config.ElevatorExpo);
		//temp = get_expo_value(temp, Config.RudderExpo);


		// Post-PID gyro input
		if (Stability || AutoLevel)
		{
			if (Config.Channel[i].roll_gyro == G_ON)
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

			if (Config.Channel[i].pitch_gyro == G_ON)
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

			if (Config.Channel[i].yaw_gyro == G_ON)
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
			if (Config.Channel[i].acc != NO_ACC)
			{
				if (Config.Channel[i].acc_polarity == REVERSED)
				{
					//temp -=	PID_ACCs[ROLL];
					temp -=	PID_ACCs[Config.Channel[i].acc];
				}
				else
				{
					//temp +=	PID_ACCs[ROLL];*/
					temp +=	PID_ACCs[Config.Channel[i].acc];
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
		else ServoOut[i] = Config.Channel[i].value;

		ServoOut[i] = Config.Channel[i].value;

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
