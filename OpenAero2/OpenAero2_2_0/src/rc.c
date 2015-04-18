//***********************************************************
//* rc.c
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include <avr/io.h>
#include <stdbool.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h> 
#include <util/delay.h>
#include "typedefs.h"
#include "isr.h"
#include "init.h"
#include "io_cfg.h"
#include "servos.h"
#include "main.h"
#include "eeprom.h"
#include "mixer.h"

//************************************************************
// Prototypes
//************************************************************

void RxGetChannels(void);
void RC_Deadband(void);
void CenterSticks(void);
void SetFailsafe(void);

//************************************************************
// Defines
//************************************************************
#define	NOISE_THRESH	5			// Max RX noise threshold

//************************************************************
// Code
//************************************************************

volatile int16_t RCinputs[MAX_RC_CHANNELS + 1];	// Normalised RC inputs
volatile int16_t MonopolarThrottle;				// Monopolar throttle

// Get raw flight channel data (~2500 to 5000) and remove zero offset
// Use channel mapping for reconfigurability
void RxGetChannels(void)
{
	static	int16_t	OldRxSum;			// Sum of all major channels
	int16_t	RxSumDiff;
	int16_t	RxSum, i;

	// Remove zero offsets
	for (i=0;i<MAX_RC_CHANNELS;i++)
	{
		RCinputs[i]	= RxChannel[i] - Config.RxChannelZeroOffset[i];
	}

	// Special handling for monopolar throttle
	MonopolarThrottle = RxChannel[THROTTLE] - Config.RxChannelZeroOffset[THROTTLE];

	// Bipolar throttle must use the nominal mid-point
	RCinputs[THROTTLE] = RxChannel[THROTTLE] - 3750;

	// Reverse primary channels as requested
	if (Config.AileronPol == REVERSED)
	{
		// Note we have to reverse the source otherwise we get a double reverse if someone sets up
		// the second aileron as AILERON
		RCinputs[AILERON] = -(RxChannel[AILERON] - Config.RxChannelZeroOffset[AILERON]);
	}

	// Only reverse second aileron if set up
	if ((Config.SecAileronPol == REVERSED) && (Config.FlapChan != NOCHAN))
	{
		// Note we have to reverse the source otherwise we get a double reverse if someone sets up
		// the second aileron as AILERON
		RCinputs[Config.FlapChan] = -(RxChannel[Config.FlapChan] - Config.RxChannelZeroOffset[Config.FlapChan]);
	}

	if (Config.ElevatorPol == REVERSED)
	{
		RCinputs[ELEVATOR] = -RCinputs[ELEVATOR];
	}

	if (Config.RudderPol == REVERSED)
	{
		RCinputs[RUDDER] = -RCinputs[RUDDER];
	}

	// Calculate RX activity
	RxSum = RCinputs[AILERON] + RCinputs[ELEVATOR] + RCinputs[GEAR] + RCinputs[RUDDER] + RCinputs[AUX1];
	RxSumDiff = RxSum - OldRxSum;

	// Set RX activity flag
	if ((RxSumDiff > NOISE_THRESH) || (RxSumDiff < -NOISE_THRESH)) 
	{
		Flight_flags |= (1 << RxActivity);
	}
	else 
	{
		Flight_flags &= ~(1 << RxActivity);
	}
	
	// Preset RCinputs[NOCHAN] for sanity
	RCinputs[NOCHAN] = 0;

	OldRxSum = RxSum;
}

 // Detect the hands-off situation
void RC_Deadband(void)
{
	int16_t	aileron_actual = 0;

	// If flaperons set up 
	if (Config.FlapChan != NOCHAN)
	{
		// Recreate actual roll signal from flaperons
		aileron_actual  = RCinputs[AILERON] + RCinputs[Config.FlapChan];
		aileron_actual  = aileron_actual >> 1;
	}
	// If flaperons not set up
	else
	{
		aileron_actual  = RCinputs[AILERON];
	}

	// Hands-free detection
	if (((aileron_actual < Config.HandsFreetrigger) && (aileron_actual > -Config.HandsFreetrigger))
	 && ((RCinputs[ELEVATOR]  < Config.HandsFreetrigger) && (RCinputs[ELEVATOR]  > -Config.HandsFreetrigger)))
	{
		Flight_flags |= (1 << HandsFree);
	}
	else
	{
		Flight_flags &= ~(1 << HandsFree);
	}
}

// Center sticks on request from Menu
void CenterSticks(void)		
{
	uint8_t i, j;
	uint16_t RxChannelZeroOffset[MAX_RC_CHANNELS] = {0,0,0,0,0,0,0,0};

	// Take an average of eight readings
	// RxChannel will auto-update every RC frame (normally 46Hz or so)
	for (i=0; i < 8; i++)
	{
		for (j=0;j<MAX_RC_CHANNELS;j++)
		{
			RxChannelZeroOffset[j] += RxChannel[j];
		}
		_delay_ms(100); // Wait for a new frame
	}

	for (i=0;i<MAX_RC_CHANNELS;i++)
	{
		Config.RxChannelZeroOffset[i] = ((RxChannelZeroOffset[i] + 4) >> 3); // Round and divide by 8
	}

	Save_Config_to_EEPROM();
}

// Set failsafe position
void SetFailsafe(void)		
{
	uint8_t i;
	int16_t failsafe;
	int16_t temp;

	// Update latest values of each channel
	ProcessMixer();

	// Update Config settings based on servo position
	for (i = 0; i < MAX_OUTPUTS; i++)
	{
		// Set primary failsafe point
		temp = Config.Channel[i].value;		// Mixer values are +/-1250
		Config.Limits[i].failsafe = temp;

		// Round and rescale and set noob-friendly mixer failsafe percentages
		failsafe = (temp + (int16_t)5) / (int16_t)10;
		
		// Bounds check 
		if (failsafe > 125)
		{
			failsafe = 125;
		}
		if (failsafe < -125)
		{
			failsafe = -125;
		}		
		
		// Save as percentage
		Config.Failsafe[i] = failsafe;
	}

	Save_Config_to_EEPROM();
}

