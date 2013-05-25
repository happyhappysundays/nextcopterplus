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
#include "..\inc\typedefs.h"
#include "..\inc\isr.h"
#include "..\inc\init.h"
#include "..\inc\io_cfg.h"
#include "..\inc\servos.h"
#include "..\inc\main.h"
#include "..\inc\eeprom.h"
#include "..\inc\mixer.h"

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
#define	NOISE_THRESH	20			// Max RX noise threshold. Increase if lost alarm keeps being reset.

//************************************************************
// Code
//************************************************************

int16_t RCinputs[MAX_RC_CHANNELS];	// Normalised RC inputs

// Get raw flight channel data and remove zero offset
// Use channel mapping for configurability
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
	
	OldRxSum = RxSum;
}

 // Reduce RxIn noise and also detect the hands-off situation
void RC_Deadband(void)
{
	int16_t	aileron_actual = 0;

	// Deadband culling
	if ((RCinputs[AILERON] < Config.DeadbandLimit) && (RCinputs[AILERON] > -Config.DeadbandLimit))
	{
		RCinputs[AILERON] = 0;
	}
	if ((RCinputs[ELEVATOR] < Config.DeadbandLimit) && (RCinputs[ELEVATOR] > -Config.DeadbandLimit)) 
	{
		RCinputs[ELEVATOR] = 0;
	}
	if ((RCinputs[RUDDER] < Config.DeadbandLimit) && (RCinputs[RUDDER] > -Config.DeadbandLimit))
	{
		RCinputs[RUDDER] = 0;
	}

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
	for (i=0;i<8;i++)
	{
		for (j=0;j<MAX_RC_CHANNELS;j++)
		{
			RxChannelZeroOffset[j] += RxChannel[j];
		}
		_delay_ms(100);
	}

	for (i=0;i<MAX_RC_CHANNELS;i++)
	{
		Config.RxChannelZeroOffset[i] = RxChannelZeroOffset[i] >> 3; // Divide by 8
	}

	Save_Config_to_EEPROM();
}

// Set failsafe position
void SetFailsafe(void)		
{
	uint8_t i;
	int16_t failsafe;

	// Update latest values of each channel
	ProcessMixer();

	// Transfer latest values of each channel to ServoOut[] and range limit
	UpdateServos();

	// Update Config settings based on servo position
	for (i = 0; i < MAX_OUTPUTS; i++)
	{
		// Set primary failsafe point
		Config.Limits[i].failsafe = ServoOut[i];

		// Rescale and set noob-friendly mixer failsafe percentages
		failsafe = (ServoOut[i] - 3750) / 12;
		Config.Failsafe[i] = failsafe;
	}

	Save_Config_to_EEPROM();
}

