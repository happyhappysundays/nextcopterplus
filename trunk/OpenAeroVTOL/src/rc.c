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

//************************************************************
// Defines
//************************************************************
#define	NOISE_THRESH	5			// Max RX noise threshold. Reduce if unintentional disarms occur.

//************************************************************
// Code
//************************************************************

int16_t RCinputs[MAX_RC_CHANNELS];	// Normalised RC inputs

// Get raw flight channel data (~2500 to 5000) and remove zero offset
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
		RCinputs[AILERON] = -RCinputs[AILERON];
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

	// Set RX activity flag if movement above noise floor or throttle above minimum
	if ((RxSumDiff > NOISE_THRESH) || (RxSumDiff < -NOISE_THRESH) || (RCinputs[THROTTLE] > THROTTLEIDLE)) 
	{
		Flight_flags |= (1 << RxActivity);
	}
	else 
	{
		Flight_flags &= ~(1 << RxActivity);
	}
	
	OldRxSum = RxSum;
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
