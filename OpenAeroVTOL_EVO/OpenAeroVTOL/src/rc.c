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
void UpdateTransition(void);

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
	for (i=0; i < MAX_RC_CHANNELS; i++)
	{
		RCinputs[i]	= RxChannel[i] - Config.RxChannelZeroOffset[i];
	}

	// Special handling for monopolar throttle
	// Preset to RxChannelZeroOffset[THROTTLE] = 2750 (-250 to 2250) for safety. 
	// Normally MonopolarThrottle is referenced to the lowest throttle position.
	MonopolarThrottle = RxChannel[THROTTLE] - Config.RxChannelZeroOffset[THROTTLE]; 

	// Bipolar throttle must use the nominal mid-point as calibration is done at throttle minimum
	RCinputs[THROTTLE] = RxChannel[THROTTLE] - 3750; 

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
	RxSum = RCinputs[AILERON] + RCinputs[ELEVATOR] + RCinputs[RUDDER];
	RxSumDiff = RxSum - OldRxSum;

	// Set RX activity flag if movement above noise floor or throttle above minimum
	if ((RxSumDiff > NOISE_THRESH) || (RxSumDiff < -NOISE_THRESH) || (MonopolarThrottle > THROTTLEIDLE)) 
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

/*
	// Debug! override the transition switch/knob
	if (flip)
	{
		RCinputs[Config.FlightChan] = 1000; // Force to P2
	}
	else
	{
		RCinputs[Config.FlightChan] = -1000; // Force to P1
	}
*/
}

// Center sticks on request from Menu
void CenterSticks(void)		
{
	uint8_t i, j;
	uint16_t RxChannelZeroOffset[MAX_RC_CHANNELS] = {0,0,0,0,0,0,0,0};

	// Take an average of eight readings
	// RxChannel will auto-update every RC frame (normally 46Hz or so)
	for (i = 0; i < 8; i++)
	{
		for (j=0; j<MAX_RC_CHANNELS; j++)
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

void UpdateTransition(void)
{
	int16_t temp = 0;
	
	// Offset RC input to (approx) -250 to 2250
	temp = RCinputs[Config.FlightChan] + 1000;

	// Trim lower end to zero (0 to 2250)
	if (temp < 0) temp = 0;

	// Convert 0 to 2250 to 0 to 125. Divide by 20
	// Round to avoid truncation errors
	transition = (temp + 10) / 20;

	// transition now has a range of 0 to 101 for 0 to 2000 input
	// Limit extent of transition value 0 to 100 (101 steps)
	if (transition > 100) transition = 100;
}