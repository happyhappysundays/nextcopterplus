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
int16_t get_expo_value (int16_t RCvalue, uint8_t Expolevel);
void RC_Deadband(void);
void CenterSticks(void);
void SetFailsafe(void);

//************************************************************
// Defines
//************************************************************
#define DEAD_BAND		10			// Centre region of RC input where no activity is processed
#define	NOISE_THRESH	20			// Max RX noise threshold. Increase if lost alarm keeps being reset.

// ROM-based exponential multiplier table
// These numbers were calculated using my Expo.xlsx spreadsheet.
// Use the spreadsheet to work out new Expo rates for you.

const char Expo[10][16] PROGMEM = 
	{
		{117,118,118,119,120,120,121,122,123,123,124,125,126,126,127,128}, // Minimal expo
		{103,108,106,107,109,111,112,114,115,117,119,121,122,124,126,128},
		{ 92, 94, 96, 98,100,102,104,107,109,112,114,117,119,122,125,128},
		{ 79, 82, 84, 87, 90, 92, 95, 98,102,105,108,112,116,120,124,128},
		{ 66, 68, 71, 74, 77, 81, 84, 88, 92, 97,101,106,111,116,122,128},
		{ 52, 55, 58, 62, 65, 69, 73, 78, 82, 87, 93, 99,105,112,120,128},
		{ 39, 42, 45, 49, 52, 56, 61, 66, 71, 77, 83, 91, 99,107,117,128},
		{ 27, 29, 32, 35, 39, 43, 47, 52, 58, 65, 72, 80, 90,101,114,128},
		{ 16, 18, 20, 23, 26, 30, 34, 39, 45, 51, 59, 69, 80, 93,109,128},
		{  7,  8,  9, 11, 13, 16, 19, 23, 28, 35, 43, 53, 66, 82,102,128}  // Extreme expo
	};

//************************************************************
// Code
//************************************************************

int16_t 	RCinputs[MAX_RC_CHANNELS];	// Normalised RC inputs
bool		RxActivity;
bool		HandsFree;


// Get raw flight channel data and remove zero offset
// Use channel mapping for configurability
void RxGetChannels(void)
{
	static	int16_t	OldRxSum;			// Sum of all major channels
	int16_t	RxSumDiff;
	int16_t	RxSum, i;

	for (i=0;i<MAX_RC_CHANNELS;i++)
	{
		RCinputs[i]	= RxChannel[i] - Config.RxChannelZeroOffset[i];
	}

	// Calculate RX activity
	RxSum = RCinputs[AILERON] + RCinputs[ELEVATOR] + RCinputs[GEAR] + RCinputs[RUDDER] + RCinputs[FLAP];
	RxSumDiff = RxSum - OldRxSum;

	// Set RX activity flag
	if ((RxSumDiff > NOISE_THRESH) || (RxSumDiff < -NOISE_THRESH)) RxActivity = true;
	else RxActivity = false;
	OldRxSum = RxSum;
}

// Get expo'd RC value
int16_t get_expo_value (int16_t RCvalue, uint8_t Expolevel)
{
	int8_t	range, expo_level;
	int32_t RCcalc, RCsum, mult;						// Max values are around +/-75000

	if (Expolevel == 0) return (RCvalue);				// No need to calculate if expo is zero
	if (Expolevel > 99) Expolevel = 99;					// Limit expo to 99%

	range 	= (abs(RCvalue) >> 6);						// Work out which band the RCinput is in (16 values)
	if (range > 15) range = 15;
	if (range <  0) range = 0;
	expo_level = Expolevel/10;							// Work out which expo level to use (0~100 -> 0~10)	
				
	mult = pgm_read_byte(&Expo[expo_level][range]); 	// Get the multiplier from the table 

	RCsum 	= RCvalue;									// Promote RCvalue to 32 bits as GCC is broken :(
	RCcalc 	= RCsum * mult;								// Do the 32-bit x 32-bit multiply
	RCcalc	= RCcalc >> 7;
	RCvalue = (int16_t) RCcalc;							// Divide by 64 to get the expo'd value

	return (RCvalue);
}

 // Reduce RxIn noise and also detect the hands-off situation
void RC_Deadband(void)
{
	int16_t	aileron_actual = 0;

	// Deadband culling
	if ((RCinputs[AILERON] < DEAD_BAND) && (RCinputs[AILERON] > -DEAD_BAND))
	{
		RCinputs[AILERON] = 0;
	}
	if ((RCinputs[ELEVATOR] < DEAD_BAND) && (RCinputs[ELEVATOR] > -DEAD_BAND)) 
	{
		RCinputs[ELEVATOR] = 0;
	}
	if ((RCinputs[RUDDER] < DEAD_BAND) && (RCinputs[RUDDER] > -DEAD_BAND))
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


	// Hands-free detection (won't work with flaperons set)
	if (((aileron_actual < Config.HandsFreetrigger) && (aileron_actual > -Config.HandsFreetrigger))
	 && ((RCinputs[ELEVATOR]  < Config.HandsFreetrigger) && (RCinputs[ELEVATOR]  > -Config.HandsFreetrigger)))
	{
		HandsFree = true;
	}
	else
	{
		HandsFree = false;
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
		Config.Channel[i].Failsafe = failsafe;
	}

	Save_Config_to_EEPROM();
}

