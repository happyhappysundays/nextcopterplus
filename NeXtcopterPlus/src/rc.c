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
#include "..\inc\typedefs.h"
#include "..\inc\isr.h"
#include "..\inc\init.h"
#include <avr/pgmspace.h> 

//************************************************************
// Prototypes
//************************************************************

void RxGetChannels(void);
int16_t get_expo_value (int16_t RCvalue);
int16_t get_acc_expo_value (int16_t ACCvalue);

//************************************************************
// Code
//************************************************************

int16_t  RxInRoll;					// RC axis values
int16_t  RxInPitch;
uint16_t RxInCollective;
int16_t  RxInYaw;

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

// Get expo'd RC value
int16_t get_expo_value (int16_t RCvalue)
{
	int8_t	range, expo_level;
	int32_t RCcalc, RCsum, mult;						// Max values are around+/-75000

	if (Config.RC_expo == 0) return (RCvalue);			// No need to calculate if expo is zero

	range 	= abs(RCvalue) >> 5;						// Work out which band the RCinput is in (16 values)
	if (range > 15) range = 15;
	if (range <  0) range = 0;
	expo_level = Config.RC_expo/10;						// Work out which expo level to use (0~100 -> 0~10)	
				
	mult = pgm_read_byte(&Expo[expo_level][range]); 	// Get the multiplier from the table 

	RCsum 	= RCvalue;									// Promote RCvalue to 32 bits as GCC is broken :(
	RCcalc 	= RCsum * mult;								// Do the 32-bit x 32-bit multiply
	RCcalc	= RCcalc >> 7;
	RCvalue = (int16_t) RCcalc;							// Divide by 128 to get the expo'd value

	return (RCvalue);
}

// Get expo'd Accelerometer value
int16_t get_acc_expo_value (int16_t ACCvalue)
{
	int8_t	range, expo_level;
	int32_t ACCcalc, ACCsum, mult;						// Max values are around+/-75000

	if (Config.ACC_expo == 0) return (ACCvalue);		// No need to calculate if expo is zero

	range 	= abs(ACCvalue) >> 3;						// Work out which band the ACCvalue is in (16 values)
	if (range > 15) range = 15;
	if (range <  0) range = 0;
	expo_level = Config.ACC_expo/10;					// Work out which expo level to use (0~100 -> 0~10)	
					
	mult = pgm_read_byte(&Expo[expo_level][range]); 	// Get the multiplier from the table 

	ACCsum 	= ACCvalue;									// Promote ACCvalue to 32 bits as GCC is broken :(
	ACCcalc = ACCsum * mult;							// Do the 32-bit x 32-bit multiply
	ACCcalc	= ACCcalc >> 7;
	ACCvalue = (int16_t) ACCcalc;						// Divide by 128 to get the expo'd value

	return (ACCvalue);
}

//--- Get and scale RX channel inputs ---
void RxGetChannels(void)
{
	int16_t  RxChannel;
	do
	{
		RxChannelsUpdatedFlag = false;
		RxInRoll = get_expo_value(RxChannel1 - Config.RxChannel1ZeroOffset);
	} 
	while (RxChannelsUpdatedFlag); 	// Re-get if updated

	do
	{
		RxChannelsUpdatedFlag = false;
		RxInPitch = get_expo_value(RxChannel2 - Config.RxChannel2ZeroOffset);
	} 
	while (RxChannelsUpdatedFlag);

	do
	{
		RxChannelsUpdatedFlag = false;
		RxChannel = RxChannel3 - Config.RxChannel3ZeroOffset;	
		if (RxChannel < 0) RxInCollective = 0; 	
		else RxInCollective = (RxChannel >> 2); 	// Scale 0->256
	} 
	while (RxChannelsUpdatedFlag);
	
	do
	{
		RxChannelsUpdatedFlag = false;
		RxInYaw = get_expo_value(RxChannel4 - Config.RxChannel4ZeroOffset);
	} 
	while (RxChannelsUpdatedFlag);
}
