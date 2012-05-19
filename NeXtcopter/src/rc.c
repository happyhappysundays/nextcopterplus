//***********************************************************
//* rc.c
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include <avr/io.h>
#include <stdbool.h>
#include <avr/interrupt.h>
#include "..\inc\typedefs.h"
#include "..\inc\isr.h"
#include "..\inc\init.h"

//************************************************************
// Prototypes
//************************************************************

void RxGetChannels(void);

//************************************************************
// Code
//************************************************************

int8_t 	RxInRoll;					// RC axis values
int8_t 	RxInPitch;
uint8_t RxInCollective;
int8_t 	RxInYaw;

//--- Get and scale RX channel inputs ---
void RxGetChannels(void)
{
	int16_t RxChannel;

	do
	{
		RxChannelsUpdatedFlag = false;
		RxChannel = RxChannel1 - Config.RxChannel1ZeroOffset;	// Normalise to +-100
		RxInRoll = (RxChannel >> 2);
	} 
	while (RxChannelsUpdatedFlag); 	// Re-get if updated

	do
	{
		RxChannelsUpdatedFlag = false;
		RxChannel = RxChannel2 - Config.RxChannel2ZeroOffset;	// Normalise
		RxInPitch = (RxChannel >> 2); 
	} 
	while (RxChannelsUpdatedFlag);

	do
	{
		RxChannelsUpdatedFlag = false;
		RxChannel = RxChannel3 - Config.RxChannel3ZeroOffset;	
		if (RxChannel < 0) RxInCollective = 0; 		// Avoid negative values in unsigned 8bit int
		else RxInCollective = (RxChannel >> 2); 	// Scale 0->200
	} 
	while (RxChannelsUpdatedFlag);
	
	do
	{
		RxChannelsUpdatedFlag = false;
		RxChannel = RxChannel4 - Config.RxChannel4ZeroOffset;		// Normalise
		RxInYaw = (RxChannel >> 2); 
	} 
	while (RxChannelsUpdatedFlag);

}
