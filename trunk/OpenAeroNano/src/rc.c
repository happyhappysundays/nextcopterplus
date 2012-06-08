//***********************************************************
//* rc.c
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include <avr/io.h>
#include <stdbool.h>
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

int16_t		RxInRoll;					// RC axis values
int16_t		RxInPitch;
uint16_t	RxInAux;
int16_t		RxInYaw;

//--- Get and scale RX channel inputs ---
void RxGetChannels(void)
{
	int16_t  RxChannel;
	do
	{
		RxChannelsUpdatedFlag = false;
		RxInRoll = RxChannel1 - Config.RxChannel1ZeroOffset;
	} 
	while (RxChannelsUpdatedFlag); 	// Re-get if updated

	do
	{
		RxChannelsUpdatedFlag = false;
		RxInPitch = RxChannel2 - Config.RxChannel2ZeroOffset;
	} 
	while (RxChannelsUpdatedFlag);

	do
	{
		RxChannelsUpdatedFlag = false;
		RxChannel = RxChannel3 - Config.RxChannel3ZeroOffset; //1500
		if (RxChannel < 0) RxInAux = 0;
		else RxInAux = RxChannel;
	} 
	while (RxChannelsUpdatedFlag);
	
	do
	{
		RxChannelsUpdatedFlag = false;
		RxInYaw = RxChannel4 - Config.RxChannel4ZeroOffset;
	} 
	while (RxChannelsUpdatedFlag);
}
