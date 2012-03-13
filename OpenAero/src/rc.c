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
#include "..\inc\io_cfg.h"

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
int32_t		RxSum, OldRxSum;			// Sum of all major channels
#if defined(STD_FLAPERON)
int16_t		RxInAux1;
#endif
bool		RxActivity;

#define		NOISE_THRESH	50			// Max RX noise threshold. Increase if lost alarm keeps being reset.

//--- Get and scale RX channel inputs ---
void RxGetChannels(void)
{
	int16_t  RxChannel;
	int32_t	 RxSumDiff;
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

#if defined(STD_FLAPERON)
	do
	{
		RxChannelsUpdatedFlag = false;
		RxInAux1 = RxChannel6 - Config.RxChannel6ZeroOffset;
	} 
	while (RxChannelsUpdatedFlag);
#endif


	RxSum = RxInRoll + RxInPitch + RxInAux + RxInYaw;
	RxSumDiff = RxSum - OldRxSum;

	if ((RxSumDiff > NOISE_THRESH) || (RxSumDiff < -NOISE_THRESH)) RxActivity = true;
	else RxActivity = false;
	OldRxSum = RxSum;
}
