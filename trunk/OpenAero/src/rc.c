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

#define		NOISE_THRESH	20			// Max RX noise threshold. Increase if lost alarm keeps being reset.

// Get raw flight channel data and remove zero offset
void RxGetChannels(void)
{
	int16_t  RxChannel;
	int32_t	 RxSumDiff;

	RxInRoll = RxChannel1 - Config.RxChannel1ZeroOffset;
	RxInPitch = RxChannel2 - Config.RxChannel2ZeroOffset;


		RxChannel = RxChannel3 - Config.RxChannel3ZeroOffset; //1500
		if (RxChannel < 0) RxInAux = 0;
		else RxInAux = RxChannel;

//	RxInAux = RxChannel3 - Config.RxChannel3ZeroOffset;



	RxInYaw = RxChannel4 - Config.RxChannel4ZeroOffset;

#if defined(STD_FLAPERON)
	RxInAux1 = RxChannel5 - Config.RxChannel5ZeroOffset;
#endif

	// Calculate RX activity
	RxSum = RxInRoll + RxInPitch + RxInAux + RxInYaw;
	RxSumDiff = RxSum - OldRxSum;

	// Set RX activity flag
	if ((RxSumDiff > NOISE_THRESH) || (RxSumDiff < -NOISE_THRESH)) RxActivity = true;
	else RxActivity = false;
	OldRxSum = RxSum;
}

