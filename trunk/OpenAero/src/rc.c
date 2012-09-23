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
#include "..\inc\servos.h"
#include "..\inc\main.h"

//************************************************************
// Prototypes
//************************************************************

void RxGetChannels(void);
void SetServoPositions(void);

//************************************************************
// Code
//************************************************************

int16_t		RxInRoll;					// RC axis values
int16_t		RxInPitch;
uint16_t	RxInAux;
int16_t		RxInYaw;
int32_t		RxSum, OldRxSum;			// Sum of all major channels
int16_t		RxInAux1;
int16_t		StabChan;

bool		RxActivity;

//************************************************************
// Defines
//************************************************************
#define		NOISE_THRESH	20			// Max RX noise threshold. Increase if lost alarm keeps being reset.
#define		CH_COMPLETED	9

// Get raw flight channel data and remove zero offset
void RxGetChannels(void)
{

	int16_t  RxChannel;
	int32_t	 RxSumDiff;

	RxInRoll = RxChannel1 - Config.RxChannel1ZeroOffset;
	RxInPitch = RxChannel2 - Config.RxChannel2ZeroOffset;

	RxChannel = RxChannel3 - Config.RxChannel3ZeroOffset; //1520
	if (RxChannel < 0) RxInAux = 0;
	else RxInAux = RxChannel;

	RxInYaw = RxChannel4 - Config.RxChannel4ZeroOffset;
	RxInAux1 = RxChannel5 - Config.RxChannel5ZeroOffset;

	// Calculate RX activity
	RxSum = RxInRoll + RxInPitch + RxInAux + RxInYaw;
	RxSumDiff = RxSum - OldRxSum;

	// Set RX activity flag
	if ((RxSumDiff > NOISE_THRESH) || (RxSumDiff < -NOISE_THRESH)) RxActivity = true;
	else RxActivity = false;
	OldRxSum = RxSum;

	// Get stability switch data from requested channel
	switch(Config.StabChan)
	{
		case 1:
			StabChan = RxChannel1;
			break;
		case 2:
			StabChan = RxChannel2;
			break;
		case 3:
			StabChan = RxChannel3;
			break;
		case 4:
			StabChan = RxChannel4;
			break;
		case 5:
			StabChan = RxChannel5;
			break;
		case 6:
			StabChan = RxChannel6;
			break;
		case 7:
			StabChan = RxChannel7;
			break;
		case 8:
			StabChan = RxChannel8;
			break;
		default:
			StabChan = RxChannel8;	// Channel 8 switches stability by default
			break;
	}
}

// Set servo position
// Note that in N6 mode, ServoOut1 redirects to M3 in servos_asm.S
void SetServoPositions(void)	
{
	if(Config.YawServo) {
		ServoOut1 = Config.RxChannel4ZeroOffset - RxInYaw;
	}
	else {
		ServoOut1 = RxChannel4;
	}

#ifndef N6_MODE
	#if defined(STD_FLAPERON)
		if(Config.RollServo) {
			ServoOut2 = Config.RxChannel5ZeroOffset - RxInRoll;
			ServoOut5 = Config.RxChannel1ZeroOffset - RxInAux1;
		}
		else {
			ServoOut2 = RxChannel5;
			ServoOut5 = RxChannel1;
		}
	#else
		if(Config.RollServo) {
			ServoOut2 = Config.RxChannel1ZeroOffset - RxInRoll;
		}
		else {
			ServoOut2 = RxChannel1;
		}
	#endif
#else
	switch(MixerMode)
	{
		case 1:						// Flaperon mixing
			if(Config.RollServo) {
				ServoOut2 = Config.RxChannel5ZeroOffset - RxInRoll;
				ServoOut5 = Config.RxChannel1ZeroOffset - RxInAux1;
			}
			else {
				ServoOut2 = RxChannel5;
				ServoOut5 = RxChannel1;
			}
			break;
		default:					// Everything else
			if(Config.RollServo) {
				ServoOut2 = Config.RxChannel1ZeroOffset - RxInRoll;
			}
			else {
				ServoOut2 = RxChannel1;
			}
			break;
	}
#endif

	if(Config.PitchServo) {
		ServoOut4 = Config.RxChannel2ZeroOffset - RxInPitch;
	}
	else {
		ServoOut4 = RxChannel2;
	}
	Throttle = RxChannel3;
}
