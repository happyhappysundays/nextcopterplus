//***********************************************************
//* isr.c
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include <stdbool.h>
#include <avr/interrupt.h>
#include "..\inc\io_cfg.h"

//************************************************************
// Interrupt vectors
//************************************************************

volatile bool RxChannelsUpdatedFlag;

volatile uint16_t RxChannel1;
volatile uint16_t RxChannel2;
volatile uint16_t RxChannel3;
volatile uint16_t RxChannel4;

uint16_t RxChannel1Start;
uint16_t RxChannel2Start;
uint16_t RxChannel3Start;
uint16_t RxChannel4Start;

// RX_ROLL
ISR(PCINT2_vect)
{
	if ( RX_ROLL )			// Rising
	{
		RxChannel1Start = TCNT1;
	} else {				// Falling
		RxChannel1 = TCNT1 - RxChannel1Start;
	}
	RxChannelsUpdatedFlag = true;
}

// RX_PITCH
ISR(INT0_vect)
{
	if (RX_PITCH)			// Rising
	{
		RxChannel2Start = TCNT1;
	} else {				// Falling
		RxChannel2 = TCNT1 - RxChannel2Start;
	}
	RxChannelsUpdatedFlag = true;
}

// RX_COLL
ISR(INT1_vect)
{
	if (RX_COLL)			// Rising
	{
		RxChannel3Start = TCNT1;

	} else {				// Falling
		RxChannel3 = TCNT1 - RxChannel3Start;
	}
	RxChannelsUpdatedFlag = true;
}

// RX_YAW
ISR(PCINT0_vect)
{
	if ( RX_YAW )			// Rising
	{
		RxChannel4Start = TCNT1;
	} else {				// Falling
		RxChannel4 = TCNT1 - RxChannel4Start;
	}
	RxChannelsUpdatedFlag = true;
}
