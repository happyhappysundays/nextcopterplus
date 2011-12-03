//***********************************************************
//* isr.c
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include <stdbool.h>
#include <avr/interrupt.h>
#include "..\inc\io_cfg.h"
#include "..\inc\main.h"

//************************************************************
// Interrupt vectors
//************************************************************

//#define PPM_MODE // Uncomment this for PPM support on CH1

volatile bool RxChannelsUpdatedFlag;

volatile uint16_t RxChannel1;
volatile uint16_t RxChannel2;
volatile uint16_t RxChannel3;
volatile uint16_t RxChannel4;
volatile uint16_t RxChannel5;
volatile uint16_t RxChannel6;
volatile uint16_t RxChannel7;
volatile uint16_t RxChannel8;
uint16_t RxChannel1Start;
uint16_t RxChannel2Start;
uint16_t RxChannel3Start;
uint16_t RxChannel4Start;
uint16_t RxChannel5Start;
uint16_t RxChannel6Start;
uint16_t RxChannel7Start;
uint16_t RxChannel8Start;
uint16_t PPMSyncStart;
uint8_t ch_num = 0;	// Channel number

#define SYNCPULSEWIDTH 10000 // Sync pulse must be more than 10us long

#ifndef PPM_MODE  	// Normal RX mode
ISR(PCINT2_vect)
{
	if (RX_ROLL)	// Rising
	{
		RxChannel1Start = TCNT1;
	} 
	else 
	{				// Falling
		RxChannel1 = TCNT1 - RxChannel1Start;
	}
	RxChannelsUpdatedFlag = true;
}

ISR(INT0_vect)
{
	if (RX_PITCH)	// Rising
	{
		RxChannel2Start = TCNT1;
	} 
	else 
	{				// Falling
		RxChannel2 = TCNT1 - RxChannel2Start;
	}
	RxChannelsUpdatedFlag = true;
}

ISR(INT1_vect)
{
	if (RX_COLL)	// Rising
	{
		RxChannel3Start = TCNT1;
	} 
	else 
	{				// Falling
		RxChannel3 = TCNT1 - RxChannel3Start;
	}
	RxChannelsUpdatedFlag = true;
}

ISR(PCINT0_vect)
{
	if (RX_YAW)	// Rising
	{
		RxChannel4Start = TCNT1;
	} 
	else 
	{				// Falling
		RxChannel4 = TCNT1 - RxChannel4Start;
	}
	RxChannelsUpdatedFlag = true;
}

#else	// PPM RX mode

ISR(PCINT2_vect)
{
	// Check to see if previous period was a sync pulse
	// If so, reset channel number
	if (RX_ROLL)
	{
		if ((TCNT1 - PPMSyncStart) > SYNCPULSEWIDTH) ch_num = 0;
	}
	switch(ch_num)
	{
		case 0:
			if (RX_ROLL) RxChannel1Start = TCNT1;		// Rising
			else 
			{
				RxChannel1 = TCNT1 - RxChannel1Start;	// Falling
				ch_num++;
			}
			break;
		case 1:
			if (RX_ROLL) RxChannel2Start = TCNT1;		// Rising
			else 
			{
				RxChannel2 = TCNT1 - RxChannel2Start;	// Falling
				ch_num++;
			}
			break;
		case 2:
			if (RX_ROLL) RxChannel3Start = TCNT1;		// Rising
			else 
			{
				RxChannel3 = TCNT1 - RxChannel3Start;	// Falling
				ch_num++;
			}
			break;
		case 3:
			if (RX_ROLL) RxChannel4Start = TCNT1;		// Rising
			else 
			{
				RxChannel4 = TCNT1 - RxChannel4Start;	// Falling
				ch_num++;
			}
			break;
		case 4:
			if (RX_ROLL) RxChannel5Start = TCNT1;		// Rising
			else 
			{
				RxChannel5 = TCNT1 - RxChannel5Start;	// Falling
				ch_num++;
			}
			break;
		case 5:
			if (RX_ROLL) RxChannel6Start = TCNT1;		// Rising
			else 
			{
				RxChannel6 = TCNT1 - RxChannel6Start;	// Falling
				ch_num++;
			}
			break;
		case 6:
			if (RX_ROLL) RxChannel7Start = TCNT1;		// Rising
			else 
			{
				RxChannel7 = TCNT1 - RxChannel7Start;	// Falling
				ch_num++;
			}
			break;
		case 7:
			if (RX_ROLL) RxChannel8Start = TCNT1;		// Rising
			else 
			{
				RxChannel8 = TCNT1 - RxChannel8Start;	// Falling
				ch_num++;
			}
			break;
		default:
			RxChannelsUpdatedFlag = true;				// Flag that data changed
			break;
	} // Switch
} // ISR(PCINT2_vect)

#endif
