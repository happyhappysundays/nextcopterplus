//***********************************************************
//* isr.c
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include <stdbool.h>
#include <avr/interrupt.h>
#include "..\inc\io_cfg.h"
//#include "..\inc\main.h"

//************************************************************
// Interrupt vectors
//************************************************************

volatile bool RxChannelsUpdatedFlag;
volatile bool Interrupted;

volatile uint16_t RxChannel1; // These variables are local but defining them 
volatile uint16_t RxChannel2; // here means that there is less pushing/popping from
volatile uint16_t RxChannel3; // the stack
volatile uint16_t RxChannel4;
uint16_t RxChannel1Start;	
uint16_t RxChannel2Start;	
uint16_t RxChannel3Start;	
uint16_t RxChannel4Start;

#ifdef CPPM_MODE
volatile uint16_t RxChannel5;
volatile uint16_t RxChannel6;
volatile uint16_t RxChannel7;
volatile uint16_t RxChannel8;
uint16_t RxChannel5Start;
uint16_t RxChannel6Start;
uint16_t RxChannel7Start;
uint16_t RxChannel8Start;

volatile uint16_t PPMSyncStart;		// Sync pulse timer
volatile uint8_t ch_num;			// Channel number
#define SYNCPULSEWIDTH 3000			// Sync pulse must be more than 3ms long

#endif //CPPM_MODE

//************************************************************
// Standard mode
//************************************************************

#ifndef CPPM_MODE  	// Normal RX mode
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
		Interrupted = true;						// Signal that interrupt block has finished
	}
	RxChannelsUpdatedFlag = true;
}

//************************************************************
// PPM RX mode 	- input on PD2 (INT0/elevator)
// NB: JR/Spectrum channel order (Th,Ai,El,Ru,5,6,7,8)
// 	   Other brands of TX will lead to the wrong channels
//	   being decoded into the RxChannel variables
//	   unless they are changed here.
//************************************************************

#else
ISR(INT0_vect)
{	// Check to see if previous period was a sync pulse
	// If so, reset channel number

	if ((TCNT1 - PPMSyncStart) > SYNCPULSEWIDTH) ch_num = 0;
	PPMSyncStart = TCNT1;
	switch(ch_num)
	{
		case 0:
			RxChannel1Start = TCNT1;
			ch_num++;
			break;
		case 1:
			RxChannel2Start = TCNT1;
			RxChannel3 = TCNT1 - RxChannel1Start;	// Ch3 - Throttle
			RxChannelsUpdatedFlag = true;			// Flag that data changed
			ch_num++;
			break;
		case 2:
			RxChannel3Start = TCNT1;
			RxChannel1 = TCNT1 - RxChannel2Start;	// Ch1 - Aileron
			RxChannelsUpdatedFlag = true;			// Flag that data changed
			ch_num++;
			break;
		case 3:
			RxChannel4Start = TCNT1;
			RxChannel2 = TCNT1 - RxChannel3Start;	// Ch2 - Elevator
			RxChannelsUpdatedFlag = true;			// Flag that data changed
			ch_num++;
			break;
		case 4:
			RxChannel5Start = TCNT1;
			RxChannel4 = TCNT1 - RxChannel4Start;	// Ch4 - Rudder
			RxChannelsUpdatedFlag = true;			// Flag that data changed
			ch_num++;
			break;
		case 5:
			RxChannel6Start = TCNT1;
			RxChannel5 = TCNT1 - RxChannel5Start;	// Ch5 - Gear
			ch_num++;
			break;
		case 6:
			RxChannel7Start = TCNT1;
			RxChannel6 = TCNT1 - RxChannel6Start;	// Ch6 - Flap 
			ch_num++;
			break;
		case 7:
			RxChannel8Start = TCNT1;
			RxChannel7 = TCNT1 - RxChannel7Start;	// Ch7 - AUX2
			ch_num++;
			break;
		case 8:
			RxChannel8 = TCNT1 - RxChannel8Start;	// Ch8 - AUX3
			ch_num++;
			Interrupted = true;						// Signal that interrupt block has finished
			break;
		default:
			break;
	} // Switch
} // ISR(INT0_vect)

#endif
