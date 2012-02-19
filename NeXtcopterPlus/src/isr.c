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

volatile bool RxChannelsUpdatedFlag;

volatile uint16_t RxChannel1; 	// These variables are local but defining them 
volatile uint16_t RxChannel2; 	// here means that there is less pushing/popping from
volatile uint16_t RxChannel3; 	// the stack
volatile uint16_t RxChannel4;
volatile uint16_t RxChannel5;
uint16_t RxChannel1Start;	
uint16_t RxChannel2Start;	
uint16_t RxChannel3Start;	
uint16_t RxChannel4Start;
uint16_t RxChannel5Start;

#ifndef CPPM_MODE
volatile uint8_t PCINT2_state;	// Need to keep track of previous PCINT2_state to separate RX_ROLL and RX_AUX
#endif

#ifdef CPPM_MODE
volatile uint16_t RxChannel6;
volatile uint16_t RxChannel7;
volatile uint16_t RxChannel8;
uint16_t RxChannel6Start;
uint16_t RxChannel7Start;
uint16_t RxChannel8Start;
volatile uint16_t PPMSyncStart;	// Sync pulse timer
volatile uint8_t ch_num;		// Channel number
#define SYNCPULSEWIDTH 3000		// Sync pulse must be more than 3ms long
#ifdef PROX_MODULE
volatile uint16_t EchoTime;		// Sonar echo duration
uint16_t EchoTimeStart;			// Sonar echo start
#endif

#endif //CPPM_MODE

//************************************************************
// Standard mode
//************************************************************

#ifndef CPPM_MODE  	// Normal RX mode

// For PCINT2 we have to somehow work out which interrupt (RX_ROLL or RX_AUX/CH5) triggered this.
// Since there is no interrupt flag, we have to create a state machine to keep track of both these
// signals. It seems to work well. I guess some people *really* wanted five PWM RX inputs :) 
ISR(PCINT2_vect)
{
	// Only one source of interrupt when we need all six motor outputs 
	#if defined(HEXA_COPTER) || defined(HEXA_X_COPTER)
	if (RX_ROLL)						// Rising
	{
		RxChannel1Start = TCNT1;
	} 
	else 
	{									// Falling
		RxChannel1 = TCNT1 - RxChannel1Start;
	}
	RxChannelsUpdatedFlag = true;

	// Use M6 as a fifth RC input channel
	#else
	switch(PCINT2_state)
	{
		case 0:							// Default state. Assume both RX_ROLL and RX_AUX were low
			if (RX_ROLL)				// RX_ROLL rising
			{
				RxChannel1Start = TCNT1;
				PCINT2_state = 1;		// Set state to RX_ROLL running
			} 
			else if (RX_AUX)			// RX_AUX rising
			{
				RxChannel5Start = TCNT1;
				PCINT2_state = 2;		// Set state to RX_AUX running
			}
			else PCINT2_state = 0;		// Otherwise reset state to both low
			break;

		case 1:							// RX_ROLL running -> falling edge
			RxChannel1 = TCNT1 - RxChannel1Start;
			RxChannelsUpdatedFlag = true;
			PCINT2_state = 0;
			break;
		case 2: 						// RX_AUX running -> falling edge
			RxChannel5 = TCNT1 - RxChannel5Start;
			RxChannelsUpdatedFlag = true;
			PCINT2_state = 0;
			break;
		default:
			PCINT2_state = 0;			// Reset PCINT2_state if lost
			break;
	}
	#endif

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

//************************************************************
// PPM RX mode 	- input on PD2 (INT0/elevator)
// NB: JR/Spectrum channel order (Th,Ai,El,Ru,5,6,7,8)
// 	   Other brands of TX will lead to the wrong channels
//	   being decoded into the NeXtcopter RxChannel variables
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
			break;
		default:
			break;
	} // Switch
} // ISR(INT0_vect)

#ifdef PROX_MODULE
ISR(INT1_vect)
{
	if (ECHO)		// Echo pulse just started
	{
		EchoTimeStart = TCNT1;
	} 
	else 
	{				// Echo pulse completed
		EchoTime = TCNT1 - EchoTimeStart;
	}
} // ISR(INT1_vect)
#endif
#endif
