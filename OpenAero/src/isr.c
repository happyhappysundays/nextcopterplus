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

volatile bool Interrupted;

volatile uint16_t RxChannel1; // These variables are local but defining them 
volatile uint16_t RxChannel2; // here means that there is less pushing/popping from
volatile uint16_t RxChannel3; // the stack
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

volatile uint16_t icp_value;		// Current ICP value
volatile uint16_t RxChannelStart;	// ICP measurement start variable
volatile uint16_t PPMSyncStart;		// Sync pulse timer
volatile uint8_t ch_num;			// Current channel number
volatile uint8_t max_chan;			// Target channel number

uint16_t gapstart;					// Start of gap measurement from current channel
uint16_t gapend;					// End of gap measurement from current channel
uint16_t gap;						// Size of gap
bool	gapfound;					// Flag to indentify that gap has been calculated
bool	gapready;					// Used to skip over the first incidence of an input

#define SYNCPULSEWIDTH 3000			// Sync pulse must be more than 3ms long
#define GAPPULSEWIDTH 5000			// Servo update gap must be more than 5ms long

//************************************************************
// Standard PWM mode - original version
//************************************************************

#ifndef CPPM_MODE  	// Normal RX mode

#ifdef LEGACY_PWM_MODE
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
}

ISR(INT1_vect)
{
	if (RX_COLL)	// Rising
	{
		RxChannel3Start = TCNT1;
	} 
	else 
	{				// Falling
		RxChannel3 = TCNT1 - RxChannel3Start;	// Normally the last channel to arrive
		Interrupted = true;						// Signal that interrupt block has finished
	}
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
}

//************************************************************
// Standard PWM mode - predictive version. Still buggy.
//************************************************************

#else

ISR(PCINT2_vect)
{
	if (RX_ROLL)						// Interrupt must have been from the rising edge
	{
		RxChannel1Start = TCNT1;		// Mark start of pulse
	} 
	else 								// Interrupt must have been from the falling edge
	{									// Falling
		RxChannel1 = TCNT1 - RxChannel1Start; // Calculate pulse width
		if (gapfound == false) 			// If gap not found yet
		{
			// If not the first interrupt (since no gapstart set up yet)
			if (gapready)
			{
				gapend = TCNT1;
				gap = gapend - gapstart;

				// If gap bigger than GAPPULSEWIDTH, mark as found
				if (gap > GAPPULSEWIDTH)
				{
					gapfound = true;	// Flag that a gap is found
					max_chan = ch_num;
				}
			}
			// All cases
			gapstart = TCNT1;			// Measure start of gap
			gapready = true;			// Done at least one interrupt
			ch_num = 1;					// Set the channel to the source channel
		}
		// Gap already found
		else if (max_chan == 1)			// If this is the selected channel number the gap starts from here
		{
			Interrupted = true;			// Signal that interrupt block has finished
		}
	}
}

ISR(INT0_vect)
{
	if (RX_PITCH)	// Rising 
	{
		RxChannel2Start = TCNT1;		// Mark start of pulse
	} 
	else 								// Interrupt must have been from the falling edge
	{				// Falling
		RxChannel2 = TCNT1 - RxChannel2Start; // Calculate pulse width
		if (gapfound == false) 			// If gap not found yet
		{
			// If not the first interrupt (since no gapstart set up yet)
			if (gapready)
			{
				gapend = TCNT1;
				gap = gapend - gapstart;

				// If gap bigger than GAPPULSEWIDTH, mark as found
				if (gap > GAPPULSEWIDTH)
				{
					gapfound = true;	// Flag that a gap is found
					max_chan = ch_num;
				}
			}
			// All cases
			gapstart = TCNT1;			// Measure start of gap
			gapready = true;			// Done at least one interrupt
			ch_num = 2;					// Set the channel to the source channel
		}
		// Gap already found
		else if (max_chan == 2)			// If this is the selected channel number the gap starts from here
		{
			Interrupted = true;			// Signal that interrupt block has finished
		}
	}
}

ISR(INT1_vect)
{
	if (RX_COLL)	// Rising
	{
		RxChannel3Start = TCNT1;		// Mark start of pulse
	} 
	else 								// Interrupt must have been from the falling edge
	{				// Falling
		RxChannel3 = TCNT1 - RxChannel3Start; // Calculate pulse width
		if (gapfound == false) 			// If gap not found yet
		{
			// If not the first interrupt (since no gapstart set up yet)
			if (gapready)
			{
				gapend = TCNT1;
				gap = gapend - gapstart;

				// If gap bigger than GAPPULSEWIDTH, mark as found
				if (gap > GAPPULSEWIDTH)
				{
					gapfound = true;	// Flag that a gap is found
					max_chan = ch_num;
				}
			}
			// All cases
			gapstart = TCNT1;			// Measure start of gap
			gapready = true;			// Done at least one interrupt
			ch_num = 3;					// Set the channel to the source channel
		}
		// Gap already found
		else if (max_chan == 3)			// If this is the selected channel number the gap starts from here
		{
			Interrupted = true;			// Signal that interrupt block has finished
		}
	}
}

ISR(PCINT0_vect)
{
	if (RX_YAW)		// Rising
	{
		RxChannel4Start = TCNT1;		// Mark start of pulse
	} 
	else 								// Interrupt must have been from the falling edge
	{				// Falling
		RxChannel4 = TCNT1 - RxChannel4Start; // Calculate pulse width
		if (gapfound == false) 			// If gap not found yet
		{
			// If not the first interrupt (since no gapstart set up yet)
			if (gapready)
			{
				gapend = TCNT1;
				gap = gapend - gapstart;

				// If gap bigger than GAPPULSEWIDTH, mark as found
				if (gap > GAPPULSEWIDTH)
				{
					gapfound = true;	// Flag that a gap is found
					max_chan = ch_num;
				}
			}
			// All cases
			gapstart = TCNT1;			// Measure start of gap
			gapready = true;			// Done at least one interrupt
			ch_num = 4;					// Set the channel to the source channel
		}
		// Gap already found
		else if (max_chan == 4)			// If this is the selected channel number the gap starts from here
		{
			Interrupted = true;			// Signal that interrupt block has finished
		}
	}
}
#endif

#elif defined(ICP_CPPM_MODE) // ICP CPPM mode

//************************************************************
// CPPM RX mode 	- input on PB0 (ICP/M3)
// NB: JR/Spectrum channel order (Th,Ai,El,Ru,5,6,7,8)
// 	   Other brands of TX will lead to the wrong channels
//	   being decoded into the RxChannel variables
//	   unless they are changed here.
//************************************************************

ISR(TIMER1_CAPT_vect)
{	// Check to see if previous period was a sync pulse
	// If so, reset channel number

	icp_value = ICR1;
	if ((icp_value - PPMSyncStart) > SYNCPULSEWIDTH) ch_num = 0;
	PPMSyncStart = icp_value;

	switch(ch_num)
	{
		case 0:
			RxChannelStart = icp_value;
			ch_num++;
			break;
		case 1:
			RxChannel3 = icp_value - RxChannelStart;
			RxChannelStart = icp_value;
			ch_num++;
			break;
		case 2:
			RxChannel1 = icp_value - RxChannelStart;
			RxChannelStart = icp_value;
			ch_num++;
			break;
		case 3:
			RxChannel2 = icp_value - RxChannelStart;
			RxChannelStart = icp_value;
			ch_num++;
			break;
		case 4:
			RxChannel4 = icp_value - RxChannelStart;
			RxChannelStart = icp_value;
			ch_num++;
			break;
		case 5:
			RxChannel5 = icp_value - RxChannelStart;
			RxChannelStart = icp_value;
			ch_num++;
			break;
		case 6:
			RxChannel6 = icp_value - RxChannelStart;
			RxChannelStart = icp_value;
			ch_num++;
			break;
		case 7:
			RxChannel7 = icp_value - RxChannelStart;
			RxChannelStart = icp_value;
			ch_num++;
			break;
		case 8:
			RxChannel8 = icp_value - RxChannelStart;
			RxChannelStart = icp_value;
			ch_num++;
			break;
		default:
			break;
	} // Switch


	// Work out the highest channel number automagically
	if (ch_num > max_chan)	
	{
		max_chan = ch_num;					// Reset max channel number
	}
	else if (ch_num == max_chan)
	{
		Interrupted = true;					// Signal that interrupt block has finished
	}
}

#else // INT0 CPPM mode

//************************************************************
// CPPM RX mode 	- input on PD2 (INT0/elevator)
// NB: JR/Spectrum channel order (Th,Ai,El,Ru,5,6,7,8)
// 	   Other brands of TX will lead to the wrong channels
//	   being decoded into the RxChannel variables
//	   unless they are changed here.
//************************************************************

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
			ch_num++;
			break;
		case 2:
			RxChannel3Start = TCNT1;
			RxChannel1 = TCNT1 - RxChannel2Start;	// Ch1 - Aileron
			ch_num++;
			break;
		case 3:
			RxChannel4Start = TCNT1;
			RxChannel2 = TCNT1 - RxChannel3Start;	// Ch2 - Elevator
			ch_num++;
			break;
		case 4:
			RxChannel5Start = TCNT1;
			RxChannel4 = TCNT1 - RxChannel4Start;	// Ch4 - Rudder
			ch_num++;
			break;
		case 5:
			RxChannel6Start = TCNT1;
			RxChannel5 = TCNT1 - RxChannel5Start;	// Ch5 - Gear/Aileron 2
			ch_num++;
			break;
		case 6:
			RxChannel7Start = TCNT1;
			RxChannel6 = TCNT1 - RxChannel6Start;	// Ch6 - Flap 1
			ch_num++;
			break;
		case 7:
			RxChannel8Start = TCNT1;
			RxChannel7 = TCNT1 - RxChannel7Start;	// Ch7 - Flap 2
			ch_num++;
			break;
		case 8:
			RxChannel8 = TCNT1 - RxChannel8Start;	// Ch8 - AUX3
			ch_num++;
			break;
		default:
			break;
	} // Switch

	// Work out the highest channel number automagically
	if (ch_num > max_chan)	
	{
		max_chan = ch_num;					// Reset max channel number
	}
	else if (ch_num == max_chan)
	{
		Interrupted = true;					// Signal that interrupt block has finished
	}
} // ISR(INT0_vect)

#endif
