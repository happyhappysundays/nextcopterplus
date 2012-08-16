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
volatile uint16_t RxChannel[MAX_RC_SOURCES]; // There are more sources than RC channels
volatile uint16_t RxChannelStart[MAX_RC_CHANNELS];	
volatile uint16_t PPMSyncStart;		// Sync pulse timer
volatile uint8_t ch_num;			// Current channel number
volatile uint8_t max_chan;			// Target channel number
bool	 RC_Lock;					// RC sync found/lost flag

#define SYNCPULSEWIDTH 6750			// Sync pulse must be more than 2.7ms long

//************************************************************
//* Standard PWM mode
//* Sequential PWM inputs from a normal RC receiver
//************************************************************

ISR(INT1_vect)
{
	if (RX_ROLL)	// Rising
	{
		RxChannelStart[AILERON] = TCNT1;
	} 
	else 
	{				// Falling
		RxChannel[AILERON] = TCNT1 - RxChannelStart[AILERON];
	}
}

ISR(INT0_vect)
{
	if (RX_PITCH)	// Rising 
	{
		RxChannelStart[ELEVATOR] = TCNT1;
	} 
	else 
	{				// Falling
		RxChannel[ELEVATOR] = TCNT1 - RxChannelStart[ELEVATOR];
	}
}

ISR(PCINT3_vect)
{	
	if (RX_COLL)	// Rising
	{
		RxChannelStart[THROTTLE] = TCNT1;
	} 
	else 
	{				// Falling
		RxChannel[THROTTLE] = TCNT1 - RxChannelStart[THROTTLE];
		if (Config.RxMode == PWM2) 
		{
			Interrupted = true;						// Signal that interrupt block has finished
			RC_Lock = true;							// RC sync established
		}
	}
}


ISR(PCINT1_vect)
{
	if (RX_AUX)	// Rising
	{
		RxChannelStart[GEAR] = TCNT1;
	} 
	else 
	{				// Falling
		RxChannel[GEAR] = TCNT1 - RxChannelStart[GEAR];
		if (Config.RxMode == PWM3) 
		{
			Interrupted = true;						// Signal that interrupt block has finished
			RC_Lock = true;							// RC sync established
		}
	}
}

//************************************************************
// INT2 is shared between RUDDER in PWM mode or CPPM in CPPM mode
// NB: Raw CPPM channel order (0,1,2,3,4,5,6,7) is 
// mapped via Config.ChannelOrder[]. Actual channel values are always
// in the sequence THROTTLE, AILERON, ELEVATOR, RUDDER, GEAR, FLAP, AUX1, AUX2
//************************************************************

ISR(INT2_vect)
{
	if (Config.RxMode != CPPM_MODE)
	{
		if (RX_YAW)	// Rising
		{
			RxChannelStart[RUDDER] = TCNT1;
		} 
		else 
		{				// Falling
			RxChannel[RUDDER] = TCNT1 - RxChannelStart[RUDDER];
			if (Config.RxMode == PWM1) 
			{
				Interrupted = true;						// Signal that interrupt block has finished
				RC_Lock = true;							// RC sync established
			}
		}
	}
	else
	{
		// Only respond to negative-going interrupts
		if (CPPM) return;
		// Check to see if previous period was a sync pulse
		// If so, reset channel number
		if ((TCNT1 - PPMSyncStart) > SYNCPULSEWIDTH) ch_num = 0;
		PPMSyncStart = TCNT1;
		switch(ch_num)
		{
			case 0:
				RxChannelStart[Config.ChannelOrder[0]] = TCNT1;
				ch_num++;
				break;
			case 1:
				RxChannelStart[Config.ChannelOrder[1]] = TCNT1;
				RxChannel[Config.ChannelOrder[0]] = TCNT1 - RxChannelStart[Config.ChannelOrder[0]];
				ch_num++;
				break;
			case 2:
				RxChannelStart[Config.ChannelOrder[2]] = TCNT1;
				RxChannel[Config.ChannelOrder[1]] = TCNT1 - RxChannelStart[Config.ChannelOrder[1]];
				ch_num++;
				break;
			case 3:
				RxChannelStart[Config.ChannelOrder[3]] = TCNT1;
				RxChannel[Config.ChannelOrder[2]] = TCNT1 - RxChannelStart[Config.ChannelOrder[2]];
				ch_num++;
				break;
			case 4:
				RxChannelStart[Config.ChannelOrder[4]] = TCNT1;
				RxChannel[Config.ChannelOrder[3]] = TCNT1 - RxChannelStart[Config.ChannelOrder[3]];
				ch_num++;
				break;
			case 5:
				RxChannelStart[Config.ChannelOrder[5]] = TCNT1;
				RxChannel[Config.ChannelOrder[4]] = TCNT1 - RxChannelStart[Config.ChannelOrder[4]];
				ch_num++;
				break;
			case 6:
				RxChannelStart[Config.ChannelOrder[6]] = TCNT1;
				RxChannel[Config.ChannelOrder[5]] = TCNT1 - RxChannelStart[Config.ChannelOrder[5]];
				ch_num++;
				break;
			case 7:
				RxChannelStart[Config.ChannelOrder[7]] = TCNT1;
				RxChannel[Config.ChannelOrder[6]] = TCNT1 - RxChannelStart[Config.ChannelOrder[6]];
				ch_num++;
				break;
			case 8:
				RxChannel[Config.ChannelOrder[7]] = TCNT1 - RxChannelStart[Config.ChannelOrder[7]];
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
			RC_Lock = true;						// RC sync established
		}
	}
} // ISR(INT2_vect)


