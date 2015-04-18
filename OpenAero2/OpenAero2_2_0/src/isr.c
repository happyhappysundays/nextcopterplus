//***********************************************************
//* isr.c
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include <stdbool.h>
#include <avr/interrupt.h>
#include "io_cfg.h"
#include "main.h"
#include <stdlib.h>
#include <string.h>

//***********************************************************
//* Prototypes
//***********************************************************

uint16_t TIM16_ReadTCNT1(void);
void init_int(void);
void Disable_RC_Interrupts(void);

//************************************************************
// Interrupt vectors
//************************************************************

volatile bool Interrupted;			// Flag that RX packet completed
volatile bool JitterFlag;			// Flag that interrupt occurred
volatile bool JitterGate;			// Area when we care about JitterFlag

volatile uint16_t RxChannel[MAX_RC_CHANNELS];
volatile uint16_t RxChannelStart[MAX_RC_CHANNELS];	
volatile uint16_t PPMSyncStart;		// Sync pulse timer
volatile uint8_t ch_num;			// Current channel number
volatile uint8_t max_chan;			// Target channel number

volatile uint8_t rcindex;			// Serial data buffer pointer
volatile uint16_t chanmask16;
volatile uint16_t checksum;
volatile uint8_t bytecount;
volatile uint16_t TMR0_counter;		// Number of times Timer 0 has overflowed
volatile uint16_t FrameRate;		// Updated frame rate for serial packets


#define SYNCPULSEWIDTH 6750			// CPPM sync pulse must be more than 2.7ms
#define MINPULSEWIDTH 750			// Minimum CPPM pulse is 300us
//#define PACKET_TIMER 2500			// Serial RC packet start timer. Minimum gap 500/2500000 = 1.0ms

#define PACKET_TIMER 7500			// Serial RC packet start timer. Minimum gap 500/2500000 = 1.0ms

#define MAX_CPPM_CHANNELS 8			// Maximum number of channels via CPPM

//************************************************************
//* Timer 0 overflow handler for extending TMR1
//************************************************************

ISR(TIMER0_OVF_vect)
{
	TMR0_counter++;
}

//************************************************************
//* Standard PWM mode
//* Sequential PWM inputs from a normal RC receiver
//************************************************************

ISR(INT1_vect)
{
	// Log interrupts that occur during PWM generation
	if (JitterGate)	JitterFlag = true;	

	if (RX_ROLL)	// Rising
	{
		RxChannelStart[AILERON] = TCNT1;
	} 
	else 
	{				// Falling
		RxChannel[AILERON] = TCNT1 - RxChannelStart[AILERON];
		if (Config.PWM_Sync == AILERON) 
		{
			Interrupted = true;						// Signal that interrupt block has finished
		}
	}
}

ISR(INT0_vect)
{
	if (JitterGate)	JitterFlag = true;	

	if (RX_PITCH)	// Rising 
	{
		RxChannelStart[ELEVATOR] = TCNT1;
	} 
	else 
	{				// Falling
		RxChannel[ELEVATOR] = TCNT1 - RxChannelStart[ELEVATOR];
		if (Config.PWM_Sync == ELEVATOR) 
		{
			Interrupted = true;						// Signal that interrupt block has finished
		}
	}
}

ISR(PCINT3_vect)
{
	if (JitterGate)	JitterFlag = true;	
		
	if (RX_COLL)	// Rising
	{
		RxChannelStart[THROTTLE] = TCNT1;
	} 
	else 
	{				// Falling
		RxChannel[THROTTLE] = TCNT1 - RxChannelStart[THROTTLE];
		if (Config.PWM_Sync == THROTTLE) 
		{
			Interrupted = true;						// Signal that interrupt block has finished
		}
	}
}


ISR(PCINT1_vect)
{
	if (JitterGate)	JitterFlag = true;

	if (RX_AUX)	// Rising
	{
		RxChannelStart[GEAR] = TCNT1;
	} 
	else 
	{				// Falling
		RxChannel[GEAR] = TCNT1 - RxChannelStart[GEAR];
		if (Config.PWM_Sync == GEAR) 
		{
			Interrupted = true;						// Signal that interrupt block has finished
		}
	}
}

//************************************************************
// INT2 is shared between RUDDER in PWM mode or CPPM in CPPM mode
// NB: Raw CPPM channel order (0,1,2,3,4,5,6,7) is 
// mapped via Config.ChannelOrder[]. Actual channel values are always
// in the sequence THROTTLE, AILERON, ELEVATOR, RUDDER, GEAR, AUX1, AUX2, AUX3
//
// Compacted CPPM RX code thanks to Edgar
//
//************************************************************

ISR(INT2_vect)
{
	if (JitterGate)	JitterFlag = true;	

    // Backup TCNT1
    uint16_t tCount;
	
    tCount = TIM16_ReadTCNT1();

	uint8_t curChannel;
	uint8_t prevChannel;

	if (Config.RxMode != CPPM_MODE)
	{
		if (RX_YAW)	// Rising
		{
			RxChannelStart[RUDDER] = tCount;
		} 
		else 
		{			// Falling
			RxChannel[RUDDER] = tCount - RxChannelStart[RUDDER];
			if (Config.PWM_Sync == RUDDER) 
			{
				Interrupted = true;					// Signal that interrupt block has finished
			}
		}
	}
	
	//************************************************************
	// CPPM code:
	// This code keeps track of the number of channels received
	// within a frame and only signals the data received when the 
	// last data is complete. This makes it compatible with any 
	// number of channels. The minimum sync pulse is 2.7ms and the
	// minimum inter-channel pulse is 300us. This suits "27ms" FrSky
	// CPPM receivers.
	//************************************************************
	else
	{
		// Only respond to negative-going interrupts
		if (CPPM) return;

		// Check to see if previous period was a sync pulse or too small to be valid
		// If so, reset the channel number
		if (((tCount - PPMSyncStart) > SYNCPULSEWIDTH) || ((tCount - PPMSyncStart) < MINPULSEWIDTH))
		{
			ch_num = 0;
		}

		// Update PPMSyncStart with current value
		PPMSyncStart = tCount;

		// Get the channel number of the current channel in the requested channel order
        curChannel = Config.ChannelOrder[ch_num];

		// Set up previous channel number based on the requested channel order
		if (ch_num > 0)
		{
			prevChannel = Config.ChannelOrder[ch_num-1];
		}
		else
		{
			prevChannel = 0;
		}

		// Measure the channel data only for the first MAX_CPPM_CHANNELS (currently 8)
		// Prevent code from over-running RxChannelStart[]
        if (ch_num < MAX_CPPM_CHANNELS)
		{
            RxChannelStart[curChannel] = tCount;
		}

		// When ch_num = 0, the first channel has not yet been measured.
		// That only occurs at the second pulse. Prevent code from over-running RxChannel[]
        if ((ch_num > 0) && (ch_num <= MAX_CPPM_CHANNELS))
        {
		   RxChannel[prevChannel] = tCount - RxChannelStart[prevChannel];
		}

        // Increment to the next channel
		ch_num++;

		// Work out the highest channel number automatically.
		// Update the maximum channel seen so far.
		if (ch_num > max_chan) 
		{
			max_chan = ch_num;					// Update max channel number
		}
		// If the current channel is the highest channel, CPPM is complete
		else if (ch_num == max_chan)
		{
			Interrupted = true;					// Signal that interrupt block has finished
		}
	
		// If the signal is ever lost, reset measured max channel number
		// and force a recalculation
		if (Overdue)
		{
			max_chan = 0;
			Overdue = false;
		}
	}
} // ISR(INT2_vect)

//************************************************************
//* Serial receive interrupt
//************************************************************

ISR(USART0_RX_vect)
{
	char temp = 0;			// RX characters
	uint16_t temp16 = 0;	// Unsigned temp reg for mask etc
	int16_t itemp16 = 0;	// Signed temp reg 
	uint8_t sindex = 0;		// Serial buffer index
	uint8_t j = 0;			// GP counter and mask index

	uint8_t chan_mask = 0;	// Common variables
	uint8_t chan_shift = 0;
	uint8_t data_mask = 0;
	
	uint16_t Save_TCNT1;	// Timer1 (16bit) - run @ 2.5MHz (400ns) - max 26.2ms
	uint16_t CurrentPeriod;

	//************************************************************
	//* Common entry code
	//************************************************************

	// Log interrupts that occur during PWM generation
	if (JitterGate)	JitterFlag = true;

	// Read byte first
	temp = UDR0;

	// Save current time stamp
	Save_TCNT1 = TIM16_ReadTCNT1();
	
	// Work out frame rate properly
	// Note that CurrentPeriod cannot be larger than 26.2ms
	
	//CurrentPeriod = Save_TCNT1 - PPMSyncStart;
	if (Save_TCNT1 < PPMSyncStart)
	{
		CurrentPeriod = (65536 - PPMSyncStart + Save_TCNT1);
	}
	else
	{
		CurrentPeriod = (Save_TCNT1 - PPMSyncStart);
	}

	// Handle start of new packet
	if (CurrentPeriod > PACKET_TIMER) // 1.0ms
	{
		// Reset variables
		rcindex = 0;
		bytecount = 0;
		ch_num = 0;
		checksum = 0;
		chanmask16 = 0;

		// Save frame rate to global
		FrameRate = CurrentPeriod;
	}

	// Timestamp this interrupt
	PPMSyncStart = Save_TCNT1;
	
	// Put received byte in buffer if space available
	if (rcindex < SBUFFER_SIZE)
	{
		sBuffer[rcindex++] = temp;		
		
		//LED1 = ~LED1; //debug	
	}

	//************************************************************
	//* XPS Xtreme format (8-N-1/250Kbps) (1480us for a 37 bytes packet)
	//*
	//* Byte 0: Bit 3 should always be 0 unless there really is a lost packet.
	//* Byte 1: RSS
	//* Byte 2: Mask 
 	//* 		The mask value determines the number of channels in the stream. 
	//*			A 6 channel stream is going to have a mask of 0x003F (00000000 00111111) 
	//*			if outputting all 6 channels.  It is possible to ouput only channels 2 
	//*			and 4 in the stream (00000000 00001010).  In which case the first word 
	//*			of data will be channel 2 and the 2nd word will be channel.
 	//*  
	//*  0x00   0x23   0x000A   0x5DC   0x5DD   0xF0
	//*  ^^^^   ^^^^   ^^^^^^   ^^^^^   ^^^^^   ^^^^
	//*  Flags  dBm     Mask    CH 2    CH 4    ChkSum
	//*
	//************************************************************

	if (Config.RxMode == XTREME)
	{
		// Look at flag byte to see if the data is meant for us
		if (bytecount == 0)
		{
			// Check top 3 bits for channel bank
			// Trash checksum if not clear
			if (temp & 0xE0)
			{
				checksum +=	0x55;
			}
		}

		// Get MSB of mask byte
		if (bytecount == 2)
		{
			chanmask16 = 0;
			chanmask16 = temp << 8;		// High byte of Mask
		}

		// Combine with LSB of mask byte
		// Work out how many channels there are supposed to be
		if (bytecount == 3)
		{
			chanmask16 += (uint16_t)temp;	// Low byte of Mask
			temp16 = chanmask16;			// Need to keep a copy od chanmask16

			// Count bits set (number of active channels)				 
			for (ch_num = 0; temp16; ch_num++)
			{
				temp16 &= temp16 - 1;
			}
		}

		// Add up checksum up until final packet
		if (bytecount < ((ch_num << 1) + 4))
		{
			checksum +=	temp;
		}
	
		// Process data when all packets received
		else
		{
			// Check checksum 
			checksum &= 0xff;

			// Ignore packet if checksum wrong
			if (checksum != temp) // temp holds the transmitted checksum byte
			{
				Interrupted = false;
				ch_num = 0;
				checksum = 0;
			}
			else
			{
				// RC sync established
				Interrupted = true;	

				// Set start of channel data per format
				sindex = 4; // Channel data from byte 5

				// Work out which channel the data is intended for from the mask bit position
				// Channels can be anywhere in the lower 16 channels of the Xtreme format
				for (j = 0; j < 16; j++)
				{
					// If there is a bit set, allocate channel data for it
					if (chanmask16 & (1 << j))
					{
						// Reconstruct word
						temp16 = (sBuffer[sindex] << 8) + sBuffer[sindex + 1];

						// Expand to OpenAero2 units if a valid channel
						if (j < MAX_RC_CHANNELS)
						{
							RxChannel[Config.ChannelOrder[j]] = ((temp16 * 10) >> 2);
						} 		

						// Within the bounds of the buffer
						if (sindex < SBUFFER_SIZE)
						{
							sindex += 2;
						}
					}
				} // For each mask bit	
			} // Checksum
		} // Check end of data
	} // (Config.RxMode == XTREME)

	//************************************************************
	//* Futaba S-Bus format (8-E-2/100Kbps) (2500us for a 25 byte packet)
	//*	S-Bus decoding algorithm borrowed in part from Arduino
	//*
	//* The protocol is 25 Bytes long and is sent every 14ms (analog mode) or 7ms (high speed mode).
	//* One Byte = 1 start bit + 8 data bit + 1 parity bit + 2 stop bit (8E2), baud rate = 100,000 bit/s
	//*
	//* The highest bit is sent first. The logic is inverted :( Stupid Futaba.
	//*
	//* [start byte] [data1] [data2] .... [data22] [flags][end byte]
	//* 
	//* 0 start byte = 11110000b (0xF0)
	//* 1-22 data = [ch1, 11bit][ch2, 11bit] .... [ch16, 11bit] (Values = 0 to 2047)
	//* 	channel 1 uses 8 bits from data1 and 3 bits from data2
	//* 	channel 2 uses last 5 bits from data2 and 6 bits from data3
	//* 	etc.
	//* 
	//* 23 flags = 
	//*		bit7 = ch17 = digital channel (0x80)
	//* 	bit6 = ch18 = digital channel (0x40)
	//* 	bit5 = Frame lost, equivalent red LED on receiver (0x20)
	//* 	bit4 = failsafe activated (0x10)
	//* 	bit3 = n/a
	//* 	bit2 = n/a
	//* 	bit1 = n/a
	//* 	bit0 = n/a
	//* 24 endbyte = 00000000b (SBUS) or (data % 0xCF) (SBUS2)
	//*
	//************************************************************

	if (Config.RxMode == SBUS)
	{
		// Flag that packet has completed
		if ((bytecount == 24) && ((temp == 0x00) || ((temp % 0xCF) == 0x04)))
		{
			// If frame lost, ignore packet
			if ((sBuffer[23] & 0x20) == 0)
			{
				// RC sync established
				Interrupted = true;
				
				//LED1 = ~LED1; //debug

				// Clear channel data
				for (j = 0; j < MAX_RC_CHANNELS; j++)
				{
					RxChannel[j] = 0;
				}

				// Start from second byte
				sindex = 1;

                // Deconstruct S-Bus data
				// 8 channels * 11 bits = 88 bits
                for (j = 0; j < 88; j++)
                {
                    if (sBuffer[sindex] & (1<<chan_mask))
                    {
						// Place the RC data into the correct channel order for the transmitted system
						RxChannel[Config.ChannelOrder[chan_shift]] |= (1<<data_mask);
                    }

                    chan_mask++;
                    data_mask++;

                    // If we have done 8 bits, move to next byte in buffer
					if (chan_mask == 8)
                    {
                        chan_mask =0;
                        sindex++;
                    }

                    // If we have reconstructed all 11 bits of one channel's data (2047)
					// increment the channel number
					if (data_mask == 11)
                    {
                        data_mask =0;
                        chan_shift++;
                    }
                }

				// Convert to  OpenAero2 values (0~2047 -> 2500~4999)
				for (j = 0; j < MAX_RC_CHANNELS; j++)
				{
					// Subtract weird-ass Futaba offset
					itemp16= RxChannel[j] - 1024;	
					
					// Expand into OpenAero2 units							
					itemp16 = itemp16 + (itemp16 >> 2) + (itemp16 >> 3) + (itemp16 >> 4) + (itemp16 >> 5); 	// Quick multiply by 1.469 :)
					//itemp16 = itemp16 + (itemp16 >> 1); // Quicker mulitply by 1.5

					// Add back in OpenAero2 offset
					RxChannel[j] = itemp16 + 3750;				
				} 	
			} // Frame lost check
			
		} // Packet ended flag
	
	} // (Config.RxMode == SBUS)

	//************************************************************
	//* Spektrum Satellite format (8-N-1/115Kbps) MSB sent first (1391us for a 16 byte packet)
	//* DX7/DX6i: One data-frame at 115200 baud every 22ms.
	//* DX7se:    One data-frame at 115200 baud every 11ms.
	//*
	//*    byte1: is a frame loss counter
	//*    byte2: [0 0 0 R 0 0 N1 N0]
	//*    byte3:  and byte4:  channel data (FLT-Mode)	= FLAP 6
	//*    byte5:  and byte6:  channel data (Roll)		= AILE A
	//*    byte7:  and byte8:  channel data (Pitch)		= ELEV E
	//*    byte9:  and byte10: channel data (Yaw)		= RUDD R
	//*    byte11: and byte12: channel data (Gear Switch) GEAR 5
	//*    byte13: and byte14: channel data (Throttle)	= THRO T
	//*    byte15: and byte16: channel data (AUX2)		= AUX2 8
	//* 
	//* DS9 (9 Channel): One data-frame at 115200 baud every 11ms,
	//* alternating frame 1/2 for CH1-7 / CH8-9
	//*
	//*   1st Frame:
	//*    byte1: is a frame loss counter
	//*    byte2: [0 0 0 R 0 0 N1 N0]
	//*    byte3:  and byte4:  channel data
	//*    byte5:  and byte6:  channel data
	//*    byte7:  and byte8:  channel data
	//*    byte9:  and byte10: channel data
	//*    byte11: and byte12: channel data
	//*    byte13: and byte14: channel data
	//*    byte15: and byte16: channel data
	//*   2nd Frame:
	//*    byte1: is a frame loss counter
	//*    byte2: [0 0 0 R 0 0 N1 N0]
	//*    byte3:  and byte4:  channel data
	//*    byte5:  and byte6:  channel data
	//*    byte7:  and byte8:  0xffff
	//*    byte9:  and byte10: 0xffff
	//*    byte11: and byte12: 0xffff
	//*    byte13: and byte14: 0xffff
	//*    byte15: and byte16: 0xffff
	//* 
	//* Each channel data (16 bit= 2byte, first msb, second lsb) is arranged as:
	//* 
	//* Bits: F 00 C3 C2 C1 C0  D9 D8 D7 D6 D5 D4 D3 D2 D1 D0 for 10-bit data (0 to 1023) or
	//* Bits: F C3 C2 C1 C0 D10 D9 D8 D7 D6 D5 D4 D3 D2 D1 D0 for 11-bit data (0 to 2047) 
	//* 
	//* R: 0 for 10 bit resolution 1 for 11 bit resolution channel data
	//* N1 to N0 is the number of frames required to receive all channel data. 
	//* F: 1 = indicates beginning of 2nd frame for CH8-9 (DS9 only)
	//* C3 to C0 is the channel number. 0 to 9 (4 bit, as assigned in the transmitter)
	//* D9 to D0 is the channel data 
	//*		(10 bit) 0xaa..0x200..0x356 for 100% transmitter-travel
	//*		(11 bit) 0x154..0x400..0x6ac for 100% transmitter-travel
	//*
	//* The data values can range from 0 to 1023/2047 to define a servo pulse width 
	//* from approximately 0.75ms to 2.25ms (1.50ms difference). 1.465us per digit.
	//* A value of 171/342 is 1.0 ms
	//* A value of 512/1024 is 1.5 ms
	//* A value of 853/1706 is 2.0 ms
	//* 0 = 750us, 1023/2047 = 2250us
	//*
	//************************************************************

	// Handle Spektrum format
	if (Config.RxMode == SPEKTRUM)
	{
		// Process data when all packets received
		if (bytecount >= 15)
		{
			// Ahem... ah... just stick the last byte into the buffer manually...(hides)
			sBuffer[15] = temp;

			// Set start of channel data per format
			sindex = 2; // Channel data from byte 3

			// Work out if this is 10 or 11 bit data
			if (sBuffer[1] & 0x10) 	// 0 for 10 bit resolution 1 for 11 bit resolution
			{
				chan_mask = 0x78;	// 11 bit (2048)
				data_mask = 0x07;
				chan_shift = 0x03;
			}
			else
			{
				chan_mask = 0x3C;	// 10 bit (1024)
				data_mask = 0x03;
				chan_shift = 0x02;
			}

			// Work out which channel the data is intended for from the channel number data
			// Channels can also be in the second packet. Spektrum has 7 channels per packet.
			for (j = 0; j < 7; j++)
			{
				// Extract channel number
				ch_num = (sBuffer[sindex] & chan_mask) >> chan_shift;

				// Reconstruct channel data
				temp16 = ((sBuffer[sindex] & data_mask) << 8) + sBuffer[sindex + 1];

				// Expand to OpenAero2 units if a valid channel
				// Blank channels have the channel number of 16
				if (ch_num < MAX_RC_CHANNELS)
				{
					// Subtract Spektrum center offset
					if (chan_shift == 0x03) // 11-bit
					{
						itemp16 = temp16 - 1024;
					}
					else
					{
						itemp16 = temp16 - 512;	
					}					

					// Quick multiply by 2.93
					itemp16 = (itemp16 << 1) + (itemp16 >> 1) + (itemp16 >> 2) + (itemp16 >> 3) + (itemp16 >> 4); 

					if (chan_shift == 0x03) // 11-bit
					{
						// Divide in case of 11-bit value
						itemp16 = itemp16 >> 1;								
					}

					// Add back in OpenAero2 offset
					itemp16 += 3750;										

					RxChannel[Config.ChannelOrder[ch_num]] = itemp16;
				}

				sindex += 2;

			} // For each pair of bytes
			
			// RC sync established
			Interrupted = true;

		} // Check end of data
		
	} // (Config.RxMode == SPEKTRUM)

	//************************************************************
	//* Common exit code
	//************************************************************

	// Increment byte count
	bytecount++;
}

//***********************************************************
//* TCNT1 atomic read subroutine
//* from Atmel datasheet
//* TCNT1 is the only 16-bit timer
//***********************************************************

uint16_t TIM16_ReadTCNT1(void)
{
	uint8_t sreg;
	uint16_t i;
	
	/* Save global interrupt flag */
	sreg = SREG;
	
	/* Disable interrupts */
	cli();
	
	/* Read TCNTn into i */
	i = TCNT1;
	
	/* Restore global interrupt flag */
	SREG = sreg;
	return i;
}

//***********************************************************
// Disable RC interrupts as required
//***********************************************************

void Disable_RC_Interrupts(void)
{
	cli();	// Disable interrupts

	// Disable PWM input interrupts
	PCMSK1 = 0;							// Disable AUX
	PCMSK3 = 0;							// Disable THR
	EIMSK  = 0;							// Disable INT0, 1 and 2

	// Disable receiver (flushes buffer)
	UCSR0B &= ~(1 << RXEN0);	

	// Disable serial interrupt	
	UCSR0B &= ~(1 << RXCIE0);
	
	// Clear interrupt flags
	PCIFR	= 0x0F;						// Clear PCIF0~PCIF3 interrupt flags
	EIFR	= 0x00; 					// Clear INT0~INT2 interrupt flags (Elevator, Aileron, Rudder/CPPM)
	
	sei(); // Re-enable interrupts
}

//***********************************************************
// Reconfigure RC interrupts
//***********************************************************

void init_int(void)
{
	cli();	// Disable interrupts
	
	switch (Config.RxMode)
	{
		case CPPM_MODE:
			PCMSK1 = 0;							// Disable AUX
			PCMSK3 = 0;							// Disable THR
			EIMSK = 0x04;						// Enable INT2 (Rudder/CPPM input)
			UCSR0B &= ~(1 << RXCIE0);			// Disable serial interrupt
			UCSR0B &= ~(1 << RXEN0);			// Disable receiver and flush buffer
			break;

		case PWM:
			PCMSK1 |= (1 << PCINT8);			// PB0 (Aux pin change mask)
			PCMSK3 |= (1 << PCINT24);			// PD0 (Throttle pin change mask)
			EIMSK  = 0x07;						// Enable INT0, 1 and 2 
			UCSR0B &= ~(1 << RXCIE0);			// Disable serial interrupt
			UCSR0B &= ~(1 << RXEN0);			// Disable receiver and flush buffer
			break;

		case XTREME:
		case SBUS:
		case SPEKTRUM:
			// Disable PWM input interrupts
			PCMSK1 = 0;							// Disable AUX
			PCMSK3 = 0;							// Disable THR
			EIMSK  = 0;							// Disable INT0, 1 and 2 
			
			// Enable serial receiver and interrupts
			UCSR0B |= (1 << RXCIE0);			// Enable serial interrupt
			UCSR0B |= (1 << RXEN0);				// Enable receiver
			break;

		default:
			break;	
	}	

	// Clear interrupt flags
	PCIFR	= 0x0F;								// Clear PCIF0~PCIF3 interrupt flags
	EIFR	= 0x00; 							// Clear INT0~INT2 interrupt flags (Elevator, Aileron, Rudder/CPPM)

	sei(); // Re-enable interrupts

} // init_int