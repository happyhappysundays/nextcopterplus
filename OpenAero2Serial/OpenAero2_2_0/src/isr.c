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

//************************************************************
// Interrupt vectors
//************************************************************

volatile bool Interrupted;			// Flag that RX packet completed

volatile uint16_t RxChannel[MAX_RC_CHANNELS];
volatile uint16_t ExtChannel[MAX_EXT_CHANNELS];
volatile uint16_t RxChannelStart[MAX_RC_CHANNELS];	
volatile uint16_t PPMSyncStart;		// Sync pulse timer
volatile uint8_t ch_num;			// Current channel number
volatile uint8_t max_chan;			// Target channel number
volatile uint8_t Spektrum_frame_in;	// Sticky frame counter

// Xtreme globals for re-transmission
volatile uint8_t Xtreme_Flags;
volatile uint8_t Xtreme_RSS;
volatile uint16_t Xtreme_Chanmask;
volatile uint16_t Spektrum_Chanmask_0; // Channel mask for Frame 0
volatile uint16_t Spektrum_Chanmask_1; // Channel mask for Frame 1
volatile uint16_t chanmask16;

// S.Bus globals for re-transmission
volatile uint8_t SBUS_Flags;

// Spektrum globals for re-transmission
volatile uint8_t Spektrum_frameloss;

volatile uint8_t rcindex;			// Serial data buffer pointer
volatile uint16_t checksum;
volatile uint8_t bytecount;
volatile uint16_t TMR0_counter;		// Number of times Timer 0 has overflowed
volatile uint32_t FramePeriod;		// Updated frame period for serial packets
volatile uint16_t FrameStart;		// Timestamp of the frame start
volatile uint8_t int_count;			// Number of times that the serial interrupt has been triggered

#define SYNCPULSEWIDTH 6750			// CPPM sync pulse must be more than 2.7ms
#define MINPULSEWIDTH 750			// Minimum CPPM pulse is 300us
#define PACKET_TIMER 2500			// Serial RC packet start timer. Minimum gap 2500/2500000 = 1.0ms
#define MAX_CPPM_CHANNELS 8			// Maximum number of channels via CPPM
#define MAX_SPEKTRUM_CHANNELS 14	// Maximum number of channels via Spektrum 	
#define MININTCOUNT 10				// Minimum number of serial interrupts to validate the frame period measurement
									// NB: Needs to be more than MAX_CPPM_CHANNELS + 1
#define FRAMEDEFAULT 75000			// 30ms in T1 cycles is 75000

//************************************************************
//* Timer 0 overflow handler for extending TMR1
//************************************************************

ISR(TIMER0_OVF_vect)
{
	TMR0_counter++;
}

//************************************************************
// PCINT3 is for CPPM input in CPPM mode
// NB: Raw CPPM channel order (0,1,2,3,4,5,6,7) is 
// mapped via Config.ChannelOrder[]. Actual channel values are always
// in the sequence THROTTLE, AILERON, ELEVATOR, RUDDER, GEAR, AUX1, AUX2, AUX3
//
// Compacted CPPM RX code thanks to Edgar
//
//************************************************************

ISR(PCINT3_vect)
{
	uint8_t curChannel;
	uint8_t prevChannel;
	uint16_t Save_TCNT1;	// Timer1 (16bit) - run @ 2.5MHz (400ns) - max 26.2ms
	uint16_t CurrentPeriod;
	
	//************************************************************
	// CPPM code:
	// This code keeps track of the number of channels received
	// within a frame and only signals the data received when the 
	// last data is complete. This makes it compatible with any 
	// number of channels. The minimum sync pulse is 2.7ms and the
	// minimum inter-channel pulse is 300us. This suits "27ms" FrSky
	// CPPM receivers.
	//************************************************************

	// Only respond to negative-going interrupts
	if (CPPM) return;

	// Save current time stamp
	Save_TCNT1 = TIM16_ReadTCNT1();
		
	// Work out frame rate properly
	// Note that CurrentPeriod cannot be larger than 26.2ms
	if (Save_TCNT1 < PPMSyncStart)
	{
		CurrentPeriod = (65536 - PPMSyncStart + Save_TCNT1);
	}
	else
	{
		CurrentPeriod = (Save_TCNT1 - PPMSyncStart);
	}

	// Handle start of new packet if gap between serial data has been more than 1.0ms
	if (CurrentPeriod > SYNCPULSEWIDTH) // 2.7ms
	{
		if (int_count < MININTCOUNT)
		{
			// increment the serial interrupt counter
			int_count++;
		}
		// Flag that a valid number of serial interrupts has occurred.
		else
		{
			// Set RxStarted flag
			Flight_flags |= (1 << RxStarted);
		}

		// FramePeriod only needs calculation before Rx has officially started
		if (!(Flight_flags & (1 << RxStarted)))
		{
			// Measure period from last FrameStart
			if (Save_TCNT1 < FrameStart)
			{
				FramePeriod = (65536 - FrameStart + Save_TCNT1);
			}
			else
			{
				FramePeriod = (Save_TCNT1 - FrameStart);
			}
		}
			
		// Reset channel counter
		ch_num = 0;
			
		// Save the timestamp for the start of data so that
		// we can do a proper full frame rate measurement
		FrameStart = Save_TCNT1;
	}
	
	// Check for pulses smaller than 300us and reset if so
	else if (CurrentPeriod < MINPULSEWIDTH)
	{
		// Reset channel counter
		ch_num = 0;
	}
	
	// Timestamp this interrupt
	PPMSyncStart = Save_TCNT1;

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
        RxChannelStart[curChannel] = Save_TCNT1;
	}

	// When ch_num = 0, the first channel has not yet been measured.
	// That only occurs at the second pulse. Prevent code from over-running RxChannel[]
    if ((ch_num > 0) && (ch_num <= MAX_CPPM_CHANNELS))
    {
		RxChannel[prevChannel] = Save_TCNT1 - RxChannelStart[prevChannel];
	}

    // Increment to the next channel
	ch_num++;

	// Work out the highest channel number automatically.
	// Update the maximum channel seen so far, but only while not officially started
	if ((ch_num > max_chan) && !(Flight_flags & (1 << RxStarted)))
	{
		max_chan = ch_num;					// Update max channel number
	}
	// If the current channel is the highest channel, CPPM is complete
	else if (ch_num == max_chan)
	{
		// RC sync established
		Interrupted = true;
					
		// Reset frame timers
		FrameDrop_Output_Rate = 0;
		Failsafe_Output_Rate = 0;
		RC_Timeout = 0;					// Reset 500ms failsafe timeout
					
		// Flag end-of-failsafe if we were in failsafe.
		// Also, clear failsafe.
		if (Flight_flags & (1 << FailsafeFlag))
		{
			Alarm_flags |= (1 << FAILSAFE_ENDED);
			Flight_flags &= ~(1 << FailsafeFlag);
		}
					
		Flight_flags &= ~(1 << FrameDrop);	// And the frame drop flag
	}
} // ISR(INT2_vect)

//************************************************************
//* Serial receive interrupt
//************************************************************

ISR(USART0_RX_vect)
{
	uint8_t temp = 0;		// RX characters
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

	// Read error flags first
	temp =  UCSR0A;

	// Check Framing error, Parity error bits
	if (temp & ((1<<FE0)|(1<<UPE0)))
	{
		// Read byte to remove from buffer
		temp = UDR0;
	}

	// Check all for Data overrun
	else if (temp & (1<<DOR0))
	{
		// Read byte to remove from buffer
		temp = UDR0;
		// Read byte to remove from buffer
		temp = UDR0;				
	}

	// Valid data
	else
	{
		// Read byte first
		temp = UDR0;
			
		// Save current time stamp
		Save_TCNT1 = TIM16_ReadTCNT1();
	
		// Work out frame rate properly
		// Note that CurrentPeriod cannot be larger than 26.2ms
		if (Save_TCNT1 < PPMSyncStart)
		{
			CurrentPeriod = (65536 - PPMSyncStart + Save_TCNT1);
		}
		else
		{
			CurrentPeriod = (Save_TCNT1 - PPMSyncStart);
		}

		// Handle start of new packet if gap between serial data has been more than 1.0ms
		if (CurrentPeriod > PACKET_TIMER) // 1.0ms
		{
			if (int_count < MININTCOUNT)
			{
				// increment the serial interrupt counter
				int_count++;		
			}
			// Flag that a valid number of serial interrupts has occurred.
			else
			{
				// Set RxStarted flag
				Flight_flags |= (1 << RxStarted);
			}

			// FramePeriod only needs calculation before Rx has officially started
			if (!(Flight_flags & (1 << RxStarted)))
			{
				// Measure period from last FrameStart
				if (Save_TCNT1 < FrameStart)
				{
					FramePeriod = (65536 - FrameStart + Save_TCNT1);
				}
				else
				{
					FramePeriod = (Save_TCNT1 - FrameStart);
				}
			}
			
			// Reset variables
			rcindex = 0;
			bytecount = 0;
			ch_num = 0;
			checksum = 0;
			chanmask16 = 0;
		
			// Save the timestamp for the start of data so that
			// we can do a proper full frame rate measurement
			FrameStart = Save_TCNT1;
		}

		// Timestamp this interrupt
		PPMSyncStart = Save_TCNT1;
	
		// Put received byte in buffer if space available
		if (rcindex < SBUFFER_SIZE)
		{
			sBuffer[rcindex++] = temp;		
		}

		//************************************************************
		//* XPS Xtreme format (8-N-1/250Kbps) (1480us for a 37 bytes packet)
		//*
		//* Byte 0: Bit 3 should always be 0 unless there really is a lost packet.
		//* Byte 1: RSS
		//* Byte 2: Mask 
 		//* 		The mask value determines the number of channels in the stream. 
		//*			A 6 channel stream is going to have a mask of 0x003F (00000000 00111111) 
		//*			if outputting all 6 channels.  It is possible to output only channels 2 
		//*			and 4 in the stream (00000000 00001010).  In which case the first word 
		//*			of data will be channel 2 and the 2nd word will be channel.
 		//*  
		//*  0x00   0x23   0x000A   0x5DC   0x5DD   0xF0
		//*  ^^^^   ^^^^   ^^^^^^   ^^^^^   ^^^^^   ^^^^
		//*  Flags  dBm     Mask    CH 2    CH 4    ChkSum
		//*
		//************************************************************

		if (Config.RxModeIn == XTREME)
		{
			// Look at flag byte to see if the data is ok
			if (bytecount == 0)
			{
				// Save a copy of the flags for retransmission
				Xtreme_Flags = temp;
			
				// Check top 3 bits for channel bank
				// Trash checksum if not clear
				if (temp & 0xE0)
				{
					checksum +=	0x55;
				}
			}
	
			// Save RSS byte
			if (bytecount == 1)
			{
				Xtreme_RSS = temp;
			}
		
			// Get MSB of mask byte
			if (bytecount == 2)
			{
				chanmask16 = temp;
			}

			// Combine with LSB of mask byte
			// Work out how many channels there are supposed to be
			if (bytecount == 3)
			{
				chanmask16 = chanmask16 << 8;	// Form high byte of mask
				chanmask16 |= (uint8_t)temp;		// Low byte of Mask
			
				temp16 = chanmask16;			// Need to keep a copy of chanmask16 to work of number of channels
				Xtreme_Chanmask = chanmask16;	// Also need to save as a global for retransmission (uint16)

				// Count bits set (number of active channels)	
				// ch_num will contain the number of channels found			 
				for (ch_num = 0; temp16; ch_num++)
				{
					temp16 &= (temp16 - 1);
				}
			}

			// Add up checksum up until final packet
			if (bytecount < ((ch_num << 1) + 4))
			{
				checksum +=	temp;
			}
	
			// Process data when whole packet received
			else
			{
				// Check checksum 
				checksum &= 0xff;

				// Ignore packet if checksum wrong
				if (checksum != temp) // temp holds the transmitted checksum byte
				{
					ch_num = 0;
					checksum = 0;
				
					LED1 = 1;
				}
			
				// Checksum OK
				else
				{
					LED1 = 0;
				
					// RC sync established
					Interrupted = true;	
			
					// Reset frame timers
					FrameDrop_Output_Rate = 0;
					Failsafe_Output_Rate = 0;
					RC_Timeout = 0;					// Reset 500ms failsafe timeout
					
					// Flag end-of-failsafe if we were in failsafe.
					// Also, clear failsafe.
					if (Flight_flags & (1 << FailsafeFlag))
					{
						Alarm_flags |= (1 << FAILSAFE_ENDED);
						Flight_flags &= ~(1 << FailsafeFlag);
					}
					
					Flight_flags &= ~(1 << FrameDrop);	// And the frame drop flag
				
					// Clear channel data
					for (j = 0; j < MAX_RC_CHANNELS; j++)
					{
						RxChannel[j] = 0;
					
						if (Config.RxModeOut == SPEKTRUM)
						{
							ExtChannel[j] = 0xFFFF;
						}
						else if (Config.RxModeOut == SBUS)
						{
							ExtChannel[j] = 0x400;
						}
						else
						{
							ExtChannel[j] = 0;
						}
					}

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
								// Convert to system values
								itemp16 = temp16;
								itemp16 -= 1500;							// Remove Xtreme offset (+/-500)		
							
								// Multiply by Xtreme-to-System factor (x2.0)
								itemp16 = (itemp16 << 1);
							
								// Add offset
								itemp16 += 3750;							// Add System offset
								temp16 = itemp16;		
																
								RxChannel[Config.ChannelOrder[j]] = temp16;
							}
						
							// Save any extra channels in extra buffer in the format suitable for retransmission
							else if (j < (MAX_EXT_CHANNELS + MAX_RC_CHANNELS))
							{
								// Extreme to Satellite
								if (Config.RxModeOut == SPEKTRUM)
								{
									// 11 bit data (1000~2000 -> 0~2047)
									itemp16 = temp16;
									itemp16 -= 1500;							// Remove Xtreme offset (+/-500)
						
									//  867.5/50 = 1.735 (1.7344)
									itemp16 = itemp16 + (itemp16 >> 2) + (itemp16 >> 3) + (itemp16 >> 4) + (itemp16 >> 5) + (itemp16 >> 6);
						
									// Add offset
									itemp16 += 1024;							// Add Satellite offset (+/-1024)
									temp16 = itemp16;
						
									temp16 &= 0x7FF;							// Mask off data bits
									temp16 |= (j << 11);						// Shift channel number up to the correct spot

									// Put back into buffer
									ExtChannel[j - MAX_RC_CHANNELS] = temp16;
								}
							
								// Extreme to S.Bus
								else if (Config.RxModeOut == SBUS)
								{
									// 11 bit data (1000~2000 -> 0~2047) 
									itemp16 = temp16;
									itemp16 -= 1500;							// Remove Xtreme offset (+/-500)
								
									// 800/500 = 1.6 (1.6016)
									itemp16 = itemp16 + (itemp16 >> 1) + (itemp16 >> 4) + (itemp16 >> 5) + (itemp16 >> 7);
								
									// Add offset
									itemp16 += 1024;							// Add S.Bus offset (+/-1024)
									temp16 = itemp16;
								
									ExtChannel[j - MAX_RC_CHANNELS] = temp16;
								}
							
								// Xtreme to Xtreme
								else
								{
									ExtChannel[j - MAX_RC_CHANNELS] = temp16;							
								}
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
		} // (Config.RxModeIn == XTREME)

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
		//* 24 endbyte = 00000000b (SBUS) or (variable) (SBUS2)
		//*
		//* Data size:	0 to 2047, centered on 1024 (1.520ms)
		//* 
		//* 0 		= 880us
		//* 224		= 1020us
		//* 1024 	= 1520us +/- 800 for 1-2ms
		//* 1824	= 2020us
		//* 2047 	= 2160us
		//*
		//************************************************************

		else if (Config.RxModeIn == SBUS)
		{
			// Flag that packet has completed
			// End bytes can be 00, 04, 14, 24, 34 and possibly 08 for FASSTest 12-channel
			//if ((bytecount == 24) && ((temp == 0x00) || (temp == 0x04) || (temp == 0x14) || (temp == 0x24) || (temp == 0x34) || (temp == 0x08)))
			if (bytecount == 24)
			{
				// Save flags
				SBUS_Flags = sBuffer[23];
			
				// If frame lost, ignore packet
				if ((sBuffer[23] & 0x20) == 0)
				{
					LED1 = 0;
				
					// RC sync established
					Interrupted = true;
			
					// Reset frame timers
					FrameDrop_Output_Rate = 0;
					Failsafe_Output_Rate = 0;
					RC_Timeout = 0;					// Reset 500ms failsafe timeout
					
					// Flag end-of-failsafe if we were in failsafe.
					// Also, clear failsafe.
					if (Flight_flags & (1 << FailsafeFlag))
					{
						Alarm_flags |= (1 << FAILSAFE_ENDED);
						Flight_flags &= ~(1 << FailsafeFlag);
					}
					
					Flight_flags &= ~(1 << FrameDrop);	// And the frame drop flag
					
					// Clear channel data
					for (j = 0; j < MAX_RC_CHANNELS; j++)
					{
						RxChannel[j] = 0;
						ExtChannel[j] = 0;
					}

					// Start from second byte
					sindex = 1;

					// Deconstruct S-Bus data
					// 16 channels * 11 bits = 176 bits
					for (j = 0; j < 176; j++)
					{
						if (sBuffer[sindex] & (1 << chan_mask))
						{
							if (chan_shift < MAX_RC_CHANNELS)
							{
								// Place the RC data into the correct channel order for the transmitted system
								RxChannel[Config.ChannelOrder[chan_shift]] |= (1 << data_mask); // RxChannel is 16 bits							
							}
							// Save any extra channels in extra buffer
							else if (chan_shift < (MAX_EXT_CHANNELS + MAX_RC_CHANNELS))
							{
								ExtChannel[chan_shift - MAX_RC_CHANNELS] |= (1 << data_mask);							
							}
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

					// S.Bus to System
					for (j = 0; j < MAX_RC_CHANNELS; j++)
					{
						// Subtract Futaba offset
						itemp16 = RxChannel[j] - 1024;	
					
						// Expand into OpenAero2 units x1.25 (1.25)	(1000/800)
						itemp16 = itemp16 + (itemp16 >> 2);

						// Add back in OpenAero2 offset
						RxChannel[j] = itemp16 + 3750;				
					}
				
					// Convert Extra channel info to the format suitable for retransmission
					for (j = 0; j < MAX_RC_CHANNELS; j++)
					{
						// S.Bus to Satellite
						if (Config.RxModeOut == SPEKTRUM)
						{
							// 11bit (Values = 0 to 2047), same as Satellite
							itemp16 = ExtChannel[j];
							itemp16 -= 1024;							// Subtract S.Bus offset (+/-1024)
						
							//  1.0844 (1.086) (867.5/800) 
							itemp16 = itemp16 + (itemp16 >> 4) + (itemp16 >> 6) + (itemp16 >> 7);
						
							// Add offset
							itemp16 += 1024;							// Add Satellite offset (+/-1024)
							temp16 = itemp16;
						
							temp16 &= 0x7FF;							// Mask off data bits
							temp16 |= ((j + MAX_RC_CHANNELS) << 11);	// Shift channel number up to the correct spot

							// Put back into buffer
							ExtChannel[j] = temp16;	
						}
					
						// S.Bus to Xtreme	
						else if (Config.RxModeOut == XTREME)
						{
							// 11-bit (0 to 2047) to (1000 to 2000) 
							itemp16 = ExtChannel[j];
							itemp16 -= 1024;							// Subtract S.bus offset (+/-1024)
						
							//  0.625 (0.625) (500/800) 
							itemp16 = (itemp16 >> 1) + (itemp16 >> 3);
						
							itemp16 += 1500;							// Add in Xtreme offset (1000~2000)
							temp16 = itemp16;
						
							// Put back into buffer
							ExtChannel[j] = temp16;						
						}
					}
				 	
				} // Frame lost check
				else
				{
					LED1 = 1;
				}
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
		//* 
		//* 0 		= 920us
		//* 157		= 1010us
		//* 1024 	= 1510us +/- 867.5 for 1-2ms
		//* 1892	= 2010us
		//* 2047 	= 2100us
		//*
		//************************************************************

		// Handle Spektrum format
		else if (Config.RxModeIn == SPEKTRUM)
		{
			// Process data when all packets received
			if (bytecount == 15)
			{
				// Just stick the last byte into the buffer manually.
				sBuffer[15] = temp;

				// Set start of channel data
				sindex = 2; // Channel data from byte 3

				// Work out which frame this is from byte 3
				if (sBuffer[2] & 0x80) 	// 0 for frame 0, 1 for frame 1
				{
					Spektrum_frame_in = 1;		// Mark as Frame 1 if bit set
					Spektrum_Chanmask_1 = 0;	// Clear frame mask 1
				}
				else
				{
					Spektrum_frame_in = 0;
					Spektrum_Chanmask_0 = 0;	// Clear frame mask 0
				}			

				// Compare old with new fame loss data
				if (sBuffer[0] != Spektrum_frameloss)
				{
					LED1 = 1;
				}
				else
				{
					LED1 = 0;
				}
			
				// Save current frame loss data
				Spektrum_frameloss = sBuffer[0];
						
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

				// Work out which channel the data is intended for from the channel number data.
				// Channels can also be in the second packet. Spektrum has 7 channels per packet.
				for (j = 0; j < 7; j++)
				{
					// Extract channel number
					ch_num = (sBuffer[sindex] & chan_mask) >> chan_shift;

					// Reconstruct channel data
					temp16 = ((sBuffer[sindex] & data_mask) << 8) + sBuffer[sindex + 1];

					// Expand to OpenAero2 units if a valid channel
					// Blank channels have the channel number of 16 (0xFFFF)
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

						// Spektrum to System
						// 1.1527 () (1000/867.5) x2 = 2.30547 (2.3047)
						itemp16 = (itemp16 << 1) + (itemp16 >> 2) + (itemp16 >> 5) + (itemp16 >> 6) + (itemp16 >> 7); 

						if (chan_shift == 0x03) // 11-bit
						{
							// Divide in case of 11-bit value (therefore x1.466)
							itemp16 = itemp16 >> 1;								
						}

						// Add back in OpenAero2 offset
						itemp16 += 3750;										

						RxChannel[Config.ChannelOrder[ch_num]] = itemp16;
					}
		
					// Save any extra channels in extra buffer as-is, complete with channel number
					else if (ch_num < MAX_SPEKTRUM_CHANNELS)
					{
						// Satellite to Xtreme
						if (Config.RxModeOut == XTREME)
						{
							// Strip off channel number (0~2047)
							temp16 = ((sBuffer[sindex] & data_mask) << 8) + sBuffer[sindex + 1];
						
							itemp16 = temp16;
							itemp16 -= 1024;							// Subtract Spektrum offset (+/-1024)
						
							// x0.5764 (.5762)	(500/867.5)
							itemp16 = (itemp16 >> 1) + (itemp16 >> 4) + (itemp16 >> 7) + (itemp16 >> 8) + (itemp16 >> 9); 
						
							// Add offset
							itemp16 += 1500;							// Add Xtreme offset (1000~2000)
							temp16 = itemp16;
						
							// Put back into buffer
							ExtChannel[ch_num - MAX_RC_CHANNELS] = temp16;
						}
					
						// Satellite to S.Bus
						else if (Config.RxModeOut == SBUS)
						{
							// Strip off channel number
							temp16 = ((sBuffer[sindex] & data_mask) << 8) + sBuffer[sindex + 1];

							itemp16 = temp16;
							itemp16 -= 1024;							// Subtract Spektrum offset (+/-1024)
						
							// 0.9222 (0.9219) (800/867.5) 
							itemp16 = (itemp16 >> 1) + (itemp16 >> 2) + (itemp16 >> 3) + (itemp16 >> 4) + (itemp16 >> 6);
						
							// Add offset
							itemp16 += 1024;							// Add S.Bus offset
							temp16 = itemp16;

							// Put back into buffer
							ExtChannel[ch_num - MAX_RC_CHANNELS] = temp16;						
						}
					
						// Spektrum to Spektrum
						else
						{
							ExtChannel[ch_num - MAX_RC_CHANNELS] = (sBuffer[sindex] << 8) + sBuffer[sindex + 1];						
						}
					}

					// Use the channel masks to count the number of channels in the Spektrum packets for each frame
					// Ignore channels above 14. This inlcudes the dummy filler bytes 0xff, 0xff which read as ch16
					if (ch_num < MAX_SPEKTRUM_CHANNELS)
					{
						if (Spektrum_frame_in == 0)
						{
							// Mark which channels are in frame 0
							Spektrum_Chanmask_0 |= (1 << ch_num);					
						}
						else
						{
							// Mark which channels are in frame 1
							Spektrum_Chanmask_1 |= (1 << ch_num);
						}

						// OR the two masks to get the sum of all channels in the pair of frames.
						Xtreme_Chanmask = (Spektrum_Chanmask_0 | Spektrum_Chanmask_1);
					}
					
					sindex += 2;

				} // For each pair of bytes
			
				// RC sync established
				Interrupted = true;
					
				// Reset frame timers
				FrameDrop_Output_Rate = 0;
				Failsafe_Output_Rate = 0;
				RC_Timeout = 0;					// Reset 500ms failsafe timeout
				
				// Flag end-of-failsafe if we were in failsafe.
				// Also, clear failsafe.
				if (Flight_flags & (1 << FailsafeFlag))
				{
					Alarm_flags |= (1 << FAILSAFE_ENDED);
					Flight_flags &= ~(1 << FailsafeFlag);
				}
				
				Flight_flags &= ~(1 << FrameDrop);	// And the frame drop flag
				
			} // Check end of data
		
		} // (Config.RxMode == SPEKTRUM)

		//************************************************************
		//* Common exit code
		//************************************************************

		// Increment byte count
		bytecount++;
	
	} // Valid data
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
// Reconfigure RC interrupts
//***********************************************************

void init_int(void)
{
	cli();	// Disable interrupts

	// Receiver side	
	switch (Config.RxModeIn)
	{
		case CPPM_MODE:
			PCMSK0 = 0;							// Disable PCINT0-7
			PCMSK1 = 0;							// Disable PCINT8-15 (AUX, Rudder)
			PCMSK2 = 0;							// Disable PCINT16-23
			PCMSK3 = 1;							// Enable  PCINT24 (THR/CPPM input)
			EIMSK  = 0;							// Disable INT0, 1 and 2 
			UCSR0B &= ~(1 << RXCIE0);			// Disable serial interrupt
			UCSR0B &= ~(1 << RXEN0);			// Disable receiver and flush buffer
			break;

		case XTREME:
		case SBUS:
		case SPEKTRUM:
			// Disable input interrupts
			PCMSK0 = 0;							// Disable PCINT0-7
			PCMSK1 = 0;							// Disable PCINT8-15 (AUX, Rudder)
			PCMSK2 = 0;							// Disable PCINT16-23
			PCMSK3 = 0;							// Disable PCINT24-31 (THR/CPPM input)
			EIMSK  = 0;							// Disable INT0, 1 and 2 
			
			// Enable serial receiver and interrupts
			UCSR0B |= (1 << RXCIE0);			// Enable serial interrupt
			UCSR0B |= (1 << RXEN0);				// Enable receiver on UART0
			break;

		default:
			break;	
	}	

	// Transmitter side
	UCSR1B = (1 << TXEN1);						// Enable transmitter on UART1

	// Clear interrupt flags
	PCIFR	= 0x0F;								// Clear PCIF0~PCIF3 interrupt flags
	EIFR	= 0x07; 							// Clear INT0~INT2 interrupt flags (Elevator, Aileron, Rudder/CPPM)

	sei(); // Re-enable interrupts

} // init_int