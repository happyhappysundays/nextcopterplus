//***********************************************************
//* uart.c
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <util/delay.h>
#include "io_cfg.h"
#include "isr.h"
#include "servos.h"
#include "main.h"

//************************************************************
// Prototypes
//************************************************************

void init_uart(void);
void USART_Transmit(uint8_t data);
void TransmitData(void);

//************************************************************
// Code
// Baud = 20000000 / (16 * (UBRRn + 1))	Where U2X0 = 0
// Baud = 20000000 / ( 8 * (UBRRn + 1))	Where U2X0 = 1
//************************************************************

// Work out best divisor for baud rate generator
#define USART_BAUDRATE_XTREME 250000
#define BAUD_PRESCALE_XTREME ((F_CPU + USART_BAUDRATE_XTREME * 8L) / (USART_BAUDRATE_XTREME * 16L) - 1) // Default RX rate for Xtreme

#define USART_BAUDRATE_SBUS 100000
#define BAUD_PRESCALE_SBUS ((F_CPU + USART_BAUDRATE_SBUS * 4L) / (USART_BAUDRATE_SBUS * 8L) - 1) // Default RX rate for S-Bus

#define USART_BAUDRATE_SPEKTRUM 115200
#define BAUD_PRESCALE_SPEKTRUM ((F_CPU + USART_BAUDRATE_SPEKTRUM * 8L) / (USART_BAUDRATE_SPEKTRUM * 16L) - 1) // Default RX rate for Spektrum

#define SPEKTRUM_1024	0x02
#define SPEKTRUM_2048	0x12
#define SBUS_START		0xF0
#define SBUS_END		0x00
#define SPEKTRUM_CHNUM	7

// Initialise UART with adjusted bit rate
void init_uart(void)
{
	cli();								// Atmel wants global interrupts disabled when changing UART setup on the fly
	
	UCSR0B &= ~(1 << RXCIE0);			// Disable serial interrupt

	while (UCSR0A & (1 << RXC0))		// Make sure there is nothing in the RX0 reg
	{
		UCSR0C = UDR0;					// So as not to annoy Studio 6, use UCSR0C as a temp reg until UDR0 empty. Sorry, UCSR0C...
	}
	
	// Reset UART regs to a known state
	UCSR0A = 0; // U2X = 0, no master mode, flags cleared 
	UCSR0B = 0; // Clear flags, disable tx/rx, 8 bits
	UCSR0C = 6; // 8N1
	UCSR1A = 0; // U2X = 0, no master mode, flags cleared
	UCSR1B = 0; // Clear flags, disable tx/rx, 8 bits
	UCSR1C = 6; // 8N1

	// Setup RX side
	switch (Config.RxModeIn)
	{
		// Xtreme 8N1 (8 data bits / No parity / 1 stop bit / 250Kbps)
		case XTREME:
			UCSR0A &= ~(1 << U2X0);						// Clear the 2x flag
			UBRR0H  = (BAUD_PRESCALE_XTREME >> 8); 		// Actual = 250000, Error = 0%
			UBRR0L  =  BAUD_PRESCALE_XTREME & 0xff;		// 0x04
			UCSR0B |=  (1 << RXEN0);					// Enable receiver
			UCSR0C &= ~(1 << USBS0); 					// 1 stop bit
			UCSR0C &=  ~(1 << UPM00) | 					// No parity
						(1 << UPM01);
			UCSR0B |=  (1 << RXCIE0);					// Enable serial interrupt
			break;
				
		// Futaba S-Bus 8E2 (8 data bits / Even parity / 2 stop bits / 100Kbps)
		case SBUS: 		
			UCSR0A |=  (1 << U2X0);						// Need to set the 2x flag
			UBRR0H  = (BAUD_PRESCALE_SBUS >> 8);  		// Actual = 100000 , Error = 0%	
			UBRR0L  =  BAUD_PRESCALE_SBUS & 0xff;		// 0x18 (24)
			UCSR0B |=  (1 << RXEN0);					// Enable receiver
			UCSR0C |=  (1 << USBS0); 					// 2 stop bits
			UCSR0C &= ~(1 << UPM00); 					// Even parity 
			UCSR0C |=  (1 << UPM01); 
			UCSR0B |=  (1 << RXCIE0);					// Enable serial interrupt
			break;

		// Spektrum 8N1 (8 data bits / No parity / 1 stop bit / 115.2Kbps)
		case SPEKTRUM: 	
			UCSR0A &=  ~(1 << U2X0);					// Clear the 2x flag
			UBRR0H  =  (BAUD_PRESCALE_SPEKTRUM >> 8); 	// Actual = 113636, Error = -1.36%
			UBRR0L  =   BAUD_PRESCALE_SPEKTRUM & 0xff;	// 0x0A (10.35)	
			UCSR0B |= 	(1 << RXEN0);					// Enable receiver
			UCSR0C &=  ~(1 << USBS0); 					// 1 stop bit
			UCSR0C &=  ~(1 << UPM00) | 					// No parity 
						(1 << UPM01); 
			UCSR0B |=  (1 << RXCIE0);					// Enable serial interrupt
			break;

		case CPPM_MODE:
			UCSR0B &= 	~(1 << RXEN0);					// Disable receiver in CPPM modes
			break;
			
		default:
			break;
	}

	// Setup TX side
	switch (Config.RxModeOut)
	{
		// Xtreme 8N1 (8 data bits / No parity / 1 stop bit / 250Kbps)
		case XTREME:
		UCSR1A &= ~(1 << U2X1);						// Clear the 2x flag
		UBRR1H  = (BAUD_PRESCALE_XTREME >> 8); 		// Actual = 250000, Error = 0%
		UBRR1L  =  BAUD_PRESCALE_XTREME & 0xff;		// 0x04
		UCSR1B |=  (1 << TXEN1);					// Enable transmitter
		UCSR1C &= ~(1 << USBS1); 					// 1 stop bit
		UCSR1C &=  ~(1 << UPM10) | 					// No parity
					(1 << UPM11);

		break;
		
		// Futaba S-Bus 8E2 (8 data bits / Even parity / 2 stop bits / 100Kbps)
		case SBUS:
		UCSR1A |=  (1 << U2X1);						// Need to set the 2x flag
		UBRR1H  = (BAUD_PRESCALE_SBUS >> 8);  		// Actual = 100000 , Error = 0%
		UBRR1L  =  BAUD_PRESCALE_SBUS & 0xff;		// 0x18 (24)
		UCSR1B |=  (1 << TXEN1);					// Enable transmitter
		UCSR1C |=  (1 << USBS1); 					// 2 stop bits
		UCSR1C &= ~(1 << UPM10); 					// Even parity
		UCSR1C |=  (1 << UPM11);
		break;

		// Spektrum 8N1 (8 data bits / No parity / 1 stop bit / 115.2Kbps)
		case SPEKTRUM:
		UCSR1A &=  ~(1 << U2X1);					// Clear the 2x flag
		UBRR1H  =  (BAUD_PRESCALE_SPEKTRUM >> 8); 	// Actual = 113636, Error = -1.36%
		UBRR1L  =   BAUD_PRESCALE_SPEKTRUM & 0xff;	// 0x0A (10.35)
		UCSR1B |= 	(1 << TXEN1);					// Enable transmitter
		UCSR1C &=  ~(1 << USBS1); 					// 1 stop bit
		UCSR1C &=  ~(1 << UPM10) | 					// No parity
					(1 << UPM11);
		break;

		default:
		break;
	}

	// Re-enable interrupts
	sei();
}

// USART_Transmit routine straight from the Atmel datasheet
void USART_Transmit(uint8_t data)
{
	/* Wait for empty transmit buffer */
	while (!( UCSR1A & (1<<UDRE1)));
	
	/* Put data into buffer, sends the data */
	UDR1 = data;
}

// Transmit data packet based on the TX mode
void TransmitData(void)
{
	uint8_t i;
	uint8_t j = 0;						// GP counter and mask index
	uint8_t temp = 0;					// TX characters
	int16_t itemp16 = 0;				// Signed temp reg 
	uint16_t temp16 = 0;				// Unsigned temp reg 
	uint16_t SpektrumMask = 0;
	uint8_t chan_mask = 0;				// Common variables
	uint8_t chan_shift = 0;
	uint8_t data_mask = 0;
	uint8_t sindex = 0;					// Serial buffer index
	
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
	//*			For OpenAero2Serial the user could have set up all eight channels,
	//*			so we need to output all as CH1 to CH8, then send any extras above CH8.
 	//*  
	//*  0x00   0x23   0x000A   0x5DC   0x5DD   0xF0
	//*  ^^^^   ^^^^   ^^^^^^   ^^^^^   ^^^^^   ^^^^
	//*  Flags  dBm     Mask    CH 2    CH 4    ChkSum
	//*
	//*	750 	= 750us
	//*	1000	= 1000us
	//*	1500 	= 1500us +/- 500 for 1-2ms
	//*	2000	= 2000us
	//*	2250 	= 2250us
	//*
	//************************************************************

	if (Config.RxModeOut == XTREME)
	{
		checksum = 0;					// Reset checksum

		Xtreme_Chanmask |= 0x00FF;		// Always transmit at least the bottom 8 channels. 

		// Fake frame loss data if necessary
		if (Config.RxModeIn != XTREME)
		{		
			Xtreme_Flags = 0;
			Xtreme_RSS = 0;
		}
		
		// S.Bus will always be transmitted as 16 channels
		// Xtreme and Spektrum will know how many channels to transmit
		if (Config.RxModeIn == SBUS)
		{
			Xtreme_Chanmask = 0xFFFF;
		}
			
		// Transmit Xtreme data
		checksum += Xtreme_Flags;
		USART_Transmit(Xtreme_Flags);	// Copy of flags byte from input
		
		checksum += Xtreme_RSS;
		USART_Transmit(Xtreme_RSS);		// Copy of RSS byte from input
		
		temp = Xtreme_Chanmask >> 8;	// High byte of channel mask
		checksum += temp;
		USART_Transmit(temp);
		
		temp = 0xFF;					// Low byte of channel mask - always set to show CH1 to CH8
		Xtreme_Chanmask |= temp;		// Re-write the CH1 to CH8 bits of the original channel mask
		
		checksum += temp;
		USART_Transmit(temp);

		// Work out which channel the data is intended for from the mask bit position
		// This way the outgoing data will match the ingoing...
		for (j = 0; j < 16; j++)
		{
			// If there is a bit set, allocate channel data for it
			if (Xtreme_Chanmask & (1 << j))
			{
				// First eight channels come from ServoOut[]
				if (j < MAX_RC_CHANNELS)
				{
					// Remove system offset
					itemp16 = ServoOut[j];
					itemp16 -= 3750;
					
					// Conversion factor x0.5 
					itemp16 = (itemp16 >> 1);
					
					// Add in Extreme offset
					itemp16 += 1500;
					temp16 = itemp16;
					
					// Send bytes				
					temp = (uint8_t)(temp16 >> 8);		// High byte first
					checksum +=	temp;					// Add to checksum
					USART_Transmit(temp);

					temp = (uint8_t)(temp16 & 0xFF);	// Low byte next
					checksum +=	temp;					// Add to checksum
					USART_Transmit(temp);
				}
				// Extra channels come from ExtChannel[]
				else if (j < (MAX_EXT_CHANNELS + MAX_RC_CHANNELS))
				{
					temp = (uint8_t)(ExtChannel[j - MAX_RC_CHANNELS] >> 8);	// High byte first
					checksum +=	temp;										// Add to checksum
					USART_Transmit(temp);

					temp = (uint8_t)(ExtChannel[j - MAX_RC_CHANNELS] & 0xFF);	// Low byte next
					checksum +=	temp;										// Add to checksum
					USART_Transmit(temp);					
				}
			}
		}
		
		temp = (uint8_t)(checksum & 0xFF);
		USART_Transmit(temp);			// Checksum
	}

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
	//* The total number of received channels is indicated by Xtreme_Chanmask.
	//* Where possible, interleave as many channels back into the packets.
	//*
	//************************************************************

	// Handle Spektrum format
	else if (Config.RxModeOut == SPEKTRUM)
	{
		// Transmit frame loss data - fake if necessary
		if (Config.RxModeIn != SPEKTRUM)
		{
			Spektrum_frameloss = 0;
		}

		// CPPM has a maximum of 8 channels. Spektrum has 7 per frame.
		// So arrange channels as best fit. CH1-6 in both frames, 7 and 8 once per pair.
		if (Config.RxModeIn == CPPM)
		{
			Spektrum_Chanmask_0 = 0x007F; // CH1-7 in frame 0
			Spektrum_Chanmask_1 = 0x00BF; // CH1-6, 8 in frame 1
		}

		// S.Bus will always be transmitted as 16 channels
		// So mark all channels as set
		if (Config.RxModeIn == SBUS)
		{
			Spektrum_Chanmask_0 = 0x007F; // CH1-7 in frame 0
			Spektrum_Chanmask_1 = 0x3F80; // CH8-14 in frame 1
		}

		// Xtreme knows how many channels to transmit
		// Copy them out into the two frames
		if (Config.RxModeIn == XTREME)
		{
			i = 0;
			Spektrum_Chanmask_0 = 0;
			Spektrum_Chanmask_1 = 0;
			
			// Transfer the Xtreme channels to the two Spektrum frames 
			for (j = 0; j < 16; j++)
			{	
				if (Xtreme_Chanmask & (1 << j))
				{
					// CH1 to 7 go to Frame 0
					if (i < SPEKTRUM_CHNUM)
					{
						Spektrum_Chanmask_0 |= (1 << j);
					}
					// CH8 to 14 go to Frame 1
					else if (i < (SPEKTRUM_CHNUM << 1))
					{
						Spektrum_Chanmask_1 |= (1 << j);
					}
					// Increment Spektrum channel number
					i++;
				}
			}
		}
		
		USART_Transmit(Spektrum_frameloss);	// Copy of last frame loss byte from input		

		// Transmit flags (hard coded to 2048 and 2 frames per packet)
		USART_Transmit(SPEKTRUM_2048);

		// Process alternate frames correctly
		// We will always transmit eight channels of data.

		i = 0; // Reset channel counter  

		// Select the appropriate channel mask for this frame
		if (Spektrum_frame_in == 0)
		{
			SpektrumMask = Spektrum_Chanmask_0; // Use mask 0
		}
		else
		{
			SpektrumMask = Spektrum_Chanmask_1; // Use mask 1
		}
			
		// Work out which channel the data is intended for from the mask bit position
		// This way the outgoing data will match the ingoing... fill in blanks with 0xFFFF
		for (j = 0; j < 16; j++)
		{
			// If there is a bit set in the mask, allocate channel data for it.
			// If not, move to next channel/bit
			if (SpektrumMask & (1 << j))
			{
				// We can only fit seven channels in each frame
				if (i < SPEKTRUM_CHNUM)
				{
					// First eight channels come from ServoOut[]
					if (j < MAX_RC_CHANNELS)
					{
						// Convert to Spektrum values (2500~5000 -> 0~2047)
						itemp16 = ServoOut[j];
						itemp16 -= 3750;				// Remove local offset (-1250~1250)
							
						// Respan to Spektrum units (0-2047)
						// x0.8675 (0.8672)
						itemp16 = (itemp16 >> 1) + (itemp16 >> 2) + (itemp16 >> 4) + (itemp16 >> 5) + (itemp16 >> 6) + (itemp16 >> 7);
							
						itemp16 += 1024;				// Add Spektrum offset
							
						// Bounds check (0 to 2047)
						if (itemp16 < 0)
						{
							itemp16 = 0;
						}
						if (itemp16 > 2047)
						{
							itemp16 = 2047;
						}
							
						// Reshuffle bits into byte pairs
						temp = (uint8_t)(itemp16 >> 8);		// OR the channel number together with the upper 3 bits of the data
						temp |= (j << 3);					// Shift channel number up to the correct spot. j = channel number
	
						// Set the "2nd frame" bit for first byte of channel data if we are generating the second frame
						// i is the count of channels output in this frame
						if ((Spektrum_frame_in == 1) && (i == 0))
						{
							temp |= 0x80;			
						}
							
						// Transmit channel data msb
						USART_Transmit(temp);
							
						// Transmit channel data lsb
						temp = (uint8_t)(itemp16 & 0xFF);
						USART_Transmit(temp);
					}
					// Ext channel - already formatted correctly
					else
					{
						// Reshuffle bits into byte pairs
						temp = (uint8_t)(ExtChannel[j - MAX_RC_CHANNELS] >> 8);
						
						// Set the "2nd frame" bit for first byte of channel data if we are generating the second frame
						if ((Spektrum_frame_in == 1) && (i == 0))
						{
							temp |= 0x80;
						}
						
						// Transmit channel data msb
						USART_Transmit(temp);
							
						// Transmit channel data lsb
						temp = (ExtChannel[j - MAX_RC_CHANNELS] & 0xFF);
						USART_Transmit(temp);
					}
			
					i++; // Count the channels inserted into this frame

				} //if (i < SPEKTRUM_CHNUM)
			} // If there is a bit set
		} // for (j = 0; j < 16; j++)

		// Fill in any extras with 0xffff
		// i is the count of channels output in this frame
		for (j = i; j < SPEKTRUM_CHNUM; j++)
		{
			// Transmit channel data msb
			USART_Transmit(0xFF);
				
			// Transmit channel data lsb
			USART_Transmit(0xFF);
		}

		// Flip to alternate frame for generating when in failsafe or not fed by a Spektrum source
		// Otherwise we will keep generating the same frame...
		if ((Flight_flags & (1 << FailsafeFlag)) || (Config.RxModeIn != SPEKTRUM))
		{
			if (Spektrum_frame_in == 0)
			{
				Spektrum_frame_in = 1;
			}
			else
			{
				Spektrum_frame_in = 0;
			}
		}
	/*	
		// Clear the channel masks after transmission in case they change.		
		else
		{
			SpektrumMask = 0;
			Spektrum_Chanmask_0 = 0;
			Spektrum_Chanmask_1 = 0;
		}
		*/
	} // (Config.RxMode == SPEKTRUM)

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
	//* Data size:	0 to 2047, centered on 1024 (1.501ms)
	//*				0	(-1024)	= 901us
	//*				168 (-855)	= 1.0ms
	//*				1024 (0)	= 1.501ms
	//*				1877 (+855) = 2.0ms
	//*				2047 (+1023) = 2098us
	//*	
	//************************************************************

	else if (Config.RxModeOut == SBUS)
	{
		// Clear entire sBuffer first
		memset(&sBuffer[0],0,(sizeof(sBuffer)));
		
		// S.Bus start byte
		sBuffer[0] = SBUS_START;

		// Convert servo data to S.Bus data size
		for (i = 0; i < MAX_OUTPUTS; i++)
		{
			// Convert to S.BUS values
			itemp16 = ServoOut[i];			// Copy to signed register (2500~5000)

			itemp16 -= 3750;				// Remove local offset (-1250~1250)
			
			// Respan to S.BUS values (x0.8) (0.8008)
			itemp16 = (itemp16 >> 1) + (itemp16 >> 2) + (itemp16 >> 5) + (itemp16 >> 6) + (itemp16 >> 8);
			
			itemp16 += 1024;				// Add S.Bus offset
			
			// Bounds check (0 to 2047)
			if (itemp16 < 0)
			{
				itemp16 = 0;
			}
			if (itemp16 > 2047)
			{
				itemp16 = 2047;
			}

			// Copy to unsigned register
			temp16 = (itemp16 & 0x7FFF);
			
			// Copy back to ServoOut for now
			ServoOut[i] = temp16;
		}
		
        // Reconstruct S-Bus data
		// 22 bytes of ineptly compressed data
		//* 1-22 data = [ch1, 11bit][ch2, 11bit] .... [ch16, 11bit] (Values = 0 to 2047)
		//* 	channel 1 uses 8 bits from data1 and 3 bits from data2
		//* 	channel 2 uses last 5 bits from data2 and 6 bits from data3
		//* 	etc.
		// Our data is 8ch x 11 bits or 88 bits but the whole space is 16ch or 176 bits (22*8)
	
		sindex = 1;	// Step over flag byte
		
        for (j = 0; j < 176; j++)
		{
			if (chan_shift < MAX_RC_CHANNELS)
			{
				// Transfer set bits from 16-bit servo data to the sBuffer
				if (ServoOut[chan_shift] & (1 << data_mask))
				{
					// Place the RC data into buffer directly
					sBuffer[sindex] |= (1 << chan_mask);
				}
			}
			// Extra channels in extra buffer
			else if (chan_shift < (MAX_EXT_CHANNELS + MAX_RC_CHANNELS))
			{
				// Transfer set bits from 16-bit extra channel data to the sBuffer
				if (ExtChannel[chan_shift - MAX_RC_CHANNELS] & (1 << data_mask))
				{
					// Place the RC data into buffer directly
					sBuffer[sindex] |= (1 << chan_mask);
				}
			}

            chan_mask++;
            data_mask++;

            // If we have done 8 bits, move to next byte in sBuffer
            if (chan_mask == 8)
            {
	            chan_mask = 0;
	            sindex++;
            }

            // If we have reconstructed all 11 bits of one channel's data (2047)
            // increment the channel number
            if (data_mask == 11)
            {
	            data_mask = 0;
	            chan_shift++;
            }			
		}

		// Transmit S.Bus flags if they exist	
		if (Config.RxModeIn != SBUS)
		{
			SBUS_Flags = 0;
		}		

		sBuffer[23] = SBUS_Flags;	
		
		// Transmit S.Bus end byte
		sBuffer[24] = SBUS_END;	
		
		// Transmit whole S.Bus packet
		for (j = 0; j < 25; j++)
		{
			USART_Transmit(sBuffer[j]);
		}
	}
}