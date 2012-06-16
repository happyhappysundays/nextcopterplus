//***********************************************************
//* uart.c
//*
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdbool.h>
#include <stdlib.h>
#include <util/delay.h>
#include "..\inc\io_cfg.h"
#include "..\inc\acc.h"
#include "..\inc\gyros.h"
#include "..\inc\servos.h"
#include "..\inc\init.h"
#include "..\inc\rc.h"
#include "..\inc\main.h"
#include "..\inc\isr.h"
#include "..\inc\pots.h"
#include "..\inc\lcd.h"

//************************************************************
// Prototypes
//************************************************************

void init_uart(void);
void send_byte(uint8_t i);
void send_word(uint16_t i);
uint8_t rx_byte(void);
uint16_t rx_word(void);
void send_multwii_data(void);
void get_multwii_data(void);
void autotune(void);
void variable_delay(uint8_t count);

//************************************************************
// Code
//************************************************************

#define USART_BAUDRATE 19200
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)  // Default RX rate
#define RX_QUALITY 8 		// Requires <= 80% success rate
#define TX_BIT_DELAY 51 	// Default TX rate

// Pass model type to MultiWii GUI
#if defined(STD_FLAPERON)
	#define MULTITYPE 13	// Flaperon aeroplane mode
#elif defined(STANDARD)
	#define MULTITYPE 12	// Aeroplane mode
#else
	#define MULTITYPE 8		// Flying wing
#endif

void init_uart(void)						// Initialise UART with adjusted bitrate
{
	UCSR0B |= (1 << RXEN0);					// Turn on the reception circuitry
	UCSR0C |= (1 << UCSZ00) | (1 << UCSZ01);// Use 8-bit character sizes

	UBRR0H = (Config.AutoTuneRX >> 8); 		// Load upper 8-bits of the baud rate value into the high byte of the UBRR register
	UBRR0L = Config.AutoTuneRX; 			// Load lower 8-bits of the baud rate value into the low byte of the UBRR register 
}

void send_byte(uint8_t ByteToSend)
{
	uint8_t mask;

	TX = 0;
	variable_delay(Config.AutoTuneTX);

	for (mask = 0x01; mask; mask <<= 1) 
	{
		if (ByteToSend & mask) TX = 1; 
		else TX = 0;
		variable_delay(Config.AutoTuneTX);
	}
	TX = 1;
	variable_delay(Config.AutoTuneTX);
}

void send_word(uint16_t i)
{
	uint8_t byte_l;
	uint8_t byte_h;
	uint16_t word;

	byte_l = (uint8_t)i;
	send_byte(byte_l);

	word = i >> 8;
	byte_h = (uint8_t)word;
	send_byte(byte_h);
}

uint8_t rx_byte(void)
{
	while ((UCSR0A & (1 << RXC0)) == 0) {}; 	// Do nothing until data is ready to be read
	return(UDR0); 
}

uint16_t rx_word(void)
{
	uint8_t bytel, byteh;
	uint16_t word;

	bytel = rx_byte();
	byteh = rx_byte();
	word = ((byteh << 8) | bytel);

	return(word);
}

void get_multwii_data(void) // 22 bytes
{
	Config.P_mult_roll = rx_byte();
	Config.I_mult_roll = rx_byte();
	Config.D_mult_roll = rx_byte();
	Config.P_mult_pitch = rx_byte();
	Config.I_mult_pitch = rx_byte();
	Config.D_mult_pitch = rx_byte();
	Config.P_mult_yaw = rx_byte();
	Config.I_mult_yaw = rx_byte();
	Config.D_mult_yaw = rx_byte();
	Config.P_mult_glevel = rx_byte();
	Config.D_mult_glevel = rx_byte();
	Config.P_mult_alevel = rx_byte();
	Config.I_mult_alevel = rx_byte();	//
	rx_byte();							// Was RC_Rate
	rx_byte();							// Was RC_Expo
	rx_byte();							// Was RollPitchRate
	rx_byte();							// Was YawRate
	Config.PowerTrigger = rx_word();	//
	Config.Modes = rx_byte();			//
	Config.AccRollZeroTrim = rx_byte();	//
	Config.AccPitchZeroTrim = rx_byte();//22
}

void send_multwii_data(void) // 66 bytes
{
	uint8_t i;

    send_byte('M');				// Send signature byte
    send_byte(VERSION);			// Firmware version 1
    for (i = 0; i < 2; i++)
	send_word(accADC[i]<<2);	// Scaled acc 
	send_byte(0);
	send_byte(0);				// 7 - Dummy word for ACC (Z)
    for (i = 0; i < 3; i++)
	send_word(gyroADC[i]); 		// Gyro 13
	send_word(ServoOut1); 		// Motors 25
	send_word(ServoOut2);
	send_word(ServoOut4);
	send_word(ServoOut5);
	send_word(ServoOut6);
	send_word(Throttle);
	send_word(RxChannel1);
	send_word(RxChannel2); 
	send_word(RxChannel3); 
	send_word(RxChannel4); 		// 33
#ifdef ICP_CPPM_MODE
	send_word(RxChannel5);
	send_word(RxChannel6); 
	send_word(RxChannel7); 
	send_word(RxChannel8); 		// 41
#else
	send_word(1500);
	send_word(1500); 
	send_word(1500); 
	send_word(1500); 			// 41
#endif
	if ((Config.Modes &16) > 0) 
	{
		flight_mode |= 16;		// 16 = LVA mode 1 = buzzer, 0 = LED
	}
	else
	{
		flight_mode &= 0xef;
	}
	if ((Config.Modes &4)  > 0) 
	{
		flight_mode |= 4;  		//  4 = Pot mode 1 = eeprom, 0 = Pots
	}
	else
	{
		flight_mode &= 0xfb;
	}
	send_byte(flight_mode);
	send_word(cycletime);		// cycleTime 44

#ifndef N6_MODE
	send_byte(MULTITYPE);		// Multitype 45
#else
	switch(MixerMode)
	{
		case 0:					// Aeroplane mixing
			send_byte(12);
			break;
		case 2:					// Flying wing mixing
			send_byte(8);
			break;
		case 1:					// Flaperon mixing
			send_byte(13);
			break;
		default:				// Default to aeroplane mixing
			send_byte(12);
			break;
	}
#endif

	send_byte(Config.P_mult_roll);
	send_byte(Config.I_mult_roll);
	send_byte(Config.D_mult_roll);
	send_byte(Config.P_mult_pitch);
	send_byte(Config.I_mult_pitch);
	send_byte(Config.D_mult_pitch);
	send_byte(Config.P_mult_yaw);
	send_byte(Config.I_mult_yaw);
	send_byte(Config.D_mult_yaw);
	send_byte(Config.P_mult_glevel);
	send_byte(Config.D_mult_glevel);
	send_byte(Config.P_mult_alevel);
	send_byte(Config.I_mult_alevel);	// 
    send_byte(0);						// Was RC rate
    send_byte(0);						// Was RC Expo
    send_byte(0);						// No longer used
    send_byte(0);						// No longer used
	send_word(vBat);					// Send battery voltage (1000 = 10.0V)
	send_word(Config.PowerTrigger);		// intPowerTrigger1 
	send_byte(Config.AccRollZeroTrim);
	send_byte(Config.AccPitchZeroTrim); // 
    send_byte('M');						// 69 Send trailer signature byte
}

void autotune(void)
{
	int start_speed	= BAUD_PRESCALE >> 1;	// Search start point
	int end_speed	= BAUD_PRESCALE + start_speed;	// Search end point
	uint8_t hit	= 0;					
	uint8_t count = 0;
	uint8_t	dummy = 0;
	uint16_t test_speed, min_speed, max_speed = 0;	// Speed markers

	min_speed 	= end_speed;			// Initialise markers
	max_speed	= start_speed;
	dummy = UDR0;						// Flush buffer!

	for (test_speed = start_speed; test_speed <= end_speed; test_speed++)
	{
		UBRR0H = (test_speed >> 8); 	// Load upper 8-bits of the baud rate value into the high byte of the UBRR register
		UBRR0L = test_speed; 			// Load lower 8-bits of the baud rate value into the low byte of the UBRR register 
		_delay_ms(1);

		for (int i = 0; i < 10; i++)	// Try 10 times at each speed (1 sec)
		{
			if ((UCSR0A & (1 << RXC0)) > 0) // Something in the RX buffer
			{
				if (UDR0 == 'w') 
				{
					count++;
					if ((count >= RX_QUALITY) && (test_speed < min_speed))
					{
						min_speed = test_speed; // Update marker
						hit = 1;
					}
					else if ((count >= RX_QUALITY) && (test_speed > max_speed))
					{
						max_speed = test_speed; // Update marker
						hit = 1;
					}
				}
			}
			_delay_ms(20);				// Wait until a character sure to be there
		}
		LED1 = !LED1;						// Flash LED during test loops
	}

	LED1 = 0;
	
	if (hit > 0)						// Only update if new value found
	{
		// Calculate RX value
		test_speed = min_speed + ((max_speed - min_speed) >> 1);
		Config.AutoTuneRX = test_speed;

		// Calculate TX value with improved accuracy
		min_speed = min_speed << 1; // Double
		max_speed = max_speed << 1; // Double
		test_speed = min_speed + ((max_speed - min_speed) >> 1);
		Config.AutoTuneTX = test_speed; 
	}

	// Print out autotune result for debugging
	LCDprint_line1("UTX:    URX:    ");	
	LCDprint_line2("LCDTX:          ");
	LCDgoTo(4);
	LCDprintstr(itoa(Config.AutoTuneTX,pBuffer,10));
	LCDgoTo(12);
	LCDprintstr(itoa(Config.AutoTuneRX,pBuffer,10));
	LCDgoTo(22);
	LCDprintstr(itoa((Config.AutoTuneTX << 1),pBuffer,10));
}

void variable_delay(uint8_t count)
{
	switch(count) {
		case 45:				// 
			_delay_us(41);
			break;
		case 46:				// 
			_delay_us(42);
			break;
		case 47:				// 
			_delay_us(43);
			break;
		case 48:				// 
			_delay_us(44);
			break;
		case 49:				// 49.0/49.3us
			_delay_us(45);
			break;
		case 50:				// 50.4/50.7us
			_delay_us(46);
			break;
		case 51:				// 51.2/51.6us
			_delay_us(47);
			break;
		case 52:				// 53.0/53.3us
			_delay_us(48);
			break;
		case 53:				// 53.3/53.7us
			_delay_us(49);
			break;
		case 54:				// 
			_delay_us(50);
			break;
		case 55:				// 
			_delay_us(51);
			break;
		case 56:				// 
			_delay_us(52);
			break;
		case 57:				// 
			_delay_us(53);
			break;
		default:
			_delay_us(47); // Same as case 51
			break;
	}
}
