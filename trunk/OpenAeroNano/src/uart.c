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
void variable_delay(uint8_t count);

//************************************************************
// Code
//************************************************************

#define USART_BAUDRATE 19200
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)  // Default RX rate
#define RX_QUALITY 8 		// Requires <= 80% success rate
#define TX_BIT_DELAY 51 	// Default TX rate


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

	LCD_TX = 0;
	variable_delay(Config.AutoTuneTX);

	for (mask = 0x01; mask; mask <<= 1) 
	{
		if (ByteToSend & mask) LCD_TX = 1; 
		else LCD_TX = 0;
		variable_delay(Config.AutoTuneTX);
	}
	LCD_TX = 1;
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
