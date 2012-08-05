//***********************************************************
//* lcd.c
//*
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include <avr/io.h>
#include <stdbool.h>
#include <util/delay.h>
#include "..\inc\io_cfg.h"
#include "..\inc\vbat.h"
#include "..\inc\adc.h"
#include "..\inc\init.h"
#include "..\inc\eeprom.h"
#include "..\inc\acc.h"
#include <avr/pgmspace.h> 

//************************************************************
// Prototypes
//************************************************************

void LCDprint(uint8_t i);
void LCDgoTo(uint8_t position);
void LCDprint_line1(const char *s);
void LCDprint_line2(const char *s);
void LCDclear(void);
void LCDclearLine(uint8_t line);
void LCDprintstr(const char *s);
void LCD_fixBL(void);

//************************************************************
// Defines
//************************************************************

#define LCD_BAUDRATE 9600
#define LCD_DELAY (1000000 / LCD_BAUDRATE)  // Default RX rate

//************************************************************
// LCD code snippets from Mike Barton.
// Borrowed from Arduino SoftUsart and the Sparkfun datasheet
//************************************************************

void LCDprint(uint8_t i)
{
	uint8_t mask;

	TX = 0;
	_delay_us(LCD_DELAY);

	for (mask = 0x01; mask; mask <<= 1) 
	{
		if (i & mask) TX = 1; 
		else TX = 0;
		_delay_us(LCD_DELAY);
	}
	TX = 1;
	_delay_us(LCD_DELAY);
}

// Position = line 1: 0-15, line 2: 16-31
void LCDgoTo(uint8_t position) 
{
	if (position<16)
	{
		LCDprint(0xFE);
		LCDprint(position+0x80);
	}
	else
	{
		LCDprint(0xFE);
		LCDprint(position+0x80+0x30);
	}
}

void LCDprint_line1(const char *s)
{
	LCDclearLine(1);
	LCDgoTo(0);

	while (*s) LCDprint(*s++);
}

void LCDprint_line2(const char *s)
{
	LCDclearLine(2);
	LCDgoTo(16);

	while (*s) LCDprint(*s++);
}

void LCDclear(void)
{
	LCDprint(0xFE);
	LCDprint(0x01);
}

void LCD_fixBL(void)
{
	LCDprint(0x7C);
	LCDprint(157);
}

void LCDclearLine(uint8_t line)
{
	int8_t j;

	if (line == 1)
	{
		LCDprint(0xFE); // Line 1
		LCDprint(0x80);
	}
	else
	{
		LCDprint(0xFE); // Line 2
		LCDprint(0xC0);
	}
	for(j=0;j<16;j++)
	{
		LCDprint(0x20);	// Clear line
	}
}

void LCDprintstr(const char *s)
{
	while (*s) LCDprint(*s++);
}
