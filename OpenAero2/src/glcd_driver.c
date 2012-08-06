//***********************************************************
//* glcd_driver.c
//***********************************************************
//*
//* $Id:$
//*
//* ST7565 LCD Driver
//*
//* Copyright (C) 2010 Limor Fried, Adafruit Industries
//*
//* This library is free software; you can redistribute it and/or
//* modify it under the terms of the GNU Lesser General Public
//* License as published by the Free Software Foundation; either
//* version 2.1 of the License, or (at your option) any later version.
//* 
//* This library is distributed in the hope that it will be useful,
//* but WITHOUT ANY WARRANTY; without even the implied warranty of
//* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//* Lesser General Public License for more details.
//* 
//* You should have received a copy of the GNU Lesser General Public
//* License along with this library; if not, write to the Free Software
//* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
//* 
//* Additional updates by David Thompson
//*
//***********************************************************
//* Includes
//***********************************************************

#include <avr/io.h>
#include <stdlib.h>
#include <string.h>
#include <util/delay.h>
#include <avr/pgmspace.h> 
#include "..\inc\glcd_driver.h"
#include "..\inc\io_cfg.h"
#include "..\inc\main.h"

#define GLCD_BAUDRATE 8000
#define GLCD_DELAY (1000000 / GLCD_BAUDRATE)  // Default RX rate (125ns)

void glcd_delay(void);
void glcd_spiwrite_asm(uint8_t byte);
void mugui_ctrl_setPixel(uint16_t x, uint16_t y, uint8_t color);

//***********************************************************
//* Low-level code
//***********************************************************

int pagemap[] PROGMEM 		= { 7, 6, 5, 4, 3, 2, 1, 0 }; 	// This makes more sense
int pagemap_logo[] PROGMEM	= { 0, 1, 2, 3, 4, 5, 6, 7 };	// This works for the logo...

// Software SPI write
inline void spiwrite(uint8_t c) 
{
	int8_t i;
	for (i=7; i>=0; i--) 
	{
		LCD_SCL = 0;
		if (c & (1 << (i)))		// Bit set?
		{
			LCD_SI = 1;
			glcd_delay();
			LCD_SCL = 1;
		}
		else					// Bit clear?
		{
			LCD_SI = 0;
			glcd_delay();
			LCD_SCL = 1;
		}
		_delay_us(1);
	}
}

// Send command to LCD
void st7565_command(uint8_t c) 
{
	LCD_A0 = 0;
	spiwrite(c);
}

// Send data to LCD
void st7565_data(uint8_t c) 
{
	LCD_A0 = 1;
	spiwrite(c);
}


// Initialise LCD
void st7565_init(void) 
{
	// Set pin directions
	LCD_SI_DIR 	= OUTPUT;
	LCD_SCL_DIR = OUTPUT;
	LCD_A0_DIR	= OUTPUT;
	LCD_RES_DIR = OUTPUT;
	LCD_CSI_DIR	= OUTPUT;

	// Toggle RST low to reset and CS low so it'll listen to us
	LCD_CSI = 0;
	LCD_RES = 0;
	_delay_ms(500);
	LCD_RES = 1;

	// LCD bias select
	st7565_command(CMD_SET_BIAS_9); 			// Check (A2)
	// ADC select
	st7565_command(CMD_SET_ADC_NORMAL); 		// Check (A0)
	// Initial display line
	st7565_command(CMD_SET_DISP_START_LINE); 	// Check (40)
	// Normal display mode
	st7565_command(CMD_SET_DISP_NORMAL); 		// Check (A6)
	// Clear RMW mode
	st7565_command(CMD_RMW_CLEAR); 				// Check (EE)
	// Set reverse scan mode
	st7565_command(CMD_SET_COM_REVERSE); 		// Check (C8)
	// Turn on voltage converter (VC=1, VR=0, VF=0)
	st7565_command(CMD_SET_POWER_CONTROL | 0x4); 
	// Wait for 50% rising
	_delay_ms(50);
	// Turn on voltage regulator (VC=1, VR=1, VF=0)
	st7565_command(CMD_SET_POWER_CONTROL | 0x6);
	// Wait >=50ms
	_delay_ms(50);
	// Turn on voltage follower (VC=1, VR=1, VF=1)
	st7565_command(CMD_SET_POWER_CONTROL | 0x7); // Check (2F)
	// Wait
	_delay_ms(10);
	// Set LCD operating voltage (regulator resistor, ref voltage resistor)
	st7565_command(CMD_SET_RESISTOR_RATIO); 	// Check (24)
	// Set up static register (OFF)
	st7565_command(CMD_SET_STATIC_OFF); 		// Check (AC)
	st7565_command(CMD_SET_STATIC_REG); 		// Check (00)
	// Set up booster
	st7565_command(CMD_SET_BOOSTER_FIRST); 		// Check (F8)
	st7565_command(CMD_SET_BOOSTER_234); 		// Check (00)
}


// Set LCD brightness
void st7565_set_brightness(uint8_t val) 
{
	st7565_command(CMD_SET_VOLUME_FIRST);
	st7565_command(CMD_SET_VOLUME_SECOND | (val & 0x3f));
}

// Write LCD buffer
void write_buffer(uint8_t *buffer) 
{
	uint8_t c, p;
	for(p = 0; p < 8; p++) 
	{
		st7565_command(CMD_SET_PAGE | pgm_read_byte(&pagemap[p]));					// Page 0 to 7
		st7565_command(CMD_SET_COLUMN_LOWER | (0x0 & 0xf));			// Column 0
		st7565_command(CMD_SET_COLUMN_UPPER | ((0x0 >> 4) & 0xf));	// Column 0
		st7565_command(CMD_RMW);									// Sets auto-increment

		for(c = 0; c < 128; c++) 
		{
			st7565_data(buffer[(128*p)+c]);
		}
	}
}

// Write LCD buffer for logo
void write_logo_buffer(uint8_t *buffer) 
{
	uint8_t c, p;
	for(p = 0; p < 8; p++) 
	{
		st7565_command(CMD_SET_PAGE | pgm_read_byte(&pagemap_logo[p]));					// Page 0 to 7
		st7565_command(CMD_SET_COLUMN_LOWER | (0x0 & 0xf));			// Column 0
		st7565_command(CMD_SET_COLUMN_UPPER | ((0x0 >> 4) & 0xf));	// Column 0
		st7565_command(CMD_RMW);									// Sets auto-increment

		for(c = 0; c < 128; c++) 
		{
			st7565_data(buffer[(128*p)+c]);
		}
	}
}

// Clear buffer
void clear_buffer(uint8_t *buff) 
{
	memset(buff, 0, 1024);
}

//***********************************************************
//* Graphics API code
//***********************************************************

// Set a single pixel - muGUI
void mugui_ctrl_setPixel(uint16_t x, uint16_t y, uint8_t color) 
{
	if ((x >= LCDWIDTH) || (y >= LCDHEIGHT))
	{
		return;
	}
	// x is which column
	if (color)
	{
		buffer[x+ (y/8)*128] |= (1 << (7-(y%8)));  
	}
	else
	{
		buffer[x+ (y/8)*128] &= ~(1 << (7-(y%8))); 
	}
}

// Set a single pixel
void setpixel(uint8_t *buff, uint8_t x, uint8_t y, uint8_t color) 
{
	if ((x >= LCDWIDTH) || (y >= LCDHEIGHT))
	{
		return;
	}
	// x is which column
	if (color)
	{
		buff[x+ (y/8)*128] |= (1 << (7-(y%8)));  
	}
	else
	{
		buff[x+ (y/8)*128] &= ~(1 << (7-(y%8))); 
	}
}

void drawbitmap(uint8_t *buff, uint8_t x, uint8_t y, const uint8_t bitmap, uint8_t w, uint8_t h, uint8_t color) 
{
	for (uint8_t j=0; j<h; j++) 
	{
		for (uint8_t i=0; i<w; i++ ) 
		{
			if (pgm_read_byte(bitmap + i + (j/8)*w) & (1 << (j%8))) 
			{
				setpixel(buff, x+i, y+j, color);
			}
		}
	}
}

// Clear a single pixel
void clearpixel(uint8_t *buff, uint8_t x, uint8_t y) 
{
	// x is which column
	buff[x+ (y/8)*128] &= ~(1 << (7-(y%8)));
}

// Bresenham's algorithm - From wikpedia
void drawline(uint8_t *buff, uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t color) 
{
	uint8_t steep = abs(y1 - y0) > abs(x1 - x0);
	if (steep) 
	{
		swap(x0, y0);
		swap(x1, y1);
	}

	if (x0 > x1) 
	{
		swap(x0, x1);
		swap(y0, y1);
	}

	uint8_t dx, dy;
	dx = x1 - x0;
	dy = abs(y1 - y0);

	int8_t err = dx / 2;
	int8_t ystep;

	if (y0 < y1) 
	{
		ystep = 1;
	} 
	else 
	{
		ystep = -1;
	}

	for (; x0<x1; x0++) 
	{
		if (steep) 
		{
			setpixel(buff, y0, x0, color);
		} 
		else 
		{
			setpixel(buff, x0, y0, color);
		}
		err -= dy;
		if (err < 0) 
		{
			y0 += ystep;
			err += dx;
		}
	}
}

// Filled rectangle
void fillrect(uint8_t *buff, uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color) 
{
	// Stupidest version - just pixels - but fast with internal buffer!
	for (uint8_t i=x; i<x+w; i++) 
	{
		for (uint8_t j=y; j<y+h; j++) 
		{
			setpixel(buff, i, j, color);
		}
	}
}


// Draw a rectangle
void drawrect(uint8_t *buff, uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color) 
{
	// Stupidest version - just pixels - but fast with internal buffer!
	for (uint8_t i=x; i<x+w; i++) 
	{
		setpixel(buff, i, y, color);
		setpixel(buff, i, y+h-1, color);
	}
	for (uint8_t i=y; i<y+h; i++) 
	{
		setpixel(buff, x, i, color);
		setpixel(buff, x+w-1, i, color);
	} 
}


// Draw a circle
void drawcircle(uint8_t *buff, uint8_t x0, uint8_t y0, uint8_t r, uint8_t color) 
{
	int8_t f = 1 - r;
	int8_t ddF_x = 1;
	int8_t ddF_y = -2 * r;
	int8_t x = 0;
	int8_t y = r;

	setpixel(buff, x0, y0+r, color);
	setpixel(buff, x0, y0-r, color);
	setpixel(buff, x0+r, y0, color);
	setpixel(buff, x0-r, y0, color);

	while (x<y) 
	{
		if (f >= 0) 
		{
			y--;
			ddF_y += 2;
			f += ddF_y;
		}

		x++;
		ddF_x += 2;
		f += ddF_x;

		setpixel(buff, x0 + x, y0 + y, color);
		setpixel(buff, x0 - x, y0 + y, color);
		setpixel(buff, x0 + x, y0 - y, color);
		setpixel(buff, x0 - x, y0 - y, color);

		setpixel(buff, x0 + y, y0 + x, color);
		setpixel(buff, x0 - y, y0 + x, color);
		setpixel(buff, x0 + y, y0 - x, color);
		setpixel(buff, x0 - y, y0 - x, color);
	}
}


// Draw a filled circle
void fillcircle(uint8_t *buff, uint8_t x0, uint8_t y0, uint8_t r, uint8_t color) 
{
	int8_t f = 1 - r;
	int8_t ddF_x = 1;
	int8_t ddF_y = -2 * r;
	int8_t x = 0;
	int8_t y = r;

	for (uint8_t i=y0-r; i<=y0+r; i++) 
	{
		setpixel(buff, x0, i, color);
	}

	while (x<y) 
	{
		if (f >= 0) 
		{
			y--;
			ddF_y += 2;
			f += ddF_y;
		}

		x++;
		ddF_x += 2;
		f += ddF_x;

		for (uint8_t i=y0-y; i<=y0+y; i++) 
		{
			setpixel(buff, x0+x, i, color);
			setpixel(buff, x0-x, i, color);
		} 
		for (uint8_t i=y0-x; i<=y0+x; i++) 
		{
			setpixel(buff, x0+y, i, color);
			setpixel(buff, x0-y, i, color);
		}    
	}
}


// Clear screen (does not clear buffer)
void clear_screen(void) 
{
	uint8_t p, c;

	for(p = 0; p < 8; p++) 
	{
		st7565_command(CMD_SET_PAGE | p);								// Set page to p
		for(c = 0; c < 128; c++) 										// Was 129, which I think is wrong...
		{
			st7565_command(CMD_SET_COLUMN_LOWER | (c & 0xf));
			st7565_command(CMD_SET_COLUMN_UPPER | ((c >> 4) & 0xf));	// Set column to c
			st7565_data(0x00);											// Clear data
		}     
	}
}

