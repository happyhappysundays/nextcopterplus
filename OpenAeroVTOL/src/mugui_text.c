//***************************************************************************
//* mugui_text.c
//***************************************************************************
//*! \addtogroup mugui_text text functions for graphical displays
//*
//* @{
//* \file    text functions for graphical displays
//* \author  www.mugui.de [FB]
//* \date    15.11.2009
//* \version 1.0.0
//* \todo
//*
//***************************************************************************
//* muGUI - micro Graphical User Interface
//* Copyright (C) 2009 www.mugui.de [FB]
//*
//* This program is free software: you can redistribute it and/or modify
//* it under the terms of the GNU General Public License as published by
//* the Free Software Foundation, either version 3 of the License, or
//* (at your option) any later version.
//*
//* This program is distributed in the hope that it will be useful,
//* but WITHOUT ANY WARRANTY; without even the implied warranty of
//* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//* GNU General Public License for more details.
//*
//* You should have received a copy of the GNU General Public License
//* along with this program.  If not, see <http://www.gnu.org/licenses/>.
//***************************************************************************
//Import data types always first

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <avr/pgmspace.h>
#include <stdbool.h>
#include "..\inc\io_cfg.h"
#include "..\inc\main.h"
#include "..\inc\glcd_driver.h"

/*************************************************************************/
// Replacement for missing include files...
/*************************************************************************/
typedef struct
{
	uint16_t	x;
	uint16_t	y;
} mugui_point16_t;

// Redefine mugui types to standard ones
#define mugui_uint16_t uint16_t
#define mugui_char_t char
#define mugui_uint8_t uint8_t
#define mugui_bool bool
#define mugui_uint32_t uint32_t
#define mugui_uint64_t uint64_t
#define mugui_int64_t int64_t

// Externals
extern uint8_t buffer[];

// Prototypes
void mugui_text_sizestring(mugui_char_t *s, prog_uchar* font, mugui_size16_t *size);
void mugui_lcd_puts(mugui_char_t *s, prog_uchar* font,mugui_uint16_t x, mugui_uint16_t y);
mugui_uint16_t mugui_lcd_putc(mugui_char_t c, prog_uchar* font,mugui_uint16_t x, mugui_uint16_t y);
void pgm_mugui_lcd_puts(prog_uchar* s, prog_uchar* font,mugui_uint16_t x, mugui_uint16_t y);
void pgm_mugui_scopy(const char *s);

/*************************************************************************/
/*! \brief  calculate the size of a string 
	\param  pointer to string s
	\param  pointer to font
	\param  pointer to string size as return value
	\return void
	\date 	12.08.2009
*/
/************************************************************************/
void mugui_text_sizestring(mugui_char_t *s, prog_uchar* font, mugui_size16_t *size)
{
	mugui_uint8_t  distance = 1;			//distance between characters
	mugui_uint16_t length = 0;				//temporary length of string
	mugui_uint16_t i = 0;					//counter variable
	mugui_uint16_t startcharacter = 0; 	    //startcharacter of the font
	mugui_uint16_t height = 0;				//height of the bitmap
	mugui_uint16_t width = 0; 				//width of the bitmap
	mugui_uint8_t  index = 0; 				//Index of the bitmap
	mugui_uint8_t  indexlowbyte = 0; 		//low byte of the bitmap address in the array
	mugui_uint8_t  indexhighbyte = 0; 		//high byte of the bitmap address in the array
	mugui_uint32_t indexaddress = 0;		//bitmap address in the array (derived from low and high byte)

	/* read header of the font          */
	/* pgm_read_byte is ATMega specific */
	length = strlen(s);
	height = pgm_read_byte(&font[4]);

	startcharacter = pgm_read_byte(&font[2]);
	for(i=0;i<length;i++) //every character in prop fonts has its own width
	{
		index = s[i] - startcharacter;
		indexhighbyte = pgm_read_byte(&font[index*2 + 5]);
		indexlowbyte = pgm_read_byte(&font[index*2 + 6]);
		indexaddress = (mugui_int64_t)indexhighbyte;
		indexaddress = indexaddress << 8;
		indexaddress += indexlowbyte;
		width += pgm_read_byte(&font[indexaddress]) + distance;
	}

	/* prepare return value */
	size->x = width;
	size->y = height;
}

/*************************************************************************/
/*! \brief 	display string from PROGMEM without auto linefeed 
	\param  pointer to string s
	\return void
	\date 	12.08.2009
	\Modified by D. Thompson 27/07/2012
*/
/************************************************************************/
void pgm_mugui_lcd_puts(prog_uchar* s, prog_uchar* font,mugui_uint16_t x, mugui_uint16_t y)
{
	mugui_uint8_t distance  = 1;			// Distance between characters
	mugui_uint16_t xpos     = 0;			// Relative xpos of character

	while(pgm_read_byte(s) != 0x00) 
	{
		xpos += mugui_lcd_putc(pgm_read_byte(s++), font, x + xpos, y) + distance;
	}
}

/*************************************************************************/
/*! \brief 	copy a string from PROGMEM to the print buffer
	\param  pointer to string s
	\return void
	\date 	30.07.2012
	\Created by D. Thompson
*/
/************************************************************************/
void pgm_mugui_scopy(const char *s)
{
	int i = 0;

	// Clear buffer first
	for (i = 0; i < 16; i++)
	{
		pBuffer[i] = 0x00;
	}

	i = 0;
	while(pgm_read_byte(s) != 0x00) 
	{
		pBuffer[i] = pgm_read_byte(s++);
		i++;
	}
}

/*************************************************************************/
/*! \brief 	display string without auto linefeed 
	\param  pointer to string s
	\return void
	\date 	12.08.2009
*/
/************************************************************************/
void mugui_lcd_puts(mugui_char_t *s, prog_uchar* font,mugui_uint16_t x, mugui_uint16_t y)
{
	mugui_uint8_t distance  = 1;			//distance between characters
	mugui_uint16_t xpos     = 0;			//relativ xpos of character
	mugui_uint16_t length 	= 0;			//temporary length of string
	mugui_uint16_t i 		= 0;			//counter variable
	
	length = strlen(s);
	for(i=0;i<length;i++)
	{
		xpos += mugui_lcd_putc(s[i], font, x + xpos, y) + distance;
	}
}

/*************************************************************************/
/*! \brief  display character for monospace and proportional fonts 
	\param  character c
	\param  pointer to font
	\param  x position (upper left corner)
	\param  y position (upper left corner)
	\return character width
	\date 	13.11.2009
	\Modified by D. Thompson 14/08/2012 - Now hard-coded for proportional, type 2 (verticalCeiling)
*/
/************************************************************************/
mugui_uint16_t mugui_lcd_putc(mugui_char_t c, prog_uchar* font,mugui_uint16_t x, mugui_uint16_t y)
{
	mugui_uint16_t startcharacter = 0; 		//startcharacter of the font
	mugui_uint16_t height = 0;				//height of the bitmap
	mugui_uint16_t width = 0; 				//width of the bitmap
	mugui_uint8_t  index = 0; 				//index of the bitmap
	mugui_uint8_t  indexlowbyte = 0; 		//low byte of the bitmap address in the array
	mugui_uint8_t  indexhighbyte = 0; 		//high byte of the bitmap address in the array
	mugui_uint32_t indexaddress = 0;		//bitmap address in the array (derived from low and high byte)
	mugui_uint16_t tx = 0;	 				//temporary x
	mugui_uint16_t ty = 0;	 				//temporary y
	mugui_uint8_t  tb= 0;     				//temporary byte
	mugui_uint8_t  data= 0;					//databyte
	mugui_uint8_t  mask= 0;					//bitmask
	mugui_uint8_t  bit= 0;					//bit value
	mugui_uint8_t  tc= 0;	 				//temorary count
	mugui_uint8_t  bytes= 0;  				//bytes per line or row

	/* Read header of the font          */
	/* pgm_read_byte is ATMega specific */
	startcharacter = pgm_read_byte(&font[2]);
	height = pgm_read_byte(&font[4]);

	/* Read the rest of the header */
	index = c - startcharacter;
	indexhighbyte = pgm_read_byte(&font[index*2 + 5]);
	indexlowbyte = pgm_read_byte(&font[index*2 + 6]);
	indexaddress = (mugui_int64_t)indexhighbyte;
	indexaddress = indexaddress << 8;
	indexaddress += indexlowbyte;
	width = pgm_read_byte(&font[indexaddress]);

	/* Determine the number of bytes for given width */ 
	bytes = ((height-1)>>3)+1;
	/* For every column */
	for(tx= 0; tx < width; tx++) //for every row
	{
		ty = 0;
		/* For every byte */
		for(tb = 0; tb < bytes; tb ++)
		{
			/* Read bytes from program memory - ATMega specific */
			data = pgm_read_byte(&font[indexaddress + 1 + bytes*tx + tb]);
			/* For every bit within the height */
			for(tc = 0;  ( (tc < 8) && (ty < height) ); tc ++)
			{
					/* Determine the bit mask */
					mask = 1<<(tc);
					bit = data & mask;
					if(bit)
					{
						setpixel(buffer,tx+x,ty+y,1);
					}
					else
					{
						setpixel(buffer,tx+x,ty+y,0);
					}
					ty++;
			}
		}
	}

	return width;
}
