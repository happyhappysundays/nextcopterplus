//***********************************************************
//* menu_driver.c
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdlib.h>
#include "..\inc\io_cfg.h"
#include "..\inc\glcd_driver.h"
#include "..\inc\mugui.h"
#include "..\inc\glcd_menu.h"
#include "..\inc\main.h"
#include "..\inc\vbat.h"
#include "..\inc\menu_ext.h"
#include "..\inc\rc.h"

#define CONTRAST 100

//************************************************************
// Prototypes
//************************************************************

// Menu frames
void print_menu_frame(uint8_t style);

// Menu management
void update_menu(uint8_t items, uint8_t start, uint8_t button, uint8_t* cursor, uint8_t* top, uint8_t* temp);
uint16_t do_menu_item(uint8_t menuitem, int16_t value, menu_range_t range, int8_t offset, uint8_t text_link);
//void print_menu_items(uint8_t top, uint8_t start, int16_t values[], const menu_range_t menu_ranges[], const uint8_t MenuOffsets[], const uint8_t text_link[], uint8_t cursor);
void print_menu_items(uint8_t top, uint8_t start, int16_t values[], const menu_range_t menu_ranges[], prog_uchar* MenuOffsets, prog_uchar* text_link, uint8_t cursor);

// Misc
void menu_beep(void);
uint8_t poll_buttons(void);
void print_cursor(uint8_t line);
void draw_expo(int16_t value);

// Special print routine - prints either numeric or text
void print_menu_text(int16_t values, uint8_t style, uint8_t text_link, uint8_t x, uint8_t y);

//************************************************************
// Print basic menu frame
// style = menu style (0 = main, 1 = sub)
//************************************************************
void print_menu_frame(uint8_t style)
{
	// Print bottom markers
	if (style == 0)
	{
		LCD_Display_Text(12, (prog_uchar*)Wingdings, 0, 57); 	// Left
		LCD_Display_Text(9, (prog_uchar*)Wingdings, 38, 59); 	// Down
		LCD_Display_Text(10, (prog_uchar*)Wingdings, 80, 59); 	// Up
		LCD_Display_Text(11, (prog_uchar*)Wingdings, 120, 57); 	// Right
	}
	else
	{
		LCD_Display_Text(16, (prog_uchar*)Verdana8, 0, 54); 	// Clear
		LCD_Display_Text(9, (prog_uchar*)Wingdings, 38, 59);	// Down
		LCD_Display_Text(10, (prog_uchar*)Wingdings, 80, 59);	// Up
		LCD_Display_Text(17, (prog_uchar*)Verdana8, 103, 54);	// Save
	}

	// Write from buffer
	write_buffer(buffer);
}

//************************************************************
// Print menu items
// 
//************************************************************
void print_menu_items(uint8_t top, uint8_t start, int16_t values[], const menu_range_t menu_ranges[], prog_uchar* MenuOffsets, prog_uchar* text_link, uint8_t cursor)
//void print_menu_items(uint8_t top, uint8_t start, int16_t values[], const menu_range_t menu_ranges[], const uint8_t MenuOffsets[], const uint8_t text_link[], uint8_t cursor)
{
	// Clear buffer before each update
	clear_buffer(buffer);

	// Print menu
	print_menu_frame(0);											// Frame
	LCD_Display_Text(top,(prog_uchar*)Verdana8,ITEMOFFSET,LINE0);	// First line
	LCD_Display_Text(top+1,(prog_uchar*)Verdana8,ITEMOFFSET,LINE1);	// Second line
	LCD_Display_Text(top+2,(prog_uchar*)Verdana8,ITEMOFFSET,LINE2);	// Third line
	LCD_Display_Text(top+3,(prog_uchar*)Verdana8,ITEMOFFSET,LINE3);	// Fourth line


	

	// Print values

	print_menu_text(values[top - start], menu_ranges[top - start].style, (pgm_read_byte(&text_link[top - start]) + values[top - start]), pgm_read_byte(&MenuOffsets[top - start]), LINE0);
	print_menu_text(values[top+1 - start], menu_ranges[top+1 - start].style, (pgm_read_byte(&text_link[top+1 - start]) + values[top+1 - start]), pgm_read_byte(&MenuOffsets[top - start+1]), LINE1);
	print_menu_text(values[top+2 - start], menu_ranges[top+2 - start].style, (pgm_read_byte(&text_link[top+2 - start]) + values[top+2 - start]), pgm_read_byte(&MenuOffsets[top - start+2]), LINE2);
	print_menu_text(values[top+3 - start], menu_ranges[top+3 - start].style, (pgm_read_byte(&text_link[top+3 - start]) + values[top+3 - start]), pgm_read_byte(&MenuOffsets[top - start+3]), LINE3);

/*	print_menu_text(values[top - start], menu_ranges[top - start].style, (text_link[top - start] + values[top - start]), MenuOffsets[top - start], LINE0);
	print_menu_text(values[top+1 - start], menu_ranges[top+1 - start].style, (text_link[top+1 - start] + values[top+1 - start]), MenuOffsets[top - start+1], LINE1);
	print_menu_text(values[top+2 - start], menu_ranges[top+2 - start].style, (text_link[top+2 - start] + values[top+2 - start]), MenuOffsets[top - start+2], LINE2);
	print_menu_text(values[top+3 - start], menu_ranges[top+3 - start].style, (text_link[top+3 - start] + values[top+3 - start]), MenuOffsets[top - start+3], LINE3);
*/
	print_cursor(cursor);												// Cursor
	write_buffer(buffer);
}

//************************************************************
// Edit curent value according to limits and increment
// button	= Total number of menu items in list
// value*	= Value of current item
// range 	= Limits of current item
//************************************************************

uint16_t do_menu_item(uint8_t menuitem, int16_t value, menu_range_t range, int8_t offset, uint8_t text_link)
{
	mugui_size16_t size;
	uint8_t button = 0;

	while (button != ENTER)
	{
		clear_buffer(buffer);

		// Print title
		gLCDprint_Menu_P((char*)pgm_read_word(&text_menu[menuitem]), (prog_uchar*)Verdana14, 0, 0);

		// Print value
		if (range.style == 0) // numeric
		{
			// Write numeric value, centered on screen
			mugui_text_sizestring(itoa(value,pBuffer,10), (prog_uchar*)Verdana22, &size);
			mugui_lcd_puts(itoa(value,pBuffer,10),(prog_uchar*)Verdana22,((128-size.x)/2)+offset,25);
		}
		else // Text
		{
			// Write text, centered on screen
			pgm_mugui_scopy((char*)pgm_read_word(&text_menu[text_link + value])); // Copy string to pBuffer

			mugui_text_sizestring((char*)pBuffer, (prog_uchar*)Verdana14, &size);
			LCD_Display_Text(text_link + value, (prog_uchar*)Verdana14,((128-size.x)/2),25);
		}

		// Draw expo graph if required
		if (offset != 0)
		{
			draw_expo(value);
		}

		// Print bottom markers
		print_menu_frame(1);

		// Write from buffer
		write_buffer(buffer);


		// Poll buttons when idle
		button = poll_buttons();
		while (button == NONE)					
		{
			button = poll_buttons();
		}

		// Handle cursor Up/Down limits
		if (button == DOWN)	
		{
			value = value - range.increment;
			_delay_ms(10);
		}

		if (button == UP)	
		{
			value = value + range.increment;
			_delay_ms(10);
		}

		if (button == BACK)	// Clear value
		{
			value = range.lower;
		}

		// Limit values to set ranges
		if (value < range.lower) value = range.lower;
		if (value > range.upper) value = range.upper;

		// Update contrast setting
		if (menuitem == CONTRAST)
		{
			st7565_set_brightness(value);
		}
	}
	return value;
}

//************************************************************
// Update menu list, cursor, calculate selected item
// items	= Total number of menu items in list
// start	= Text list start position
// cursor* 	= Location of cursor
// top*		= Item number currently on top line
// temp*	= Currently selected item number
//************************************************************

void update_menu(uint8_t items, uint8_t start, uint8_t button, uint8_t* cursor, uint8_t* top, uint8_t* temp)
{
	// Calculate which function has been requested
	if (button == ENTER)
	{
		switch(*cursor) 
		{
			case LINE0:
				*temp = *top;
				break;
			case LINE1:
				*temp = *top + 1;
				break;	
			case LINE2:
				*temp = *top + 2;
				break;
			case LINE3:
				*temp = *top + 3;
				break;
			default:
				break;
		}
		menu_beep();
		_delay_ms(200);
	}

	// Handle cursor Up/Down limits
	if (button == DOWN)	
	{
		switch(*cursor) 
		{
			case LINE0:
				if (items > 1) *cursor = LINE1;
				break;	
			case LINE1:
				if (items > 2) *cursor = LINE2;
				break;	
			case LINE2:
				if (items > 3) *cursor = LINE3;
				break;
			case LINE3:
				if (items > 4) *cursor = NEXTLINE;
				break;
			default:
				*cursor = NEXTLINE;
				break;
		}
		menu_beep();
		_delay_ms(200);
	}

	if (button == UP)	
	{
		switch(*cursor) 
		{
			case LINE3:
				*cursor = LINE2;
				break;	
			case LINE2:
				*cursor = LINE1;
				break;
			case LINE1:
				*cursor = LINE0;
				break;
			case LINE0:
				*cursor = PREVLINE;
				break;
			default:
				*cursor = PREVLINE;
				break;
		}
		menu_beep();
		_delay_ms(200);
	}

	// When cursor is at limits and button pressed
	if (*cursor == PREVLINE)						
	{
		*cursor  = LINE0;
		if (*top > start) *top = *top - 1;				// Shuffle list up
	}
	if (*cursor == NEXTLINE)
	{
		*cursor  = LINE3;
		if ((*top+3) < ((start + items)-1)) *top = *top + 1;	// Shuffle list down
	}
}

//************************************************************
// Special subroutine to print either numeric or text
//************************************************************

void print_menu_text(int16_t values, uint8_t style, uint8_t text_link, uint8_t x, uint8_t y)
{
	if (style == 0) // Numeral
	{
		mugui_lcd_puts(itoa(values,pBuffer,10),(prog_uchar*)Verdana8,x,y);
	}
	else 			// Text
	{
		LCD_Display_Text(text_link, (prog_uchar*)Verdana8,x,y);
	}
}

//************************************************************
// Misc subroutines
//************************************************************

uint8_t poll_buttons(void)
{
	uint8_t buttons;

	buttons = (PINB & 0xf0);	
	_delay_ms(10);


	if (buttons != (PINB & 0xf0))
	{
		buttons = 0;	// Debounce buttons
	}

	return buttons;
}

void menu_beep(void)
{
	LVA = 1;
	_delay_ms(25);
	LVA = 0;
}

void print_cursor(uint8_t line)
{
	LCD_Display_Text(13, (prog_uchar*)Wingdings, CURSOROFFSET, line);
}

void draw_expo(int16_t value)
{
	mugui_size16_t a, b;
	int16_t expo = 0;
	
	drawline(buffer, 127, 0, 127, 52, 1); 				// Y-axis
	drawline(buffer, 55, 52, 127, 52, 1); 				// X-axis

	a.x = 60;
	a.y = 52;
	b.x = 0;
	b.y = 0;

	for (int16_t i = 0; i < 928; i+= 50)
	{
		if (value < 5) value = 5; 						// Bodge

		expo = get_expo_value (i, (uint8_t)value-1);	// Get expo value from table

		b.x = 70 + (i/16);								// Calculate next point location
		b.y = 52 - (expo/40);

		if (b.x > 127) b.x = 127;						// Limit graph points
		if (b.x <= 0) b.x = 0;
		if (b.y > 63) b.y = 63;
		if (b.y <= 0) b.y = 0;

		drawline(buffer, a.x, a.y, b.x, b.y, 1);		// Draw segment
		a.x =	b.x;
		a.y =	b.y;
	}
}
