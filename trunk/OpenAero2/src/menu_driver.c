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

#define CONTRAST 160 // Contrast item number <--- This sucks... move somewhere sensible!!!!!

//************************************************************
// Prototypes
//************************************************************

// Menu frames
void print_menu_frame(uint8_t style);

// Menu management
void update_menu(uint8_t items, uint8_t start, uint8_t button, uint8_t* cursor, uint8_t* top, uint8_t* temp);
uint16_t do_menu_item(uint8_t menuitem, int16_t value, menu_range_t range, int8_t offset, uint8_t text_link);
void print_menu_items(uint8_t top, uint8_t start, int8_t values[], int8_t size, prog_uchar* menu_ranges, uint8_t MenuOffsets, prog_uchar* text_link, uint8_t cursor);
void print_menu_items_16(uint8_t top, uint8_t start, int16_t values[], prog_uchar* menu_ranges, uint8_t MenuOffsets, prog_uchar* text_link, uint8_t cursor);
void print_menu_items_core(uint8_t top, uint8_t start, int16_t values[], prog_uchar* menu_ranges, uint8_t MenuOffsets, prog_uchar* text_link, uint8_t cursor);

// Misc
void menu_beep(uint8_t beeps);
uint8_t poll_buttons(void);
void print_cursor(uint8_t line);
void draw_expo(int16_t value);
uint8_t button;
menu_range_t get_menu_range (prog_uchar* menu_ranges, uint8_t menuitem);

// Special print routine - prints either numeric or text
void print_menu_text(int16_t values, uint8_t style, uint8_t text_link, uint8_t x, uint8_t y);

// Hard-coded line positions
uint8_t lines[4] = {LINE0, LINE1, LINE2, LINE3};

// Button acceleration
uint8_t button_multiplier;


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
		LCD_Display_Text(10, (prog_uchar*)Wingdings, 38, 59); 	// Up
		LCD_Display_Text(9, (prog_uchar*)Wingdings, 80, 59); 	// Down
		LCD_Display_Text(11, (prog_uchar*)Wingdings, 120, 57); 	// Right
	}
	else
	{
		LCD_Display_Text(16, (prog_uchar*)Verdana8, 0, 54); 	// Clear
		LCD_Display_Text(10, (prog_uchar*)Wingdings, 38, 59);	// Up
		LCD_Display_Text(9, (prog_uchar*)Wingdings, 80, 59);	// Down
		LCD_Display_Text(17, (prog_uchar*)Verdana8, 103, 54);	// Save
	}

	// Write from buffer
	write_buffer(buffer,1);
}

//**********************************************************************
// Print menu items primary subroutine
// Why are there two? 8-bit menu structures take up much less space
// but are obviously limited to +/-127.
// We need a 16-bit version for things like the battery menu.
// GCC doesn't allow over-driving functions like C++ so...
// print_menu_items() and print_menu_items_16() are wrappers for
// print_menu_items_core().
//
// Usage:
// top = position in submenu list
// start = start of submenu text list. (top - start) gives the offset into the list.
// values = pointer to array of values to change
// menu_ranges = pointer to array of min/max/inc/style/defaults
// MenuOffsets = originally an array, now just a fixed horizontal offset for the value text
// text_link = pointer to the text list for the values if not numeric
// cursor = cursor position
//**********************************************************************
void print_menu_items_core(uint8_t top, uint8_t start, int16_t values[], prog_uchar* menu_ranges, uint8_t MenuOffsets, prog_uchar* text_link, uint8_t cursor)
{
	menu_range_t	range1;
	
	// Clear buffer before each update
	clear_buffer(buffer);
	print_menu_frame(0);
	
	// Print each line
	for (uint8_t i = 0; i < 4; i++)
	{
		LCD_Display_Text(top+i,(prog_uchar*)Verdana8,ITEMOFFSET,lines[i]);
		memcpy_P(&range1, &menu_ranges[(top+i - start)* sizeof(range1)], sizeof(range1));
		print_menu_text(values[top+i - start], range1.style, (pgm_read_byte(&text_link[top+i - start]) + values[top+i - start]), MenuOffsets, lines[i]);
	}

	print_cursor(cursor);	// Cursor
	write_buffer(buffer,1);
	poll_buttons();
}

//*******************************************************************
// Print menu items (int8_t) for handling signed 8-bit +/-127 values
//*******************************************************************
void print_menu_items(uint8_t top, uint8_t start, int8_t values[], int8_t size, prog_uchar* menu_ranges, uint8_t MenuOffsets, prog_uchar* text_link, uint8_t cursor)
{
	int16_t big_values[size];
	int8_t	i = 0;

	for (i = 0; i < size; i++)	// Promote 8-bit structure to 16-bit ones
	{
		big_values[i] = values[i];
	}

	print_menu_items_core(top, start, &big_values[0], (prog_uchar*)menu_ranges, MenuOffsets, (prog_uchar*)text_link, cursor);
}

//**********************************************************************
// Print menu items (int16_t) for handling signed 16-bit +/-32768 values
//**********************************************************************
void print_menu_items_16(uint8_t top, uint8_t start, int16_t values[], prog_uchar* menu_ranges, uint8_t MenuOffsets, prog_uchar* text_link, uint8_t cursor)
{
	print_menu_items_core(top, start, &values[0], (prog_uchar*)menu_ranges, MenuOffsets, (prog_uchar*)text_link, cursor);
}

//************************************************************
// get_menu_range - Get raneg info from PROGMEM for a specific item
//************************************************************

// Get range from Program memory
menu_range_t get_menu_range (prog_uchar* menu_ranges, uint8_t menuitem)
{
	menu_range_t	range;
	memcpy_P(&range, &menu_ranges[menuitem * sizeof(range)], sizeof(range));
	return (range);
}

//************************************************************
// Edit curent value according to limits and increment
// menuitem = Item reference
// value	= Value of item
// range 	= Limits of item
// offset	= Horizontal offset on screen
// text_link = Start of text list for the values if not numeric
//************************************************************

uint16_t do_menu_item(uint8_t menuitem, int16_t value, menu_range_t range, int8_t offset, uint8_t text_link)
{
	mugui_size16_t size;

	button = NONE;

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
		write_buffer(buffer,1);


		// Poll buttons when idle
		poll_buttons();

		// Handle cursor Up/Down limits
		if (button == DOWN)	
		{
			value = value - (range.increment * button_multiplier);
			_delay_ms(50);
		}

		if (button == UP)	
		{
			value = value + (range.increment * button_multiplier);
			_delay_ms(50);
		}

		if (button == BACK)	// Clear value
		{
			value = range.default_value;
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
	button = ENTER;
	return value;
}

//************************************************************
// Update menu list, cursor, calculate selected item
// items	= Total number of menu items in list
// start	= Text list start position
// button	= Current button state/value
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
		menu_beep(1);
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
		menu_beep(1);
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
		menu_beep(1);
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
// values = current value of item
// style = flag to indicate if value is numeric or a text link
// text_link = index of text to display
// x = horizontal location on screen
// y = vertical location on screen
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
// Poll buttons, wait until something pressed, debounce and 
// return button info.
//************************************************************

uint8_t poll_buttons(void)
{
	static uint8_t button_count = 0;
	uint8_t buttons = 0;

	button = (PINB & 0xf0); // button is global, buttons is local

	while (button == NONE)					
	{
		buttons = (PINB & 0xf0);	
		_delay_ms(10);

		if (buttons != (PINB & 0xf0))
		{
			buttons = 0; // Buttons different
		}
		else // Buttons the same - update global
		{
			button = buttons;
		}

		// Reset button acceleration
		button_count = 0;
		button_multiplier = 1;
	}

	// Check for buttons being held down
	if (button != NONE)
	{
		// Count the number of times incremented
		button_count++; 
		if (button_count >= 10)
		{
			button_count = 0;
			button_multiplier ++;
		}
	}

	return buttons;
}

//************************************************************
// Beep required number of times
//************************************************************

void menu_beep(uint8_t beeps)
{
	uint8_t i;

	for (i=0; i < beeps; i++)
	{ 
		LVA = 1;
		_delay_ms(25);
		LVA = 0;
		_delay_ms(25);
	}
}

//************************************************************
// Print cursor on specified line
//************************************************************

void print_cursor(uint8_t line)
{
	LCD_Display_Text(13, (prog_uchar*)Wingdings, CURSOROFFSET, line);
}

//************************************************************
// Draw expo chart on pop-up window.
// uses actual expo table to draw graph.
// Really painful to maintain as it includes many hard-coded 
// values that are highly sensitive to changes in the source data.
// Just as well it's cute...
//************************************************************

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
		b.y = 52 - (expo/20);

		if (b.x > 127) b.x = 127;						// Limit graph points
		if (b.x <= 0) b.x = 0;
		if (b.y > 63) b.y = 63;
		if (b.y <= 0) b.y = 0;

		drawline(buffer, a.x, a.y, b.x, b.y, 1);		// Draw segment
		a.x =	b.x;
		a.y =	b.y;
	}
}
