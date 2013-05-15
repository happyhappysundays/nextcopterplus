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
#include <avr/interrupt.h>
#include "..\inc\mixer.h"

#define CONTRAST 170 // Contrast item number <--- This sucks... move somewhere sensible!!!!!

//************************************************************
// Prototypes
//************************************************************

// Menu frames
void print_menu_frame(uint8_t style);

// Menu management
void update_menu(uint8_t items, uint8_t start, uint8_t offset, uint8_t button, uint8_t* cursor, uint8_t* top, uint8_t* temp);
void do_menu_item(uint8_t menuitem, int8_t *values, uint8_t mult, menu_range_t range, int8_t offset, uint8_t text_link, bool servo_enable, int16_t servo_number);
void print_menu_items(uint8_t top, uint8_t start, int8_t values[], uint8_t mult, prog_uchar* menu_ranges, uint8_t rangetype, uint8_t MenuOffsets, prog_uchar* text_link, uint8_t cursor);

// Misc
void menu_beep(uint8_t beeps);
uint8_t poll_buttons(bool acceleration);
void print_cursor(uint8_t line);
void draw_expo(int16_t value);
menu_range_t get_menu_range (prog_uchar* menu_ranges, uint8_t menuitem);

// Special print routine - prints either numeric or text
void print_menu_text(int16_t values, uint8_t style, uint8_t text_link, uint8_t x, uint8_t y);

// Servo driver
void output_servo_ppm_asm3(int16_t servo_number, int16_t value);

// Hard-coded line positions
uint8_t lines[4] PROGMEM = {LINE0, LINE1, LINE2, LINE3};

// Menu globals
uint8_t button_multiplier;
uint8_t button;
uint8_t cursor = LINE0;
uint8_t menu_temp = 0;

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
	write_buffer(buffer);
}

//**********************************************************************
// Print menu items primary subroutine
//
// Usage:
// top = position in submenu list
// start = start of submenu text list. (top - start) gives the offset into the list.
// values = pointer to array of values to change
// multiplier = display/actual if type = 2, otherwise defaults to 1
// menu_ranges = pointer to array of min/max/inc/style/defaults
// rangetype = unique (0) all values are different, copied (1) all values are the same
// MenuOffsets = originally an array, now just a fixed horizontal offset for the value text
// text_link = pointer to the text list for the values if not numeric
// cursor = cursor position
//**********************************************************************
void print_menu_items(uint8_t top, uint8_t start, int8_t values[], uint8_t mult, prog_uchar* menu_ranges, uint8_t rangetype, uint8_t MenuOffsets, prog_uchar* text_link, uint8_t cursor)
{
	menu_range_t	range1;
	uint8_t multiplier;
		
	// Clear buffer before each update
	clear_buffer(buffer);
	print_menu_frame(0);
	
	// Print each line
	for (uint8_t i = 0; i < 4; i++)
	{
		LCD_Display_Text(top+i,(prog_uchar*)Verdana8,ITEMOFFSET,(uint8_t)pgm_read_byte(&lines[i]));

		// Handle unique or copied ranges (to reduce space)
		if (rangetype == 0)
		{
			// Use each unique entry
			memcpy_P(&range1, &menu_ranges[(top+i - start)* sizeof(range1)], sizeof(range1));
		}
		else
		{
			// Use just the first entry in array for all 
			memcpy_P(&range1, &menu_ranges[0], sizeof(range1));
		}
	
		if (range1.style == 2)
		{
			multiplier = mult;
		}
		else
		{
			multiplier = 1;
		}

		print_menu_text((values[top+i - start] * multiplier), range1.style, (pgm_read_byte(&text_link[top+i - start]) + values[top+i - start]), MenuOffsets, (uint8_t)pgm_read_byte(&lines[i]));
	}

	print_cursor(cursor);	// Cursor
	write_buffer(buffer);
	poll_buttons(true);
}

//************************************************************
// get_menu_range - Get range info from PROGMEM for a specific item
//************************************************************

menu_range_t get_menu_range(prog_uchar* menu_ranges, uint8_t menuitem)
{
	menu_range_t	range;
	memcpy_P(&range, &menu_ranges[menuitem * sizeof(range)], sizeof(range));
	return (range);
}

//************************************************************
// Edit curent value according to limits and increment
// menuitem = Item reference
// values = pointer to value to change
// multiplier = display/actual
// range 	= Limits of item
// offset	= Horizontal offset on screen
// text_link = Start of text list for the values if not numeric
// servo_enable = Enable real-time updating of servo position
// servo_number = Servo number to update
//************************************************************

void do_menu_item(uint8_t menuitem, int8_t *values, uint8_t mult, menu_range_t range, int8_t offset, uint8_t text_link, bool servo_enable, int16_t servo_number)
{
	mugui_size16_t size;
	int16_t temp16;
	int8_t i;
	int16_t value = (int8_t)*values;
	uint8_t display_update = 0;
	uint8_t servo_update = 0;
	uint8_t button_update = 0;
	uint8_t button_inc = 0;
	bool	button_lock = false;
	bool	first_time = true;

	// Multiply value for display only if style is 2
	if (range.style == 2)
	{
		value = value * mult;
	}
	else mult = 1;

	button = NONE;

	// Reset servo to neutral unless it is for the throttle channel in CPPM mode
	if (servo_enable && !((Config.Channel[servo_number].source_a == THROTTLE) && (Config.RxMode == CPPM_MODE)))
	{
		temp16 = Config.Limits[servo_number].trim;
		temp16 = ((temp16 << 2) / 10); 		// Span back to what the output wants

		// Give servos time to settle
		for (i = 0; i < 25; i++)
		{
			cli();
			output_servo_ppm_asm3(servo_number, temp16);
			sei();

			_delay_ms(10);
		}
	}

	// This is a loop that cycles until Button 4 is pressed (Save)
	// The GLCD updating slows servo updates down too much so only update the GLCD periodically
	// When not updating the GLCD the servo should be updated at 50Hz (20ms)
	while (button != ENTER)
	{
		// Increment loopcount so that we can time various things
		display_update++;
		servo_update++;

		// Vary the button increment delay depending on the function
		if (servo_enable)
		{
			button_inc = 20; // For servos

		}
		else
		{
			button_inc = 1;	// For everything else
		}

		// Increment button timer when pressed
		if (button != NONE)
		{
			button_update++;

			// Release button lock after button_inc loops
			if (button_update > button_inc)
			{
				button_lock = false;
				button_update = 0;
			} 
		}
		// Remove lock when not pressed
		else 
		{
			button_update = 0;
			button_lock = false;
		}

		// Display update
		if 	(!servo_enable || 									// Non-servo value or
			((display_update >= 32) && (button != NONE)) || 	// Servo value and 32 cycles passed but only with a button pressed or...
			 (first_time))										// First time into routine
		{
			display_update = 0;
			first_time = false;

			clear_buffer(buffer);

			// Print title
			gLCDprint_Menu_P((char*)pgm_read_word(&text_menu[menuitem]), (prog_uchar*)Verdana14, 0, 0);

			// Print value
			if ((range.style == 0) || (range.style == 2)) // numeric and numeric * 4
			{
				// Write numeric value, centered on screen
				mugui_text_sizestring(itoa(value,pBuffer,10), (prog_uchar*)Verdana14, &size);
				mugui_lcd_puts(itoa(value,pBuffer,10),(prog_uchar*)Verdana14,((128-size.x)/2)+offset,25);
			}
			else // text
			{
				// Write text, centered on screen
				pgm_mugui_scopy((char*)pgm_read_word(&text_menu[text_link + value])); // Copy string to pBuffer

				mugui_text_sizestring((char*)pBuffer, (prog_uchar*)Verdana14, &size);
				LCD_Display_Text(text_link + value, (prog_uchar*)Verdana14,((128-size.x)/2),25);
			}

			// Print bottom markers
			print_menu_frame(1);

			// Write from buffer
			write_buffer(buffer);
		}

		// Poll buttons when idle
		// Don't use button acceleration when moving servos
		// And don't block the code with poll_buttons()
		if (servo_enable)
		{
			button = (PINB & 0xf0);	
			button_multiplier = 1;
		}
		else
		{
			poll_buttons(true);
		}

		// Handle cursor Up/Down limits
		if (button == DOWN)
		{
			if (button_lock == false)
			{
				button_lock = true;
				value = value - (range.increment * button_multiplier);
				button_update = 0;
			}
		}

		if (button == UP)
		{
			if (button_lock == false)
			{
				button_lock = true;
				value = value + (range.increment * button_multiplier);
				button_update = 0;
			}
		}

		if (button == BACK)	
		{
			value = (range.default_value * mult);
		}

		// Limit values to set ranges
		if (value < (range.lower * mult)) 
		{
			value = range.lower * mult;
		}
		
		if (value > (range.upper * mult)) 
		{
			value = range.upper * mult;
		}

		// Update contrast setting
		if (menuitem == CONTRAST)
		{
			st7565_set_brightness(value); // debug
		}

		// Set servo position if required and update every 4 * 5ms = 20ms
		if ((servo_enable) && (servo_update >= 4))
		{
			servo_update = 0;

			temp16 = scale_percent(value);	// Convert to servo position (from %)
			temp16 = ((temp16 << 2) / 10); 	// Span back to what the output wants

			cli();
			output_servo_ppm_asm3(servo_number, temp16);
			sei();
		}

		// Loop rate = 5ms (200Hz)
		_delay_ms(5);

	} // while (button != ENTER)


	// Exit
	button = ENTER;

	// Divide value from that displayed if style = 2
	if (range.style == 2)
	{
		value = value / mult;
	}

	*values = (int8_t)value;
}

//************************************************************
// Update menu list, cursor, calculate selected item
// items	= Total number of menu items in list
// start	= Text list start position
// offset	= Offset into special lists
// button	= Current button state/value
// cursor* 	= Location of cursor
// top*		= Item number currently on top line
// temp*	= Currently selected item number
//************************************************************

void update_menu(uint8_t items, uint8_t start, uint8_t offset, uint8_t button, uint8_t* cursor, uint8_t* top, uint8_t* temp)
{
	// Temporarily add in offset :(
	*top = *top + offset;
	start = start + offset;

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
	if (*cursor == PREVLINE)								// Up				
	{
		*cursor  = LINE0;
		if (*top > start) *top = *top - 1;					// Shuffle list up
	}
	if (*cursor == NEXTLINE)								// Down
	{
		*cursor  = LINE3;
		if ((*top+3) < ((start + items)-1)) *top = *top + 1;// Shuffle list down
	}

	// Remove temporary offset
	*top = *top - offset;
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
	if ((style == 0) || (style == 2)) // Numeral
	{
		mugui_lcd_puts(itoa(values,pBuffer,10),(prog_uchar*)Verdana8,x,y);
	}
	else if (style == 1) // Text
	{
		LCD_Display_Text(text_link, (prog_uchar*)Verdana8,x,y);
	}
}

//************************************************************
// Poll buttons, wait until something pressed, debounce and 
// return button info.
//************************************************************

uint8_t poll_buttons(bool acceleration)
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

	// Check for buttons being held down if requested
	if ((button != NONE) && (acceleration))
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
