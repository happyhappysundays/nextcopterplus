//***********************************************************
//* menu_driver.c
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include "compiledefs.h"
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdlib.h>
#include <string.h>
#include "io_cfg.h"
#include "glcd_driver.h"
#include "mugui.h"
#include "glcd_menu.h"
#include "main.h"
#include "vbat.h"
#include "menu_ext.h"
#include "rc.h"
#include <avr/interrupt.h>
#include "mixer.h"

#define CONTRAST 160 // Contrast item number <--- This sucks... move somewhere sensible!!!!!

//************************************************************
// Prototypes
//************************************************************

// Menu frames
void print_menu_frame(uint8_t style);

// Menu management
void update_menu(uint16_t items, uint16_t start, uint16_t offset, uint8_t button, uint16_t* cursor, uint16_t* top, uint16_t* temp);
void do_menu_item(uint16_t menuitem, int8_t *values, uint8_t mult, menu_range_t range, int8_t offset, uint16_t text_link, bool servo_enable, int16_t servo_number);
void print_menu_items(uint16_t top, uint16_t start, int8_t values[], const unsigned char* menu_ranges, uint8_t rangetype, const uint16_t* MenuOffsets, const uint16_t* text_link, uint16_t cursor);
void edit_curve_item(uint8_t curve, uint8_t type);

// Misc
void menu_beep(uint8_t beeps);
uint8_t poll_buttons(bool acceleration);
void print_cursor(uint8_t line);
void draw_expo(int16_t value);
menu_range_t get_menu_range (const unsigned char* menu_ranges, uint8_t menuitem);

// Special print routine - prints either numeric or text
void print_menu_text(int16_t values, uint8_t style, uint16_t text_link, uint8_t x, uint8_t y);

// Servo driver
void output_servo_ppm_asm3(int16_t servo_number, int16_t value);

// Hard-coded line positions
const uint8_t lines[4] PROGMEM = {LINE0, LINE1, LINE2, LINE3};

// Menu globals
uint8_t button_multiplier;
uint8_t button;
uint16_t cursor = LINE0;
uint16_t menu_temp = 0;

// Defines
#define CURVESTARTE 421
#define CURVESTARTM 442

//************************************************************
// Print basic menu frame
// style = menu style
//************************************************************
void print_menu_frame(uint8_t style)
{
	LCD_Display_Text(10, (const unsigned char*)Wingdings, 38, 59); 	// Up
	LCD_Display_Text(9, (const unsigned char*)Wingdings, 80, 59); 	// Down

	switch (style)
	{
		case BASIC:
			// Print bottom markers
			LCD_Display_Text(12, (const unsigned char*)Wingdings, 0, 57); 	// Left
			LCD_Display_Text(11, (const unsigned char*)Wingdings, 120, 57); // Right
			break;
			
		case EDIT:
			// For editing items
			LCD_Display_Text(16, (const unsigned char*)Verdana8, 0, 54); 	// Def.
			LCD_Display_Text(17, (const unsigned char*)Verdana8, 103, 54);	// Save
			break;
			
		case ABORT:
			// Save or abort
			LCD_Display_Text(280, (const unsigned char*)Verdana8, 0, 54); 	// Abort
			LCD_Display_Text(17, (const unsigned char*)Verdana8, 103, 54);	// Save
			break;
			
		case LOG:
			// Clear or exit
			LCD_Display_Text(12, (const unsigned char*)Wingdings, 0, 57); 	// Left
			LCD_Display_Text(291, (const unsigned char*)Verdana8, 100, 54);	// Clear
			break;			

		case CURVE:
			// Curve edit screen
			LCD_Display_Text(12, (const unsigned char*)Wingdings, 0, 57); 	// Left
			LCD_Display_Text(11, (const unsigned char*)Wingdings, 120, 57); // Right
			break;

		case OFFSET:
			// Offset curve edit screen
			LCD_Display_Text(48, (const unsigned char*)Verdana8, 10, 54); 	// P1
			LCD_Display_Text(12, (const unsigned char*)Wingdings, 0, 57); 	// Left
			LCD_Display_Text(49, (const unsigned char*)Verdana8, 55, 54); 	// P1.n	
			LCD_Display_Text(11, (const unsigned char*)Wingdings, 120, 57); // Right
			LCD_Display_Text(50, (const unsigned char*)Verdana8, 107, 54);	// P2
			break;		
			
		default:
			break;
	}

	// Write from buffer
	write_buffer(buffer);
}

//**********************************************************************
// Print menu items primary subroutine
//
// Usage:
// top = position in sub-menu list
// start = start of sub-menu text list. (top - start) gives the offset into the list.
// values = pointer to array of values to change
// menu_ranges = pointer to array of min/max/inc/style/defaults
// rangetype = unique (0) all values are different, copied (1) all values are the same
// MenuOffsets = originally an array, now just a fixed horizontal offset for the value text
// text_link = pointer to the text list for the values if not numeric
// cursor = cursor position
//**********************************************************************

void print_menu_items(uint16_t top, uint16_t start, int8_t values[], const unsigned char* menu_ranges, uint8_t rangetype, const uint16_t* MenuOffsets, const uint16_t* text_link, uint16_t cursor)
{
	menu_range_t	range1;
	uint16_t base = 0;
	uint16_t offset = 0;
	uint16_t text = 0;
	uint8_t text_offset = 0;
		
	// Clear buffer before each update
	clear_buffer(buffer);
	print_menu_frame(BASIC);
	
	// Print each line
	for (uint8_t i = 0; i < 4; i++)
	{
		LCD_Display_Text(top+i,(const unsigned char*)Verdana8,ITEMOFFSET,(uint8_t)pgm_read_byte(&lines[i]));

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

		// Calculate location of text to display
		base = pgm_read_word(&text_link[top + i - start]);
		offset = values[top + i - start];
		text = base + offset;
		
		// Calculate horizontal offset of text to display
		text_offset = (uint8_t)pgm_read_word(&MenuOffsets[top + i - start]);
		
		
		print_menu_text((values[top+i - start]), range1.style, text, text_offset, (uint8_t)pgm_read_byte(&lines[i]));
	}

	print_cursor(cursor);	// Cursor
	write_buffer(buffer);
	poll_buttons(true);
}

//************************************************************
// get_menu_range - Get range info from PROGMEM for a specific item
//************************************************************

menu_range_t get_menu_range(const unsigned char* menu_ranges, uint8_t menuitem)
{
	menu_range_t	range;
	memcpy_P(&range, &menu_ranges[menuitem * sizeof(range)], sizeof(range));
	return (range);
}

//************************************************************
// Edit current value according to limits and increment
// menuitem = Item reference
// values = pointer to value to change
// multiplier = display/actual
// range 	= Limits of item
// offset	= Horizontal offset on screen
// text_link = Start of text list for the values if not numeric
// servo_enable = Enable real-time updating of servo position
// servo_number = Servo number to update
//************************************************************

void do_menu_item(uint16_t menuitem, int8_t *values, uint8_t mult, menu_range_t range, int8_t offset, uint16_t text_link, bool servo_enable, int16_t servo_number)
{
	mugui_size16_t size;
	int16_t temp16;
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

	// This is a loop that cycles until Button 4 is pressed (Save) or an Abort button
	// The GLCD updating slows servo updates down too much so only update the GLCD periodically
	// When not updating the GLCD the servo should be updated at 50Hz (20ms)
	while ((button != ENTER) && (button != ABORT))
	{
		// Increment loop count so that we can time various things
		display_update++;
		servo_update++;

		// Vary the button increment delay depending on the function
		if (servo_enable)
		{
			button_inc = 5; // For servos
		}
		else
		{
			button_inc = 1;	// For everything else (numbers)
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
			((display_update >= 8) && (button != NONE)) || 		// Servo value and 8 cycles passed but only with a button pressed or...
			 (first_time))										// First time into routine
		{
			display_update = 0;
			first_time = false;

			clear_buffer(buffer);

			// Print warning
			if (range.style == 4)
			{
				LCD_Display_Text(281,(const unsigned char*)Verdana8,0,0);	// Warning
				LCD_Display_Text(282,(const unsigned char*)Verdana8,25,12);
			}
			// Print title
			else
			{
				gLCDprint_Menu_P((char*)pgm_read_word(&text_menu[menuitem]), (const unsigned char*)Verdana14, 0, 0);				
			}

			// Print value
			if ((range.style == 0) || (range.style == 2) || (range.style == 3)) // numeric, numeric * 4, servo limits
			{
				// Write numeric value, centered on screen
				mugui_text_sizestring(itoa(value,pBuffer,10), (const unsigned char*)Verdana14, &size);
				mugui_lcd_puts(itoa(value,pBuffer,10),(const unsigned char*)Verdana14,((128-size.x)/2)+offset,25);
			}
			else // text (style 1 or 4)
			{
				// Write text, centered on screen
				// NB: pBuffer obviously has to be larger than the longest text string printed... duh...
				pgm_mugui_scopy((char*)pgm_read_word(&text_menu[text_link + value])); // Copy string to pBuffer

				mugui_text_sizestring((char*)pBuffer, (const unsigned char*)Verdana14, &size);
				LCD_Display_Text(text_link + value, (const unsigned char*)Verdana14,((128-size.x)/2),25);
			}

			// Print appropriate menu frame
			// Save/Abort screen
			if (range.style == 4)
			{
				// Print bottom markers
				print_menu_frame(ABORT);				
			}
			// Save/default screen
			else
			{
				// Print bottom markers
				print_menu_frame(EDIT);				
			}

			// Write from buffer
			write_buffer(buffer);
		}
	
		// Slow the loop rate more for text items (1 and 4) less for servos (3)
		switch (range.style)
		{
			case 0:
				// Loop rate = 50ms (10Hz)
				_delay_ms(100);
				break;
			case 1:
				// Loop rate = 250ms (4Hz)
				_delay_ms(250);		
				break;
			case 2:
				// Loop rate = 50ms (10Hz)
				_delay_ms(100);
				break;
			case 3:				
				// Loop rate = 20ms (50Hz)
				_delay_ms(20);		
				break;		
			case 4:
				// Loop rate = 250ms (4Hz)
				_delay_ms(250);	
				break;		
		}

		// Poll buttons when idle.
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

		// Release button lock when pressed
		// unless a servo
		if ((button != NONE) && (!servo_enable))
		{
			button_lock = false;
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

		// Handle button 1
		if (button == BACK)	
		{
			// Save/Abort screen
			if (range.style == 4)
			{
				button = ABORT;
			}
			else
			{
				value = (range.default_value * mult);				
			}
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
			st7565_set_brightness(value);
		}

		// Set servo position if required
		// Ignore if the output is marked as a motor
		if	(
				(servo_enable) &&
				(Config.Channel[servo_number].Motor_marker != MOTOR)
			)
		{
			servo_update = 0;

			temp16 = scale_percent(value);	// Convert to servo position (from %)

			// Scale motor from 2500~5000 to 1000~2000
			temp16 = ((temp16 << 2) + 5) / 10; 	// Round and convert

			cli();
			output_servo_ppm_asm3(servo_number, temp16);
			sei();
		}

	} // while ((button != ENTER) && (button != ABORT))

	// Divide value from that displayed if style = 2
	if (range.style == 2)
	{
		value = value / mult;
	}

	*values = (int8_t)value;
}

//************************************************************
// Edit current curve points according to limits and increment
// curve				= curve number to edit
// type					= CURVE or OFFSET
// Curves_menu_ranges	= pointer to list of ranges for this curve
//************************************************************

void edit_curve_item(uint8_t curve, uint8_t type)
{
	mugui_size16_t size;
	int16_t value = 0;
	uint8_t Point_ref = 0;				// Current focus
	menu_range_t range;
	int16_t Points[NUMBEROFPOINTS];
	int8_t InterPoints[NUMBEROFPOINTS];
	int8_t i = 0;
	int8_t Point_x = 0;
	int8_t Point_y = 0;
	int8_t varbox_x = 0;
	int8_t varbox_y = 0;
	int8_t chanbox_y = 0;	
	int8_t channel = THROTTLE;
	uint16_t reference = CURVESTARTE;
	
	button = NONE;

	// Set the correct text list for the selected reference
	if (Config.P1_Reference != MODEL)
	{
		reference = CURVESTARTE;
	}
	else
	{
		reference = CURVESTARTM;
	}

	// This is a loop that cycles until Button 4 is pressed (Save) or an Abort button
	while ((button != ENTER) && (button != ABORT))
	{
		// Handle offset curves differently
		if (type == OFFSET)
		{
			// Get curve point ranges
			range = get_menu_range ((const unsigned char*)Offsets_menu_ranges[curve], Point_ref);
		}
		else
		{
			range = get_menu_range ((const unsigned char*)Curves_menu_ranges[curve], Point_ref);
		}

		// Display update
		clear_buffer(buffer);
		
		// Handle offset curves differently
		if (type == OFFSET)
		{
			// Print graph frame
			print_menu_frame(OFFSET);
		}
		else
		{
			// Print graph frame
			print_menu_frame(CURVE);
		}
		
		// Print axes
		if (type == OFFSET)
		{
			drawline(buffer, 64, 0, 64, 52, 1);		// Vertical
		}
		else
		{
			drawline(buffer, 64, 0, 64, 57, 1);		// Vertical
		}
		
		drawline(buffer, 0, 29, 128, 29, 1);	// Horizontal
				
		// Get the current curve's data
		if (type == OFFSET)
		{
			Points[0] = Config.Offsets[curve].Point1;
			Points[1] = Config.Offsets[curve].Point2;
			Points[2] = Config.Offsets[curve].Point3;
			Points[3] = Config.Offsets[curve].Point4;
			Points[4] = Config.Offsets[curve].Point5;
			Points[5] = Config.Offsets[curve].Point6;
			Points[6] = Config.Offsets[curve].Point7;
			channel	= Config.Offsets[curve].channel;
		}
		else
		{
			Points[0] = Config.Curve[curve].Point1;
			Points[1] = Config.Curve[curve].Point2;
			Points[2] = Config.Curve[curve].Point3;
			Points[3] = Config.Curve[curve].Point4;
			Points[4] = Config.Curve[curve].Point5;
			Points[5] = Config.Curve[curve].Point6;
			Points[6] = Config.Curve[curve].Point7;
			channel	= Config.Curve[curve].channel;			
		}

		// Calculate and draw points
		for (i = 0; i < 7; i++)
		{
			// Interpolate points for the offset graph
			if (type == CURVE)
			{
				// Curves 0 to 100
				if (curve < 2)
				{
					InterPoints[i] = (int8_t)(54 - ((Points[i] * 50) / 100));				
				}
				// Curves -100 to -100
				else
				{
					InterPoints[i] = (int8_t)(29 - ((Points[i] * 50) / 200));			
				}
			}
			// Curves -125 to -125
			else
			{
				InterPoints[i] = (int8_t)(29 - ((Points[i] * 50) / 250));
			}

			// Draw boxes on the five points
			switch(i)
			{
				case 0:
					Point_x = 2;
					break;
				case 1:
					Point_x = 22;
					break;
				case 2:
					Point_x = 42;
					break;
				case 3:
					Point_x = 62;
					break;
				case 4:
					Point_x = 82;
					break;
				case 5:
					Point_x = 102;
					break;
				case 6:
					Point_x = 121;
					break;
			}
			
			// Black box surrounding point (vertical origin is offset by 2)
			fillrect(buffer, Point_x, InterPoints[i] - 2, 5, 5, 1);
		}

		// Draw lines between the points
		drawline(buffer, 4, InterPoints[0], 24, InterPoints[1], 1);
		drawline(buffer, 24, InterPoints[1], 44, InterPoints[2], 1);
		drawline(buffer, 44, InterPoints[2], 64, InterPoints[3], 1);
		drawline(buffer, 64, InterPoints[3], 84, InterPoints[4], 1);
		drawline(buffer, 84, InterPoints[4], 104, InterPoints[5], 1);
		drawline(buffer, 104, InterPoints[5], 123, InterPoints[6], 1);

		// Highlight the current point
		switch(Point_ref)
		{
			case 0:
			Point_x = 0;
			break;
			case 1:
			Point_x = 20;
			break;
			case 2:
			Point_x = 40;
			break;
			case 3:
			Point_x = 60;
			break;
			case 4:
			Point_x = 80;
			break;
			case 5:
			Point_x = 100;
			break;
			case 6:
			Point_x = 119;
			break;
			case 7:
			Point_x = 119;
			break;
		}
		
		// Adjust box coordinates
		Point_y = (InterPoints[Point_ref] - 4);

		// Channel numbers are highlighted differently
		if (Point_ref == 7)
		{
			pgm_mugui_scopy((char*)pgm_read_word(&text_menu[reference + Config.Curve[curve].channel]));		// Copy string to pBuffer
			mugui_text_sizestring((char*)pBuffer, (const unsigned char*)Verdana8, &size);					// Calculate size
			drawrect(buffer,(123 - size.x),(chanbox_y - 1 - size.y), (size.x + 5), (size.y + 4), 1);		// Outline
		}
		else
		{
			drawrect(buffer,Point_x,Point_y, 9, 9, 1);
		}

		// Print value of current object in a box somewhere
		if (type == CURVE)
		{
			switch(Point_ref)
			{
				case 0:
					value = Config.Curve[curve].Point1;
					break;
				case 1:
					value = Config.Curve[curve].Point2;
					break;
				case 2:
					value = Config.Curve[curve].Point3;
					break;
				case 3:
					value = Config.Curve[curve].Point4;
					break;
				case 4:
					value = Config.Curve[curve].Point5;
					break;
				case 5:
					value = Config.Curve[curve].Point6;
					break;
				case 6:
					value = Config.Curve[curve].Point7;
					break;
				case 7:
					value = Config.Curve[curve].channel;
					break;
			}
		}
		// Offsets
		else
		{
			switch(Point_ref)
			{
				case 0:
					value = Config.Offsets[curve].Point1;
					break;
				case 1:
					value = Config.Offsets[curve].Point2;
					break;
				case 2:
					value = Config.Offsets[curve].Point3;
					break;
				case 3:
					value = Config.Offsets[curve].Point4;
					break;
				case 4:
					value = Config.Offsets[curve].Point5;
					break;
				case 5:
					value = Config.Offsets[curve].Point6;
					break;
				case 6:
					value = Config.Offsets[curve].Point7;
					break;
				case 7:
					value = Config.Offsets[curve].channel;
					break;
			}
		}
		
		// Move value box when point 1 is in the way
		if (((Config.Curve[curve].Point1 < 50) && (type == CURVE)) || ((Config.Offsets[curve].Point1 < 50) && (type == OFFSET)))
		{
			varbox_y = 0;
		}
		else
		{
			varbox_y = 40;			
		}

		// Move channel box when points 6 and 7 are in the way
		if (((Config.Curve[curve].Point6 + Config.Curve[curve].Point7) < 0) && (type == CURVE))
		{
			chanbox_y = 12;
		}
		else
		{
			chanbox_y = 51;
		}
		
		// Print the graph point values
		if (Point_ref < 7)
		{
			mugui_text_sizestring(itoa(value,pBuffer,10), (const unsigned char*)Verdana8, &size);			// Get dimensions of text
			fillrect(buffer,varbox_x,varbox_y, (size.x + 5), (size.y + 4), 0);								// White box
			drawrect(buffer,varbox_x,varbox_y, (size.x + 5), (size.y + 4), 1);								// Outline
			mugui_lcd_puts(itoa(value,pBuffer,10),(const unsigned char*)Verdana8,varbox_x + 3,varbox_y + 3);// Value
		}
		
		// Print associated channel somewhere for the Generic curve
		if ((curve >= 4) && (type == CURVE))
		{
			pgm_mugui_scopy((char*)pgm_read_word(&text_menu[reference + Config.Curve[curve].channel]));		// Copy string to pBuffer
			mugui_text_sizestring((char*)pBuffer, (const unsigned char*)Verdana8, &size);					// Calculate size
			fillrect(buffer,(124 - size.x),(chanbox_y - size.y), (size.x + 3), (size.y + 2), 0);			// White box
			print_menu_text(0, 1, (reference + channel), (126 - size.x), (chanbox_y + 2 - size.y));			// Channel
		}

		// Write from buffer
		write_buffer(buffer);

		// Slow the loop rate
		_delay_ms(100);		

		// Poll buttons when idle. This updates the button multiplier
		poll_buttons(true);
	
		// Handle cursor Up/Down limits
		if (button == DOWN)
		{
			value = value - (range.increment * button_multiplier);
			
			// Limit values to set ranges
			if (value <= range.lower)
			{
				value = range.lower;
			}
		}

		if (button == UP)
		{
			value = value + (range.increment * button_multiplier);
			
			// Limit values to set ranges
			if (value >= range.upper)
			{
				value = range.upper;
			}
		}

		// Update values for next loop
		if (type == CURVE)
		{
			switch(Point_ref)
			{
				case 0:
					Config.Curve[curve].Point1 = value;
					break;
				case 1:
					Config.Curve[curve].Point2 = value;
					break;
				case 2:
					Config.Curve[curve].Point3 = value;
					break;
				case 3:
					Config.Curve[curve].Point4 = value;
					break;
				case 4:
					Config.Curve[curve].Point5 = value;
					break;
				case 5:
					Config.Curve[curve].Point6 = value;
					break;
				case 6:
					Config.Curve[curve].Point7 = value;
					break;
				case 7:
					Config.Curve[curve].channel = value;
					break;
			}
		}
		else
		{
			switch(Point_ref)
			{
				case 0:
					Config.Offsets[curve].Point1 = value;
					break;
				case 1:
					Config.Offsets[curve].Point2 = value;
					break;
				case 2:
					Config.Offsets[curve].Point3 = value;
					break;
				case 3:
					Config.Offsets[curve].Point4 = value;
					break;
				case 4:
					Config.Offsets[curve].Point5 = value;
					break;
				case 5:
					Config.Offsets[curve].Point6 = value;
					break;
				case 6:
					Config.Offsets[curve].Point7 = value;
					break;
				case 7:
					Config.Offsets[curve].channel = value;
					break;
			}
		}

		// Handle button 4
		if (button == ENTER)
		{
			// Cursor at far right
			if  (
					((Point_ref == (NUMBEROFPOINTS - 1)) && (curve < 4) && (type == CURVE)) ||
					((Point_ref == NUMBEROFPOINTS) && (curve >= 4) && (type == CURVE)) ||
					((Point_ref == (NUMBEROFPOINTS - 1)) && (type == OFFSET))
				)
			{
				button = ENTER;
			}
			// Move cursor right
			else
			{
				Point_ref++;
				button = NONE;	
			}
		}

		// Handle button 1
		if (button == BACK)
		{
			// Cursor at far left
			if (Point_ref == 0)
			{
				//button = ABORT;
				button = ENTER;
			}
			// Move cursor left
			else
			{
				Point_ref--;
				button = NONE;
			}
		}

	} // while ((button != ENTER) && (button != ABORT))
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

void update_menu(uint16_t items, uint16_t start, uint16_t offset, uint8_t button, uint16_t* cursor, uint16_t* top, uint16_t* temp)
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
	}

	if (button != NONE)	
	{
		menu_beep(1);
		_delay_ms(200);
	}

	// When cursor is at limits and button pressed
	if (*cursor == PREVLINE)								// Up				
	{
		*cursor  = LINE0;
		if (*top > start) 
		{
			*top = *top - 1;								// Shuffle list up
		}
	}
	
	if (*cursor == NEXTLINE)								// Down
	{
		*cursor  = LINE3;
		if ((*top + 3) < (uint16_t)((start + items) - 1))	// Compiler throws a warning here without the cast. top is uint16_t, start is uint8_t, items = uint8_t
		{
			*top = *top + 1;								// Shuffle list down
		}
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

void print_menu_text(int16_t values, uint8_t style, uint16_t text_link, uint8_t x, uint8_t y)
{
	if ((style == 0) || (style == 2) || (style == 3)) // Numeral
	{
		mugui_lcd_puts(itoa(values,pBuffer,10),(const unsigned char*)Verdana8,x,y);
	}
	else
	{
		LCD_Display_Text(text_link, (const unsigned char*)Verdana8,x,y);
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
		_delay_ms(5);

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

	// Check buzzer mode first
	if (Config.Buzzer == ON)
	{
		for (i=0; i < beeps; i++)
		{
			LVA = 1;
			_delay_ms(1);
			LVA = 0;
			_delay_ms(3);
		}		
	}

}

//************************************************************
// Print cursor on specified line
//************************************************************

void print_cursor(uint8_t line)
{
	LCD_Display_Text(13, (const unsigned char*)Wingdings, CURSOROFFSET, line);
}
