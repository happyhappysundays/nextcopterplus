//***********************************************************
//* display_status.c
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include <avr/io.h>
#include <stdlib.h>
#include "..\inc\io_cfg.h"
#include "..\inc\glcd_driver.h"
#include "..\inc\mugui.h"
#include <avr/pgmspace.h>
#include "..\inc\glcd_menu.h"
#include "..\inc\main.h"
#include "..\inc\vbat.h"
#include <util/delay.h>
#include "..\inc\menu_ext.h"

extern uint16_t StackCount(void);	

//************************************************************
// Prototypes
//************************************************************

void Display_status(void);

//************************************************************
// Code
//************************************************************

void Display_status(void)
{
	int16_t temp, min, max, range, scale;
	uint16_t vbat_temp, free_ram;
	int8_t	pos1, pos2, pos3;
	mugui_size16_t size;

	clear_buffer(buffer);

	// Display text
	LCD_Display_Text(4,(prog_uchar*)Verdana8,0,0); 		// Mode
	LCD_Display_Text(3,(prog_uchar*)Verdana8,0,11); 	// Version text
	LCD_Display_Text(5,(prog_uchar*)Verdana8,0,22); 	// RX sync
	LCD_Display_Text(6,(prog_uchar*)Verdana8,0,33); 	// Profile
	LCD_Display_Text(133,(prog_uchar*)Verdana8,0,44); 	// Free RAM:

	// Display menu and markers
	LCD_Display_Text(9, (prog_uchar*)Wingdings, 0, 59);	// Down
	LCD_Display_Text(14,(prog_uchar*)Verdana8,10,55);	// Menu

	// Display values
	print_menu_text(0, 1, (22 + Config.MixMode), 33, 0);
	print_menu_text(0, 1, (62 + Config.RxMode), 45, 22);
	mugui_lcd_puts(itoa((Config.Flight + 1),pBuffer,10),(prog_uchar*)Verdana8,45,33);

	// Display unused RAM
	free_ram = StackCount();
	mugui_lcd_puts(itoa(free_ram,pBuffer,10),(prog_uchar*)Verdana8,52,44);

	// Draw battery
	drawrect(buffer, 100,4, 28, 50, 1);					// Battery body
	drawrect(buffer, 110,0, 8, 4, 1);					// Battery terminal

	vbat_temp = GetVbat();

	min = Config.MinVoltage * Config.BatteryCells * 4;	// Calculate battery voltage limits
	max = Config.MaxVoltage * Config.BatteryCells * 4;
	range = max - min;
	scale = range / 50;

	// Look out for that divide-by-zero... :)
	if ((vbat_temp >= min) && (scale > 0))
	{
		temp =(vbat_temp - min) / scale;
	}
	else
	{
		temp = 0;
	}

	if (temp > 50) temp = 50;

	fillrect(buffer, 100,54-temp, 28, temp, 1);				// Battery filler (max is 60)

	// Display voltage
	uint8_t x_loc = 102;	// X location of voltage display
	uint8_t y_loc = 55;		// Y location of voltage display

	temp = vbat_temp/100;	// Display whole decimal part first
	mugui_text_sizestring(itoa(temp,pBuffer,10), (prog_uchar*)Verdana8, &size);
	mugui_lcd_puts(itoa(temp,pBuffer,10),(prog_uchar*)Verdana8,x_loc,y_loc);
	pos1 = size.x;

	vbat_temp = vbat_temp - (temp * 100); // Now display the parts to the right of the decimal point

	LCD_Display_Text(7,(prog_uchar*)Verdana8,(x_loc + pos1),y_loc);
	mugui_text_sizestring(".", (prog_uchar*)Verdana8, &size);
	pos3 = size.x;
	mugui_text_sizestring("0", (prog_uchar*)Verdana8, &size);
	pos2 = size.x;

	if (vbat_temp >= 10)
	{
		mugui_lcd_puts(itoa(vbat_temp,pBuffer,10),(prog_uchar*)Verdana8,(x_loc + pos1 + pos3),y_loc);
	}
	else
	{
		LCD_Display_Text(8,(prog_uchar*)Verdana8,(x_loc + pos1 + pos3),y_loc);
		mugui_lcd_puts(itoa(vbat_temp,pBuffer,10),(prog_uchar*)Verdana8,(x_loc + pos1 + pos2 + pos3),y_loc);
	}

	// Draw error messages, if any
	if (General_error != 0)
	{
		// Create message box
		fillrect(buffer, 14,8, 96, 48, 0);	// White box
		drawrect(buffer, 14,8, 96, 48, 1); 	// Outline

		// Prioritise error from top to bottom
		if((General_error & (1 << SENSOR_ERROR)) != 0)
		{
			LCD_Display_Text(72,(prog_uchar*)Verdana14,35,14); // Sensor
			LCD_Display_Text(98,(prog_uchar*)Verdana14,43,34); // Error
			menu_beep(9);
		}
		else if((General_error & (1 << LOW_BATT)) != 0)
		{
			LCD_Display_Text(134,(prog_uchar*)Verdana14,33,14); // Battery
			LCD_Display_Text(119,(prog_uchar*)Verdana14,46,34); // Low
		}
		else if((General_error & (1 << NO_SIGNAL)) != 0)
		{
			LCD_Display_Text(75,(prog_uchar*)Verdana14,51,13); 	// No
			LCD_Display_Text(76,(prog_uchar*)Verdana14,39,33);  // Signal
		}
		else if((General_error & (1 << LOST_MODEL)) != 0)
		{
			LCD_Display_Text(99,(prog_uchar*)Verdana14,45,14); // Lost
			LCD_Display_Text(100,(prog_uchar*)Verdana14,40,34);// Model
		}
		else if((General_error & (1 << THROTTLE_HIGH)) != 0)
		{
			LCD_Display_Text(105,(prog_uchar*)Verdana14,28,14); // Throttle
			LCD_Display_Text(120,(prog_uchar*)Verdana14,46,34);	// High
		}
	}

	// Write buffer to complete
	write_buffer(buffer);
	clear_buffer(buffer);
}
