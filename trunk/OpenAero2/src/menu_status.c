//***********************************************************
//* menu_status.c
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

//************************************************************
// Prototypes
//************************************************************

void Display_status(void);

//************************************************************
// Code
//************************************************************

int8_t	General_error;	// Global error flag

void Display_status(void)
{
	int16_t temp, min, max, range, scale;
	int8_t	pos1, pos2, pos3;
	mugui_size16_t size;

	clear_buffer(buffer);

	// Display text
	LCD_Display_Text(4,(prog_uchar*)Verdana8,0,0); 		// Mode
	LCD_Display_Text(3,(prog_uchar*)Verdana8,0,11); 	// Version text
	LCD_Display_Text(5,(prog_uchar*)Verdana8,0,22); 	// Input
	LCD_Display_Text(46,(prog_uchar*)Verdana8,0,33); 	// Stability
	LCD_Display_Text(47,(prog_uchar*)Verdana8,0,44); 	// Autolevel

	// Display menu and markers
	LCD_Display_Text(9, (prog_uchar*)Wingdings, 0, 59);	// Down
	LCD_Display_Text(14,(prog_uchar*)Verdana8,10,55);	// Menu

	// Display values
	print_menu_text(0, 1, (18 + Config.RxMode), 50, 22);
	LCD_Display_Text(0,(prog_uchar*)Verdana8,50,11); 
	print_menu_text(0, 1, (22 + Config.MixMode), 33, 0);
	print_menu_text(0, 1, (101 + Stability), 50, 44);
	print_menu_text(0, 1, (101 + AutoLevel), 50, 33);

	// Draw battery
	drawrect(buffer, 100,4, 28, 50, 1);					// Battery body
	drawrect(buffer, 110,0, 8, 4, 1);					// Battery terminal

	GetVbat();

	min = Config.MinVoltage * Config.BatteryCells;		// Calculate battery voltage limits
	max = Config.MaxVoltage * Config.BatteryCells;
	range = max - min;
	scale = range / 50;

	if (vBat >= min) 
	{
		temp =(vBat - min) / scale;
	}
	else
	{
		temp = 0;
	}
	if (temp <= 0) temp = 0;
	if (temp > 50) temp = 50;

	fillrect(buffer, 100,54-temp, 28, temp, 1);				// Battery filler (max is 60)

	// Display voltage
	uint8_t x_loc = 102;	// X location of voltage display
	uint8_t y_loc = 55;		// Y location of voltage display

	temp = vBat/100;		// Display whole decimal part first
	mugui_text_sizestring(itoa(temp,pBuffer,10), (prog_uchar*)Verdana8, &size);
	mugui_lcd_puts(itoa(temp,pBuffer,10),(prog_uchar*)Verdana8,x_loc,y_loc);
	pos1 = size.x;

	vBat = vBat - (temp * 100); // Now display the parts to the right of the decimal point

	LCD_Display_Text(7,(prog_uchar*)Verdana8,(x_loc + pos1),y_loc);
	mugui_text_sizestring(".", (prog_uchar*)Verdana8, &size);
	pos3 = size.x;
	mugui_text_sizestring("0", (prog_uchar*)Verdana8, &size);
	pos2 = size.x;

	if (vBat >= 10)
	{
		mugui_lcd_puts(itoa(vBat,pBuffer,10),(prog_uchar*)Verdana8,(x_loc + pos1 + pos3),y_loc);
	}
	else
	{
		LCD_Display_Text(8,(prog_uchar*)Verdana8,(x_loc + pos1 + pos3),y_loc);
		mugui_lcd_puts(itoa(vBat,pBuffer,10),(prog_uchar*)Verdana8,(x_loc + pos1 + pos2 + pos3),y_loc);
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
			LCD_Display_Text(82,(prog_uchar*)Verdana14,33,14); 	// Battery
			LCD_Display_Text(119,(prog_uchar*)Verdana14,46,34); // Low
		}
		else if((General_error & (1 << NO_SIGNAL)) != 0)
		{
			LCD_Display_Text(75,(prog_uchar*)Verdana14,51,13); 	// No
			LCD_Display_Text(76,(prog_uchar*)Verdana14,39,33);  // Signal
			menu_beep(3);
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
			menu_beep(6);
		}
	}

	// Write buffer to complete
	write_buffer(buffer,1);
	clear_buffer(buffer);
}
