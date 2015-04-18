//***********************************************************
//* display_status.c
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include <avr/io.h>
#include <stdlib.h>
#include "io_cfg.h"
#include "glcd_driver.h"
#include "mugui.h"
#include <avr/pgmspace.h>
#include "glcd_menu.h"
#include "main.h"
#include "vbat.h"
#include <util/delay.h>
#include "menu_ext.h"
#include "mixer.h"
#include "main.h"
#include "init.h"

//************************************************************
// Prototypes
//************************************************************

void Display_status(void);

//************************************************************
// Code
//************************************************************

void Display_status(void)
{
	int16_t temp, range, scale;
	uint16_t vbat_temp;
	int8_t	pos1, pos2, pos3;
	mugui_size16_t size;

	clear_buffer(buffer);

	// Display text
	LCD_Display_Text(4,(const unsigned char*)Verdana8,0,0); 	// Preset
	LCD_Display_Text(3,(const unsigned char*)Verdana8,0,11); 	// Version text
	LCD_Display_Text(5,(const unsigned char*)Verdana8,0,22); 	// RX sync
	LCD_Display_Text(6,(const unsigned char*)Verdana8,0,33); 	// Profile

	// Display menu and markers
	LCD_Display_Text(9, (const unsigned char*)Wingdings, 0, 59);// Down
	LCD_Display_Text(14,(const unsigned char*)Verdana8,10,55);	// Menu

	// Display values
	print_menu_text(0, 1, (22 + Config.MixMode), 45, 0);
	print_menu_text(0, 1, (48 + Config.RxMode), 45, 22);
	mugui_lcd_puts(itoa((Config.Flight + 1),pBuffer,10),(const unsigned char*)Verdana8,45,33);

	// Interrupt counter
	if (Config.RxMode == PWM)
	{
		LCD_Display_Text(18,(const unsigned char*)Verdana8,0,44); // Interrupt counter text
		mugui_lcd_puts(itoa(InterruptCount,pBuffer,10),(const unsigned char*)Verdana8,45,44); // Interrupt counter
	}

	// Draw battery
	drawrect(buffer, 100,4, 28, 50, 1);					// Battery body
	drawrect(buffer, 110,0, 8, 5, 1);					// Battery terminal

	vbat_temp = GetVbat();

	// Calculate battery voltage limits
	range = SystemVoltage - Config.PowerTriggerActual;
	scale = range / 50;

	// Look out for that divide-by-zero... :)
	if ((vbat_temp >= Config.PowerTriggerActual) && (scale > 0))
	{
		temp = (vbat_temp - Config.PowerTriggerActual) / scale;
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
	mugui_text_sizestring(itoa(temp,pBuffer,10), (const unsigned char*)Verdana8, &size);
	mugui_lcd_puts(itoa(temp,pBuffer,10),(const unsigned char*)Verdana8,x_loc,y_loc);
	pos1 = size.x;

	vbat_temp = vbat_temp - (temp * 100); // Now display the parts to the right of the decimal point

	LCD_Display_Text(7,(const unsigned char*)Verdana8,(x_loc + pos1),y_loc);
	mugui_text_sizestring(".", (const unsigned char*)Verdana8, &size);
	pos3 = size.x;
	mugui_text_sizestring("0", (const unsigned char*)Verdana8, &size);
	pos2 = size.x;

	if (vbat_temp >= 10)
	{
		mugui_lcd_puts(itoa(vbat_temp,pBuffer,10),(const unsigned char*)Verdana8,(x_loc + pos1 + pos3),y_loc);
	}
	else
	{
		LCD_Display_Text(8,(const unsigned char*)Verdana8,(x_loc + pos1 + pos3),y_loc);
		mugui_lcd_puts(itoa(vbat_temp,pBuffer,10),(const unsigned char*)Verdana8,(x_loc + pos1 + pos2 + pos3),y_loc);
	}

	// Draw error messages, if any
	if (General_error != 0)
	{
		// Create message box
		fillrect(buffer, 14,8, 96, 48, 0);	// White box
		drawrect(buffer, 14,8, 96, 48, 1); 	// Outline

		// Prioritise error from top to bottom
		if((General_error & (1 << LVA_ALARM)) != 0)
		{
			LCD_Display_Text(134,(const unsigned char*)Verdana14,33,14); // Battery
			LCD_Display_Text(119,(const unsigned char*)Verdana14,46,34); // Low
		}
		else if((General_error & (1 << NO_SIGNAL)) != 0)
		{
			LCD_Display_Text(75,(const unsigned char*)Verdana14,51,13); // No
			LCD_Display_Text(76,(const unsigned char*)Verdana14,39,33); // Signal
		}
		else if((General_error & (1 << LOST_MODEL)) != 0)
		{
			LCD_Display_Text(131,(const unsigned char*)Verdana14,45,14); // Lost
			LCD_Display_Text(132,(const unsigned char*)Verdana14,40,34);// Model
		}
		else if((General_error & (1 << THROTTLE_HIGH)) != 0)
		{
			LCD_Display_Text(105,(const unsigned char*)Verdana14,28,14); // Throttle
			LCD_Display_Text(121,(const unsigned char*)Verdana14,46,34); // High
		}
	}

	// Write buffer to complete
	write_buffer(buffer);
	clear_buffer(buffer);
}
