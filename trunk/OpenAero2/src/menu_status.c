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
	LCD_Display_Text(59,(prog_uchar*)Verdana8,0,33); 	// Stability
	LCD_Display_Text(60,(prog_uchar*)Verdana8,0,44); 	// Autolevel

	// Display menu and markers
	LCD_Display_Text(9, (prog_uchar*)Wingdings, 0, 59);	// Down
	LCD_Display_Text(105,(prog_uchar*)Verdana8,10,55);	// Refresh
	LCD_Display_Text(14,(prog_uchar*)Verdana8,54,55);	// Menu
	LCD_Display_Text(9, (prog_uchar*)Wingdings, 80, 59);// Down

	// Display values
	print_menu_text(0, 1, (29 + Config.RxMode), 50, 22);
	LCD_Display_Text(163,(prog_uchar*)Verdana8,50,11); 
	print_menu_text(0, 1, (33 + Config.MixMode), 36, 0);
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

	write_buffer(buffer);
}
