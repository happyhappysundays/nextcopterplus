//***********************************************************
//* menu_balance.c
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
#include "..\inc\isr.h"
#include <util/delay.h>
#include "..\inc\acc.h"
#include "..\inc\menu_ext.h"

//************************************************************
// Prototypes
//************************************************************

void Display_balance(void);

//************************************************************
// Code
//************************************************************

void Display_balance(void)
{
	while(BUTTON1 != 0)
	{
		if (BUTTON4 == 0)
		{
			_delay_ms(500);
			CalibrateAcc();
		}

		int16_t	x_pos, y_pos;

		ReadAcc();

		x_pos = accADC[PITCH] + 32;
		if (x_pos < 0) x_pos = 0;
		if (x_pos > 64) x_pos = 64;

		y_pos = 64 - accADC[ROLL];
		if (y_pos < 0) y_pos = 0;
		if (y_pos > 128) y_pos = 128;

		// Print bottom markers
		LCD_Display_Text(12, (prog_uchar*)Wingdings, 2, 55); 	// Left
		LCD_Display_Text(73, (prog_uchar*)Verdana8, 108, 53); 	// Calibrate

		// Draw balance meter
		drawrect(buffer, 0, 0, 128, 64, 1);
		drawcircle(buffer, 64, 32, 10, 1);
		drawline(buffer, 64, 8, 64, 56, 1); 
		drawline(buffer, 32, 32, 96, 32, 1); 
		fillcircle(buffer, y_pos, x_pos, 8, 1);

		write_buffer(buffer,1);
		clear_buffer(buffer);
		_delay_ms(20);
	}
	menu_beep(1);
	_delay_ms(200);
}
