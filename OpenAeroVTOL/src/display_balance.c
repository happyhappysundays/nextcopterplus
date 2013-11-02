//***********************************************************
//* display_balance.c
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
	int16_t	x_pos, y_pos;

	while(BUTTON1 != 0)
	{
		ReadAcc();

		// HORIZONTAL: 	Pitch = X, Roll = Y
		// UPSIDEDOWN:	Pitch = X, Roll = Y
		// AFT:			Pitch = X, Roll = Y
		// VERTICAL:	Pitch = Y, Roll = X
		// SIDEWAYS:	Pitch = Y, Roll = X

		if ((Config.Orientation == VERTICAL) || (Config.Orientation == SIDEWAYS))
		{
			x_pos = ((int8_t)pgm_read_byte(&Acc_Pol[Config.Orientation][ROLL]) * accADC[ROLL]) + 32;
			y_pos = ((int8_t)pgm_read_byte(&Acc_Pol[Config.Orientation][PITCH]) * accADC[PITCH]) + 64;	
		}
		else
		{
			x_pos = ((int8_t)pgm_read_byte(&Acc_Pol[Config.Orientation][PITCH]) * accADC[PITCH]) + 32;
			y_pos = ((int8_t)pgm_read_byte(&Acc_Pol[Config.Orientation][ROLL]) * accADC[ROLL]) + 64;
		}

		if (x_pos < 0) x_pos = 0;
		if (x_pos > 64) x_pos = 64;
		if (y_pos < 0) y_pos = 0;
		if (y_pos > 128) y_pos = 128;

		// Print bottom markers
		LCD_Display_Text(12, (prog_uchar*)Wingdings, 2, 55); 	// Left

		// Draw balance meter
		drawrect(buffer, 0, 0, 128, 64, 1);		// Border
		drawrect(buffer, 54, 22, 21, 21, 1);	// Target
		drawline(buffer, 64, 8, 64, 56, 1); 	// Crosshairs
		drawline(buffer, 32, 32, 96, 32, 1); 
		fillcircle(buffer, y_pos, x_pos, 8, 1);	// Bubble

		write_buffer(buffer);
		clear_buffer(buffer);
		_delay_ms(20);
	}
}
