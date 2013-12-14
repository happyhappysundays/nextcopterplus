//***********************************************************
//* display_balance.c
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
#include "isr.h"
#include <util/delay.h>
#include "acc.h"
#include "menu_ext.h"

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
	int8_t	roll_axis, pitch_axis;

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
			roll_axis = PITCH;
			pitch_axis = ROLL;
		}
		else
		{
			roll_axis = ROLL;
			pitch_axis = PITCH;
		}

//
//#ifdef KK21
//		x_pos = ((int8_t)pgm_read_byte(&Acc_Pol[Config.Orientation][PITCH]) * accADC[pitch_axis]) + 32;
//		y_pos = ((int8_t)pgm_read_byte(&Acc_Pol[Config.Orientation][ROLL]) * accADC[roll_axis]) + 64;	
//#else
		x_pos = ((int8_t)pgm_read_byte(&Acc_Pol[Config.Orientation][pitch_axis]) * accADC[pitch_axis]) + 32;
		y_pos = ((int8_t)pgm_read_byte(&Acc_Pol[Config.Orientation][roll_axis]) * accADC[roll_axis]) + 64;	
//#endif

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

		write_buffer(buffer,1);
		clear_buffer(buffer);
		_delay_ms(20);
	}
}
