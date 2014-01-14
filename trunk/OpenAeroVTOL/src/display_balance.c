//***********************************************************
//* display_balance.c
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include "compiledefs.h"
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
#include "adc.h"
#include "imu.h"

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


		// Refresh accSmooth values
		// Note that because it takes 4.096ms to refresh the whole GLCD this loop cannot run 
		// faster than 244Hz, but that's close enough to the actual loop time so that the 
		// actual Acc LPF effect is closely mirrored on the balance meter.
		getEstimatedAttitude(); 

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

		// We need to reverse the polarity reversal so that the meter is once again
		// related to the KK2.0, not the model.
		// For some reason, pitch has to be reversed on he KK2.1
#ifdef KK21
		x_pos = ((int8_t)pgm_read_byte(&Acc_Pol[Config.Orientation][pitch_axis]) * -accSmooth[pitch_axis]) + 32;
#else
		x_pos = ((int8_t)pgm_read_byte(&Acc_Pol[Config.Orientation][pitch_axis]) * accSmooth[pitch_axis]) + 32;
#endif
		y_pos = ((int8_t)pgm_read_byte(&Acc_Pol[Config.Orientation][roll_axis]) * accSmooth[roll_axis]) + 64;

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

		// Debug - show IMU angles
		//mugui_lcd_puts(itoa(angle[ROLL],pBuffer,10),(prog_uchar*)Verdana8,10,15);
		//mugui_lcd_puts(itoa(angle[PITCH],pBuffer,10),(prog_uchar*)Verdana8,10,25);

		// Refresh GLCD 
		write_buffer(buffer,1);
		clear_buffer(buffer);
	}
}
