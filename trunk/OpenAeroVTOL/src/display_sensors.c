//***********************************************************
//* display_sensors.c
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
#include "gyros.h"
#include "acc.h"
#include "menu_ext.h"

//************************************************************
// Prototypes
//************************************************************

void Display_sensors(void);

//************************************************************
// Code
//************************************************************

void Display_sensors(void)
{
	while(BUTTON1 != 0)
	{
		if (BUTTON4 == 0)
		{
			_delay_ms(500);
			CalibrateAcc(NORMAL);
			CalibrateGyrosFast();
		}

		if (BUTTON3 == 0)
		{
			_delay_ms(500);
			CalibrateAcc(REVERSED);
		}

		ReadGyros();
		ReadAcc();

		LCD_Display_Text(26,(prog_uchar*)Verdana8,37,0); 	// Gyro
		LCD_Display_Text(30,(prog_uchar*)Verdana8,77,0); 	// Acc
		LCD_Display_Text(27,(prog_uchar*)Verdana8,5,15);	// Roll
		LCD_Display_Text(28,(prog_uchar*)Verdana8,5,25);	// Pitch
		LCD_Display_Text(29,(prog_uchar*)Verdana8,5,35);	// Yaw/Z

		mugui_lcd_puts(itoa(gyroADC[ROLL],pBuffer,10),(prog_uchar*)Verdana8,40,15);
		mugui_lcd_puts(itoa(gyroADC[PITCH],pBuffer,10),(prog_uchar*)Verdana8,40,25);
		mugui_lcd_puts(itoa(gyroADC[YAW],pBuffer,10),(prog_uchar*)Verdana8,40,35);

		mugui_lcd_puts(itoa(accADC[ROLL],pBuffer,10),(prog_uchar*)Verdana8,80,15);
		mugui_lcd_puts(itoa(accADC[PITCH],pBuffer,10),(prog_uchar*)Verdana8,80,25);
		mugui_lcd_puts(itoa(accADC[YAW],pBuffer,10),(prog_uchar*)Verdana8,80,35);

		// Print bottom markers
		LCD_Display_Text(12, (prog_uchar*)Wingdings, 0, 57); 	// Left
		LCD_Display_Text(37, (prog_uchar*)Verdana8, 75, 55); 	// Inverted Calibrate
		LCD_Display_Text(60, (prog_uchar*)Verdana8, 108, 55); 	// Calibrate

		// Update buffer
		write_buffer(buffer,1);
		clear_buffer(buffer);
		_delay_ms(100);
	}
}
