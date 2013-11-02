//***********************************************************
//* display_sensors.c
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
#include "..\inc\gyros.h"
#include "..\inc\menu_ext.h"

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
			CalibrateGyros();
		}

		if (BUTTON3 == 0)
		{
			_delay_ms(500);
			CalibrateAcc(REVERSED);
		}

		if (BUTTON2 == 0)
		{
			_delay_ms(500);
			CalibrateGyros();
		}

		ReadGyros();
		ReadAcc();

		LCD_Display_Text(26,(prog_uchar*)Verdana8,0,0); 	// Gyro
		LCD_Display_Text(30,(prog_uchar*)Verdana8,70,0); 	// Acc
		LCD_Display_Text(27,(prog_uchar*)Verdana8,10,15);	// X
		LCD_Display_Text(28,(prog_uchar*)Verdana8,10,25);	// Y
		LCD_Display_Text(29,(prog_uchar*)Verdana8,10,35);	// Z

		mugui_lcd_puts(itoa(gyroADC[PITCH],pBuffer,10),(prog_uchar*)Verdana8,30,15);
		mugui_lcd_puts(itoa(gyroADC[ROLL],pBuffer,10),(prog_uchar*)Verdana8,30,25);
		mugui_lcd_puts(itoa(gyroADC[YAW],pBuffer,10),(prog_uchar*)Verdana8,30,35);
		mugui_lcd_puts(itoa(accADC[PITCH],pBuffer,10),(prog_uchar*)Verdana8,80,15);
		mugui_lcd_puts(itoa(accADC[ROLL],pBuffer,10),(prog_uchar*)Verdana8,80,25);
		mugui_lcd_puts(itoa(accADC[YAW],pBuffer,10),(prog_uchar*)Verdana8,80,35);

		// Print bottom markers
		LCD_Display_Text(12, (prog_uchar*)Wingdings, 0, 57); 	// Left
		LCD_Display_Text(26, (prog_uchar*)Verdana8, 32, 55); 	// Gyro
		LCD_Display_Text(37, (prog_uchar*)Verdana8, 75, 55); 	// Inverted Calibrate
		LCD_Display_Text(60, (prog_uchar*)Verdana8, 108, 55); 	// Calibrate

		// Update buffer
		write_buffer(buffer);
		clear_buffer(buffer);
		_delay_ms(100);
	}
}
