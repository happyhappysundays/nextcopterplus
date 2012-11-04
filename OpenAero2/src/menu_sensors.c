//***********************************************************
//* menu_sensors.c
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
			CalibrateAcc();
			CalibrateGyros();
		}

		ReadGyros();
		ReadAcc();

		LCD_Display_Text(26,(prog_uchar*)Verdana8,20,0); LCD_Display_Text(27,(prog_uchar*)Verdana8,50,0);
		LCD_Display_Text(26,(prog_uchar*)Verdana8,20,10); LCD_Display_Text(28,(prog_uchar*)Verdana8,50,10);
		LCD_Display_Text(26,(prog_uchar*)Verdana8,20,20); LCD_Display_Text(29,(prog_uchar*)Verdana8,50,20);
		LCD_Display_Text(30,(prog_uchar*)Verdana8,20,30); LCD_Display_Text(27,(prog_uchar*)Verdana8,50,30);
		LCD_Display_Text(30,(prog_uchar*)Verdana8,20,40); LCD_Display_Text(28,(prog_uchar*)Verdana8,50,40);
		LCD_Display_Text(30,(prog_uchar*)Verdana8,20,50); LCD_Display_Text(29,(prog_uchar*)Verdana8,50,50);

		mugui_lcd_puts(itoa(gyroADC[PITCH],pBuffer,10),(prog_uchar*)Verdana8,80,0);
		mugui_lcd_puts(itoa(gyroADC[ROLL],pBuffer,10),(prog_uchar*)Verdana8,80,10);
		mugui_lcd_puts(itoa(gyroADC[YAW],pBuffer,10),(prog_uchar*)Verdana8,80,20);
		mugui_lcd_puts(itoa(accADC[PITCH],pBuffer,10),(prog_uchar*)Verdana8,80,30);
		mugui_lcd_puts(itoa(accADC[ROLL],pBuffer,10),(prog_uchar*)Verdana8,80,40);
		mugui_lcd_puts(itoa(accADC[YAW],pBuffer,10),(prog_uchar*)Verdana8,80,50);

		// Print bottom markers
		LCD_Display_Text(12, (prog_uchar*)Wingdings, 0, 57); 	// Left
		LCD_Display_Text(60, (prog_uchar*)Verdana8, 108, 55); 	// Calibrate

		// Update buffer
		write_buffer(buffer,1);
		clear_buffer(buffer);
		_delay_ms(100);
	}
}
