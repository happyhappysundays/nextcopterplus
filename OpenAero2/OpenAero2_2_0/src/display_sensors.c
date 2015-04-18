//***********************************************************
//* display_sensors.c
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
#include "gyros.h"
#include "acc.h"
#include "menu_ext.h"
#include "i2c.h"
#include "MPU6050.h"
#include "vbat.h"
#include "imu.h"

//************************************************************
// Prototypes
//************************************************************

void Display_sensors(void);

//************************************************************
// Code
//************************************************************

void Display_sensors(void)
{
	bool	first_time = true;
		
	clear_buffer(buffer);
		
	// While BACK not pressed
	while(BUTTON1 != 0)
	{
		ReadGyros();
		ReadAcc();

		LCD_Display_Text(26,(const unsigned char*)Verdana8,37,0); 	// Gyro
		LCD_Display_Text(30,(const unsigned char*)Verdana8,77,0); 	// Acc
		//
		LCD_Display_Text(27,(const unsigned char*)Verdana8,5,13);	// Roll
		LCD_Display_Text(28,(const unsigned char*)Verdana8,5,23);	// Pitch
		LCD_Display_Text(29,(const unsigned char*)Verdana8,5,33);	// Yaw/Z
		//
		mugui_lcd_puts(itoa(gyroADC[ROLL],pBuffer,10),(const unsigned char*)Verdana8,40,13);
		mugui_lcd_puts(itoa(gyroADC[PITCH],pBuffer,10),(const unsigned char*)Verdana8,40,23);
		mugui_lcd_puts(itoa(gyroADC[YAW],pBuffer,10),(const unsigned char*)Verdana8,40,33);
		mugui_lcd_puts(itoa(accADC[ROLL],pBuffer,10),(const unsigned char*)Verdana8,80,13);
		mugui_lcd_puts(itoa(accADC[PITCH],pBuffer,10),(const unsigned char*)Verdana8,80,23);
		mugui_lcd_puts(itoa(accADC[YAW],pBuffer,10),(const unsigned char*)Verdana8,80,33);

		// Print bottom markers
		LCD_Display_Text(12, (const unsigned char*)Wingdings, 0, 57); 	// Left
		LCD_Display_Text(37, (const unsigned char*)Verdana8, 75, 55); 	// Inverted Calibrate
		LCD_Display_Text(60, (const unsigned char*)Verdana8, 108, 55); 	// Calibrate

		// Update buffer
		write_buffer(buffer);
		clear_buffer(buffer);

		if (first_time)
		{
			// Wait until finger off button
			Wait_BUTTON4();
			
			first_time = false;
		}
		
		// Normal calibrate button pressed
		if (BUTTON4 == 0)
		{
			// Wait until finger off button
			Wait_BUTTON4();
			
			// Pause until steady
			_delay_ms(250);
			
			// Calibrate sensors
			CalibrateGyrosFast();
			CalibrateAcc(NORMAL);
		}

		// Inverted calibrate button pressed
		if (BUTTON3 == 0)
		{
			// Wait until button snap dissipated
			_delay_ms(250);
			CalibrateAcc(REVERSED);
		}
	}
}
