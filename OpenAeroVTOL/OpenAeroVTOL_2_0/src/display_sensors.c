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
#ifdef DISPLAYLOG
	uint8_t i, y = 0;
	int16_t index = 0;
	data_pointer = 0;
#endif
		
	while(BUTTON1 != 0)
	{
#ifdef DISPLAYLOG
		//************************************************************
		// Display ROLL gyro data log
		//************************************************************		
		data_pointer = index;
		
		// Draw graph
		for (i = 0; i < 128; i++)
		{
			data_pointer++;
			
			y = (uint8_t)(datalog[data_pointer] + 32);
			
			if (y > 63) y = 63;
			if (y < 0) y = 0;

			setpixel(buffer, i, y, 1);
		}

		if (BUTTON3 == 0)
		{
			index += 5;
			if (index > 896) index = 896;
			data_pointer = index;

		}

		if (BUTTON2 == 0)
		{
			index -= 5;
			if (index < 0)
			{
				index = 0;
				data_pointer = 0;
			}
		}

		// Display index
		mugui_lcd_puts(itoa(index,pBuffer,10),(const unsigned char*)Verdana8,105,1);
#endif		
	
		if (BUTTON4 == 0)
		{
			// Wait until finger off button
			_delay_ms(500);
			
			// Calibrate sensors
#ifdef AIRSPEED
			CalibrateAirspeed();
#endif
			CalibrateGyrosFast();		
			CalibrateAcc(NORMAL);
		}
		
#ifndef DISPLAYLOG
		if (BUTTON3 == 0)
		{
			_delay_ms(500);
			CalibrateAcc(REVERSED);
		}

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
#endif
		
#ifdef AIRSPEED
		LCD_Display_Text(53,(const unsigned char*)Verdana8,5,45);		// Airspeed
		mugui_lcd_puts(itoa((GetAirspeed() - Config.AirspeedZero),pBuffer,10),(const unsigned char*)Verdana8,55,45);
#endif

		// Print bottom markers
		LCD_Display_Text(12, (const unsigned char*)Wingdings, 0, 57); 	// Left
		LCD_Display_Text(60, (const unsigned char*)Verdana8, 108, 55); 	// Calibrate
		
#ifdef DISPLAYLOG
		LCD_Display_Text(12, (const unsigned char*)Wingdings, 45, 55); 	// Left
		LCD_Display_Text(11, (const unsigned char*)Wingdings, 75, 55); 	// Right
#else
		LCD_Display_Text(25, (const unsigned char*)Verdana8, 75, 55); 	// Inverted Calibrate		
#endif

		// Update buffer
		write_buffer(buffer,1);
		clear_buffer(buffer);
	}
}
