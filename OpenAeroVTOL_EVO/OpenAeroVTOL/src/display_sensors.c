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
#include "rc.h"

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
	uint16_t Save_TCNT1 = 0;
	uint16_t ticker_16 = 0;
	uint32_t interval = 0;			// IMU interval	
	
	clear_buffer(buffer);
	
	// While BACK not pressed
	while(BUTTON1 != 0)
	{
		RxGetChannels();						// Check state of transition switch
		UpdateTransition();						// Update the transition variable

		ReadGyros();
		ReadAcc();

		//************************************************************
		//* Update IMU
		// TMR1 is a 16-bit counter that counts at 2.5MHz. Max interval is 26.2ms
		// TMR0 is an 8-bit counter that counts at 19.531kHz. Max interval is 13.1ms
		// These two are concatenated to create a virtual timer that can measure up to 
		// 256 x 26.2ms = 6.7072s at which point the "period" is 16,768,000, a 24-bit number
		//************************************************************
		
		// Safely get current value of TCNT1
		Save_TCNT1 = TIM16_ReadTCNT1();

		// Reset Timer0 count
		TCNT0 = 0;

		// Handle TCNT1 overflow correctly - this actually seems necessary...
		// ticker_16 will hold the most recent amount measured by TCNT1
		// Timer1 (16bit) - run @ 2.5MHz (400ns) - max 26.2ms
		if (Save_TCNT1 < LoopStartTCNT1)
		{
			ticker_16 = (65536 - LoopStartTCNT1) + Save_TCNT1;
		}
		else
		{
			ticker_16 = (Save_TCNT1 - LoopStartTCNT1);
		}
		
		interval = ticker_16; // uint16_t
		
		// Store old TCNT for next measurement
		LoopStartTCNT1 = Save_TCNT1;
		
		// Handle both Timer1 under- and over-run cases
		// If TMR0_counter is less than 2, ICNT1 has not overflowed
		if (TMR0_counter < 2)
		{
			interval = ticker_16; // uint16_t
		}
		
		// If TMR0_counter is 2 or more, then TCNT1 has overflowed
		// So we use chunks of TCNT0, counted during the loop interval
		// to work out the exact period.
		// Timer0 (8bit) - run @ 20MHz / 1024 = 19.531kHz or 51.2us - max 13.1ms
		else
		{
			interval = ticker_16 + (TMR0_counter * 32768);
		}

		TMR0_counter = 0;
		
		// Refresh accSmooth values and AccVert
		// Fake the IMU period as accSmooth doesn't need that
		imu_update(interval);

		LCD_Display_Text(26,(const unsigned char*)Verdana8,37,0); 	// Gyro
		LCD_Display_Text(30,(const unsigned char*)Verdana8,72,0); 	// Acc
		LCD_Display_Text(31,(const unsigned char*)Verdana8,107,0); 	// IMU
		//
		LCD_Display_Text(27,(const unsigned char*)Verdana8,5,13);	// Roll
		LCD_Display_Text(28,(const unsigned char*)Verdana8,5,23);	// Pitch
		LCD_Display_Text(29,(const unsigned char*)Verdana8,5,33);	// Yaw/Z
		//
		mugui_lcd_puts(itoa(gyroADCalt[ROLL],pBuffer,10),(const unsigned char*)Verdana8,40,13);
		mugui_lcd_puts(itoa(gyroADCalt[PITCH],pBuffer,10),(const unsigned char*)Verdana8,40,23);
		mugui_lcd_puts(itoa(gyroADCalt[YAW],pBuffer,10),(const unsigned char*)Verdana8,40,33);
		mugui_lcd_puts(itoa(accADC[ROLL],pBuffer,10),(const unsigned char*)Verdana8,75,13);
		mugui_lcd_puts(itoa(accADC[PITCH],pBuffer,10),(const unsigned char*)Verdana8,75,23);
		mugui_lcd_puts(itoa(accADC[YAW],pBuffer,10),(const unsigned char*)Verdana8,75,33);
		mugui_lcd_puts(itoa(angle[ROLL]/100,pBuffer,10),(const unsigned char*)Verdana8,107,13);
		mugui_lcd_puts(itoa(angle[PITCH]/100,pBuffer,10),(const unsigned char*)Verdana8,107,23);

		// These are very useful for debugging the AccZ calibration
		/*
		mugui_lcd_puts(itoa(Config.AccZero_P1[YAW],pBuffer,10),(const unsigned char*)Verdana8,100,13);
		mugui_lcd_puts(itoa(Config.AccZeroNormZ_P1,pBuffer,10),(const unsigned char*)Verdana8,100,23);
		mugui_lcd_puts(itoa(Config.AccZeroInvZ_P1,pBuffer,10),(const unsigned char*)Verdana8,100,33);
		mugui_lcd_puts(itoa(Config.AccZeroDiff_P1,pBuffer,10),(const unsigned char*)Verdana8,100,43);
		
		mugui_lcd_puts(itoa(Config.AccZero_P2[YAW],pBuffer,10),(const unsigned char*)Verdana8,105,13);
		mugui_lcd_puts(itoa(Config.AccZeroNormZ_P2,pBuffer,10),(const unsigned char*)Verdana8,105,23);
		mugui_lcd_puts(itoa(Config.AccZeroInvZ_P2,pBuffer,10),(const unsigned char*)Verdana8,105,33);
		mugui_lcd_puts(itoa(Config.AccZeroDiff_P2,pBuffer,10),(const unsigned char*)Verdana8,105,43);
		*/
		
		// AccVert
		LCD_Display_Text(229,(const unsigned char*)Verdana8,5,45);	// AccVert
		mugui_lcd_puts(itoa(accVert,pBuffer,10),(const unsigned char*)Verdana8,60,45);

		// Print bottom markers
		LCD_Display_Text(12, (const unsigned char*)Wingdings, 0, 57); 	// Left
		LCD_Display_Text(60, (const unsigned char*)Verdana8, 108, 55); 	// Calibrate
		LCD_Display_Text(25, (const unsigned char*)Verdana8, 75, 55); 	// Inverted Calibrate		

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
