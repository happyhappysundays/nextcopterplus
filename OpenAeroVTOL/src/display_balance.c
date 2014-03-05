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

#include "gyros.h"

//************************************************************
// Prototypes
//************************************************************

void Display_balance(void);

//************************************************************
// Code
//************************************************************

void Display_balance(void)
{
	uint16_t ticker_16 = 0;
	uint16_t LoopTCNT1 = 0;
	int16_t	x_pos, y_pos;

	while(BUTTON1 != 0)
	{
		// Read sensors
		ReadGyros();
		ReadAcc();

		// Time the loop for the IMU
		// ticker_16 is incremented at 2.5MHz (400ns) - max 26.2ms
		ticker_16 = (uint16_t)((uint16_t)TCNT1 - LoopTCNT1);	
		LoopTCNT1 = TCNT1;	

		// Refresh accSmooth values
		// Note that because it takes 4.096ms to refresh the whole GLCD this loop cannot run 
		// faster than 244Hz, but that's close enough to the actual loop time so that the 
		// actual Acc LPF effect is closely mirrored on the balance meter.
		getEstimatedAttitude(ticker_16); 

		// Convert acc signal to a pixel position
		x_pos = accSmooth[PITCH] + 32;
		y_pos = accSmooth[ROLL] + 64;

		if (x_pos < 0) x_pos = 0;
		if (x_pos > 64) x_pos = 64;
		if (y_pos < 0) y_pos = 0;
		if (y_pos > 128) y_pos = 128;

		// Print bottom markers
		LCD_Display_Text(12, (const unsigned char*)Wingdings, 2, 55); 	// Left

		// Draw balance meter
		drawrect(buffer, 0, 0, 128, 64, 1);		// Border
		drawrect(buffer, 54, 22, 21, 21, 1);	// Target
		drawline(buffer, 64, 8, 64, 56, 1); 	// Crosshairs
		drawline(buffer, 32, 32, 96, 32, 1); 
		fillcircle(buffer, y_pos, x_pos, 8, 1);	// Bubble

		// Refresh GLCD 
		write_buffer(buffer,1);
		clear_buffer(buffer);
	}
}
