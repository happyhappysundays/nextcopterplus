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
	int16_t	x_pos, y_pos;
	int16_t count = 0;

	#ifdef KK2Mini
	st7565_set_brightness(28);
	#endif

	while(BUTTON1 != 0)
	{
		// Read accs
		ReadAcc();

		// Refresh accSmooth values
		// Fake the IMU period as accSmooth doesn't need that
		imu_update(0);

		count++;
		
		// Only display once per 10 loops
		if (count > 10)
		{
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
			write_buffer(buffer);
			clear_buffer(buffer);
			
			count = 0;
		}
	}

	#ifdef KK2Mini
	clear_buffer(buffer);
	write_buffer(buffer);
	st7565_set_brightness(Config.Contrast);
	#endif
}
