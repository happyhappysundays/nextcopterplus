//***********************************************************
//* display_wizard.c
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
#include "rc.h"
#include "menu_ext.h"
#include "mixer.h"
#include "eeprom.h"

//************************************************************
// Prototypes
//************************************************************

void Display_sticks(void);

//************************************************************
// Code
//************************************************************

void Display_sticks(void)
{
	int8_t	i;
	int8_t	offset;
	int8_t	temp_aileron, temp_elevator, temp_rudder;
	bool	CalibrateDone = false;
	bool	CalibrateStarted = false;

	// Save original settings in case user aborts
	temp_aileron = Config.AileronPol;
	temp_elevator = Config.ElevatorPol;
	temp_rudder = Config.RudderPol;

	// Reset to defaults - not ideal, but it works
	Config.AileronPol = NORMAL;
	Config.ElevatorPol =  NORMAL;
	Config.RudderPol = NORMAL;

	// Until exit button pressed or complete
	while((BUTTON1 != 0) && (!CalibrateDone))
	{
		// Draw first stick on the left
		offset = 0;

		// Clear screen buffer
		clear_buffer(buffer);

		// Draw graphic
		for (i = 0; i < 2; i++)
		{
			drawrect(buffer, 17 + offset, 0, 40, 40, 1);			// Box
			drawline(buffer, 38 + offset,20, 48 + offset,  3, 1); 	// Line 1
			drawline(buffer, 41 + offset,21, 56 + offset,  6, 1); 	// Line 2
			fillcircle(buffer, 38 + offset, 21, 2, 1);				// Center
			fillcircle(buffer, 51 + offset, 5, 4, 1);				// End

			// Draw second stick on the right
			offset = 52;
		}

		// Print bottom text and markers
		LCD_Display_Text(12, (const unsigned char*)Wingdings, 0, 57); 		// Left

		// If uncalibrated
		if (!CalibrateDone)
		{
			RxGetChannels();
			
			// Display warning if sticks not centered or no RC signal while not started calibrating
			if (((RCinputs[AILERON] < -2000) || (RCinputs[AILERON] > 2000)) && !CalibrateStarted)
			{
				LCD_Display_Text(135,(const unsigned char*)Verdana14,16,43); 	// "No RX signal?"
			}

			// Sticks have not moved far enough but RC being received
			else if ((RCinputs[AILERON] < 500) && (RCinputs[AILERON] > -500))
			{
				CalibrateStarted = true;
				LCD_Display_Text(136,(const unsigned char*)Verdana14,9,43); 	// "Hold as shown"
			}

			// Sticks should now be in the right position
			// Reverse wrong input channels
			else
			{
				if (RCinputs[AILERON] < 0)
				{
					Config.AileronPol = REVERSED;
				}

				if (RCinputs[ELEVATOR] < 0)
				{
					Config.ElevatorPol = REVERSED;
				}

				if (RCinputs[RUDDER] < 0)
				{
					Config.RudderPol = REVERSED;
				}

				// If all positive - done!
				if ((RCinputs[AILERON] > 0) && (RCinputs[ELEVATOR] > 0) && (RCinputs[RUDDER] > 0))
				{
					CalibrateDone = true;
				}
			}
		}

		// Update buffer
		write_buffer(buffer,1);
	}

	// Save value and return
	if (CalibrateDone)
	{
		// We are too poor to have this tiny feature in KK2.0
#ifdef KK21
		LCD_Display_Text(137,(const unsigned char*)Verdana14,40,43); 	// "Done!"
		// Update buffer
		write_buffer(buffer,1);
		clear_buffer(buffer);
		// Pause so that the "Done!" text is readable
		_delay_ms(500);
#endif			
		Save_Config_to_EEPROM();
 	}
	else
	{
		// Restore old settings if failed
		Config.AileronPol = temp_aileron;
		Config.ElevatorPol = temp_elevator;
		Config.RudderPol = temp_rudder;
	}
}


