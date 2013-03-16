//***********************************************************
//* display_wizard.c
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
#include "..\inc\rc.h"
#include "..\inc\menu_ext.h"
#include "..\inc\mixer.h"
#include "..\inc\eeprom.h"

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
	int8_t	temp_aileron, temp_2ndaileron, temp_elevator, temp_rudder;
	bool	CalibrateDone = false;

	// Save original settings in case user aborts
	temp_aileron = Config.AileronPol;
	temp_2ndaileron = Config.SecAileronPol;
	temp_elevator = Config.ElevatorPol;
	temp_rudder = Config.RudderPol;

	// Reset to defaults - not ideal, but it works
	Config.AileronPol = NORMAL;
	Config.SecAileronPol = NORMAL;
	Config.ElevatorPol =  NORMAL;
	Config.RudderPol = NORMAL;

	// Until exit button pressed
	while((BUTTON1 != 0) && (!CalibrateDone))
	{
		offset = 0;

		// Clear screen buffer
		clear_buffer(buffer);

		// Draw graphic
		for (i = 0; i < 2; i++)
		{
			drawrect(buffer, 17 + offset, 0, 40, 40, 1);			// Box
			drawline(buffer, 38 + offset,20, 48 + offset,  3, 1); 	// Line 1
			drawline(buffer, 41 + offset,21, 56 + offset,  6, 1); 	// Line 2
			fillcircle(buffer, 38 + offset, 21, 2, 1);				// Centre
			fillcircle(buffer, 51 + offset, 5, 4, 1);				// End

			offset = 52;
		}

		// Print bottom text and markers
		LCD_Display_Text(12, (prog_uchar*)Wingdings, 0, 57); 		// Left

		// If uncalibrated
		if (!CalibrateDone)
		{
			RxGetChannels();

			// Display "No RX signal" if no input detected
			if(RxChannel[AILERON] == 0)
			{
				LCD_Display_Text(135,(prog_uchar*)Verdana14,14,43); 	// "No RX signal"
			}

			// Sticks have not moved far enough
			else if ((RxChannel[AILERON] > 3000) && (RxChannel[AILERON] < 4500))
			{
				LCD_Display_Text(136,(prog_uchar*)Verdana14,9,43); 		// "Hold as shown"
			}

			// Sticks should now be in the right position
			// Reverse wrong input channels
			else
			{
				if (RCinputs[AILERON] < 0)
				{
					Config.AileronPol = REVERSED;
				}

				// Only reverse 2nd aileron if set up as one
				if ((Config.FlapChan != NOCHAN) && (RCinputs[Config.FlapChan] < 0))
				{
					Config.SecAileronPol = REVERSED;
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
		_delay_ms(100);
	}

	// Save value and return
	if (CalibrateDone)
	{
		LCD_Display_Text(137,(prog_uchar*)Verdana14,40,43); 	// "Done!"
		// Update buffer
		write_buffer(buffer,1);
		clear_buffer(buffer);
		Save_Config_to_EEPROM();
		_delay_ms(500);
 	}
	else
	{
		// Restore old settings if failed
		Config.AileronPol = temp_aileron;
		Config.SecAileronPol = temp_2ndaileron;
		Config.ElevatorPol = temp_elevator;
		Config.RudderPol = temp_rudder;
	}
}


//************************************************************
// Much better version, but too big :(
//************************************************************
/*
void Display_sticks(void)
{
	int8_t	i;
	int8_t	offset;
	int8_t	test_stage = 0;
	int8_t	temp_aileron, temp_2ndaileron, temp_elevator, temp_rudder;
	bool	CalibrateDone = false;
	int16_t	temp_min, MinAileron = 0;
	int16_t	temp_max, MaxAileron = 0;


	// Save original settings in case user aborts
	temp_aileron = Config.AileronPol;
	temp_2ndaileron = Config.SecAileronPol;
	temp_elevator = Config.ElevatorPol;
	temp_rudder = Config.RudderPol;
	temp_min = Config.MinAileron;
	temp_max = Config.MaxAileron;

	// Reset to defaults - not ideal, but it works
	Config.AileronPol = NORMAL;
	Config.SecAileronPol = NORMAL;
	Config.ElevatorPol =  NORMAL;
	Config.RudderPol = NORMAL;

	// Until exit button pressed or tests done
	while((BUTTON1 != 0) && (!CalibrateDone))
	{
		offset = 0;

		// Clear screen buffer
		clear_buffer(buffer);

		// Print bottom text and markers
		LCD_Display_Text(12, (prog_uchar*)Wingdings, 0, 57); 				// Left

		// Update screen based on test mode
		switch (test_stage)
		{
			case 0:
				break;

			case 1:
				// Top right sticks
				for (i = 0; i < 2; i++)
				{
					drawrect(buffer, 17 + offset, 0, 40, 40, 1);			// Box
					drawline(buffer, 38 + offset,21, 48 + offset,  3, 1); 	// Line 1
					drawline(buffer, 39 + offset,20, 56 + offset,  6, 1); 	// Line 2
					fillcircle(buffer, 51 + offset, 5, 4, 1);				// End
					fillcircle(buffer, 38 + offset, 21, 2, 1);				// Centre
					offset = 52;
				}
				LCD_Display_Text(136,(prog_uchar*)Verdana14,9,43); 			// "Hold as shown"
				break;

			case 2:
			case 3:
			case 4:
			case 5:
				// Top left sticks
				for (i = 0; i < 2; i++)
				{
					drawrect(buffer, 17 + offset, 0, 40, 40, 1);			// Box
					drawline(buffer, 38 + offset,21, 19 + offset,  7, 1); 	// Line 1
					drawline(buffer, 39 + offset,20, 25 + offset,  3, 1); 	// Line 2
					fillcircle(buffer, 22 + offset, 5, 4, 1);				// End
					fillcircle(buffer, 38 + offset, 21, 2, 1);				// Centre
					offset = 52;
				}
				LCD_Display_Text(136,(prog_uchar*)Verdana14,9,43); 			// "Hold as shown"
				break;
			default:

				break;
		}

		// Get RC data (may reverse part-way through this setup
		RxGetChannels();

		// Process based on test mode
		switch (test_stage)
		{
			case 0:
				if(RxChannel[AILERON] == 0)
				{
					LCD_Display_Text(135,(prog_uchar*)Verdana14,14,25); 	// "No RX signal"
				}
				else
				{
					test_stage = 1; // Proceed to next stage
					menu_beep(test_stage);
				}
				break;

			case 1:
				// Sticks have not moved far enough
				if ((RxChannel[AILERON] < 3000) || (RxChannel[AILERON] > 4500))
				{
					test_stage = 2; // Proceed to next stage
					menu_beep(test_stage);
				}
				break;

			case 2:
				if (RCinputs[AILERON] < 0)
				{
					Config.AileronPol = REVERSED;
				}

				// Only reverse 2nd aileron if set up as one
				if ((Config.FlapChan != NOCHAN) && (RCinputs[Config.FlapChan] < 0))
				{
					Config.SecAileronPol = REVERSED;
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
					test_stage = 3; // Proceed to next stage
					menu_beep(test_stage);
				}				
				break;

			case 3:
				// Test max travel right
				// Measure max over 1 second
				for (i = 0; i < 10; i++)
				{
					if (RCinputs[AILERON] > MaxAileron)
					{
						MaxAileron = RCinputs[AILERON];
					}
					_delay_ms(100);
				}

				Config.MaxAileron = MaxAileron;

				test_stage = 4; // Proceed to next stage
				menu_beep(test_stage);
				break;

			case 4:
				// Test max travel left
				// Measure max over 2 seconds
				for (i = 0; i < 20; i++)
				{
					if (RCinputs[AILERON] < MinAileron)
					{
						MinAileron = RCinputs[AILERON];
					}
					_delay_ms(100);
				}

				Config.MinAileron = MinAileron;

				test_stage = 5; // Proceed to next stage
				menu_beep(test_stage);
				break;

			case 5:
				CalibrateDone = true;
				break;

			default:
				break;
		}

		// Update display
		write_buffer(buffer,1);
		_delay_ms(100);


	} //while((BUTTON1 != 0) && (!CalibrateDone))

	// Save value and return
	if (CalibrateDone)
	{
		// Clear screen buffer
		clear_buffer(buffer);

		LCD_Display_Text(137,(prog_uchar*)Verdana14,40,25); 	// "Done!"
		_delay_ms(500);

		// Update buffer
		write_buffer(buffer,1);
		clear_buffer(buffer);
		Save_Config_to_EEPROM();

		//debug
		//mugui_lcd_puts(itoa(MinAileron,pBuffer,10),(prog_uchar*)Verdana14,10,40);
		//mugui_lcd_puts(itoa(MaxAileron,pBuffer,10),(prog_uchar*)Verdana14,70,40);
		//write_buffer(buffer,1);
		//_delay_ms(3000);

 	}
	else
	{
		// Restore old settings if failed
		Config.AileronPol = temp_aileron;
		Config.SecAileronPol = temp_2ndaileron;
		Config.ElevatorPol = temp_elevator;
		Config.RudderPol = temp_rudder;
		Config.MinAileron = temp_min;
		Config.MaxAileron = temp_max;
	}
}
*/
