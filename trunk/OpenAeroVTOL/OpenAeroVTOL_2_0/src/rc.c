//***********************************************************
//* rc.c
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include <avr/io.h>
#include <stdbool.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h> 
#include <util/delay.h>
#include "typedefs.h"
#include "glcd_driver.h"
#include "glcd_menu.h"
#include "menu_ext.h"
#include "mugui.h"
#include "isr.h"
#include "init.h"
#include "io_cfg.h"
#include "servos.h"
#include "main.h"
#include "eeprom.h"
#include "mixer.h"

//************************************************************
// Prototypes
//************************************************************

void RxGetChannels(void);
void RC_Deadband(void);
void CenterSticks(void);
#ifdef KK21
void CenterRPYSticks(void);
void Erase_Trims(void);
void Transfer_Trims(void);
void ResetItermNeutral(void);
#endif
//************************************************************
// Defines
//************************************************************

#define	NOISE_THRESH	5			// Max RX noise threshold

//************************************************************
// Code
//************************************************************

int16_t RCinputs[MAX_RC_CHANNELS + 1];	// Normalised RC inputs
int16_t MonopolarThrottle;				// Monopolar throttle

// Get raw flight channel data (~2500 to 5000) and remove zero offset
// Use channel mapping for configurability
void RxGetChannels(void)
{
	static	int16_t	OldRxSum;			// Sum of all major channels
	int16_t	RxSumDiff;
	int16_t	RxSum, i;

	// Remove zero offsets
	for (i=0;i<MAX_RC_CHANNELS;i++)
	{
		RCinputs[i]	= RxChannel[i] - Config.RxChannelZeroOffset[i];
	}

	// Special handling for monopolar throttle
	MonopolarThrottle = RxChannel[THROTTLE] - Config.RxChannelZeroOffset[THROTTLE];

	// Bipolar throttle must use the nominal mid-point
	RCinputs[THROTTLE] = RxChannel[THROTTLE] - 3750;

	// Reverse primary channels as requested
	if (Config.AileronPol == REVERSED)
	{
		RCinputs[AILERON] = -RCinputs[AILERON];
	}

	if (Config.ElevatorPol == REVERSED)
	{
		RCinputs[ELEVATOR] = -RCinputs[ELEVATOR];
	}

	if (Config.RudderPol == REVERSED)
	{
		RCinputs[RUDDER] = -RCinputs[RUDDER];
	}

	// Calculate RX activity
	RxSum = RCinputs[AILERON] + RCinputs[ELEVATOR] + RCinputs[RUDDER];
	RxSumDiff = RxSum - OldRxSum;

	// Set RX activity flag if movement above noise floor or throttle above minimum
	if ((RxSumDiff > NOISE_THRESH) || (RxSumDiff < -NOISE_THRESH) || (MonopolarThrottle > THROTTLEIDLE)) 
	{
		Flight_flags |= (1 << RxActivity);
	}
	else 
	{
		Flight_flags &= ~(1 << RxActivity);
	}

#ifdef KK21
	// Handsfree reset is a bit less complex
	// Set RX activity flag if movement above current neutral
	if ((RxSumDiff > NOISE_THRESH) || (RxSumDiff < -NOISE_THRESH))
	{
		Flight_flags |= (1 << HANDSFREE);
	}
	else
	{
		Flight_flags &= ~(1 << HANDSFREE);
	}
#endif
	
	// Preset RCinputs[NOCHAN] for sanity
	RCinputs[NOCHAN] = 0;

	OldRxSum = RxSum;
}

// Center sticks on request from Menu
void CenterSticks(void)		
{
	uint8_t i, j;
	uint16_t RxChannelZeroOffset[MAX_RC_CHANNELS] = {0,0,0,0,0,0,0,0};

	// Take an average of eight readings
	// RxChannel will auto-update every RC frame (normally 46Hz or so)
	for (i=0; i<8; i++)
	{
		for (j=0; j<MAX_RC_CHANNELS; j++)
		{
			RxChannelZeroOffset[j] += RxChannel[j];
		}
		_delay_ms(100); // Wait for a new frame
	}

	for (i=0;i<MAX_RC_CHANNELS;i++)
	{
		Config.RxChannelZeroOffset[i] = ((RxChannelZeroOffset[i] + 4) >> 3); // Round and divide by 8
	}

	Save_Config_to_EEPROM();
}

#ifdef KK21

// Center RPY sticks
void CenterRPYSticks(void)
{
	// Only update P1's offsets when fully in P2
	if ((Transition_state == TRANS_P2) || (transition == 100))
	{
		// Set P1 RC offsets based on current RC minus any extra inherited from P2
		Config.RCinputsOffset[P1][ROLL] = -RCinputs[AILERON] - Config.RCinputsOffset[P2][ROLL];
		Config.RCinputsOffset[P1][PITCH] = RCinputs[ELEVATOR] - Config.RCinputsOffset[P2][PITCH];
		Config.RCinputsOffset[P1][YAW] = RCinputs[RUDDER] - Config.RCinputsOffset[P2][YAW];
	}
		
	// Only update P2's offsets when fully in P1
	if ((Transition_state == TRANS_P1) || (transition == 0))
	{
		// Set P2 RC offsets based on current RC minus any extra inherited from P1
		Config.RCinputsOffset[P2][ROLL] = -RCinputs[AILERON] - Config.RCinputsOffset[P1][ROLL];
		Config.RCinputsOffset[P2][PITCH] = RCinputs[ELEVATOR] - Config.RCinputsOffset[P1][PITCH];
		Config.RCinputsOffset[P2][YAW] = RCinputs[RUDDER] - Config.RCinputsOffset[P1][YAW];
	}
	
	// Also reset I-term neutrals
	ResetItermNeutral();
}

// Reset I-term neutral
void ResetItermNeutral(void)
{
	// Only update P1's I-term offsets when fully in P1
	if ((Transition_state == TRANS_P1) || (transition == 0))
	{
		// Set P2 I-term offsets so that RC does not wind up I-term
		Config.RC_Iterm_Offset[P1][ROLL] = -RCinputs[AILERON];
		Config.RC_Iterm_Offset[P1][PITCH] = RCinputs[ELEVATOR];
		Config.RC_Iterm_Offset[P1][YAW] = RCinputs[RUDDER];
	}
	
	// Only update P2's I-term offsets when fully in P2
	if ((Transition_state == TRANS_P2) || (transition == 100))
	{
		// Set P1 I-term offsets so that RC does not wind up I-term
		Config.RC_Iterm_Offset[P2][ROLL] = -RCinputs[AILERON];
		Config.RC_Iterm_Offset[P2][PITCH] = RCinputs[ELEVATOR];
		Config.RC_Iterm_Offset[P2][YAW] = RCinputs[RUDDER];
	}
}

void Erase_Trims(void)
{
	uint8_t i, j;
	bool	done = false;

	// Clear screen buffer
	clear_buffer(buffer);

	// Pop up warning
	LCD_Display_Text(257,(const unsigned char*)Verdana14,20,10); 	// "Erase trims"
	LCD_Display_Text(258,(const unsigned char*)Verdana8,35,33); 	// "Are you sure?"
	LCD_Display_Text(259,(const unsigned char*)Verdana8,108,55); 	// Yes
	LCD_Display_Text(75,(const unsigned char*)Verdana8,2,55); 		// No

	// Update buffer
	write_buffer(buffer,1);

	while((BUTTON1 != 0) && (!done))
	{
		// Yes button pressed
		if (BUTTON4 == 0)
		{
			for (i=0; i<NUMBEROFAXIS; i++)
			{
				for (j=0; j<FLIGHT_MODES; j++)
				{
					Config.RC_Iterm_Offset[j][i] = 0;
					Config.RCinputsOffset[j][i] = 0;
					Config.TXOffset[j][i] = 0;
				}
			}	
			
			// Save
			Save_Config_to_EEPROM();
			
			// Exit loop
			done = true;		
		}
	}
}

void Transfer_Trims(void)
{
	uint8_t i, j = 0;
	bool	done = false;
	bool	abort = false;
	int16_t Old_RCinputs[NUMBEROFAXIS];
	int16_t TX_Trim_Diff[NUMBEROFAXIS];

	// Clear screen buffer
	clear_buffer(buffer);

	// Pop up warning
	LCD_Display_Text(260,(const unsigned char*)Verdana14,15,0); 	// "Transfer TX""
	LCD_Display_Text(265,(const unsigned char*)Verdana14,45,20); 	// "trims"
	LCD_Display_Text(258,(const unsigned char*)Verdana8,35,40); 	// "Are you sure?"
	LCD_Display_Text(259,(const unsigned char*)Verdana8,108,55); 	// Yes
	LCD_Display_Text(75,(const unsigned char*)Verdana8,2,55); 		// No

	// Update buffer
	write_buffer(buffer,1);
	
	// Ask user for confirmation
	while((BUTTON1 != 0) && (!done) && (!abort))
	{
		// Yes button pressed
		if (BUTTON4 == 0)
		{
			// Note current trims
			RxGetChannels();
			
			// Save current sticks
			Old_RCinputs[ROLL] = -RCinputs[AILERON];
			Old_RCinputs[PITCH] = RCinputs[ELEVATOR];
			Old_RCinputs[YAW] = RCinputs[RUDDER];

			// Ask user to center TX trims then press button
			clear_buffer(buffer);
			LCD_Display_Text(263,(const unsigned char*)Verdana14,0,10); 	// "Center all trims"
			LCD_Display_Text(264,(const unsigned char*)Verdana8,18,33); 	// "Press OK to continue"	
			LCD_Display_Text(261,(const unsigned char*)Verdana8,110,55); 	// OK
			LCD_Display_Text(262,(const unsigned char*)Verdana8,2,55); 		// Abort
			write_buffer(buffer,1);			

			// Wait until finger off the button
			while (BUTTON4 == 0)
			{
				_delay_ms(50);
			}
			
			// Wait here for user to confirm or abort
			while ((!abort) && (BUTTON4 != 0))
			{
				// Abort button pressed
				if (BUTTON1 == 0)
				{
					abort = true;
				}	
			}	
	
			// Button pressed
			// If not aborted, process new trim
			if (!abort)
			{
				// Note changed trim
				RxGetChannels();

				// Calculate difference	
				TX_Trim_Diff[ROLL] =  RCinputs[AILERON] - Old_RCinputs[ROLL];
				TX_Trim_Diff[PITCH] = RCinputs[ELEVATOR] - Old_RCinputs[PITCH];
				TX_Trim_Diff[YAW] = RCinputs[RUDDER] - Old_RCinputs[YAW];

				// Calculate new offsets
				for (j=0; j<FLIGHT_MODES; j++)
				{
					for (i=0; i<NUMBEROFAXIS; i++)
					{
						Config.TXOffset[j][i] = Config.RCinputsOffset[j][i] + TX_Trim_Diff[i];						
					}
				}
				
				// Zero old offsets
				for (j=0; j<FLIGHT_MODES; j++)
				{
					for (i=0; i<NUMBEROFAXIS; i++)
					{
						Config.RC_Iterm_Offset[j][i] = 0;
						Config.RCinputsOffset[j][i] = 0;
					}
				}
					
				// Save
				Save_Config_to_EEPROM();

				// Exit loop
				done = true;
			}
		} // Yes loop
	} // while loop
}

#endif