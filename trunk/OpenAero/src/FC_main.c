// **************************************************************************
// OpenAero software
// =================
// Version 1.12
// Inspired by KKmulticopter
// Based on assembly code by Rolf R Bakke, and C code by Mike Barton
// OpenAero code by David Thompson
//
// Includes PID and Auto-level functions inspired by the open-sourced MultiWii project
// Compatible with KK boards fitted with X and Y accelerometers 
// on Roll/Pitch pot inputs. LCD board or GUI required for setup of PID constants.
//
// Tested only on Atmega168P boards (KK+)
//
// **************************************************************************
// * 						GNU GPL V3 notice
// **************************************************************************
// * Copyright (C) 2012 David Thompson, based on work by Rolf R Bakke,
// * Mike Barton and others
// * 
// * This program is free software: you can redistribute it and/or modify
// * it under the terms of the GNU General Public License as published by
// * the Free Software Foundation, either version 3 of the License, or
// * (at your option) any later version.
// * 
// * This program is distributed in the hope that it will be useful,
// * but WITHOUT ANY WARRANTY; without even the implied warranty of
// * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// * GNU General Public License for more details.
// * 
// * You should have received a copy of the GNU General Public License
// * along with this program.If not, see <http://www.gnu.org/licenses/>.
// * 
// * NB: Summary - all derivative code MUST be released with the source code!
// *
// **************************************************************************
//
// Config Modes (at power-up) NOTE: CLOCKWISE on most pots is ZERO
// ==============================================================
//
// Stick Centering
// ---------------
// Set Yaw Gain pot to max
// Set Tx trims to centre
// Power on
// LED flashes 3 times
// Restore gain pot
// Restart
//
// AutoTune
// --------
// Set Yaw Gain pot to min
// Start KK autotune on GUI
// Power on
// LED flashes 12-13 times
// Restore gain pot
// Restart
//
// Gyro direction reversing (non Accelerometer versions only)
// ------------------------
// Set Roll Gain pot to zero
// Power on
// LED flashes 6 times
// Move stick Up/Left for Normal, Down/Right for reverse
// eg: to set Roll Gyro Normal, move Tx Roll stick left, or to reverse it move stick right
//
// Stability mode selection
// ------------------------
// Hold Yaw stick on TX left or right
// Power on
// LED flashes 4 times
// Mode will toggle between the following:
// 		Stability switchable
// 		Stability always ON
//
// Autolevel mode selection
// ------------------------
// Hold Roll stick on TX left or right
// Power on
// LED flashes 5 times
// Mode will toggle between the following:
// 		Autolevel switchable
//		Autolevel always OFF
//
// Menu modes (direction may depend on TX manufacturer)
// ----------
// YAW RIGHT + PITCH FWD + ROLL LEFT	= Enter LCD menu
// YAW RIGHT + PITCH BACK				= Leave LCD menu
// 
// **************************************************************************
// Version History
// ===============
// V1.0a	Based on NeXtcopter V1.1d code
//			Initial code base.
// V1.01a	First release candidate
// V1.02a	Much improved servo jitter!
// V1.03a   Calibrated servos - first flown version
// V1.04a	Added ability to use pots to adjust roll/pitch/yaw P and I terms 
//			if selected fom menu
//			Roll pot 	= Roll/Pitch P-term
//			Pitch pot 	= Roll/Pitch I-term
//			Yaw pot 	= Yaw P-term (Watch out you don't leave it near either 
//						  end as that activates special modes)
// V1.05a	Reversed gyros for FWING mode
// V1.06a	Added gyro reversing via the LCD menu and VERTICAL orientation option
//			Fixed Flying Wing mode mixing
// V1.07a	Re-added pot-based gyro direction reversing
// V1.08a   Added LCD-based servo reversing, added fix for corrupted LCD backlights
//			Former throttle input now switches stability when not in CPPM mode 
// V1.09a	Added Throttle pass-through to THR in CPPM mode
//			Fixed LVA mode bug in GUI
// V1.10a	Added lost model alarm and changed GUI lockout system
//			Added configurable stability and autolevel switch modes.
//			Fixed cycle timer :)
// V1.11	Added flaperon configuration for split ailerons on M3/M4
// V1.12	New output pulse generation code. Removed odd channels.
//			Added ICP input option for CPPM mode. Three and six-channel modes.
//			Virtually no jitter now.
//			Fixed flaperon and vertical mode bugs.
//
//***********************************************************
//* To do
//***********************************************************
//
// 
//
//***********************************************************
//* Flight configurations (Servo number)
//***********************************************************

/*
Standard mode

			 X <--  THR (Throttle - CPPM mode)
             |
     M2 -----+----- Aileron
             |
             |
        M4 --+--    Elevator
             |
             M1    Rudder


Standard Flaperon mode (CPPM only)

			 X <--  THR (Throttle - CPPM mode)
             |
     M2 -----+----- M5 Flaperons
             |
             |
        M4 --+--    Elevator
             |
             M1    Rudder

Flying Wing - Assumes mixing done in the transmitter

		 	X <-- THR (Throttle - CPPM mode)
         __/^\__
        /       \
      /           \
     |______|______|
     |_____/|\_____|

        M2     M4 (Elevons/Flaperons)
       Left   Right

            M1
          Rudder
*/

//***********************************************************
//* Includes
//***********************************************************

#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <util/delay.h>
#include <string.h>
#include "..\inc\io_cfg.h"
#include "..\inc\eeprom.h"
#include "..\inc\adc.h"
#include "..\inc\rc.h"
#include "..\inc\Servos.h"
#include "..\inc\pots.h"
#include "..\inc\gyros.h"
#include "..\inc\init.h"
#include "..\inc\acc.h"
#include "..\inc\lcd.h"
#include "..\inc\uart.h"
#include "..\inc\isr.h"

//***********************************************************
//* Defines
//***********************************************************
// Sets the rate of the main loop (Normally 500Hz)
#define LOOP_RATE 500				// in Hz
#define LOOP_INTERVAL (1000000 / LOOP_RATE ) // 2000

// Defines output rate to the servo (Normally 55+Hz)
#define SERVO_RATE 60				// in Hz

// Servo travel limits
#define MAX_TRAVEL 2000				// Maximum travel allowed
#define MIN_TRAVEL 1000				// Minimum travel allowed

// Misc
#define DEAD_BAND 10				// Centre region of RC input where no activity is processed

// PID constants
#define ITERM_LIMIT_RP 250			// Max I-term sum for Roll/Pitch axis in normal modes
#define ITERM_LIMIT_YAW 500			// Max I-term sum for Yaw axis (Heading hold)
#define ITERM_LIMIT_LEVEL 250		// Max I-term sum for Roll/Pitch axis in AUTOLEVEL mode
#define PR_LIMIT 50					// Limit pitch/roll contribution of I-term to avoid saturation
#define YAW_LIMIT 50				// Limit yaw contribution to avoid saturation
#define AVGLENGTH 65				// Accelerometer filter buffer length + 1 (64 new + 1 old)

// Stick Arming
// If you cannot either arm or disarm, lower this value
#define STICKARM_POINT 200			// Defines how far stick must be moved (for 16-bit accurate RC)
#define GUI_TIMEOUT 3000			// Time after which GUI entry is impossible (3000 ~ 10 sec)

//***********************************************************
//* Code and Data variables
//***********************************************************

// Axis variables
int32_t Roll;						// Temp axis values. Seems we need 32 bits here.
int32_t Pitch;
int32_t Yaw;
int8_t  AccRollTrim;					// Normalised acc trim 
int8_t  AccPitchTrim;
int8_t  AvgIndex;
int8_t  OldIndex;
int32_t AvgRollSum;	
int32_t AvgPitchSum;
int16_t AvgRoll;
int16_t AvgPitch;
int16_t AvgAccRoll[AVGLENGTH];		// Circular buffer of historical accelerometer roll readings
int16_t AvgAccPitch[AVGLENGTH];		// Circular buffer of historical accelerometer pitch readings

// PID variables
int32_t	IntegralgPitch;				// PID I-terms (gyro) for each axis
int32_t	IntegralgRoll;
int32_t	IntegralaPitch;				// PID I-terms (acc.) for each axis
int32_t	IntegralaRoll;
int32_t	IntegralYaw;
int32_t P_term_gRoll;				// Calculated P-terms (gyro) for each axis
int32_t P_term_gPitch;
int32_t P_term_aRoll;				// Calculated P-terms (acc.) for each axis
int32_t P_term_aPitch;
int32_t I_term_gRoll;				// Calculated I-terms (gyro) for each axis
int32_t I_term_gPitch;
int32_t I_term_aRoll;				// Calculated I-terms (acc.) for each axis
int32_t I_term_aPitch;
int32_t I_term_Yaw;
int16_t currentError[4];			// Used with lastError to keep track of D-Terms in PID calculations
int16_t lastError[4];
int16_t DifferentialGyro;			// Holds difference between last two errors (angular acceleration)

// GUI variables
bool 	GUIconnected;				// Set when ever GUI activated (suppresses servos)
bool 	GUI_lockout;				// Lockout the GUI after GUI_TIMEOUT
uint8_t flight_mode;				// Global flight mode flag
uint16_t cycletime;					// Loop time in microseconds

// Misc
bool 	AutoLevel;					// AutoLevel = 1
bool 	LCD_active;					// Mode flags
bool 	freshmenuvalue;
bool 	Model_lost;					// Model lost flag
bool 	LMA_Alarm;					// Lost model alarm active
bool 	LVA_Alarm;					// Low voltage alarm active
bool 	firsttimeflag;
char 	pBuffer[16];				// Print buffer
uint16_t Change_LCD;				// Long timers for LCD entry and LMA
uint32_t Change_LostModel;
uint8_t LCD_TCNT2;
uint8_t Lost_TCNT2;
uint8_t rxin;
uint8_t MenuItem;
int16_t MenuValue;
uint16_t uber_loop_count;			// Used to count main loops for everything else :)
uint16_t LoopStartTCNT1, LoopElapsedTCNT1, LoopCurrentTCNT1;
uint16_t loop_padding, servo_skip;

//************************************************************
// Main loop
//************************************************************

int main(void)
{
	OldIndex = 1;
	GUI_lockout = false;
	firsttimeflag = true;
	init();								// Do all init tasks

//************************************************************
// Test code - start
//************************************************************
if (0) 
{
	while(1)
	{
		// Display results
		LCDgoTo(0);								
		LCDprintstr("    ");
		LCDgoTo(0);
		LCDprintstr(itoa(RxChannel1,pBuffer,10));
		//
		LCDgoTo(7);								
		LCDprintstr("    ");
		LCDgoTo(7);
		LCDprintstr(itoa(RxChannel2,pBuffer,10));
		//
		LCDgoTo(16);					
		LCDprintstr("    ");
		LCDgoTo(16);
		LCDprintstr(itoa(RxChannel3,pBuffer,10));
		//
		LCDgoTo(23);
		LCDprintstr("    ");
		LCDgoTo(23);
		LCDprintstr(itoa(RxChannel4,pBuffer,10));
		//
		while (Interrupted == false){};
		Interrupted = false;
		ServoOut1 = RxChannel1 >> 1;
		ServoOut2 = RxChannel2 >> 1;
		ServoOut4 = RxChannel3 >> 1;
		//output_servo_ppm_asm(ServoOut1, ServoOut2, ServoOut4);
	}
	// Test new servo code
	while(1)
	{
		RxGetChannels();
		ServoOut1 = RxChannel2;
		ServoOut2 = RxChannel3;
		ServoOut4 = 700;
		ServoOut5 = 1000;
		ServoOut6 = 1500;
		Throttle = RxChannel3;
		while (Interrupted == false){};
		Interrupted = false;
		ServoOut1 = ServoOut1 >> 1;
		ServoOut2 = ServoOut2 >> 1;
		ServoOut4 = ServoOut4 >> 1;
		ServoOut5 = ServoOut5 >> 1;
		//output_servo_ppm_asm(ServoOut1, ServoOut2, ServoOut4);	
	}
}

//************************************************************
// Test code - end
//************************************************************

	for (int i = 0; i < AVGLENGTH; i++)		// Shouldn't need to do this but... we do
	{
		AvgAccRoll[i] = 0;					// Initialise all elements of the averaging arrays
		AvgAccPitch[i] = 0;
	}
	rxin = UDR0;							// Flush RX buffer so that we don't go into GUI mode

	LED = 1;								// Switch on LED whenever on

	// Main loop
	while (1)
	{
		AccRollTrim = Config.AccRollZeroTrim - 127; // GUI I/F bug won't pass signed numbers...
		AccPitchTrim = Config.AccPitchZeroTrim - 127;

		RxGetChannels();

		//************************************************************
		//* Alarms
		//************************************************************

		// Lost model alarm
		Change_LostModel += (uint8_t) (TCNT2 - Lost_TCNT2);
		Lost_TCNT2 = TCNT2;

		// Reset count if any RX activity or LCD/GUI active
		if ((RxActivity) || (LCD_active) || (GUIconnected))	
		{														
			Change_LostModel = 0;
			Model_lost = false;			
		}
		// Wait for 60s then trigger lost model alarm (468720 * 1/7812Hz = 60s)
		if (Change_LostModel > 468720)	Model_lost = true;
		if (((uber_loop_count &128) > 0) && (Model_lost))
		{
			LMA_Alarm = true;				// Turn on buzzer
		} 	
		else LMA_Alarm = false;				// Otherwise turn off buzzer


		// Low-voltage alarm (LVA)
		if ((uber_loop_count &128) > 0)		// Check every 500ms or so
		{ 
			GetVbat();						// Get battery voltage
		}

		if ((Config.Modes &16) > 0)			// Buzzer mode
		{
			// Beep buzzer if Vbat lower than trigger
			if ((vBat < Config.PowerTrigger) && ((uber_loop_count &128) > 0)) LVA_Alarm = true;
			else LVA_Alarm = false;				// Otherwise turn off buzzer
		}
		else if ((Config.Modes &16) == 0)	// LED mode
		{	// Flash LEDs if Vbat lower than trigger
			if ((vBat < Config.PowerTrigger) && (((uber_loop_count >> 8) &2) > 0)) LVA_Alarm = false;	
			else LVA_Alarm = true;				// Otherwise leave LEDs on
		}

		// Turn on buzzer if in alarm state
		if ((LVA_Alarm) || (LMA_Alarm))
		{
			LVA = 1;
		}
		else
		{
			LVA = 0;
		}

		//************************************************************
		//* GUI
		//************************************************************

		// Lock GUI out after 10 seconds or so if not already in GUI mode
		if ((uber_loop_count > GUI_TIMEOUT) && (!GUIconnected)) 
		{
			GUI_lockout = true;
		}

		// Check to see if any requests are coming from the GUI
		if ((UCSR0A & (1 << RXC0)) && (!GUI_lockout)) // Data waiting in RX buffer and GUI not yet locked out
		{
			GUIconnected = true;			// Set GUI flag so that servos can be disabled
			rxin = rx_byte();				// Get RX byte
			switch(rxin) 
			{
				case 'M':					// Send MultiWii data
					ReadGyros();
					ReadAcc();
					send_multwii_data();	
					break;
				case 'E':					// Calibrate gyros
					CalibrateGyros();
					break;
				case 'S':
					CalibrateAcc();			// Calibrate accelerometers
					break;
				case 'W':					// Receive data from GUI and save to eeProm
					get_multwii_data();
					Save_Config_to_EEPROM();
					LED = !LED;
					_delay_ms(500);
					LED = !LED;
					break;
				case 'D':					// Load eeProm defaults
					Set_EEPROM_Default_Config();
					break;
				case 'C':
					CenterSticks();			// Do stick centering
					break;
				default:
					break;
			}
		} // GUI

		//************************************************************
		//* LCD
		//************************************************************

		// LCD menu system - enter with pitch up, roll right and yaw left in stable mode
		// Check for stick hold
		Change_LCD += (uint8_t) (TCNT2 - LCD_TCNT2);
		LCD_TCNT2 = TCNT2;

		if ((RxInPitch > -STICKARM_POINT) || 	// Reset count if not up enough
			(RxInYaw < STICKARM_POINT) ||		// Reset count if not left yaw enough
			(RxInRoll > -STICKARM_POINT))		// Reset count if not right roll enough
		{														
			Change_LCD = 0;			
		}

		freshmenuvalue = true;
		firsttimeflag = true;

		// 1 sec, about 8000. (8000 * 1/7812Hz = 1.024s)
		// 2 sec, about 15624. (15624 * 1/7812Hz = 1.0s)
		if (Change_LCD > 15624)
		{	
			MenuItem = 0;
			LCD_active = true;
			while (LCD_active)									// Keep option of bailing out
			{
				// Wait for sticks to move back from initial state, display splash
				if (firsttimeflag) 
				{
					LCD_fixBL();
					LCD_Display_Menu(MenuItem);
					LCDprint_line2(" V1.12  (c)2012 ");
					_delay_ms(1500);
					firsttimeflag = false;
					GUIconnected = false;
					MenuItem = 1;
				}
				//
				RxGetChannels();								// Check sticks
				//
				if (RxInPitch >= STICKARM_POINT) 
				{	// Move up through menu list
					if (MenuItem == (MENUITEMS-1)) MenuItem = 1;// Wrap back to start	
					else MenuItem += 1;
					freshmenuvalue = true;
				}
				if (RxInPitch <= -STICKARM_POINT) 
				{	// Pitch down to move down menu
					if  (MenuItem <= 1) MenuItem = (MENUITEMS-1);// Wrap back to end
					else MenuItem -= 1;
					freshmenuvalue = true;
				}
				if (RxInRoll > STICKARM_POINT-100) 
				{	// Increase value if within range
					freshmenuvalue = false;
					if (MenuValue < get_menu_range(MenuItem).upper) MenuValue += 1;
				}
				if (RxInRoll < -STICKARM_POINT+100) 
				{	// Decrease value
					freshmenuvalue = false;
					if (MenuValue > get_menu_range(MenuItem).lower) MenuValue -= 1;
				}
				if (RxInYaw < -STICKARM_POINT) 
				{	// Save value when Yaw left
					set_menu_item(MenuItem, MenuValue);
					LCDprint_line2("  Value saved   ");
					_delay_ms(1000);	
					freshmenuvalue = true;						// OK to get new values
				}

				// Get stored value of current item
				if (freshmenuvalue) MenuValue = get_menu_item(MenuItem);// Get current value only when not being changed
		
				// Refresh changed data prior to delay to make LCD more responsive
				LCD_Display_Menu(MenuItem);						// Display menu top line
				if (MenuItem == 14) 							// Special case for LVA mode
				{
					LCDclearLine(2);
					LCDgoTo(26);
					LCDprintstr("<-Save");						// Setup save line
					LCDgoTo(17);								// Position cursor at nice spot
					if (MenuValue == 0) LCDprintstr("LED");
					else LCDprintstr("Buzzer");
				}
				else if ((MenuItem == 13)||(MenuItem == 15))	// Special case for voltages
				{
					LCDprint_line2("         Volts  ");			// Setup placeholder
					LCDgoTo(19);								// Position cursor at nice spot
					if (MenuValue >= 100)
					{
						itoa(MenuValue,pBuffer,10);				// Put voltage text in buffer
						char length = strlen(pBuffer);			// Shift digits right (including null byte)
						memcpy(pBuffer + (length-1), pBuffer + (length-2),2);				
						pBuffer[length-2] = '.';				// Insert a decimal point
						if (length == 3) pBuffer[length+1] = ' '; // Bodge for trailing digits :(
						LCDprintstr(pBuffer);					// Print battery voltage string
					}
					else LCDprintstr("LOW");					// Even bigger bodge for less than 3 digits lol
				}
				else if (MenuItem == 16) 						// Special case for LVA mode
				{
					LCDclearLine(2);
					LCDgoTo(26);
					LCDprintstr("<-Save");						// Setup save line
					LCDgoTo(16);								// Position cursor at nice spot
					if (MenuValue == 0) LCDprintstr("Use pots");
					else LCDprintstr("Use eeprom");
				}
				else if ((MenuItem >= 17) && (MenuItem < 23)) 	// Print value to change
				{
					LCDclearLine(2);
					LCDgoTo(26);
					LCDprintstr("<-Save");						// Setup save line
					LCDgoTo(17);								// Position cursor at nice spot
					if (MenuValue == 0) LCDprintstr("Normal");
					else LCDprintstr("Reversed");	
				}
				else if ((MenuItem > 0) && (MenuItem < 17)) 	// Print value to change
				{
					LCDclearLine(2);
					LCDgoTo(26);
					LCDprintstr("<-Save");						// Setup save line
					LCDgoTo(17);								// Position cursor at nice spot
					LCDprintstr(itoa(MenuValue,pBuffer,10)); 	
				}
				else if ((MenuItem >= 23) && (MenuItem < 25))	// For commands
				{
					LCDprint_line2("      <- Execute");	
				}
				else if (MenuItem == 25) 						// Special case for StabMode
				{
					LCDclearLine(2);
					LCDgoTo(26);
					LCDprintstr("<-Save");						// Setup save line
					LCDgoTo(16);								// Position cursor at nice spot
					if (MenuValue == 0) LCDprintstr("Switchable");
					else LCDprintstr("Always ON ");
				}
				else if (MenuItem >= 26) 						// Special case for ALMode
				{
					LCDclearLine(2);
					LCDgoTo(26);
					LCDprintstr("<-Save");						// Setup save line
					LCDgoTo(16);								// Position cursor at nice spot
					if (MenuValue == 0) LCDprintstr("Switchable");
					else LCDprintstr("Always OFF");
				}

				// Stick volume variable delay (four speeds)
				if 	   ((RxInRoll > STICKARM_POINT+150) || (RxInRoll < -STICKARM_POINT-150)) _delay_ms(10);
				else if ((RxInRoll > STICKARM_POINT+50) || (RxInRoll < -STICKARM_POINT-50)) _delay_ms(50);
				else if ((RxInRoll > STICKARM_POINT-50) || (RxInRoll < -STICKARM_POINT+50)) _delay_ms(250);
				else _delay_ms(500);

				// Exit if Yaw right and Pitch down for at least 500ms
				if ((RxInYaw > STICKARM_POINT) && (RxInPitch > STICKARM_POINT)) 
				{
					_delay_ms(1000);	
					RxGetChannels();							// Check sticks
					if ((RxInYaw > STICKARM_POINT) && (RxInPitch > STICKARM_POINT))  
					{
						LCD_active = false;
						GUIconnected = false;
					}
					else LCD_active = true;
				}
			} // While LCD mode
			LCDclear();			// Clear and reset LCD entry mode
			Change_LCD = 0;
		} // LCD activated (if (Change_LCD > 15624))

		//************************************************************
		//* Servo reversing
		//************************************************************

		// Reverse servo outputs as required
		if(Config.YawServo) {		// Rudder
			ServoOut1 = Config.RxChannel4ZeroOffset - RxInYaw; 
		}
		else {
			ServoOut1 = RxChannel4;
		}

		#if defined(STD_FLAPERON) 	// Ailerons controlled by separate channels
			if(Config.RollServo) {
				ServoOut2 = Config.RxChannel1ZeroOffset - RxInRoll;
				ServoOut5 = Config.RxChannel1ZeroOffset - RxInAux1;
			}
			else {
				ServoOut2 = RxChannel5;
				ServoOut5 = RxChannel1;
			}
		#else						// Only one aileron channel
			if(Config.RollServo) {
				ServoOut2 = Config.RxChannel1ZeroOffset - RxInRoll;
			}
			else {
				ServoOut2 = RxChannel1;
			}
		#endif

		if(Config.PitchServo) {		// Elevator
			ServoOut4 = Config.RxChannel2ZeroOffset - RxInPitch;
		}
		else {
			ServoOut4 = RxChannel2;
		}
		Throttle = RxChannel3;

		//************************************************************
		//* Autolevel mode selection
		//************************************************************

		// Autolevel is only available if you have an Accelerometer
		#if defined(ACCELEROMETER)

			// Use same switch for Autolevel as for stability based on preset 
			// Autolevel enabled if Config.ALMode = 0
			// Autolevel always OFF if Config.ALMode = 1 (default)

			// For CPPM mode, use RxChannel8 input (maybe make this selectable in future versions)
			#if defined(CPPM_MODE)
				if ((RxChannel8 < 1600) && (Config.ALMode == 0))	// RxChannel8 ON and AL is available

			// For non-CPPM mode, use the(THR) input
			#else
				if ((RxChannel3 < 1600) && (Config.ALMode == 0))	// RxInAux ON and AL is available
			#endif
				{									// When channel is activated
					AutoLevel = true;				// Activate autolevel mode
					flight_mode |= 1;				// Notify GUI that mode has changed
				}
				else
				{
					AutoLevel = false;				// De-activate autolevel mode
					flight_mode &= 0xfe;			// Notify GUI that mode has changed
					firsttimeflag = true;			// Reset flag
				}

		#else // No accelerometer fitted

			AutoLevel = false;				// De-activate autolevel mode
			flight_mode &= 0xfe;			// Notify GUI that mode has changed
			firsttimeflag = true;			// Reset flag

		#endif // #if defined(ACCELEROMETER)

		//************************************************************
		//* Stability mode selection
		//************************************************************

		// For CPPM mode, use Ch.8 for Stability
		#ifdef CPPM_MODE
		if ((RxChannel8 > 1600) && (Config.StabMode == 0))	// RxChannel8 enables stability if Config.StabMode = 0 (default)
		#else
		if ((RxChannel3 > 1600) && (Config.StabMode == 0))	// RxChannel3 (formerly throttle) enables stability if Config.StabMode = 0 (default)
		#endif												// Stability always ON if Config.StabMode = 1
		{
			// Notify GUI that mode has changed
			flight_mode &= 0xf7;

			// Reset I-terms when stabilise is off
			IntegralgPitch = 0;	 
			IntegralgRoll = 0;
			IntegralaPitch = 0;	 
			IntegralaRoll = 0;
			IntegralYaw = 0;
		}

		// Stability mode ON (RxChannel3 > 1600)
		else
		{
			// Notify GUI that mode has changed to stability mode
			flight_mode |= 8;				

			if ((Config.Modes &4) == 0)		// Pots mode
			{
				ReadGainValues();
			}

			//--- Read sensors ---
			ReadGyros();
			// Only read Accs if in AutoLevel mode
			if (AutoLevel) ReadAcc();

			// Accelerometer low-pass filter
			if (AutoLevel) {

				// Average accelerometer readings properly to create a genuine low-pass filter
				// Note that this has exactly the same effect as a complementary filter but with vastly less overhead.
				AvgAccRoll[AvgIndex] = accADC[ROLL];
				AvgAccPitch[AvgIndex] = accADC[PITCH];	// Add new values into buffer

				//Calculate rolling average with latest 64 values
				AvgRollSum = AvgRollSum + accADC[ROLL] - AvgAccRoll[OldIndex]; // Add new value to sum, subtract oldest
				AvgPitchSum = AvgPitchSum + accADC[PITCH] - AvgAccPitch[OldIndex];

				AvgRoll = ((AvgRollSum >> 6) - AccRollTrim); // Divide by 64 to get rolling average then adjust for acc trim
				AvgPitch = ((AvgPitchSum >> 6) - AccPitchTrim);

				AvgIndex ++;
				if (AvgIndex >= AVGLENGTH) AvgIndex = 0; // Wrap both indexes properly to create circular buffer
				OldIndex = AvgIndex + 1;
				if (OldIndex >= AVGLENGTH) OldIndex = 0;
			}

			//***********************************************************************
			//                --- Calculate roll gyro output ---
			//***********************************************************************

			if ((RxInRoll < DEAD_BAND) && (RxInRoll > -DEAD_BAND)) RxInRoll = 0; // Reduce RxIn noise into the I-term
			//Roll = RxInRoll + gyroADC[ROLL];
			Roll = gyroADC[ROLL];

			IntegralgRoll += Roll;									// Gyro I-term
			if (IntegralgRoll > ITERM_LIMIT_RP) IntegralgRoll = ITERM_LIMIT_RP; // Anti wind-up limit
			else if (IntegralgRoll < -ITERM_LIMIT_RP) IntegralgRoll = -ITERM_LIMIT_RP;

			if (AutoLevel) // Autolevel mode (Use averaged accelerometer to calculate attitude)
			{
				// Gyro PI terms
				P_term_gRoll = Roll * Config.P_mult_glevel;			// Multiply P-term (Max gain of 256)
				P_term_gRoll = P_term_gRoll * 3;					// Multiply by 3, so max effective gain is 768
				I_term_gRoll = IntegralgRoll * Config.I_mult_glevel;// Multiply I-term (Max gain of 256)
				I_term_gRoll = I_term_gRoll >> 3;					// Divide by 8, so max effective gain is 16

				// Acc PI terms
				P_term_aRoll = AvgRoll * Config.P_mult_alevel;		// P-term of accelerometer (Max gain of 256)
				IntegralaRoll += AvgRoll;							// Acc I-term
				if (IntegralaRoll > ITERM_LIMIT_LEVEL) IntegralaRoll = ITERM_LIMIT_LEVEL; // Anti wind-up limit
				else if (IntegralaRoll < -ITERM_LIMIT_LEVEL) IntegralaRoll = -ITERM_LIMIT_LEVEL;
				I_term_aRoll = IntegralaRoll * Config.I_mult_alevel;// Multiply I-term (Max gain of 256)
				I_term_aRoll = I_term_aRoll >> 3;					// Divide by 8, so max effective gain is 16
	
				// Sum Gyro P and I terms + Acc P and I terms
				Roll = P_term_gRoll + I_term_gRoll - P_term_aRoll - I_term_aRoll;
				Roll = Roll >> 6;									// Divide by 64 to rescale values back to normal

			}
			else // Normal mode (Just use raw gyro errors to guess at attitude)
			{
				// Gyro PID terms
				if ((Config.Modes &4) > 0)							// eeprom mode
				{
					P_term_gRoll = Roll * Config.P_mult_roll;		// Multiply P-term (Max gain of 256)
					I_term_gRoll = IntegralgRoll * Config.I_mult_roll;	// Multiply I-term (Max gain of 256)
				}
				else 
				{
					P_term_gRoll = Roll * GainInADC[ROLL];			// Multiply P-term (Max gain of 256)
					I_term_gRoll = IntegralgRoll * GainInADC[PITCH];// Multiply I-term (Max gain of 256)
				}
				P_term_gRoll = P_term_gRoll * 3;					// Multiply by 3, so max effective gain is 768
				I_term_gRoll = I_term_gRoll >> 3;					// Divide by 8, so max effective gain is 16

				// Sum	
				Roll = P_term_gRoll + I_term_gRoll; 				// P + I
				Roll = Roll >> 6;									// Divide by 64 to rescale values back to normal
			}

			//--- (Add)Adjust roll gyro output to Servos
			#ifdef STANDARD
			ServoOut2 -= Roll;
			#elif defined(FWING)
			ServoOut2 += Roll;
			ServoOut4 += Roll;
			#elif defined(STD_FLAPERON)
			ServoOut2 -= Roll; // Left
			ServoOut5 -= Roll; // Right
			#else
			#error No configuration defined !!!!
			#endif

			//***********************************************************************
			//                --- Calculate pitch gyro output ---
			//***********************************************************************

			if ((RxInPitch < DEAD_BAND) && (RxInPitch > -DEAD_BAND)) RxInPitch = 0; // Reduce RxIn noise into the I-term
			//Pitch = RxInPitch + gyroADC[PITCH];
			Pitch = gyroADC[PITCH];

			IntegralgPitch += Pitch;								// I-term (32-bit)
			if (IntegralgPitch > ITERM_LIMIT_RP) IntegralgPitch = ITERM_LIMIT_RP; // Anti wind-up limit
			else if (IntegralgPitch < -ITERM_LIMIT_RP) IntegralgPitch = -ITERM_LIMIT_RP;

			if (AutoLevel) // Autolevel mode (Use averaged accelerometer to calculate attitude)
			{
				// Gyro PI terms
				P_term_gPitch = Pitch * Config.P_mult_glevel;		// Multiply P-term (Max gain of 256)
				P_term_gPitch = P_term_gPitch * 3;					// Multiply by 3, so max effective gain is 768
				I_term_gPitch = IntegralgPitch * Config.I_mult_glevel;// Multiply I-term (Max gain of 256)
				I_term_gPitch = I_term_gPitch >> 3;					// Divide by 8, so max effective gain is 16
	
				// Acc PI terms
				P_term_aPitch = AvgPitch * Config.P_mult_alevel;	// P-term of accelerometer (Max gain of 256)
				IntegralaPitch += AvgPitch;							// Acc I-term
				if (IntegralaPitch > ITERM_LIMIT_LEVEL) IntegralaPitch = ITERM_LIMIT_LEVEL; // Anti wind-up limit
				else if (IntegralaPitch < -ITERM_LIMIT_LEVEL) IntegralaPitch = -ITERM_LIMIT_LEVEL;
				I_term_aPitch = IntegralaPitch * Config.I_mult_alevel;// Multiply I-term (Max gain of 256)
				I_term_aPitch = I_term_aPitch >> 3;					// Divide by 8, so max effective gain is 16

				// Sum Gyro P and I terms + Acc P and I terms
				Pitch = P_term_gPitch + I_term_gPitch - P_term_aPitch - I_term_aPitch;
				Pitch = Pitch >> 6;									// Divide by 64 to rescale values back to normal
			}
			else // Normal mode (Just use raw gyro errors to guess at attitude)
			{
				// Gyro PID terms
				if ((Config.Modes &4) > 0)							// eeprom mode
				{
					P_term_gPitch = Pitch * Config.P_mult_pitch;	// Multiply P-term (Max gain of 256)
					I_term_gPitch = IntegralgPitch * Config.I_mult_pitch;// Multiply I-term (Max gain of 256)
				}
				else
				{
					P_term_gPitch = Pitch * GainInADC[ROLL];		// Multiply P-term (Max gain of 256)
					I_term_gPitch = IntegralgPitch * GainInADC[PITCH];// Multiply I-term (Max gain of 256)
				}
				P_term_gPitch = P_term_gPitch * 3;					// Multiply by 3, so max effective gain is 768
				I_term_gPitch = I_term_gPitch >> 3;					// Divide by 8, so max effective gain is 16

				// Sum
				Pitch = P_term_gPitch + I_term_gPitch;				// P + I
				Pitch = Pitch >> 6;									// Divide by 64 to rescale values back to normal
			}

			//--- (Add)Adjust pitch gyro output to Servos
			#if (defined (STANDARD) || defined(STD_FLAPERON))
			ServoOut4 -= Pitch;
			#elif defined(FWING)
			ServoOut2 -= Pitch;
			ServoOut4 += Pitch;
			#else
			#error No configuration defined !!!!
			#endif

			//***********************************************************************
			//                 --- Calculate yaw gyro output ---
			//***********************************************************************

			if ((RxInYaw < DEAD_BAND) && (RxInYaw > -DEAD_BAND)) RxInYaw = 0; // Reduce RxIn noise into the I-term
			//Yaw = RxInYaw + gyroADC[YAW];	
			Yaw = gyroADC[YAW];

			IntegralYaw += Yaw;									// I-term (32-bit)
			if (IntegralYaw > ITERM_LIMIT_YAW) IntegralYaw = ITERM_LIMIT_YAW;
			else if (IntegralYaw < -ITERM_LIMIT_YAW) IntegralYaw = -ITERM_LIMIT_YAW;// Anti wind-up (Experiment with value)

			if ((Config.Modes &4) > 0)							// eeprom mode
			{
				Yaw *= Config.P_mult_yaw;						// Multiply P-term (Max gain of 256)
				I_term_Yaw = IntegralYaw * Config.I_mult_yaw;	// Multiply IntegralYaw by up to 256
			}
			else
			{
				Yaw *= GainInADC[YAW];							// Multiply P-term (Max gain of 256)
				I_term_Yaw = IntegralYaw * Config.I_mult_yaw;	// Multiply IntegralYaw by up to 256
			}
			Yaw = Yaw * 3;	
			I_term_Yaw = I_term_Yaw >> 3;						// Divide by 8, so max effective gain is 16

			Yaw = Yaw - I_term_Yaw;								// P + I
			Yaw = Yaw >> 6;										// Divide by 64 to rescale values back to normal

			//--- (Add)Adjust yaw gyro output to servos
			#if (defined(STANDARD) || defined(FWING) || defined(STD_FLAPERON))
			ServoOut1 -= Yaw;
			#else
			#error No configuration defined !!!!
			#endif

		} // AUX 1 over-rides stability

		//--- Servo travel limits ---
		if ( ServoOut1 < MIN_TRAVEL )	ServoOut1 = MIN_TRAVEL;	
		if ( ServoOut2 < MIN_TRAVEL )	ServoOut2 = MIN_TRAVEL;	
		if ( ServoOut4 < MIN_TRAVEL )	ServoOut4 = MIN_TRAVEL;
		if ( ServoOut5 < MIN_TRAVEL )	ServoOut5 = MIN_TRAVEL;	
		if ( ServoOut6 < MIN_TRAVEL )	ServoOut6 = MIN_TRAVEL;
		if ( Throttle  < MIN_TRAVEL )	Throttle  = MIN_TRAVEL;	
	
		if ( ServoOut1 > MAX_TRAVEL )	ServoOut1 = MAX_TRAVEL;	
		if ( ServoOut2 > MAX_TRAVEL )	ServoOut2 = MAX_TRAVEL;	
		if ( ServoOut6 > MAX_TRAVEL )	ServoOut6 = MAX_TRAVEL;
		if ( ServoOut4 > MAX_TRAVEL )	ServoOut4 = MAX_TRAVEL;
		if ( ServoOut5 > MAX_TRAVEL )	ServoOut5 = MAX_TRAVEL;	
		if ( ServoOut6 > MAX_TRAVEL )	ServoOut6 = MAX_TRAVEL;
		if ( Throttle  > MAX_TRAVEL )	Throttle  = MAX_TRAVEL;	

		// Loop governor is here so that the loop is regulated to LOOP_RATE Hz. 
		// This is important for the averaging filters.
		LoopCurrentTCNT1 = TCNT1;
		if (LoopCurrentTCNT1 > LoopStartTCNT1) LoopElapsedTCNT1 = LoopCurrentTCNT1 - LoopStartTCNT1;
		else LoopElapsedTCNT1 = (0xffff - LoopStartTCNT1) + LoopCurrentTCNT1;

		// If loop period less than LOOP_INTERVAL, pad it out in 8us chunks. (NB: deliberately blocking code)
		loop_padding = (LOOP_INTERVAL - LoopElapsedTCNT1) / 8; // Measure remaining interval

		if ((loop_padding > 0) && (GUIconnected == false))
		{
			TIFR0 &= ~(1 << TOV0);		// Clear overflow
			TCNT0 = 0;					// Reset counter
			for (int i=0;i<loop_padding;i++)
			{
				while (TCNT0 < 8);		// 1MHz = 1us * 8 = 8us/loop
				TCNT0 -= 8;
			}
		}

		// Ensure that output_servo_ppm() is only called at SERVO_RATE.
		// If SERVO_RATE due, update servos, otherwise bypass
		// Inhibit servos while GUI connected
		if ((servo_skip >= (LOOP_RATE/SERVO_RATE)) && (GUIconnected == false))
		{
			Interrupted = false;
			servo_skip = 0;
			while (Interrupted == false){};	// Wait here for any interrupts to complete
			Interrupted = false;
			output_servo_ppm();				// Output servo signal
		}

		// Update cycle time for GUI
		cycletime = LoopElapsedTCNT1;

		// Measure period of loop from here
		LoopStartTCNT1 = TCNT1;
		uber_loop_count++;
		servo_skip++;

	} // main loop
} // main()

