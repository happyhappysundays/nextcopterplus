// **************************************************************************
// OpenAero software
// =================
// Version 1.14.1 Release
// Inspired by KKmulticopter
// Based on assembly code by Rolf R Bakke, and C code by Mike Barton
// OpenAero code by David Thompson, included open-source code as per references
//
// Includes PID and Auto-level functions inspired by the open-sourced MultiWii project
// Compatible with KK boards fitted with X and Y accelerometers 
// on Roll/Pitch pot inputs. LCD board or GUI required for setup of PID constants.
//
// Tested only on Atmega168P/PA boards (KK+, Blackboards, HobbyKing V1 & 2)
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
// Hold Yaw stick on TX left (direction may depend on TX)
// Power on
// LED flashes 4 times
// Mode will toggle between the following:
// 		Stability switchable
// 		Stability always ON
//
// Autolevel mode selection
// ------------------------
// Hold Yaw stick on TX right (direction may depend on TX)
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
//			Virtually no jitter now. Fixed flaperon and vertical mode bugs.
//			Added auto-configuring PWM and CPPM input code.
//			Corrected loop timing bug. Servo outputs now match incoming data rate.
// V1.13	Add stability channel selection via LCD in CPPM modes.
//			Removed loop rate control, changed all the long timers to suit.
//			Added servo overdue timeout to guarantee servo output regardless of input. 
//			Fixed "Automagic PWM" input mode. Added failsafe setting via holding M6 LOW.
//			Failsafe holds outputs at predetermined position if RC sync lost.
//			Automated compiledefs.h. Tweaked failsafe timing. Adjusted overdue define for slower RXs.
//			Optimised code - thanks to wargh from RCG. 
//			Added support for Eagle N6 / HobbyKing i86 hardware. S1 selects failsafe, S3 & S4 select mixer modes.
//			Removed gyro I-term as it was dangerous/confusing, Gyro PID now a P+D loop.
//				Roll pot 	= Roll/Pitch P-term
//				Pitch pot 	= Roll/Pitch D-term
//				Yaw pot 	= Yaw P-term
//			Power-on modes now no longer lock up and confuse those that won't read manuals.
//			Added X-MODE thanks to Cesco. Added LEGACY_PWM_MODE1 and LEGACY_PWM_MODE2 compile options.
//			Beta 6: Fixed flaperon mode on N6. Fixed Pitch/Roll gyros swapped on N6. Oops...
//			Beta 7: Removed usused LCD menu items where appropriate.
//			Changed failsafe timeout to 500ms and added servo rate timer to set servo failsafe rate to 50Hz.
//			Beta 8: Added 3-position switch mode for selection of Stability/Autolevel.
// 			Beta 9: Added PWM flaperon mode using M3 as the additional input
// V1.14	V1.14 release
// V1.14.1	Bugfix for stability switch in V1.14. Oops.
//
//***********************************************************
//* To do
//***********************************************************
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
             M1     Rudder (M3 on i86/N6)


Standard Flaperon mode (CPPM only - second aileron channel on Ch.5)

			 X <--  THR (Throttle - CPPM mode)
             |
     M2 -----+----- M5 Flaperons
             |
             |
        M4 --+--    Elevator
             |
             M1     Rudder (M3 on i86/N6)

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
          Rudder (M3 on i86/N6)

  i86/N6 Switch bank modes
  ------------------------
  S1 = Failsafe mode if held on for > 1 second. Leave OFF.
  S2 = Spare
  +-----------------------------------+
  |S3 |S4 | Mode                      |  
  +-----------------------------------+
  |0FF|OFF| Aeroplane mode            |
  |OFF|ON | Flying Wing mode          |
  |ON |OFF| Flaperon mode             |
  |ON |ON | Defaults to Aeroplane mode|
  +-----------------------------------+
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
#include "..\inc\i2cmaster.h"
#include "..\inc\i2c.h"

//***********************************************************
//* Defines
//***********************************************************

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

// Timeouts
#define GUI_TIMEOUT 78120			// Time after which GUI entry is impossible (78,120 = 10s)
#define	SERVO_OVERDUE 3906			// Number of T2 cycles before servo will be overdue = 3906 * 1/7812 = 500ms
#define	SERVO_RATE 156				// Requested servo rate when in failsafe mode. 7812 / 50(Hz) = 156
#define LCD_TIMEOUT 15624			// Number or T2 cycles before LCD engages (2 seconds)
#define LMA_TIMEOUT 468720			// Number or T2 cycles before Lost Model alarm sounds (1 minute)
#define FS_TIMEOUT 7812				// Number or T2 cycles before Failsafe setting engages (1 seconds)
#define	PWM_DELAY 250				// Number of T0 cycles to wait between "Interrupted" and starting the PWM pulses 250 = 2ms

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
int32_t	IntegralaPitch;				// PID I-terms (acc.) for each axis
int32_t	IntegralaRoll;
int32_t P_term_gRoll;				// Calculated P-terms (gyro) for each axis
int32_t P_term_gPitch;
int32_t P_term_aRoll;				// Calculated P-terms (acc.) for each axis
int32_t P_term_aPitch;
int32_t I_term_aRoll;				// Calculated I-terms (acc.) for each axis
int32_t I_term_aPitch;
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
bool 	firsttimeflag;
bool	Overdue;
bool	ServoTick;
bool	Failsafe;
char 	pBuffer[16];				// Print buffer
uint8_t rxin;
uint8_t MenuItem;
int16_t MenuValue;
uint8_t	MixerMode;					// Mixer mode from switch bank

// Timing
uint16_t Change_LCD;				// Long timers increment at 7812kHz or 128us
uint32_t Change_LostModel;
uint32_t Ticker_Count;
uint16_t Servo_Timeout;
uint16_t Servo_Rate;
uint16_t Failsafe_count;

uint8_t LCD_TCNT2;
uint8_t Lost_TCNT2;
uint8_t Ticker_TCNT2;
uint8_t Servo_TCNT2;
uint8_t ServoRate_TCNT2;
uint8_t Failsafe_TCNT2;
uint16_t LoopStartTCNT1;

bool 	BUZZER_ON;
bool 	LED_ON;
bool 	Model_lost;					// Model lost flag
bool 	LMA_Alarm;					// Lost model alarm active
bool 	LVA_Alarm;					// Low voltage alarm active

//************************************************************
// Main loop
//************************************************************

int main(void)
{
#ifndef ICP_CPPM_MODE
	uint16_t i;
#endif
	OldIndex = 1;
	GUI_lockout = false;
	firsttimeflag = true;
	init();									// Do all init tasks

//************************************************************
// Test code - start
//************************************************************

while(0)
{
	ServoOut1 = RxChannel4;
	ServoOut2 = RxChannel1;
	ServoOut4 = RxChannel2;
	ServoOut5 = RxChannel5;
	output_servo_ppm();				// Output servo signal
	_delay_ms(20);
}

while(0)
{
	MixerMode = ((PIND >> 6) & 0x03);	// Process mixer switch (S3~4) setting
	LCDclear();
	LCDgoTo(0);
	LCDprintstr("Mode:      ");
	LCDgoTo(6);
	LCDprintstr(itoa(MixerMode,pBuffer,16)); // Print data in hex
	_delay_ms(100);
}

#ifdef N6_MODE
while (0) 
{
	_delay_ms(2000);
	CalibrateGyros();
	writeI2Cbyte(L3G4200D_ADDRESS, 0x20, 0x8F); // 400Hz ODR, 20Hz cut-off
	writeI2Cbyte(L3G4200D_ADDRESS, 0x23, 0x20); // 500dps
	writeI2Cbyte(L3G4200D_ADDRESS, 0x24, 0x02);

	LCDclear();
	LCDgoTo(0);
	LCDprintstr("Ad:0x   ");
	LCDgoTo(5);
	LCDprintstr(itoa(L3G4200D_ADDRESS,pBuffer,16)); // Print address in hex

	while(1)
	{
		ReadGyros();
		LCDgoTo(8);
		LCDprintstr("X:      ");
		LCDgoTo(10);
		LCDprintstr(itoa(gyroADC[ROLL],pBuffer,10));
		LCDgoTo(16);
		LCDprintstr("Y:      ");
		LCDgoTo(18);
		LCDprintstr(itoa(gyroADC[PITCH],pBuffer,10));
		LCDgoTo(24);
		LCDprintstr("Z:      ");
		LCDgoTo(26);
		LCDprintstr(itoa(gyroADC[YAW],pBuffer,10));
		_delay_ms(100);
	}
}
#endif

//************************************************************
// Test code - end
//************************************************************

	for (int i = 0; i < AVGLENGTH; i++)
	{
		AvgAccRoll[i] = 0;					// Initialise all elements of the averaging arrays
		AvgAccPitch[i] = 0;
	}
	rxin = UDR0;							// Flush RX buffer so that we don't go into GUI mode
	LED2 = 1;								// Switch on Red LED whenever running
	

	//************************************************************
	//* Wait for RC synchronisation in APWM mode
	//************************************************************
	#ifdef AUTOMAGIC_PWM_MODE
	while (max_chan == 0)					// No pulses yet
	{
		_delay_ms(50);
		LED2 = !LED2;						// Flash Red LED rapidly
	}

	LED2 = 0;								// LED OFF
	_delay_ms(500);							// Allow RC time to settle
	LED2 = 1;								// LED back ON

	gapfound = true;						// Stop channel search
	#endif

	//************************************************************
	//* Main loop
	//************************************************************
	while (1)
	{
		// GUI I/F bug won't pass signed numbers...
		AccRollTrim = Config.AccRollZeroTrim - 127; 
		AccPitchTrim = Config.AccPitchZeroTrim - 127;

		// Update RC channel data
		RxGetChannels();

		//************************************************************
		//* System ticker
		//* 
		//* (Ticker_Count &128) 	 	= 60Hz
		//* ((Ticker_Count >> 8) &2) 	= 15Hz
		//* ((Ticker_Count >> 8) &4) 	= 3.75Hz (LMA and LVA alarms)
		//* ((Ticker_Count >> 8) &32)	= 0.47Hz (LED mode alarms)
		//* 
		//************************************************************

		// Ticker_Count increments at 7812 kHz, in loop cycle chunks
		Ticker_Count += (uint8_t) (TCNT2 - Ticker_TCNT2);
		Ticker_TCNT2 = TCNT2;

		if ((Ticker_Count >> 8) &4) BUZZER_ON = true; 	// 3.75Hz beep
		else BUZZER_ON = false;

		if ((Ticker_Count >> 8) &32) LED_ON = true;		// 0.47Hz flash
		else LED_ON = false;


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
		// Wait for 60s then trigger lost model alarm (LMA_TIMEOUT * 1/7812Hz = 60s)
		if (Change_LostModel > LMA_TIMEOUT)	Model_lost = true;

		if (BUZZER_ON && Model_lost) LMA_Alarm = true;	// Turn on buzzer
		else LMA_Alarm = false;				// Otherwise turn off buzzer

		#ifndef N6_MODE // No LVA for i86/N6
		// Low-voltage alarm (LVA)
		GetVbat();							// Check battery

		if ((Config.Modes &16) > 0)			// Buzzer mode
		{
			// Beep buzzer if Vbat lower than trigger
			if ((vBat < Config.PowerTrigger) && BUZZER_ON) LVA_Alarm = true;
			else LVA_Alarm = false;				// Otherwise turn off buzzer
		}
		else 								// LED mode
		{	// Flash LEDs if Vbat lower than trigger
			if ((vBat < Config.PowerTrigger) && LED_ON) LVA_Alarm = false;	
			else LVA_Alarm = true;				// Otherwise leave LEDs on
		}
		#endif

		// Turn on buzzer if in alarm state
		if ((LVA_Alarm) || (LMA_Alarm)) LVA = 1;
		else LVA = 0;

		//************************************************************
		//* GUI
		//************************************************************

		// Lock GUI out after 10 seconds or so if not already in GUI mode
		if ((Ticker_Count > GUI_TIMEOUT) && (!GUIconnected)) 
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
					LED1 = !LED1;
					_delay_ms(500);
					LED1 = !LED1;
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

		// 2 sec, about 15624. (LCD_TIMEOUT * 1/7812Hz = 2.0s)
		if (Change_LCD > LCD_TIMEOUT)
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
					LCDprint_line2(" V1.13  (c)2012 ");
					_delay_ms(1500);
					firsttimeflag = false;
					GUIconnected = false;
					MenuItem = LCD_increment(MenuItem);
				}
				//
				RxGetChannels();								// Check sticks
				//
				if (RxInPitch >= STICKARM_POINT) 
				{	// Move up through menu list
					MenuItem = LCD_increment(MenuItem);
					freshmenuvalue = true;
				}
				if (RxInPitch <= -STICKARM_POINT) 
				{	// Pitch down to move down menu
					MenuItem = LCD_decrement(MenuItem);
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
				else if (((MenuItem > 0) && (MenuItem < 17)) || (MenuItem == 27)) 	// Print value to change
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
				else if (MenuItem == 26) 						// Special case for ALMode
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
		} // LCD activated (if (Change_LCD > LCD_TIMEOUT))

		//************************************************************
		//* RC input to servos and servo reversing
		//************************************************************

		SetServoPositions();

		//************************************************************
		//* Check for Failsafe mode request if M6 is held LOW for 1 second
		//************************************************************

		Failsafe_count += (uint8_t) (TCNT2 - Failsafe_TCNT2);
		Failsafe_TCNT2 = TCNT2;
		
		// Reset count if FS is HIGH (normal)
		#ifndef N6_MODE
		if (FS) Failsafe_count = 0;
		#else
		if (!FS) Failsafe_count = 0;	 // Switch inverted on N6
		#endif
		if (Failsafe_count > FS_TIMEOUT) // Trigger failsafe setting after 1 second
		{
			SetFailsafe();
		}

		//************************************************************
		//* Autolevel mode selection
		//************************************************************

		// Autolevel is only available if you have an Accelerometer
		#if defined(ACCELEROMETER)

		// Use same switch for Autolevel as for stability based on preset 
		// Autolevel enabled if Config.ALMode = 0
		// Autolevel always OFF if Config.ALMode = 1 (default)

		// For CPPM mode, use StabChan input
		#ifdef ICP_CPPM_MODE
			if ((StabChan < 1600) && (Config.ALMode == 0))	// StabChan ON and AL is available

		// For 3-position switch, use the high range of THR input
		#elif defined (THREE_POS)
			if (RxInAux > 200)

		// For non-CPPM mode, use the(THR) input
		#else
			if ((RxInAux < 200) && (Config.ALMode == 0))	// RxInAux ON and AL is available
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

		// For CPPM mode, use StabChan for Stability
		#ifdef ICP_CPPM_MODE
		if ((StabChan > 1600) && (Config.StabMode == 0))	// StabChan enables stability if Config.StabMode = 0 (default)

		// For 3-position switch, use medium to high range of THR input
		// The range of less than -100 has both Stability and Autolevel OFF
		#elif defined (THREE_POS)
		if (RxInAux < 200)

		// For non-CPPM mode, use the(THR) input
		#else
		if ((RxInAux < 200) && (Config.StabMode == 0))		// RxInAux enables stability if Config.StabMode = 0 (default)
		#endif												// Stability always ON if Config.StabMode = 1
		{
			// Notify GUI that mode has changed
			flight_mode &= 0xf7;

			// Reset I-terms when stabilise is off
			IntegralaPitch = 0;	 
			IntegralaRoll = 0;
		}

		// Stability mode ON
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

			// Accelerometer low-pass filter
			if (AutoLevel) {

				ReadAcc();	// Only read Accs if in AutoLevel mode

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

			if ((RxInRoll < DEAD_BAND) && (RxInRoll > -DEAD_BAND)) RxInRoll = 0; // Reduce RxIn noise
			Roll = gyroADC[ROLL];

			currentError[ROLL] = Roll;								// D-term
			DifferentialGyro = currentError[ROLL] - lastError[ROLL];
			lastError[ROLL] = currentError[ROLL];

			if (AutoLevel) // Autolevel mode (Use averaged accelerometer to calculate attitude)
			{
				// Gyro PD terms
				P_term_gRoll = Roll * Config.P_mult_glevel;			// Multiply P-term (Max gain of 256)
				P_term_gRoll = P_term_gRoll * 3;					// Multiply by 3, so max effective gain is 768
				DifferentialGyro *= Config.D_mult_glevel;			// Multiply D-term by up to 256

				// Acc PI terms
				P_term_aRoll = AvgRoll * Config.P_mult_alevel;		// P-term of accelerometer (Max gain of 256)
				IntegralaRoll += AvgRoll;							// Acc I-term
				if (IntegralaRoll > ITERM_LIMIT_LEVEL) IntegralaRoll = ITERM_LIMIT_LEVEL; // Anti wind-up limit
				else if (IntegralaRoll < -ITERM_LIMIT_LEVEL) IntegralaRoll = -ITERM_LIMIT_LEVEL;
				I_term_aRoll = IntegralaRoll * Config.I_mult_alevel;// Multiply I-term (Max gain of 256)
				I_term_aRoll = I_term_aRoll >> 3;					// Divide by 8, so max effective gain is 16
	
				// Sum Gyro P and D terms + Acc P and I terms
				Roll = P_term_gRoll + DifferentialGyro - P_term_aRoll - I_term_aRoll;
				Roll = Roll >> 6;									// Divide by 64 to rescale values back to normal

			}
			else // Normal mode (Just use raw gyro errors to guess at attitude)
			{
				// Gyro PD terms
				if ((Config.Modes &4) > 0)							// eeprom mode
				{
					P_term_gRoll = Roll * Config.P_mult_roll;		// Multiply P-term (Max gain of 256)
					DifferentialGyro *= Config.D_mult_roll;			// Multiply D-term by up to 256
				}
				else 
				{
					P_term_gRoll = Roll * GainInADC[ROLL];			// Multiply P-term (Max gain of 256)
					DifferentialGyro *= GainInADC[PITCH];			// Multiply D-term by up to 256
				}
				P_term_gRoll = P_term_gRoll * 3;					// Multiply by 3, so max effective gain is 768

				// Sum	
				Roll = P_term_gRoll + DifferentialGyro; 			// P + D; 
				Roll = Roll >> 6;									// Divide by 64 to rescale values back to normal
			}

			//***********************************************************************
			//                --- Calculate pitch gyro output ---
			//***********************************************************************

			if ((RxInPitch < DEAD_BAND) && (RxInPitch > -DEAD_BAND)) RxInPitch = 0; // Reduce RxIn noise
			Pitch = gyroADC[PITCH];

			currentError[PITCH] = Pitch;							// D-term
			DifferentialGyro = currentError[PITCH] - lastError[PITCH];
			lastError[PITCH] = currentError[PITCH];

			if (AutoLevel) // Autolevel mode (Use averaged accelerometer to calculate attitude)
			{
				// Gyro PD terms
				P_term_gPitch = Pitch * Config.P_mult_glevel;		// Multiply P-term (Max gain of 256)
				P_term_gPitch = P_term_gPitch * 3;					// Multiply by 3, so max effective gain is 768
				DifferentialGyro *= Config.D_mult_glevel;			// Multiply D-term by up to 256
	
				// Acc PI terms
				P_term_aPitch = AvgPitch * Config.P_mult_alevel;	// P-term of accelerometer (Max gain of 256)
				IntegralaPitch += AvgPitch;							// Acc I-term
				if (IntegralaPitch > ITERM_LIMIT_LEVEL) IntegralaPitch = ITERM_LIMIT_LEVEL; // Anti wind-up limit
				else if (IntegralaPitch < -ITERM_LIMIT_LEVEL) IntegralaPitch = -ITERM_LIMIT_LEVEL;
				I_term_aPitch = IntegralaPitch * Config.I_mult_alevel;// Multiply I-term (Max gain of 256)
				I_term_aPitch = I_term_aPitch >> 3;					// Divide by 8, so max effective gain is 16

				// Sum Gyro P and I terms + Acc P and I terms
				Pitch = P_term_gPitch + DifferentialGyro - P_term_aPitch - I_term_aPitch;
				Pitch = Pitch >> 6;									// Divide by 64 to rescale values back to normal
			}
			else // Normal mode (Just use raw gyro errors to guess at attitude)
			{
				// Gyro PD terms
				if ((Config.Modes &4) > 0)							// eeprom mode
				{
					P_term_gPitch = Pitch * Config.P_mult_pitch;	// Multiply P-term (Max gain of 256)
					DifferentialGyro *= Config.D_mult_roll;			// Multiply D-term by up to 256
				}
				else
				{
					P_term_gPitch = Pitch * GainInADC[ROLL];		// Multiply P-term (Max gain of 256)
					DifferentialGyro *= GainInADC[PITCH];			// Multiply D-term by up to 256
				}
				P_term_gPitch = P_term_gPitch * 3;					// Multiply by 3, so max effective gain is 768

				// Sum
				Pitch = P_term_gPitch + DifferentialGyro; 			// P + D; 
				Pitch = Pitch >> 6;									// Divide by 64 to rescale values back to normal
			}

			//***********************************************************************
			//                 --- Calculate yaw gyro output ---
			//***********************************************************************

			if ((RxInYaw < DEAD_BAND) && (RxInYaw > -DEAD_BAND)) RxInYaw = 0; // Reduce RxIn noise
			Yaw = gyroADC[YAW];

			currentError[YAW] = Yaw;							// D-term
			DifferentialGyro = currentError[YAW] - lastError[YAW];
			lastError[YAW] = currentError[YAW];

			if ((Config.Modes &4) > 0)							// eeprom mode
			{
				Yaw *= Config.P_mult_yaw;						// Multiply P-term (Max gain of 256)
				DifferentialGyro *= Config.D_mult_roll;			// Multiply D-term by up to 256
			}
			else
			{
				Yaw *= GainInADC[YAW];							// Multiply P-term (Max gain of 256)
				DifferentialGyro *= GainInADC[PITCH];			// Multiply D-term by up to 256
			}
			Yaw = Yaw * 3;

			// Sum
			Yaw = Yaw + DifferentialGyro; 						// P + D; 
			Yaw = Yaw >> 6;										// Divide by 64 to rescale values back to normal

			//***********************************************************************
			//                 --- Output mixer ---
			//***********************************************************************
			#ifndef N6_MODE 					// Standard KK board mixer configuration
			#ifdef STANDARD
			ServoOut1 -= Yaw;
			ServoOut2 -= Roll;
			ServoOut4 -= Pitch;
			#elif defined(FWING)
			ServoOut1 -= Yaw;
			ServoOut2 += Roll;
			ServoOut2 -= Pitch;
			ServoOut4 += Roll;
			ServoOut4 += Pitch;
			#elif defined(STD_FLAPERON)
			ServoOut1 -= Yaw;
			ServoOut2 -= Roll; 		// Left
			ServoOut5 -= Roll; 		// Right
			ServoOut4 -= Pitch;
			#else
			#error No mixer configuration defined
			#endif
			#else
			// Note that in N6 mode, ServoOut1 redirects to M3 in servos_asm.S
			switch(MixerMode)
			{
				case 0:						// Aeroplane mixing
					ServoOut1 -= Yaw;
					ServoOut2 -= Roll;
					ServoOut4 -= Pitch;
					break;
				case 2:						// Flying wing mixing
					ServoOut1 -= Yaw;
					ServoOut2 += Roll;
					ServoOut2 -= Pitch;
					ServoOut4 += Roll;
					ServoOut4 += Pitch;
					break;
				case 1:						// Flaperon mixing
					ServoOut1 -= Yaw;
					ServoOut2 -= Roll;
					ServoOut5 -= Roll;
					ServoOut4 -= Pitch;
					break;
				default:					// Default to aeroplane mixing
					ServoOut1 -= Yaw;
					ServoOut2 -= Roll;
					ServoOut4 -= Pitch;
					break;
			}
			#endif
		} // Stability mode

		//--- Servo travel limits ---
		if ( ServoOut1 < MIN_TRAVEL )	ServoOut1 = MIN_TRAVEL;	
		if ( ServoOut2 < MIN_TRAVEL )	ServoOut2 = MIN_TRAVEL;	
		if ( ServoOut4 < MIN_TRAVEL )	ServoOut4 = MIN_TRAVEL;
		if ( ServoOut5 < MIN_TRAVEL )	ServoOut5 = MIN_TRAVEL;	
		if ( ServoOut6 < MIN_TRAVEL )	ServoOut6 = MIN_TRAVEL;
		if ( Throttle  < MIN_TRAVEL )	Throttle  = MIN_TRAVEL;	
	
		if ( ServoOut1 > MAX_TRAVEL )	ServoOut1 = MAX_TRAVEL;	
		if ( ServoOut2 > MAX_TRAVEL )	ServoOut2 = MAX_TRAVEL;	
		if ( ServoOut4 > MAX_TRAVEL )	ServoOut4 = MAX_TRAVEL;
		if ( ServoOut5 > MAX_TRAVEL )	ServoOut5 = MAX_TRAVEL;	
		if ( ServoOut6 > MAX_TRAVEL )	ServoOut6 = MAX_TRAVEL;
		if ( Throttle  > MAX_TRAVEL )	Throttle  = MAX_TRAVEL;	

		// Servo_Timeout increments at 7812 kHz, in loop cycle chunks
		// After SERVO_OVERDUE of T2 cycles, 3906 * 1/7812 = 500ms
		Servo_Timeout += (uint8_t) (TCNT2 - Servo_TCNT2);
		Servo_TCNT2 = TCNT2;
		if (Servo_Timeout > SERVO_OVERDUE)
		{
			Overdue = true;
		}

		// Servo_Rate increments at 7812 kHz, in loop cycle chunks
		// After SERVO_RATE of T2 cycles, 156 * 1/7812 = 20ms
		Servo_Rate += (uint8_t) (TCNT2 - ServoRate_TCNT2);
		ServoRate_TCNT2 = TCNT2;
		if (Servo_Rate > SERVO_RATE)
		{
			ServoTick = true;
		}

		// Check for failsafe condition (Had RC lock but now overdue)
		if (Overdue && RC_Lock)
		{
			Failsafe = true;
			RC_Lock = false;
		}

		// Set failsafe positions when RC lock lost - disable for GUI mode
		// as GUI tends to trigger failsafe as the loop rate is slower.
		if (Failsafe && !GUIconnected)
		{
			ServoOut1 = Config.Failsafe_1;
			ServoOut2 = Config.Failsafe_2;
			Throttle  = Config.Failsafe_3;
			ServoOut4 = Config.Failsafe_4;
			ServoOut5 = Config.Failsafe_5;
			ServoOut6 = Config.Failsafe_6;
		}

		// Ensure that output_servo_ppm() is synchronised to the RC interrupts
		// Inhibit servos while GUI connected
		if (Interrupted && !GUIconnected)
		{
			Interrupted = false;			// Reset interrupted flag

			Failsafe = false;				// Cancel failsafe
			Servo_Timeout = 0;				// Reset servo failsafe timeout
			Overdue = false;				// And no longer overdue...

			#ifndef ICP_CPPM_MODE
			// Short delay to ensure no residual interrupt activity from ISR
			TIFR0 &= ~(1 << TOV0);			// Clear overflow
			TCNT0 = 0;						// Reset counter
			for (i=0;i<PWM_DELAY;i++)		// PWM_DELAY * 8us = 1ms
			{
				while (TCNT0 < 64);			// 1/8MHz * 64 = 8us
				TCNT0 -= 64;
			}
			#endif
			output_servo_ppm();				// Output servo signal
		}
		// If in failsafe, just output unsynchronised. Inhibit servos while GUI connected
		else if (Failsafe && Overdue && ServoTick && !GUIconnected)
		{
			ServoTick = false;				// Reset servo update ticker
			Servo_Rate = 0;					// Reset servo rate timer

			output_servo_ppm();				// Output servo signal
		}

		// Measure the current loop rate
		cycletime = TCNT1 - LoopStartTCNT1;	// Update cycle time for GUI
		LoopStartTCNT1 = TCNT1;				// Measure period of loop from here

	} // main loop
} // main()

