// **************************************************************************
// NeXtcopter Plus software
// ========================
// Version 1.1h
// Inspired by KKmulticopter
// Based on assembly code by Rolf R Bakke, and C code by Mike Barton
//
// Includes PID and Auto-level functions inspired by the open-sourced MultiWiiproject
// Compatible with KK Plus boards fitted with X and Y accelerometers 
// on Roll/Pitch pot inputs. LCD board or GUI required for setup of PID constants.
//
// Tested only on ATmega168 boards (KK+)
// Only QUAD(+), QUAD(X), HEXACOPTER and HEXACOPTER(X) supported at this stage
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

// Stick Centering
// ---------------
// Set Yaw Gain pot to max
// Set Tx trims to centre
// Power on
// LED flashes 3 times
// Restore gain pot
// Restart
//
// ESC throttle calibration
// ------------------------
// Set Yaw Gain pot to zero
// Put throttle stick to full
// Power on
// LED flashes 3 times
// Wait for motor signal (double beep)
// Set throttle to zero
// Wait for motor confirm signal
// Power off
// Restore gain pot
//
// Arming modes (all with throttle off)
// ------------
// YAW LEFT + PITCH NEUTRAL = Arm (Normal)
// YAW LEFT + PITCH FWD		= Arm (Acro)
// YAW LEFT + PITCH BACK	= Arm (Warthox)
// YAW LEFT + ROLL LEFT		= Arm + AUTOTUNE MODE (Needs GUI)
// YAW RIGHT 				= Disarm
// YAW RIGHT + PITCH FWD	= Enter LCD menu
// YAW RIGHT + PITCH BACK	= Leave LCD menu
// 
// **************************************************************************
// Version History
// ===============
// V1.0a	Based on NeXtcopter V1.0a code
// V1.1a 	Includes PID loop control on all axis
//			Added code for accelerometers and auto-level functions
//			Added support for custom MultiWii GUI
//			Added LCD menu system
//			Added Low Voltage Alarm support for LEDs or piezo buzzer
//			Increased RC input resolution through the FC calculations
// V1.1b	Fixed D-term, rate set bugs. Autolevel now 2 x PI loops
//			Added support for 8-ch CPPM input on ELEV input (use CPPM_MODE define)
//			Removed UFO mode, replaced with Warthox mode
//			Moved all menu text and support routines to Program memory
// V1.1c	Added exponential rate system. Autotune now updates LCD TX rate
//			Added formatted battery voltage displays to menu
//			Added support for V1.1 MEMS module (use MEMS_MODULE define)
// V1.1d	Preliminary support for proximity module (use PROX_MODULE define)
//			Height set when entering HeightHold mode (CH7 > 50%)
//			Added Hexacopter, Hexacopter-X support, Vref now assumed to be 2.4V
// V1.1e	Added exponential setting for accelerometer.
//			Added GUI lockout after about 10 seconds after power-up.
//			Added lost model alarm. Fixed stick centering bug.
// V1.1f	Fixed GUI lockout bug when disarming during GUI mode.
// V1.1g	Updated default PID settings. Fixed GUI cycle count.
//			Changed "Normal" sensitivity mode to "User" mode, fixed mode change issues.
//			Added experimental Y4 mode. Added CH5 input on M6 option in quad, Y4 modes.
//			As autolevel now switchable with both CPPM and 5-channel PWM, removed
//			Autolevel arming mode.
// V1.1h	Added NO_ACC compile option to permanently disable autolevel for boards
//			without acceleromaters.
//
//***********************************************************
//* To do
//***********************************************************
//
//	1. Get proximity sensor and Altitude Hold working, maybe...
//
//***********************************************************
//* Flight configurations (Motor number and rotation)
//***********************************************************

/*
Quad+                 Quad X
      M1 CW
        |             M1 CW     M2 CCW
        |                \        / 
      +---+                \ -- /
M2 ---|   |----M3          |    |
CCW   +---+    CCW         / -- \
        |                /        \ 
        |             M4 CCW    M3 CW
      M4 CW

Hexacopter            Hexacopter-X
 		 
       M1 CW
         |
M6 CCW   |     M2 CCW        M1 CW      M2 CCW
   \     |      /               \         /
     \ +---+  /                   \_____/
      -|   |-                     |     |
     / +---+  \       M6 CCW -----|     |----- M3 CW
   /	 |      \                 |_____|
M5 CW    |     M3 CW              /     \ 
         |                      /         \
       M4 CCW                M5 CW      M4 CCW


Y4

 M1CW        M2 CCW
    \         / 
      \ ___ /
       /   \
       \___/
         |
         |
        / \
   M4CCW   M3CW <-- Mounted at 30 degrees from vertical

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
#include "..\inc\motors.h"
#include "..\inc\pots.h"
#include "..\inc\gyros.h"
#include "..\inc\init.h"
#include "..\inc\acc.h"
#include "..\inc\lcd.h"
#include "..\inc\uart.h"
#include "..\inc\isr.h"
#include "..\inc\proximity.h"

//***********************************************************
//* Defines
//***********************************************************

// Motors
#define MAX_COLLECTIVE 190			// Limits the maximum stick collective (range 160->200 200=Off)
#define MOTOR_IDLE 20				// Minimum motor speed allowed.
#define DEAD_BAND 10				// Centre region of RC input where no activity is processed

// PID constants
#define ITERM_LIMIT_RP 250			// Max I-term sum for Roll/Pitch axis in normal modes
#define ITERM_LIMIT_YAW 500			// Max I-term sum for Yaw axis (Heading hold)
#define ITERM_LIMIT_LEVEL 250		// Max I-term sum for Roll/Pitch axis in AUTOLEVEL mode
#define ALT_FALL_LIMIT 100			// Limit altitude contribution to avoid saturation (falling)
#define ALT_RISE_LIMIT 25			// Limit altitude contribution to avoid saturation (rising)
#define YAW_LIMIT 50				// Limit yaw contribution to avoid saturation
#define AVGLENGTH 65				// Accelerometer filter buffer length + 1 (64 new + 1 old)

//#define AVGLENGTH 17

#ifdef PROX_MODULE
// Altitude constants
#define MAX_HEIGHT 350				// Maximum allowable height (in cm) in alt-hold mode
#endif

// Misc
#define STICKARM_POINT 200			// Defines how far stick must be moved (for 16-bit accurate RC)
#define GUI_TIMEOUT 3000			// Time after which GUI entry is impossible (3000 ~ 10 sec)
#define MAXDELTA 5					// Maximum instantaneous change allowed for raw ACC readings 

//***********************************************************
//* Code and Data variables
//***********************************************************

// Axis variables
int32_t Roll;						// Temp axis values. Seems we need 32 bits here.
int32_t Pitch;
int32_t Yaw;
int8_t AccRollTrim;					// Normalised acc trim 
int8_t AccPitchTrim;
int8_t AvgIndex;
int8_t OldIndex;
int32_t AvgRollSum;	
int32_t AvgPitchSum;
int16_t AvgRoll;
int16_t AvgPitch;
int16_t AvgAccRoll[AVGLENGTH];		// Circular buffer of historical accelerometer roll readings
int16_t AvgAccPitch[AVGLENGTH];		// Circular buffer of historical accelerometer pitch readings
#ifdef PROX_MODULE
int16_t Altitude;					// Current height as measured by the proximity module
int16_t Set_height;					// Desired height
int16_t AvgAltitude[AVGLENGTH];		// Circular buffer of historical altimeter readings
int32_t AvgAltSum;
int16_t AvgAlt;
uint8_t AltIndex;
uint8_t OldAltIndex;
uint8_t AltLimit; 
#endif
uint8_t	RollPitchRate;				// Current Roll/Pitch rate
uint8_t	Yawrate;					// Current Yaw rate

int16_t TargetRoll;
int16_t OldTargetRoll;
int16_t TargetPitch;
int16_t OldTargetPitch;

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
bool GUIconnected;
uint8_t flight_mode;				// Global flight mode flag
uint16_t cycletime;					// Data TX cycle counter
uint16_t LoopStartTCNT1, LoopElapsedTCNT1, LoopCurrentTCNT1;

// Misc
bool AutoLevel;						// AutoLevel = 1
bool HeightHold;					// HeightHold = 1
bool LCD_active;					// Mode flags
bool freshmenuvalue;
bool GUI_lockout;					// Lockout the GUI after GUI_TIMEOUT
bool Model_lost;					// Model lost flag
bool LMA_Alarm;						// Lost model alarm active
bool LVA_Alarm;						// Low voltage alarm active
bool firsttimeflag;
char pBuffer[16];					// Print buffer
uint16_t Change_Arming;				// Arming timers
uint16_t Change_LCD;
uint32_t Change_LostModel;
uint8_t Arming_TCNT2;
uint8_t LCD_TCNT2;
uint8_t Lost_TCNT2;
uint8_t rxin;
uint8_t MenuItem;
int16_t MenuValue;
uint16_t uber_loop_count;			// Used to count main loops for everything else :)

// Test code
uint8_t lcd_temp;
uint16_t lcd_tempi;


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
// Test code
//************************************************************
if (0) 
{
	while(1)
	{
		// Display results
		LCDgoTo(2);								
		LCDprintstr(" ");
		LCDgoTo(2);
		if (M5) LCDprintstr("1");
		else LCDprintstr("0");
		//
		LCDgoTo(4);								
		LCDprintstr("     ");
		LCDgoTo(4);
		LCDprintstr(itoa(RxChannel5,pBuffer,10));
		//
		LCDgoTo(12);					
		LCDprintstr("    ");
		LCDgoTo(12);
		LCDprintstr(itoa(RxChannel1,pBuffer,10));
		//
		lcd_tempi = accADC[ROLL];			
		LCDgoTo(20);
		LCDprintstr("    ");
		LCDgoTo(20);
		LCDprintstr(itoa(RxChannel2,pBuffer,10));
		//
		lcd_tempi = gyroADC[ROLL];			
		LCDgoTo(28);
		LCDprintstr("    ");
		LCDgoTo(28);
		LCDprintstr(itoa(RxChannel4,pBuffer,10));
		//
		_delay_ms(200);		
	}

/*
	uint16_t dist = 0;
	_delay_ms(1500);
	while (1)
	{
		LCDprint_line1("Pinging...      ");	
		Ping();								// Send out burst
	//	_delay_ms(50);						// Wait for response

		dist = GetDistance();				// Get latest distance measurement
		LCDprint_line2("Dist.       (cm)");
		LCDgoTo(22);
		LCDprintstr(itoa(dist,pBuffer,10));
	//	_delay_ms(50);						// Mustn't sent out pulse within 50ms of previous pulse
	}
*/

}

//************************************************************
// /Test code
//************************************************************

	for (int i = 0; i < AVGLENGTH; i++)		// Shouldn't need to do this but... we do
	{
		AvgAccRoll[i] = 0;					// Initialise all elements of the averaging arrays
		AvgAccPitch[i] = 0;
		#ifdef PROX_MODULE
		AvgAltitude[i] = 0;
		#endif
	}
	rxin = UDR0;							// Flush RX buffer

	// Main loop
	while (1)
	{
		AccRollTrim = Config.AccRollZeroTrim - 127; // GUI I/F bug won't pass signed numbers...
		AccPitchTrim = Config.AccPitchZeroTrim - 127;

		RxGetChannels();

		//************************************************************
		//* Switchable flight modes
		//************************************************************

		// Assign TX channels to functions
		#if defined(CPPM_MODE) && defined(PROX_MODULE)
			if (RxChannel7 > 1600)					// AUX 1 controls altitude hold
			{
				if (firsttimeflag) {
					HeightHold = true;
					Altitude = GetDistance();		// Get latest distance measurement
					Set_height = Altitude;			// Use this as target height
					firsttimeflag = false;			// Reset flag
				}
			}
			else
			{
				HeightHold = false;
				firsttimeflag = true;				// Reset first time flag
			}
		#endif // CPPM_MODE && PROX_MODULE

		#if defined(QUAD_COPTER) || defined(QUAD_X_COPTER) || defined(Y4) || defined(CPPM_MODE) // Where M6 is free
			if (RxChannel5 > 1600)					// CH5 controls autolevel
			{										// When CH5 is activated
				AutoLevel = true;					// Activate autolevel mode
				flight_mode |= 1;					// Notify GUI that mode has changed
				RollPitchRate = NORMAL_ROLL_PITCH_STICK_SCALE;
				Yawrate = NORMAL_YAW_STICK_SCALE;
			}
			else
			{	
				AutoLevel = false;					// Deactivate autolevel mode
				flight_mode &= 0xfe;				// Notify GUI that mode has changed
			}
		#endif // M6 is free

		#ifdef NO_ACC
			AutoLevel = false;						// No autolevel in NO_ACC mode
			flight_mode &= 0xfe;				// Notify GUI that mode has changed
		#endif

		//************************************************************
		//* Proximity module
		//************************************************************

		// Process proximity module data if fitted
		#ifdef PROX_MODULE
		if ((uber_loop_count &8) > 0)				// Check every 16ms or so (0.016 * 64 = 1.024s)
		{ 
			Ping();									// Get Altitude
			Altitude = GetDistance();

			// Average height readings properly to create a genuine low-pass filter
			AvgAltitude[AltIndex] = Altitude;		// Add new values into buffer

			//Calculate rolling average with latest 64 values
			AvgAltSum = AvgAltSum + Altitude - AvgAltitude[OldAltIndex]; // Add new value to sum, subtract oldest
			AvgAlt = (AvgAltSum >> 6); 				// Divide by 64 to get rolling average

			AltIndex ++;
			if (AltIndex >= AVGLENGTH) AltIndex = 0; // Wrap both indexes properly to create circular buffer
			OldAltIndex = AltIndex + 1;
			if (OldAltIndex >= AVGLENGTH) OldAltIndex = 0;
		}
		#endif

		//************************************************************
		//* GUI lockout
		//************************************************************

		// Lock GUI out after 10 seconds or so if not already in GUI mode and if armed
		if ((uber_loop_count > GUI_TIMEOUT) && (!GUIconnected) && (Armed)) 
		{
			GUI_lockout = true;
		}

		//************************************************************
		//* Alarms
		//************************************************************

		// Lost model alarm
		Change_LostModel += (uint8_t) (TCNT2 - Lost_TCNT2);
		Lost_TCNT2 = TCNT2;
		// Reset count if throttle non-zero or LCD/GUI active
		if ((RxInCollective > 0) || (LCD_active) || (GUIconnected))	
		{														
			Change_LostModel = 0;	
			Model_lost = false;																																																																																																																																																		
		}
		// Wait for 60s then trigger lost model alarm (468720 * 1/7812Hz = 60s)
		if (Change_LostModel > 468720)	Model_lost = true; //468720
		if (((uber_loop_count &128) > 0) && (Model_lost))
		{
			LMA_Alarm = 1;
		} 	
		else LMA_Alarm = 0;					// Otherwise turn off lost model alarm


		// Do LVA-related tasks
		if ((uber_loop_count &128) > 0)		// Check every 500ms or so
		{ 
			GetVbat();						// Get battery voltage
		}

		if (((Config.Modes &16) > 0) && (!Model_lost))	// Buzzer mode
		{
			// Beep buzzer if Vbat lower than trigger
			if ((vBat < Config.PowerTrigger) && ((uber_loop_count &128) > 0))	LVA_Alarm = 1; 	
			else LVA_Alarm = 0;					// Otherwise turn off buzzer
		}
		else if (((Config.Modes &16) == 0) && (!Model_lost))	// LED mode
		{	// Flash LEDs if Vbat lower than trigger
			if ((vBat < Config.PowerTrigger) && (((uber_loop_count >> 8) &2) > 0))	LVA_Alarm = 0; 	
			else LVA_Alarm = 1;					// Otherwise leave LEDs on
		}

		if (LVA_Alarm || LMA_Alarm)
		{
			LVA = 1;
		}
		else LVA = 0;


		//************************************************************
		//* GUI
		//************************************************************

		// Check to see if any requests are coming from the GUI
		if ((UCSR0A & (1 << RXC0)) && (!GUI_lockout)) // Data waiting in RX buffer and GUI not yet locked out
		{
			GUIconnected = true;			// Set GUI flag so that motors can be disabled
			rxin = rx_byte();				// Get RX byte
			switch(rxin) 
			{
				case 'M':					// Send MultiWii data
					ReadGyros();
					ReadAcc();
					send_multwii_data();	
					cycletime++;
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

		// LCD menu system - enter when not armed and pitch up and yaw right
		if ((RxInCollective == 0) && !Armed) 
		{
			// Check for stick hold
			Change_LCD += (uint8_t) (TCNT2 - LCD_TCNT2);
			LCD_TCNT2 = TCNT2;

			if ((RxInPitch > -STICKARM_POINT) || 					// Reset count if not up enough
				(RxInYaw < STICKARM_POINT))	
			{														// Reset count if not right enough
				Change_LCD = 0;			
			}

			freshmenuvalue = true;
			firsttimeflag = true;

			// 6000 * 1/7812Hz = 0.768s)
			if (Change_LCD > 6000)									// Has to be quicker than ARM timer
			{	
				LCD_active = true;
				MenuItem = 0;
				while (LCD_active) 									// Keep option of bailing out
				{
					// Wait for sticks to move back from initial state, display splash
					if (firsttimeflag) 
					{
						LCD_Display_Menu(MenuItem);
						LCDprint_line2(" V1.1h (c) 2012 ");
						_delay_ms(1000);
						firsttimeflag = false;
						MenuItem = 1;
					}
					
					RxGetChannels();								// Check sticks
					
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
					if (MenuItem == 21) 							// Special case for LVA mode
					{
						LCDprint_line2("          <-Save");			// Setup save line
						LCDgoTo(17);								// Position cursor at nice spot
						if (MenuValue == 0) LCDprintstr("LED");
						else LCDprintstr("Buzzer");
					}
					else if ((MenuItem == 20)||(MenuItem == 22))	// Special case for voltages
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
					else if ((MenuItem > 0) && (MenuItem < 23)) 	// Print value to change (except for base menu and commands)
					{
						LCDprint_line2("          <-Save");			// Setup save line
						LCDgoTo(17);								// Position cursor at nice spot
						LCDprintstr(itoa(MenuValue,pBuffer,10)); 	
					}
					else if (MenuItem >= 23)						// For commands
					{
						LCDprint_line2("      <- Execute");	
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
							LCDclear();
							Change_LCD = 0;
							Change_Arming = 0;
						}
						else LCD_active = true;
					}
				} // While LCD mode
			} // LCD activated
		} // LCD menu system

		//************************************************************
		//* Arming modes
		//************************************************************

		// Process arming and stick-selected flight modes
		if (RxInCollective == 0) {
			// Check for stick arming (Timer2 @ 8MHz/1024 = 7812.5KHz)
			Change_Arming += (uint8_t) (TCNT2 - Arming_TCNT2);
			Arming_TCNT2 = TCNT2;

			// Change_Arming will keep being reset *unless* held withing the arm/disarm
			// positions. If so, Change_Arming will start to increase.
			if (Armed) 
			{
				if (RxInYaw<STICKARM_POINT) Change_Arming = 0;		// Reset count
			} 
			else 
			{
				if (RxInYaw>-STICKARM_POINT) Change_Arming = 0;		// Reset count
			}

			// Reset I-term sums at rest.
			IntegralgPitch = 0;	 
			IntegralgRoll = 0;
			IntegralaPitch = 0;	 
			IntegralaRoll = 0;
			IntegralYaw = 0;

			// 1 sec, about 8000. (8000 * 1/7812Hz = 1.024s)
			if (Change_Arming > 8000)
			{
				Armed = ! Armed;
				LED = 0;
				AutoLevel = false;
				if (Armed) 
				{
					CalibrateGyros();						// Initialise everything
					IntegralgPitch = 0;	 
					IntegralgRoll = 0;
					IntegralaPitch = 0;	 
					IntegralaRoll = 0;
					IntegralYaw = 0;
					
					uint8_t nBlink = 1;

					if (RxInPitch > STICKARM_POINT) 		// ACRO
					{
						RollPitchRate = ACRO_ROLL_PITCH_STICK_SCALE;
						Yawrate = ACRO_YAW_STICK_SCALE;
						flight_mode |= 4;
						nBlink = 3;
		 			}
			 		else if (RxInPitch < -STICKARM_POINT) 	// WARTHOX
			 		{
						RollPitchRate = WARTHOX_ROLL_PITCH_STICK_SCALE;
						Yawrate = WARTHOX_YAW_STICK_SCALE;
						flight_mode |= 8;
						nBlink = 2;
			 		}
					else if (RxInRoll < -STICKARM_POINT)	// Autotune mode
					{
						Armed = 0;							// Disarm for Autotune
						autotune();
						init_uart();
						Save_Config_to_EEPROM(); 			// Save to eeProm	
						LED = !LED;
						_delay_ms(500);
						LED = !LED;
						nBlink = 2;
					}
			 		else // NORMAL
			 		{
						// Use user-defined rates set from LCD or GUI
						RollPitchRate = Config.RollPitchRate;
						Yawrate = Config.Yawrate;
						flight_mode |= 2;
						nBlink = 1;
			 		}

			 		// Flash LED to indicate flight mode
					for (uint8_t i=0;i<nBlink;i++)
					{
						LED = 1;
						_delay_ms(400);
						LED = 0;
						_delay_ms(400);
					}

					if (Armed) LED = 1; // Light LED to indicate armed.

				} // if (Armed)
				else 
				{								// If newly disarmed
					AutoLevel = false;
					GUIconnected = false;		// Disconnect GUI
					LED = 0;
					flight_mode = 0;
				}
			} // if (Change_Arming)
		} // if ( RxInCollective == 0)

		//************************************************************
		//* Flight control
		//************************************************************

		//--- Read sensors ---
		ReadGyros();
		// Only read Accs if in AutoLevel mode
		if (AutoLevel) ReadAcc();

		//--- Start mixing by setting collective to motor input 1,2,3 and 4 ---
		if (RxInCollective > MAX_COLLECTIVE) RxInCollective = MAX_COLLECTIVE;
		MotorOut1 = RxInCollective;
		MotorOut2 = RxInCollective;
		MotorOut3 = RxInCollective;
		MotorOut4 = RxInCollective; 
		MotorOut5 = RxInCollective;
		MotorOut6 = RxInCollective;

		// Accelerometer low-pass filter + magic
		if (AutoLevel) 
		{
			// Prefilter the raw ACC data coming in to remove spikes, limit slew rate
			if ((AvgRoll - accADC[ROLL]) >  10 ) accADC[ROLL] = AvgRoll - MAXDELTA; // Lower than avg
			if ((AvgRoll - accADC[ROLL]) < -10 ) accADC[ROLL] = AvgRoll + MAXDELTA; // Higher than avg
			if ((AvgPitch - accADC[PITCH]) >  10 ) accADC[PITCH] = AvgPitch - MAXDELTA; // Lower than avg
			if ((AvgPitch - accADC[PITCH]) < -10 ) accADC[PITCH] = AvgPitch + MAXDELTA; // Higher than avg

			// Average accelerometer readings properly to create a genuine low-pass filter
			// Note that this has exactly the same effect as a complementary filter but with vastly less overhead.
			AvgAccRoll[AvgIndex] = accADC[ROLL];
			AvgAccPitch[AvgIndex] = accADC[PITCH];	// Add new values into buffer

			//Calculate rolling average with latest values
			AvgRollSum = AvgRollSum + accADC[ROLL] - AvgAccRoll[OldIndex]; // Add new value to sum, subtract oldest
			AvgPitchSum = AvgPitchSum + accADC[PITCH] - AvgAccPitch[OldIndex];

			AvgRoll = ((AvgRollSum >> 6) - AccRollTrim); // Divide by 64 to get rolling average then adjust for acc trim
			AvgPitch = ((AvgPitchSum >> 6) - AccPitchTrim);

			//AvgRoll = ((AvgRollSum >> 4) - AccRollTrim); // Divide by 16 to get rolling average then adjust for acc trim
			//AvgPitch = ((AvgPitchSum >> 4) - AccPitchTrim);

			AvgIndex ++;
			if (AvgIndex >= AVGLENGTH) AvgIndex = 0; // Wrap both indexes properly to create circular buffer
			OldIndex = AvgIndex + 1;
			if (OldIndex >= AVGLENGTH) OldIndex = 0;

			// Dynamic damping about level point
			// For Avg values between -10 and 10, only allow a gradual change in value
			if ((AvgRoll > 10) || (AvgRoll < -10)) TargetRoll = AvgRoll;
			else if (TargetRoll > OldTargetRoll) 
			{
				AvgRoll += 1;
			}
			else if (TargetRoll < OldTargetRoll) 
			{
				AvgRoll -= 1;
			}

			if ((AvgPitch > 10) || (AvgPitch < -10)) TargetPitch = AvgPitch;
			else if (TargetPitch > OldTargetPitch) 
			{
				AvgPitch += 1;
			}
			else if (TargetPitch < OldTargetPitch) 
			{
				AvgPitch -= 1;
			}
			OldTargetPitch = TargetPitch;
		}

		//***********************************************************************
		//                --- Calculate roll gyro output ---
		// NB: IF YOU CHANGE THIS CODE, YOU MUST REMOVE PROPS BEFORE TESTING !!!
		//***********************************************************************

		if ((RxInRoll < DEAD_BAND) && (RxInRoll > -DEAD_BAND)) RxInRoll = 0; // Reduce RxIn noise into the I-term
		RxInRoll = RxInRoll >> RollPitchRate;					// Reduce RxInRoll rate per flying mode
		Roll = RxInRoll + gyroADC[ROLL];

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
			P_term_aRoll = get_acc_expo_value(AvgRoll) * Config.P_mult_alevel;		// P-term of accelerometer (Max gain of 256)

			IntegralaRoll += AvgRoll;							// Acc I-term
			if (IntegralaRoll > ITERM_LIMIT_LEVEL) IntegralaRoll = ITERM_LIMIT_LEVEL; // Anti wind-up limit
			else if (IntegralaRoll < -ITERM_LIMIT_LEVEL) IntegralaRoll = -ITERM_LIMIT_LEVEL;
			I_term_aRoll = IntegralaRoll * Config.I_mult_alevel;// Multiply I-term (Max gain of 256)
			I_term_aRoll = I_term_aRoll >> 3;					// Divide by 8, so max effective gain is 16
	
			// Sum Gyro P and I terms + Acc P and I terms
			Roll = P_term_gRoll + I_term_gRoll - P_term_aRoll - I_term_aRoll;
			Roll = Roll >> 7;									// Divide by 128 to rescale values back to normal

		}
		else // Normal mode (Just use raw gyro errors to guess at attitude)
		{
			// Gyro PID terms
			P_term_gRoll = Roll * Config.P_mult_roll;			// Multiply P-term (	Max gain of 256)
			P_term_gRoll = P_term_gRoll * 3;					// Multiply by 3, so max effective gain is 768

			I_term_gRoll = IntegralgRoll * Config.I_mult_roll;	// Multiply I-term (Max gain of 256)
			I_term_gRoll = I_term_gRoll >> 3;					// Divide by 8, so max effective gain is 16

			currentError[ROLL] = Roll;							// D-term
			DifferentialGyro = currentError[ROLL] - lastError[ROLL];
			lastError[ROLL] = currentError[ROLL];
			DifferentialGyro *= Config.D_mult_roll;				// Multiply D-term by up to 256

			// Sum	
			Roll = P_term_gRoll + I_term_gRoll + DifferentialGyro; // P + I + D
			Roll = Roll >> 7;									// Divide by 128 to rescale values back to normal
		}

		//--- (Add)Adjust roll gyro output to motors
		#ifdef QUAD_COPTER
		MotorOut2 += Roll;
		MotorOut3 -= Roll;
		#elif defined(QUAD_X_COPTER)
		Roll	= (Roll >> 1);	
		MotorOut1 += Roll;
		MotorOut2 -= Roll;
		MotorOut3 -= Roll;
		MotorOut4 += Roll;
		#elif defined(HEXA_COPTER)
		Roll	= (Roll * 7) >> 3;	//RxInRoll  *= 0.875 (Sine 60 = 0.866)
		MotorOut2 -= Roll;
		MotorOut3 -= Roll;
		MotorOut5 += Roll;
		MotorOut6 += Roll;
		#elif defined(HEXA_X_COPTER)
		MotorOut3 -= Roll;
		MotorOut6 += Roll;
		Roll	= (Roll >> 1);		//RxInRoll  *= 0.5 (Sine 30 = 0.5)
		MotorOut1 += Roll;
		MotorOut2 -= Roll;
		MotorOut4 -= Roll;
		MotorOut5 += Roll;
		#elif defined(Y4)
		MotorOut1 += Roll;
		MotorOut2 -= Roll;
		#else
		#error No Copter configuration defined !!!!
		#endif

		//***********************************************************************
		//                --- Calculate pitch gyro output ---
		// NB: IF YOU CHANGE THIS CODE, YOU MUST REMOVE PROPS BEFORE TESTING !!!
		//***********************************************************************

		if ((RxInPitch < DEAD_BAND) && (RxInPitch > -DEAD_BAND)) RxInPitch = 0; // Reduce RxIn noise into the I-term
		RxInPitch = RxInPitch >> RollPitchRate;					// Reduce RxInPitch rate per flying mode
		Pitch = RxInPitch + gyroADC[PITCH];

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
			P_term_aPitch = get_acc_expo_value(AvgPitch) * Config.P_mult_alevel;	// P-term of accelerometer (Max gain of 256)

			IntegralaPitch += AvgPitch;							// Acc I-term
			if (IntegralaPitch > ITERM_LIMIT_LEVEL) IntegralaPitch = ITERM_LIMIT_LEVEL; // Anti wind-up limit
			else if (IntegralaPitch < -ITERM_LIMIT_LEVEL) IntegralaPitch = -ITERM_LIMIT_LEVEL;
			I_term_aPitch = IntegralaPitch * Config.I_mult_alevel;// Multiply I-term (Max gain of 256)
			I_term_aPitch = I_term_aPitch >> 3;					// Divide by 8, so max effective gain is 16

			// Sum Gyro P and I terms + Acc P and I terms
			Pitch = P_term_gPitch + I_term_gPitch - P_term_aPitch - I_term_aPitch;
			Pitch = Pitch >> 7;									// Divide by 128 to rescale values back to normal
		}
		else // Normal mode (Just use raw gyro errors to guess at attitude)
		{
			// Gyro PID terms
			P_term_gPitch = Pitch * Config.P_mult_pitch;		// Multiply P-term (Max gain of 256)
			P_term_gPitch = P_term_gPitch * 3;					// Multiply by 3, so max effective gain is 768

			I_term_gPitch = IntegralgPitch * Config.I_mult_pitch;// Multiply I-term (Max gain of 256)
			I_term_gPitch = I_term_gPitch >> 3;					// Divide by 8, so max effective gain is 16

			currentError[PITCH] = Pitch;						// D-term
			DifferentialGyro = currentError[PITCH] - lastError[PITCH];
			lastError[PITCH] = currentError[PITCH];	
			DifferentialGyro *= Config.D_mult_pitch;			// Multiply D-term by up to 256

			// Sum
			Pitch = P_term_gPitch + I_term_gPitch + DifferentialGyro;// P + I + D
			Pitch = Pitch >> 7;									// Divide by 128 to rescale values back to normal
		}

		//--- (Add)Adjust pitch gyro output to motors
		#ifdef QUAD_COPTER
		MotorOut1 += Pitch;
		MotorOut4 -= Pitch;
		#elif defined(QUAD_X_COPTER)
		Pitch	= (Pitch >> 1);	
		MotorOut1 += Pitch;
		MotorOut2 += Pitch;
		MotorOut3 -= Pitch;
		MotorOut4 -= Pitch;
		#elif defined(HEXA_COPTER)
		MotorOut1 += Pitch;
		MotorOut4 -= Pitch;
		Pitch = (Pitch >> 1); 		// (Sine 30 = 0.5)
		MotorOut2 += Pitch;	
		MotorOut3 -= Pitch;
		MotorOut5 -= Pitch;
		MotorOut6 += Pitch;
		#elif defined(HEXA_X_COPTER)
		Pitch	= (Pitch * 7) >> 3;	//RxInPitch  *= 0.875 (Sine 60 = 0.866)
		MotorOut1 += Pitch;
		MotorOut2 += Pitch;
		MotorOut4 -= Pitch;
		MotorOut5 -= Pitch;
		#elif defined(Y4)
		Pitch	= (Pitch >> 1);	
		MotorOut1 += Pitch;
		MotorOut2 += Pitch;
		MotorOut3 -= Pitch;
		MotorOut4 -= Pitch;
		#else
		#error No Copter configuration defined !!!!
		#endif

		//***********************************************************************
		//                 --- Calculate yaw gyro output ---
		// NB: IF YOU CHANGE THIS CODE, YOU MUST REMOVE PROPS BEFORE TESTING !!!
		//***********************************************************************

		if ((RxInYaw < DEAD_BAND) && (RxInYaw > -DEAD_BAND)) RxInYaw = 0; // Reduce RxIn noise into the I-term
		RxInYaw = RxInYaw >> Yawrate;						// Reduce RxInYaw rate per flying mode
		Yaw = RxInYaw + gyroADC[YAW];	

		IntegralYaw += Yaw;									// I-term (32-bit)
		if (IntegralYaw > ITERM_LIMIT_YAW) IntegralYaw = ITERM_LIMIT_YAW;
		else if (IntegralYaw < -ITERM_LIMIT_YAW) IntegralYaw = -ITERM_LIMIT_YAW;// Anti wind-up (Experiment with value)

		Yaw *= Config.P_mult_yaw;							// Multiply P-term (Max gain of 768)
		Yaw *= 3;	

		I_term_Yaw = IntegralYaw * Config.I_mult_yaw;		// Multiply IntegralYaw by up to 256
		I_term_Yaw = I_term_Yaw >> 3;						// Divide by 8, so max effective gain is 16

		currentError[YAW] = Yaw;							// D-term
		DifferentialGyro = currentError[YAW] - lastError[YAW];
		lastError[YAW] = currentError[YAW];	
		DifferentialGyro *= Config.D_mult_yaw;				// Multiply D-term by up to 256

		Yaw = Yaw - I_term_Yaw - DifferentialGyro;			// P + I + D
		Yaw = Yaw >> 7;										// Divide by 128 to rescale values back to normal

		if (Yaw > YAW_LIMIT) Yaw = YAW_LIMIT;		
		else if (Yaw < -YAW_LIMIT) Yaw = -YAW_LIMIT; 		// Apply YAW limit to PID calculation

		//--- (Add)Adjust yaw gyro output to motors
		#ifdef QUAD_COPTER
		MotorOut1 -= Yaw;
		MotorOut2 += Yaw;
		MotorOut3 += Yaw;
		MotorOut4 -= Yaw;
		#elif defined(QUAD_X_COPTER)
		MotorOut1 -= Yaw;
		MotorOut2 += Yaw;
		MotorOut3 -= Yaw;
		MotorOut4 += Yaw;
		#elif defined(HEXA_COPTER) || defined(HEXA_X_COPTER)
		MotorOut1 -= Yaw;
		MotorOut2 += Yaw;
		MotorOut3 -= Yaw;
		MotorOut4 += Yaw;
		MotorOut5 -= Yaw;
		MotorOut6 += Yaw;
		#elif defined(Y4)
		MotorOut3 += Yaw;
		MotorOut4 -= Yaw;
		#else
		#error No Copter configuration defined !!!!
		#endif

		#ifdef PROX_MODULE
		//***********************************************************************
		//                 --- Calculate height adjustment ---
		// NB: IF YOU CHANGE THIS CODE, YOU MUST REMOVE PROPS BEFORE TESTING !!!
		//***********************************************************************

		currentError[ALT] = AvgAlt;							// Save current height
		if (AvgAlt > MAX_HEIGHT) AvgAlt = MAX_HEIGHT;
		if (AvgAlt < 0) AvgAlt = 0;

		Altitude = Set_height - AvgAlt;						// Calculate target height error (-ve = too high, +ve = too low)
		DifferentialGyro = currentError[ALT] - lastError[ALT]; 	// Moderate acceleration up or down 
																// +ve = falling, -ve = rising
		Altitude *= 10;										// Multiply gain

		if (DifferentialGyro < 0)							// Reduce power limit when rising
		{
			AltLimit = ALT_RISE_LIMIT;
		}
		else												// Increase power limit when falling
		{
			AltLimit = ALT_FALL_LIMIT;
		}

		lastError[ALT] = currentError[ALT];					// Save previous error 

		Altitude = Altitude - DifferentialGyro;				// P + D - Adds when falling, subtracts when rising
		Altitude = Altitude >> 1;							// Average P and D terms

		if (Altitude > AltLimit) Altitude = AltLimit;		
		else if (Altitude < -AltLimit) Altitude = -AltLimit;// Apply ALT_LIMIT to PID calculation

		// Only in HeightHold mode and when height under maximum controllable
		if ((HeightHold) && (currentError[ALT] <= MAX_HEIGHT))
		{
			//--- (Add)Adjust Altitude output to motors
			#ifdef QUAD_COPTER
			MotorOut1 += Altitude;
			MotorOut2 += Altitude;
			MotorOut3 += Altitude;
			MotorOut4 += Altitude;
			#elif defined(QUAD_X_COPTER)
			MotorOut1 += Altitude;
			MotorOut2 += Altitude;
			MotorOut3 += Altitude;
			MotorOut4 += Altitude;
			#elif defined(HEXA_COPTER) || defined(HEXA_X_COPTER)
			MotorOut1 += Altitude;
			MotorOut2 += Altitude;
			MotorOut3 += Altitude;
			MotorOut4 += Altitude;
			MotorOut5 += Altitude;
			MotorOut6 += Altitude;
			#else
			#error No Copter configuration defined !!!!
			#endif
		}

		#endif // PROX_MODULE

		//--- Limit the lowest value to avoid stopping of motor if motor value is under-saturated ---
		if ( MotorOut1 < MOTOR_IDLE )	MotorOut1 = MOTOR_IDLE;	
		if ( MotorOut2 < MOTOR_IDLE )	MotorOut2 = MOTOR_IDLE;	
		if ( MotorOut3 < MOTOR_IDLE )	MotorOut3 = MOTOR_IDLE;
		if ( MotorOut4 < MOTOR_IDLE )	MotorOut4 = MOTOR_IDLE;
		if ( MotorOut5 < MOTOR_IDLE )	MotorOut5 = MOTOR_IDLE;	
		if ( MotorOut6 < MOTOR_IDLE )	MotorOut6 = MOTOR_IDLE;	

		//--- Output to motor ESC's ---
		if (RxInCollective < 1 || !Armed)	// Turn off motors if collective below 1% 
		{		
			MotorOut1 = 0;
			MotorOut2 = 0;
			MotorOut3 = 0;
			MotorOut4 = 0;
			MotorOut5 = 0;
			MotorOut6 = 0;
		}
		
		// Calculate loop period (LoopCurrentTCNT1 - LoopStartTCNT1)
		// Also, handle the odd case where the TCNT1 rolls over and TCNT1 < LoopStartTCNT1
		LoopCurrentTCNT1 = TCNT1;
		if (LoopCurrentTCNT1 > LoopStartTCNT1) LoopElapsedTCNT1 = LoopCurrentTCNT1 - LoopStartTCNT1;
		else LoopElapsedTCNT1 = (0xffff - LoopStartTCNT1) + LoopCurrentTCNT1;

		cycletime = LoopElapsedTCNT1;	// Update cycle time for GUI
		LoopStartTCNT1 = TCNT1;			// Measure period of loop from here

		if (Armed) output_motor_ppm();	// Output ESC signal
		uber_loop_count++;

	} // main loop
} // main()

