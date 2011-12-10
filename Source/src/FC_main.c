// **************************************************************************
// NeXtcopter Plus software
// ========================
// Version 1.1a
// Inspired by KKmulticopter
// Based on assembly code by Rolf R Bakke, and C code by Mike Barton
//
// Includes PID and Auto-level functions inspired by the open-sourced MultiWiiproject
//
// Compatible with KK Plus boards fitted with X and Y accelerometers 
// on Roll/Pitch pot inputs. LCD board or GUI required for setup of PID constants.
//
// Tested only on ATmega168 boards (KK+)
// Only QUAD(+) and QUAD(X) supported at this stage
//
// **************************************************************************
// * 						GNU GPL V3 notice
// **************************************************************************
// * Copyright (C) 2011 David Thompson, based on work by Rolf R Bakke,
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
// YAW LEFT + ROLL RIGHT	= Arm (Autolevel)
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
// V1.1b	Fixed D-term, rate set bugs
//			Added support for 8-ch CPPM input on ELEV input
//			Removed UFO mode, replaced with Warthox mode
//			Moved all Menu text and support routines to Program memory
//
//***********************************************************
//* Flight configurations (Motor number and rotation)
//***********************************************************

/*
Quad +
      M1 CW
        |
        |
      +---+
M2 ---|   |----M3
CCW   +---+    CCW
        |
        |
      M4 CW

Quad X
 		 
  M1 CW   M2 CCW
    \        / 
      \ -- /
      |    |
      / -- \
    /        \ 
  M4 CCW  M3 CW

*/

//***********************************************************
//* Includes
//***********************************************************

#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <util/delay.h>
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
#include "..\inc\isr.h" // Debug

//***********************************************************
//* Compilation options
//***********************************************************

//#define QUAD_COPTER		// Choose this for + config
#define QUAD_X_COPTER		// Choose this for X config

//***********************************************************
//* Defines
//***********************************************************

// Max Collective
// Limits the maximum stick collective (range 160->200 200=Off)
// This allows gyros to stabilise better when full throttle applied
#define MAX_COLLECTIVE 190
#define MOTOR_IDLE 20				// Minimum motor speed allowed.
#define DEAD_BAND 10				// Centre region of RC input where no activity is processed

// PID constants
#define ITERM_LIMIT_RP 250			// Max I-term sum for Roll/Pitch axis in normal modes
#define ITERM_LIMIT_YAW 5000		// Max I-term sum for Yaw axis (Heading hold)
#define ITERM_LIMIT_LEVEL 1000		// Max I-term sum for Roll/Pitch axis in AUTOLEVEL mode
#define PR_LIMIT 50					// Limit pitch/roll contribution of I-term to avoid saturation
#define YAW_LIMIT 50				// Limit yaw contribution to avoid saturation
#define AVGLENGTH 65				// Accelerometer filter buffer length + 1 (64 new + 1 old)

// Stick Arming
// If you cannot either arm or disarm, lower this value
#define STICKARM_POINT 200			// Defines how far stick must be moved (for 16-bit accurate RC)

//***********************************************************
//* Code and Data variables
//***********************************************************

int32_t	IntegralPitch;			// PID I-terms for each axis
int32_t	IntegralRoll;
int32_t	IntegralYaw;
int32_t	IntegralRollAngle;		// Constantly integrated roll angle I-term
int32_t	IntegralPitchAngle; 	// Constantly integrated pitch angle I-term
uint16_t cycletime;			// Data TX cycle counter
bool AutoLevel;				// AutoLevel = 1
bool GUIconnected;
//
int16_t AvgAccRoll[AVGLENGTH];			// Circular buffer of historical accelerometer roll readings
int16_t AvgAccPitch[AVGLENGTH];			// Circular buffer of historical accelerometer pitch readings
//
uint16_t Change_Arming;		// Arming timers
uint16_t Change_LCD;
uint8_t Arming_TCNT2;
//
bool LCD_active;				// Mode flags
bool freshmenuvalue;
bool firsttimeflag;
char pBuffer[16];						// Print buffer
//
int32_t Roll;							// Calculated P-terms for each axis
int32_t Pitch;
int32_t Yaw;
int32_t I_term_Roll;				// Calculated I-terms for each axis
int32_t I_term_Pitch;
int32_t I_term_Yaw;
//
int16_t currentGyroError[3];	// Used with lastGyroError to keep track of D-Terms in PID calculations
int16_t lastGyroError[3];
int16_t DifferentialGyro;			// Holds difference between last two gyro errors (angular acceleration)
uint16_t uber_loop_count;			// Used to count main loops for everything else :)
//
int8_t AccRollTrim;					// Normalised acc trim 
int8_t AccPitchTrim;
//
uint8_t rxin;
uint8_t MenuItem;
int16_t MenuValue;


int8_t AvgIndex;
int8_t OldIndex;
int32_t AvgRollSum;					// Can be 16-bit
int32_t AvgPitchSum;
int16_t AvgRoll;
int16_t AvgPitch;


//************************************************************
// Main loop
//************************************************************

int main(void)
{
	OldIndex = 1;
	firsttimeflag = true;

	//
	init();									// Do all init tasks


//************************************************************
// Test code - PPM input test
//************************************************************
if (0) {
	_delay_ms(1500);					// To get past LCD splash screen
	char pBuffer[16];					// Print buffer
	int8_t i;
	while (1) 
	{
		// Print CH1 to CH4
		for (i=0; i < 50; i++)
		{
			LCDprint_line1("1:      2:      ");	
			LCDprint_line2("3:      4:      ");
			LCDgoTo(2);
			LCDprintstr(itoa(RxChannel1,pBuffer,10));
			LCDgoTo(10);
			LCDprintstr(itoa(RxChannel2,pBuffer,10));
			LCDgoTo(18);
			LCDprintstr(itoa(RxChannel3,pBuffer,10));
			LCDgoTo(26);
			LCDprintstr(itoa(RxChannel4,pBuffer,10));
			_delay_ms(100);
		}

		// Print RC inputs
		for (i=0; i < 50; i++)
		{
			LCDprint_line1("A:      E:      ");	
			LCDprint_line2("T:      Y:      ");	
			RxGetChannels();
			LCDgoTo(2);
			LCDprintstr(itoa(RxInRoll,pBuffer,10));
			LCDgoTo(10);
			LCDprintstr(itoa(RxInPitch,pBuffer,10));
			LCDgoTo(18);
			LCDprintstr(itoa(RxInCollective,pBuffer,10));
			LCDgoTo(26);
			LCDprintstr(itoa(RxInYaw,pBuffer,10));
			_delay_ms(100);
		}
	}
}
//************************************************************
// Test code - PPM input test
//************************************************************


	//
	for (int i = 0; i < AVGLENGTH; i++)		// Shouldn't need to do this but... we do
	{
		AvgAccRoll[i] = 0;					// Initialise all elements of the averaging arrays
		AvgAccPitch[i] = 0;
	}
	rxin = UDR0;							// Flush RX buffer

	// Main loop
	while (1)
	{
		AccRollTrim = Config.AccRollZeroTrim - 127; // GUI I/F bug won't pass signed numbers...
		AccPitchTrim = Config.AccPitchZeroTrim - 127;

		RxGetChannels();

		// Do LVA-related tasks
		LVA = 0;
		if ((uber_loop_count &128) > 0)		// Check every 500ms or so
		{ 
			GetVbat();						// Get battery voltage
		}

		if ((Config.Modes &16) > 0)			// Buzzer mode
		{
			// Beep buzzer if Vbat lower than trigger
			if ((vBat < Config.PowerTrigger) && ((uber_loop_count &128) > 0))	LVA = 1; 	
			else LVA = 0;					// Otherwise turn off buzzer
		}
		else if ((Config.Modes &16) == 0)	// LED mode
		{	// Flash LEDs if Vbat lower than trigger
			if ((vBat < Config.PowerTrigger) && (((uber_loop_count >> 8) &2) > 0))	LVA = 0; 	
			else LVA = 1;					// Otherwise leave LEDs on
		}

		// Check to see if any requests are coming from the GUI
		if (UCSR0A & (1 << RXC0)) 			// Data waiting in RX buffer (add test for (RxInCollective == 0))
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

		// LCD menu system - enter when not armed and pitch up and yaw right
		if ((RxInCollective == 0) && !Armed) {

			// Check for stick hold
			Change_LCD += (uint8_t) (TCNT2 - Arming_TCNT2);

			if ((RxInPitch > -STICKARM_POINT) || 					// Reset count if not up enough
				(RxInYaw < STICKARM_POINT))	
			{														// Reset count if not right enough
				Change_LCD = 0;			
			}

			LCD_active = true;
			freshmenuvalue = true;
			firsttimeflag = true;

			// 6000 * 1/7812Hz = 0.768s)
			if (Change_LCD > 6000)									// Has to be quicker than ARM timer
			{	
				MenuItem = 0;
				while (LCD_active) 									// Keep option of bailing out
				{
					// Wait for sticks to move back from initial state, display splash
					if (firsttimeflag) 
					{
						LCD_Display_Menu(MenuItem);
						LCDprint_line2("   (c) 2011     ");
						_delay_ms(1000);
						firsttimeflag = false;
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
					
					// Refresh changed data prior to delay to make LCD more repsonsive
					LCD_Display_Menu(MenuItem);						// Display menu top line
					LCDprint_line2("          <-Save");				// Setup save line
					LCDgoTo(17);									// Position cursor at nice spot
					if (MenuItem == 17) 							// Special case for LVA mode
					{
						if (MenuValue == 0) LCDprintstr("LED");
						else LCDprintstr("Buzzer");
					}
					else if ((MenuItem > 0) && (MenuItem != 18)) LCDprintstr(itoa(MenuValue,pBuffer,10)); // Print value to change (except for base menu)

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
			IntegralPitch = 0;	 
			IntegralRoll = 0;
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
					IntegralPitch = 0;	 
					IntegralRoll = 0;
					IntegralYaw = 0;
					
					uint8_t nBlink = 1;

					if (RxInPitch > STICKARM_POINT) 		// ACRO
					{
						Config.RollPitchRate = ACRO_ROLL_PITCH_STICK_SCALE;
						Config.Yawrate = ACRO_YAW_STICK_SCALE;
						nBlink = 3;
		 			}
			 		else if (RxInPitch < -STICKARM_POINT) 	// WARTHOX
			 		{
						Config.RollPitchRate = WARTHOX_ROLL_PITCH_STICK_SCALE;
						Config.Yawrate = WARTHOX_YAW_STICK_SCALE;
						nBlink = 2;
			 		}
					else if (RxInRoll > STICKARM_POINT) 	// Autolevel
					{
						AutoLevel = true;
						Config.RollPitchRate = NORMAL_ROLL_PITCH_STICK_SCALE;
						Config.Yawrate = NORMAL_YAW_STICK_SCALE;
						nBlink = 5;
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
				}
			} // if (Change_Arming)
		} // if ( RxInCollective == 0)


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
		// NB: IF YOU CHANGE THIS CODE, YOU MUST REMOVE PROPS BEFORE TESTING !!!
		//***********************************************************************

		if ((RxInRoll < DEAD_BAND) && (RxInRoll > -DEAD_BAND)) RxInRoll = 0; // Reduce RxIn noise into the I-term
		RxInRoll = RxInRoll >> Config.RollPitchRate;			// Reduce RxInRoll rate per flying mode
		Roll = RxInRoll + gyroADC[ROLL];

		if (AutoLevel) // Autolevel mode (Use averaged accelerometer to calculate attitude)
		{
			Roll *= Config.P_mult_level;						// Multiply P-term (Max gain of 256)
			Roll = Roll * 3;									// Multiply by 3, so max effective gain is 768
			Roll = Roll >> 7;									// Divide by 128 to rescale values back to normal

			I_term_Roll = AvgRoll * Config.I_mult_level;		// Not an I-term, just the acc offset
			I_term_Roll = I_term_Roll >> 4;

			if (I_term_Roll > PR_LIMIT) I_term_Roll = PR_LIMIT;		
			else if (I_term_Roll < -PR_LIMIT) I_term_Roll = -PR_LIMIT; 	// Apply Roll limit to PID calculation
	
			Roll = Roll - I_term_Roll;							// P + Acc correction offset
		}
		else // Normal mode (Just use raw gyro errors to guess at attitude)
		{
			IntegralRoll += Roll;						// I-term (32-bit)
			if (IntegralRoll > ITERM_LIMIT_RP) IntegralRoll = ITERM_LIMIT_RP;
			else if (IntegralRoll < -ITERM_LIMIT_RP) IntegralRoll = -ITERM_LIMIT_RP;// Anti wind-up

			Roll *= Config.P_mult_roll;					// Multiply P-term (-Max gain of 256)
			Roll = Roll * 3;							// Multiply by 3, so max effective gain is 768

			I_term_Roll = IntegralRoll * Config.I_mult_roll;	// Multiply I-term (Max gain of 256)
			I_term_Roll = I_term_Roll >> 2;				// Divide by 4, so max effective gain is 64

			//currentGyroError[ROLL] = gyroADC[ROLL];		// D-term
			currentGyroError[ROLL] = Roll;				// D-term
			DifferentialGyro = currentGyroError[ROLL] - lastGyroError[ROLL];
			lastGyroError[ROLL] = currentGyroError[ROLL];
			
			DifferentialGyro *= Config.D_mult_roll;		// Multiply D-term by up to 1024
			//DifferentialGyro = DifferentialGyro << 2;

	
			Roll = Roll + I_term_Roll + DifferentialGyro;// P + I + D
			Roll = Roll >> 7;							// Divide by 128 to rescale values back to normal
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
		#else
		#error No Copter configuration defined !!!!
		#endif

		//***********************************************************************
		//                --- Calculate pitch gyro output ---
		// NB: IF YOU CHANGE THIS CODE, YOU MUST REMOVE PROPS BEFORE TESTING !!!
		//***********************************************************************

		if ((RxInPitch < DEAD_BAND) && (RxInPitch > -DEAD_BAND)) RxInPitch = 0; // Reduce RxIn noise into the I-term
		RxInPitch = RxInPitch >> Config.RollPitchRate;			// Reduce RxInPitch rate per flying mode
		Pitch = RxInPitch + gyroADC[PITCH];

		if (AutoLevel) // Autolevel mode (Use averaged accelerometer to calculate attitude)
		{
			Pitch *= Config.P_mult_level;						// Multiply P-term (Max gain of 256)
			Pitch = Pitch * 3;									// Multiply by 3, so max effective gain is 768
			Pitch = Pitch >> 7;									// Divide by 128 to rescale values back to normal

			I_term_Pitch = AvgPitch * Config.I_mult_level;		// Not an I-term, just the acc offset
			I_term_Pitch = I_term_Pitch >> 4;
			
			if (I_term_Pitch > PR_LIMIT) I_term_Pitch = PR_LIMIT;		
			else if (I_term_Pitch < -PR_LIMIT) I_term_Pitch = -PR_LIMIT; // Apply Pitch limit to calculation

			Pitch = Pitch - I_term_Pitch;						// P + Acc correction offset
		}
		else // Normal mode (Just use raw gyro errors to guess at attitude)
		{
			IntegralPitch += Pitch;							// I-term (32-bit)
			if (IntegralPitch > ITERM_LIMIT_RP) IntegralPitch = ITERM_LIMIT_RP;
			else if (IntegralPitch < -ITERM_LIMIT_RP) IntegralPitch = -ITERM_LIMIT_RP;// Anti wind-up

			Pitch *= Config.P_mult_pitch;					// Multiply P-term (Max gain of 256)
			Pitch = Pitch * 3;								// Multiply by 3, so max effective gain is 768

			I_term_Pitch = IntegralPitch * Config.I_mult_pitch;	// Multiply I-term (Max gain of 256)
			I_term_Pitch = I_term_Pitch >> 3;				// Divide by 8, so max effective gain is 16

			currentGyroError[PITCH] = Pitch;		// D-term
			//currentGyroError[PITCH] = gyroADC[PITCH];		// D-term
			DifferentialGyro = currentGyroError[PITCH] - lastGyroError[PITCH];
			lastGyroError[PITCH] = currentGyroError[PITCH];	
	
			DifferentialGyro *= Config.D_mult_pitch;		// Multiply D-term by up to 1024
			//DifferentialGyro = DifferentialGyro << 2;

			Pitch = Pitch + I_term_Pitch + DifferentialGyro;// P + I + D
			Pitch = Pitch >> 7;								// Divide by 128 to rescale values back to normal
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
		#else
		#error No Copter configuration defined !!!!
		#endif

		//***********************************************************************
		//                 --- Calculate yaw gyro output ---
		// NB: IF YOU CHANGE THIS CODE, YOU MUST REMOVE PROPS BEFORE TESTING !!!
		//***********************************************************************

		if ((RxInYaw < DEAD_BAND) && (RxInYaw > -DEAD_BAND)) RxInYaw = 0; // Reduce RxIn noise into the I-term

		RxInYaw = RxInYaw >> Config.Yawrate;			// Reduce RxInYaw rate per flying mode
		Yaw = RxInYaw + gyroADC[YAW];	

		IntegralYaw += Yaw;								// I-term (32-bit)
		if (IntegralYaw > ITERM_LIMIT_YAW) IntegralYaw = ITERM_LIMIT_YAW;
		else if (IntegralYaw < -ITERM_LIMIT_YAW) IntegralYaw = -ITERM_LIMIT_YAW;// Anti wind-up (Experiment with value)

		Yaw *= Config.P_mult_yaw;						// Multiply P-term (Max gain of 768)
		Yaw *= 3;

		I_term_Yaw = IntegralYaw * Config.I_mult_yaw;	// Multiply IntegralYaw by up to 256
		I_term_Yaw = I_term_Yaw >> 3;					// Divide by 8, so max effective gain is 16

		//currentGyroError[YAW] = gyroADC[YAW];			// D-term
		currentGyroError[YAW] = Yaw;					// D-term
		DifferentialGyro = currentGyroError[YAW] - lastGyroError[YAW];
		lastGyroError[YAW] = currentGyroError[YAW];	
	
		DifferentialGyro *= Config.D_mult_yaw;			// Multiply D-term by up to 1024
		//DifferentialGyro = DifferentialGyro << 2;

		Yaw = Yaw - I_term_Yaw - DifferentialGyro;		// P + I + D
		Yaw = Yaw >> 7;									// Divide by 128 to rescale values back to normal

		if (Yaw > YAW_LIMIT) Yaw = YAW_LIMIT;		
		else if (Yaw < -YAW_LIMIT) Yaw = -YAW_LIMIT; 	// Apply YAW limit to PID calculation

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
		#else
		#error No Copter configuration defined !!!!
		#endif

		//--- Limit the lowest value to avoid stopping of motor if motor value is under-saturated ---
		if ( MotorOut1 < MOTOR_IDLE )	MotorOut1 = MOTOR_IDLE;	
		if ( MotorOut2 < MOTOR_IDLE )	MotorOut2 = MOTOR_IDLE;	
		if ( MotorOut3 < MOTOR_IDLE )	MotorOut3 = MOTOR_IDLE;
		if ( MotorOut4 < MOTOR_IDLE )	MotorOut4 = MOTOR_IDLE;
	
		//--- Output to motor ESC's ---
		if (RxInCollective < 1 || !Armed)	// Turn off motors if collective below 1% 
		{		
			MotorOut1 = 0;
			MotorOut2 = 0;
			MotorOut3 = 0;
			MotorOut4 = 0;
		}

		if (Armed) output_motor_ppm();		// Output ESC signal
		uber_loop_count++;
	} // main loop
} // main()

