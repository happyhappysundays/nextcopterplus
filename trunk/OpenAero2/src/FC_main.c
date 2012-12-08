// **************************************************************************
// OpenAero2 software for KK2.0
// ===========================
// Version 1.1 Beta 5 - December 2012
//
// Contains trace elements of old KK assembly code by Rolf R Bakke, and C code by Mike Barton
// OpenAero code by David Thompson, included open-source code as per quoted references
//
// **************************************************************************
// * 						GNU GPL V3 notice
// **************************************************************************
// * Copyright (C) 2012 David Thompson
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
// * along with this program. If not, see <http://www.gnu.org/licenses/>.
// * 
// * NB: Summary - all derivative code MUST be released with the source code!
// *
// **************************************************************************
// Version History
// ===============
// V1.0		Based on OpenAero V1.13 Beta 8 code
//			Initial code base.
// Alpha 1	First release to alpha testers
// Alpha 2  Fixed channel number bug, added second aileron to default mixer
// Alpha 3	Enabled autolevel and acc settings in mixer menus, gyro reversing works.
//			Contrast updates visually during modification. Status menu updated.
// Alpha 4	Battery auto-calculates LVA setting. Restore to default instead of clear.
//			*Everything* that can be in PROGMEM is in PROGMEM. JR/Spk notation.
//			Version added to Status menu. Added separate accs to mixer.
//			Increased GLCD write speed to maximum.
//			Working Status menu auto refresh!
// Alpha 5	Added audible and visual error messages for LVA, no RX, Gyro error, throttle high
//			and Lost Model.	Failsafe setting now possible from the RCinputs screen. 
//			Expo now works nicely. Changed min/max values to (us). Rearranged menu.
//			FlapChan now updates mixer so that M7 has the same source as FlapChan.
//			Enabled source volume - now adjustable from between 0% and 125%
//			Fixed niggling hiccup from servos due to status screen refreshing.
//			Fixed servo jitter in CPPM mode. Add four presets for use as source channels.
// Beta 1	First public release. Fixed failsafe bug. Removed unused menu items.
//			Small, ugly hack to modify battery defaults if NiMh selected.
// Beta 2	Change gyro setup in mixer. Change limits to percentages.
//			Large menu size reduction. Trash KK2 eeprom locations.
//			Added two, two-channel RC mixers - mixes to new channels "MIX1" and "MIX2".
//			Added adjustable Lost Model Alarm timeout. 0 = disabled, otherwise 1 to 10 minutes
//			without RC input. Fixed min/max travel bug. Startup beep now after last gyro check.
//			Fixed issue where changing anything in the general menu over-wrote the mixers.
//			Removed other font options from mugui_text.c. Adjusted F.Wing default sources.
//			Force factory reset for new eeprom structure changes.
//			Added back CamStab enable so that camera stability can be done without RC.
//			Added CamStab mixer preset. Restored full PID functionality.
//			Completely new idle screen. Status screen now has user settable timeout.
//			Greatly increased acc gain.
// Beta 3	Added anti-gyro noise gate into PID calculations.
//			Fixed bug where CamStab and failsafe would clash and lock the menu.
//			Completely reworked PID code to enable heading hold on all axis.
//			Autolevel menu now only handles accelerometers. Tweaked camstab mixer preset.
//			Added settable I-term limits in %. Added servo rate adjustment for CamStab mode.
//			In CamStab mode, switched off M5-M8 mixing and outputs to increase loop speed to about 400Hz.
//			Added separate P settings for Roll and Pitch accelerometers. Servo HIGH rate set to 300Hz.
//			Added preliminary IMU code. Removed small, ugly NiMh hack.
//			Changed autolevel setting to only do autolevel. No implied stability control.
// Beta 4	Fixed noob-level f*ck-up resulting in no I-term for Yaw in Beta3
//			Added menu setting for Yaw heading-hold auto-center "Yaw magic".
//			Corrected voltage scaling text. Fixed LiPo default voltage.
//			Menu now remembers last position. Added button acceleration.
//			Updated Aeroplane mixer preset. Removed 3-pos setting from General menu as not needed.
//			Added stack monitoring function. Reversed menu and value UP/DOWN sense. 
//			Hopefully fixed false throttle high alarm.
//			Completed and integrated basic IMU code (again).
//			Added General menu settings for Acc LPF and IMU CF factor.
// Beta 4.1 Trial fix for Autolevel switch-on issue (Successful).
//	
// V1.1		Based on OpenAero2 Beta 4.1 code
// Alpha 1	Stability and Autolevel switch setpoint adjustment	
//			RC mixers moved to main mixers. Fixed balance meter movement
//			Main mixers can now cross-mix up to four channels
//			Launch delay mode. Tweaked menu navigation and driver.
//			Added 120 degree swashplate preset. 
// Alpha 2	Removed PRESET channels and replaced with per-output trims.
//			Added HANDSFREE autolevel modes (autolevel at centered sticks)
//			Added working differential mode (finally)
// Alpha 3	Added IMU fixes for inverted flight. New inverted calibration mode
//			Code now 100.0% full. Had to remove one mixer preset.
// Beta 1	Bugfix for three-position function. Minor tweaks for Beta 1
// Beta 2	Bugfix - fixed vertical orientation mode.
// Beta 3	New I-term modes: Normal, Auto and 3D.
//			Reversed Yaw gyro setting in aeroplane mixer preset...
// Beta 4	Completely changed PID loop to take sticks into account.
//			Added offset calculation to exclude flaperon movement.
//			Totally changed mixer so that stability/autolevel has exclusive control of outputs (no RC).
//			Added Maximum turn angle setting in Autolevel mode.
//			Updated Z cal to make it easier to do and harder to screw up.
//			Restored missing ACC trims. Fixed I-term constrain calculation.
//			Added Normal/FlyByWire flight modes.
//			Added Dynamic gain setting. Many bug fixes.
// Beta 5	Fixed 3D mode mixing.
//
//***********************************************************
//* To do
//***********************************************************
//
//
//***********************************************************
//* Includes
//***********************************************************

#include <avr/io.h>
#include <avr/pgmspace.h> 
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <util/delay.h>
#include <string.h>
#include "..\inc\io_cfg.h"
#include "..\inc\rc.h"
#include "..\inc\Servos.h"
#include "..\inc\vbat.h"
#include "..\inc\gyros.h"
#include "..\inc\init.h"
#include "..\inc\acc.h"
#include "..\inc\isr.h"
#include "..\inc\glcd_driver.h"
#include "..\inc\pid.h"
#include "..\inc\mixer.h"
#include "..\inc\glcd_buffer.h"
#include "..\inc\mugui.h"
#include "..\inc\glcd_menu.h"
#include "..\inc\menu_ext.h"
#include "..\inc\main.h"
#include "..\inc\imu.h"
#include "..\inc\eeprom.h"

//***********************************************************
//* Fonts
//***********************************************************

#include "..\inc\Font_Verdana.h" 		// 8 (text), 14 (titles), and 22 (numbers to edit) points
#include "..\inc\Font_WingdingsOE2.h"	// Cursor and markers

//***********************************************************
//* Defines
//***********************************************************

#define	SERVO_OVERDUE 9765			// Number of T2 cycles before servo will be overdue = 9765 * 1/19531 = 500ms
#define	SERVO_RATE_LOW 390			// Requested servo rate when in failsafe/Camstab LOW mode. 19531 / 50(Hz) = 390
#define	SERVO_RATE_HIGH 65			// Requested servo rate when in Camstab HIGH mode 300(Hz) = 65
#define LMA_TIMEOUT 1171860			// Number or T2 cycles before Lost Model alarm sounds (1 minute)
#define FS_TIMEOUT 19531			// Number or T2 cycles before Failsafe setting engages (1 second)
#define	PWM_DELAY 250				// Number of 8us blocks to wait between "Interrupted" and starting the PWM pulses 250 = 2ms
#define REFRESH_TIMEOUT 39060		// Amount of time to wait after last RX activity before refreshing LCD (2 seconds)
#define STATUS_TIMER 19531			// Unit of timing for showing the status screen (seconds)
#define LAUNCH_TIMER 195310			// Hand-launch timer (10 seconds)

//***********************************************************
//* Code and Data variables
//***********************************************************

// Flight variables
uint16_t cycletime;					// Loop time
uint32_t ticker_32;					// Incrementing system ticker

// Misc
bool AutoLevel;						// AutoLevel = 1
bool Stability;						// Stability = 1
bool Failsafe;
bool Refresh_safe;
char pBuffer[16];					// Print buffer

//************************************************************
//* Main loop
//************************************************************

int main(void)
{
	bool Overdue = false;
	bool ServoTick = false;
	// Alarms
	bool BUZZER_ON = false;
	bool Model_lost = false;		// Model lost flag
	bool LMA_Alarm = false;			// Lost model alarm active
	bool LVA_Alarm = false;			// Low voltage alarm active

	// Launch mode
	bool Launch_Mode = false;		// Launch mode ON flag
	bool Launch_Block = false;		// Launch mode autolevel block flag

	// Timers
	uint32_t Status_timeout = 0;
	uint32_t UpdateStatus_timer = 0;
	uint32_t LostModel_timer = 0;
	uint32_t Launch_timer = 0;
	uint32_t Ticker_Count = 0;
	uint16_t Servo_Timeout = 0;
	uint16_t Servo_Rate = 0;
	uint8_t Status_TCNT2 = 0;
	uint8_t Refresh_TCNT2 = 0;
	uint8_t Lost_TCNT2 = 0;
	uint8_t Launch_TCNT2 = 0;
	uint8_t Ticker_TCNT2 = 0;
	uint8_t Servo_TCNT2 = 0;
	uint8_t ServoRate_TCNT2 = 0;

	uint16_t LoopStartTCNT1 = 0;
	uint8_t	LMA_minutes = 0;
	uint8_t Status_seconds = 0;

	uint8_t Menu_mode = STATUS_TIMEOUT;

	init();							// Do all init tasks

	// Main loop
	while (1)
	{
		//************************************************************
		//* State machine for switching between screens safely
		//************************************************************

		switch(Menu_mode) 
		{
			// In IDLE mode, the text "Press any button for status" is displayed ONCE.
			// If a button is pressed the mode changes to REQ_STATUS
			case IDLE:
				if((PINB & 0xf0) != 0xf0)
				{
					Menu_mode = REQ_STATUS;
					// Reset the status screen timeout
					Status_seconds = 0;
					menu_beep(1);
				}
				break;

			// Request the status be updated when safe
			case REQ_STATUS:
				// Reset safe to refresh flag
				Refresh_safe = false;
				Menu_mode = WAITING_STATUS;
				break;

			// Waiting for status screen to be updated
			case WAITING_STATUS:
				// Next time Refresh_safe is set, switch to status screen
				if (Refresh_safe)
				{
					Menu_mode = STATUS;
				}
				break;

			// Status screen first display
			case STATUS:
				// Reset the status screen period
				UpdateStatus_timer = 0;
				// Update status screen
				Display_status();
				// Force resync on next RC packet
				Interrupted = false;	
				// Wait for timeout
				Menu_mode = WAITING_TIMEOUT_BD;
				break;

			// Status screen up, but button still down ;)
			case WAITING_TIMEOUT_BD:
				if(BUTTON1 == 0)
				{
					Menu_mode = WAITING_TIMEOUT_BD;
				}
				else
				{
					Menu_mode = WAITING_TIMEOUT;
				}
				break;
												
			// Status screen up, waiting for timeout or action
			case WAITING_TIMEOUT:
				// In status screen, change back to idle after timing out
				if (Status_seconds >= Config.Status_timer)
				{
					Menu_mode = STATUS_TIMEOUT;
				}

				// Update status screen while waiting to time out
				else if (UpdateStatus_timer > STATUS_TIMER)
				{
					Menu_mode = REQ_STATUS;
				}

				// Jump to menu if button pressed
				else if(BUTTON1 == 0)
				{
					Menu_mode = MENU;
					menu_beep(1);
				}
				break;

			// In STATUS_TIMEOUT mode, the idle screen is displayed and the mode changed to IDLE
			case STATUS_TIMEOUT:
				// Pop up the Idle screen
				idle_screen();
				// Switch to IDLE mode
				Menu_mode = IDLE;
				break;

			// In MENU mode, 
			case MENU:
				menu_main();
				// Switch back to status screen when leaving menu
				Menu_mode = STATUS;
				// Reset timeout once back in status screen
				Status_seconds = 0;
				break;

			default:
				break;
		}

		//************************************************************
		//* Status menu refreshing
		//************************************************************

		// Update status timeout
		Status_timeout += (uint8_t) (TCNT2 - Status_TCNT2);
		Status_TCNT2 = TCNT2;

		// Count elapsed seconds
		if (Status_timeout > STATUS_TIMER)
		{
			Status_seconds++;
			Status_timeout = 0;
		}

		// Update status provided no RX activity for REFRESH_TIMEOUT seconds (1s)
		UpdateStatus_timer += (uint8_t) (TCNT2 - Refresh_TCNT2);
		Refresh_TCNT2 = TCNT2;

		//************************************************************
		//* Reconfigure interrupts if menu changed
		//************************************************************

		// Reconfigure interrupts for PWM/CPPM modes
		if (Config.RxMode != CPPM_MODE)
		{
			PCMSK1 |= (1 << PCINT8);			// PB0 (Aux pin change mask)
			PCMSK3 |= (1 << PCINT24);			// PD0 (Throttle pin change mask)
			EIMSK  = 0x07;						// Enable INT0, 1 and 2 
		}
		else // CPPM mode
		{
			PCMSK1 = 0;							// Disable AUX
			PCMSK3 = 0;							// Disable THR
			EIMSK = 0x04;						// Enable INT2 (Rudder/CPPM input)
		}

		//************************************************************
		//* System ticker - based on TCNT2 (19.531kHz)
		//* 
		//* ((Ticker_Count >> 8) &8) 	= 4.77Hz (LMA and LVA alarms)
		//************************************************************

		// Ticker_Count increments at 19.531 kHz, in loop cycle chunks
		Ticker_Count += (uint8_t) (TCNT2 - Ticker_TCNT2);
		Ticker_TCNT2 = TCNT2;

		if ((Ticker_Count >> 8) &8) 
		{
			BUZZER_ON = true; 	// 4.77Hz beep
		}
		else 
		{
			BUZZER_ON = false;
		}

		//************************************************************
		//* Alarms
		//************************************************************

		// Lost model alarm
		LostModel_timer += (uint8_t) (TCNT2 - Lost_TCNT2);
		Lost_TCNT2 = TCNT2;

		// Reset LMA count if any RX activity, LMA of, or CamStab (no RC used)
		if (RxActivity || (Config.LMA_enable == 0) || (Config.CamStab == ON))
		{														
			LostModel_timer = 0;
			Model_lost = false;	
			LMA_minutes = 0;
			General_error &= ~(1 << LOST_MODEL); // Clear lost model bit		
		}
		
		if (LostModel_timer > LMA_TIMEOUT)
		{
			LMA_minutes++;
			LostModel_timer = 0;
		}

		// Trigger lost model alarm if enabled and due
		if ((LMA_minutes >= Config.LMA_enable) && (Config.LMA_enable != 0))	
		{
			Model_lost = true;
			General_error |= (1 << LOST_MODEL); // Set lost model bit
		}

		if (BUZZER_ON && Model_lost) 
		{
			LMA_Alarm = true;					// Turn on buzzer
		}
		else 
		{
			LMA_Alarm = false;					// Otherwise turn off buzzer
		}

		// Low-voltage alarm (LVA)
		GetVbat();								// Check battery


		// Beep buzzer if Vbat lower than trigger
		if ((vBat < Config.PowerTrigger) && BUZZER_ON) 
		{
			LVA_Alarm = true;
			General_error |= (1 << LOW_BATT); 	// Set low battery bit
		}
		else 
		{
			LVA_Alarm = false;					// Otherwise turn off buzzer
			General_error &= ~(1 << LOW_BATT); 	// Clear low battery bit
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
		//* Hand-launch mode handling
		//************************************************************

		if (Config.LaunchMode == ON)
		{
			// Increment timer only if Launch mode on to save cycles
			Launch_timer += (uint8_t) (TCNT2 - Launch_TCNT2);
			Launch_TCNT2 = TCNT2;

			// Reset Launch count if Launch_Mode false
			if (Launch_Mode == false)
			{														
				Launch_timer = 0;
			}
		
			// Re-enable autolevel if timer expires while autolevel blocked
			if ((Launch_Block) && (Launch_timer > LAUNCH_TIMER))
			{
				Launch_Block = false;
			}
		}

		// If first time into Launch mode
		if ((Config.LaunchMode == ON) && (Launch_Mode == false))	
		{
			// Launch mode throttle position exceeded
			if (RxChannel[THROTTLE] > Config.Launchtrigger)	
			{
				Launch_Mode = true;				// Start 10 second countdown
				Launch_Block = true;			// Disable autolevel
				menu_beep(1);					// Signal launch mode timer start
			}
		}

		//************************************************************
		//* Autolevel mode selection
		//************************************************************
		//* Primary override:
		//*		Autolevel always OFF if Config.AutoMode = OFF (default)
		//*		Autolevel disabled if Launch_Block = true
		//*
		//* Five switchable modes:
		//*		1. Disabled by Config.AutoMode = OFF
		//*		2. Enabled by "AutoChan" channel number
		//*		3. Enabled by user-set triggers of "ThreePos" channel number
		//*		4. Enabled if HandsFree and Config.AutoMode = HANDSFREE
		//*		5. Enabled by Config.AutoMode = ON
		//************************************************************

		// Update zeroed RC channel data
		RxGetChannels();

		// Clear Throttle High error once throtle reset
		if (RCinputs[THROTTLE] < 250)
		{
			General_error &= ~(1 << THROTTLE_HIGH);	
		}

		switch(Config.AutoMode)
		{
			case DISABLED:
				AutoLevel = false;				// De-activate autolevel mode
				break;
			case AUTOCHAN:
			case THREEPOS:
				if (RxChannel[Config.AutoChan] > Config.Autotrigger)
				{
					AutoLevel = true;			// Activate autolevel mode
				}	
				else
				{
					AutoLevel = false;			// De-activate autolevel mode
				}	
				break;
			case HANDSFREE:
				if (HandsFree)					// If hands free
				{
					AutoLevel = true;			// Activate autolevel mode
				}	
				else
				{
					AutoLevel = false;			// De-activate autolevel mode
				}
				break;
			case ALWAYSON:
				AutoLevel = true;				// Activate autolevel mode
				break;
			default:							// Disable by default
				break;
		}

		// Check for Launch blocking
		if (Launch_Block)
		{
			AutoLevel = false;					// De-activate autolevel mode
		}

		//************************************************************
		//* Stability mode selection
		//************************************************************
		//* Primary override:
		//*		Stability enabled if Config.StabMode = ON
		//*		Stability always OFF if Config.StabMode = OFF (default)
		//*
		//* Four switchable modes:
		//*		1. Disabled by Config.StabMode = OFF
		//*		2. Enabled by "StabChan" channel number
		//*		3. Enabled by user-set triggers of "ThreePos" channel number
		//*		4. Always ON
		//************************************************************

		switch(Config.StabMode)
		{
			case DISABLED:
				Stability = false;				// De-activate autolevel mode
				break;
			case STABCHAN:
			case THREEPOS:
				if (RxChannel[Config.StabChan] > Config.Stabtrigger)
				{
					Stability = true;			// Activate autolevel mode
				}	
				else
				{
					Stability = false;			// De-activate autolevel mode
				}	
				break;
			case ALWAYSON:
				Stability = true;			// Activate autolevel mode
				break;
			default:							// Disable by default
				break;
		}

		if (!Stability)
		{
			// Reset I-terms when stabilise is off
			IntegralGyro[ROLL] = 0;	
			IntegralGyro[PITCH] = 0;
			IntegralGyro[YAW] = 0;
		}

		// Read gyros when required
		if (Stability || AutoLevel)
		{
			ReadGyros();		// Read sensors
		}

		// Autolevel mode ON
		if (AutoLevel) 
		{
			ReadAcc();			// Only read Accs if in AutoLevel mode
			getEstimatedAttitude();
		}
        else
        {
            // Reset IMU each time autolevel restarted
            FirstTimeIMU = true;
        }

		// Remove RC noise and detect when sticks centered
		RC_Deadband();

		// Calculate PID
		Calculate_PID();

		// Calculate mix
		ProcessMixer();

		// Transfer Config.Channel[i].value data to ServoOut[i] and check limits
		UpdateServos();

		//************************************************************
		//* Process servos, failsafe mode
		//************************************************************

		// Signal servo overdue after SERVO_OVERDUE time (500ms)
		// Servo_Timeout increments at 19.531 kHz, in loop cycle chunks
		Servo_Timeout += (uint8_t) (TCNT2 - Servo_TCNT2);
		Servo_TCNT2 = TCNT2;
		if (Servo_Timeout > SERVO_OVERDUE)
		{
			Overdue = true;
		}

		// Assures even without synchronous RX, something will come out at SERVO_RATE (50Hz)
		// Servo_Rate increments at 19.531 kHz, in loop cycle chunks
		Servo_Rate += (uint8_t) (TCNT2 - ServoRate_TCNT2);
		ServoRate_TCNT2 = TCNT2;


		// If in CabStab mode (no RC to synch) AND the Servo Rate is set to HIGH run at full speed
		if ((Config.CamStab == ON) && (Config.Servo_rate == HIGH))
		{
			if (Servo_Rate > SERVO_RATE_HIGH)
			{
				ServoTick = true;
			}
		}
		else if (Servo_Rate > SERVO_RATE_LOW)
		{
				ServoTick = true;
		}

		// If simply overdue, signal RX error message
		// If in independant camstab mode, don't bother
		if (Overdue && (Config.CamStab == OFF))
		{
			General_error |= (1 << NO_SIGNAL);	// Set NO_SIGNAL bit
		}
		else
		{
			General_error &= ~(1 << NO_SIGNAL);	// Clear NO_SIGNAL bit
		}

		// Check for failsafe condition (Had RC lock but now overdue)
		if (Overdue && RC_Lock)
		{
			Failsafe = true;
			RC_Lock = false;
		}

		// Set failsafe positions when RC lock lost
		// Obviously not desired in CamStab mode (no RC)
		if (Failsafe && (Config.CamStab == OFF))
		{
			uint8_t i;
			for (i = 0; i < MAX_OUTPUTS; i++)
			{
				ServoOut[i] = Config.Limits[i].failsafe;
			}
		}

		// Ensure that output_servo_ppm() is synchronised to the RC interrupts
		if (Interrupted)
		{
			Interrupted = false;			// Reset interrupted flag
			Refresh_safe = true;			// Safe to try and refresh status screen
			Failsafe = false;				// Cancel failsafe
			Servo_Timeout = 0;				// Reset servo failsafe timeout
			Overdue = false;				// And no longer overdue...

			output_servo_ppm();				// Output servo signal
		}

		// If in failsafe, or when doing independant camstab, just output unsynchronised
		else if ((Overdue && ServoTick) && (Failsafe || (Config.CamStab == ON)))
		{
			ServoTick = false;				// Reset servo update ticker
			Servo_Rate = 0;					// Reset servo rate timer
			Refresh_safe = true;			// Safe to try and refresh status screen
			output_servo_ppm();				// Output servo signal
		}

		if (Overdue && ServoTick)
		{
			Refresh_safe = true;			// Safe to try and refresh status screen
		}

		// Measure the current loop rate
		cycletime = TCNT1 - LoopStartTCNT1;	// Update cycle time
		LoopStartTCNT1 = TCNT1;				// Measure period of loop from here
		ticker_32 += cycletime;


	} // main loop
} // main()

