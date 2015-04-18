// **************************************************************************
// OpenAero2 software for KK2.0 and KK2.1
// ======================================
// Version 1.4 Beta 2 - April 2015
//
// Some receiver format decoding code from Jim Drew of XPS and the Papparazzi project
// OpenAero code by David Thompson, included open-source code as per quoted references
//
// **************************************************************************
// * 						GNU GPL V3 notice
// **************************************************************************
// * Copyright (C) 2014 David Thompson
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
// V1.4		Based on OpenAero2 V1.2 Beta 10
//
// Beta 1	Compiler clean-up. Contrast bug fixed.
//			Axis lock stick rate now calibrated to 10, 20, 40, 80, 160, 320 and 640 deg/sec.
// 			Servo limits corrected to accurately move to the stated position.
// 			Divides rounded correctly, not truncated. Tidied up CPPM code
// 			Fixed the balance meter display for all orientations
// 			Battery voltage calculation faster and more accurate.
// 			IMU update. Pitch response works the same as roll in autolevel when inverted.
// 			Fixed balance meter response in KK2.1. Added Manual mixer option.
//			Removed Source Mix setting as per OpenAero VTOL.
//			Vbat now displays correctly for both KK2.0 and KK2.1. S.Bus timing tweaked.
//			RC noise threshold changed from +/-20 to +/-5. Tweaked battery display.
// 			Fixed acc calibration issues. Fixed Acc Z initial offset for KK2.1.
//			Fixed small bug where Acc LPF would not turn off when set to minimum.
//			Balance meter uses AccSmooth, so now shows an approximation of the Acc LPF.	
//			Better rounding in Acc LPF calculation. Tidied up IMU multiplies to make sure there were no casting gotchas.			
//			Saved code space in the CPPM section thanks to Edgar.		
//			Fixed code where RCinputs[NOCHAN] broke things.
//			Added test for first time though the main loop so that UpdateLimits() is done with the correct flight mode.
//			Increased Acc LPF range to 127. Gyro calibrate on initialisation waits for stability.
//			RX modes changed to CPPM, PWM, S.Bus and Satellite.
//			Add new menu item for selecting PWM sync source. Now all five PWM inputs can be selectable as a sync source.
//			Screen contrast updated before logo - logo prettier.
//			Lock rate default changed to 3.
//			Fixed MPU6050 setup codes. Now actually correct! Currently 500 deg/sec and 4G full-scale.
//			Added user-settable chip LPF setting for KK2.1 version.
//			Edge-clip bug fixed in fillcircle()
// Beta 2	Ported to AVR Studio 6 with automated KK2.0 and KK2.1 support.
//			Incorporated latest updates from OpenAeroVTOL Beta 38.
//			Reversed sense of dynamic gain control.
//			Fixed CPPM mis-detection bug. Profile triggers now hard-coded.
//			Mixer presets redone to mimic conventional channel order.
//			Display wizard improved so that bad RX selection doesn't read as stick inputs.
//			Updated to latest IMU from OpenAeroVTOL V1.0
//			Merged almost all improvements from OpenAeroVTOL V1.1.
//			Resolved issues with Servos[] adjustments being non-atomic.
//			Sped up slow flaps to suit new code.
//			Made failsafe work with new high-speed code
//			Fixed inverted cal for KK2.0.
//
//***********************************************************
//* To do
//***********************************************************
//
//	Bugs:	Some funny jitter in the flap/dual-aileron system when aileron difference is non-zero.
//
//***********************************************************
//* Includes
//***********************************************************

#include "compiledefs.h"
#include <avr/io.h>
#include <avr/pgmspace.h> 
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <util/delay.h>
#include <string.h>
#include "io_cfg.h"
#include "rc.h"
#include "servos.h"
#include "vbat.h"
#include "gyros.h"
#include "init.h"
#include "acc.h"
#include "isr.h"
#include "glcd_driver.h"
#include "pid.h"
#include "mixer.h"
#include "glcd_buffer.h"
#include "mugui.h"
#include "glcd_menu.h"
#include "menu_ext.h"
#include "main.h"
#include "imu.h"
#include "eeprom.h"
#include "uart.h"

//***********************************************************
//* Fonts
//***********************************************************

#include "Font_Verdana.h" 		// 8 (text) and 14 (titles) points
#include "Font_WingdingsOE2.h"	// Cursor and markers

//***********************************************************
//* Defines
//***********************************************************

#define	RC_OVERDUE 9765				// Number of T2 cycles before RC will be overdue = 9765 * 1/19531 = 500ms
#define	SLOW_RC_RATE 41667			// Slowest RC rate tolerable for Plan A syncing = 2500000/60 = 16ms
#define	SERVO_RATE_LOW 300			// A.servo rate. 19531/65(Hz) = 300
#define SECOND_TIMER 19531			// Unit of timing for seconds
#define LMA_TIMEOUT 1171860			// Number or T2 cycles before Lost Model alarm sounds (1 minute)
#define STATUS_OFF_TIME 3			// Number of seconds before the status scren disappears
#define SBUS_PERIOD	6250			// Period for S.Bus data to be transmitted (no margin) (2.5ms)
#define SBUS_MARGIN	8750			// Period for S.Bus data to be transmitted (+ 1ms margin) (3.5ms)
#define PWM_PERIOD 12500			// Average PWM generation period (5ms)
#define PWM_PERIOD_WORST 20833		// PWM generation period (8.3ms - 120Hz)
#define PWM_PERIOD_BEST 8333		// PWM generation period (3.333ms - 300Hz)
#define FASTSYNCLIMIT 293			// Max time from end of PWM to next interrupt (15ms)

//***********************************************************
//* Code and Data variables
//***********************************************************

// Flags
uint8_t	volatile General_error = 0;
uint8_t	volatile Flight_flags = 0;
uint8_t	volatile Main_flags = 0;
uint8_t	volatile Alarm_flags = 0;

// Global buffers
char pBuffer[PBUFFER_SIZE];			// Print buffer (25 bytes)

// Serial buffer
char sBuffer[SBUFFER_SIZE];			// Serial buffer (38 bytes)

// Misc globals
volatile uint16_t	InterruptCount = 0;
volatile uint16_t	LoopStartTCNT1 = 0;
volatile bool		Overdue = false;
volatile uint8_t	LoopCount = 0;

//************************************************************
//* Main loop
//************************************************************

int main(void)
{
	// Flags
	bool RCrateMeasured = false;
	bool PWMBlocked = false;
	bool RCInterruptsON = false;
	bool ServoTick = false;
	bool ResampleRCRate = false;
	bool PWMOverride = false;
	bool Interrupted_Clone = false;
	bool SlowRC = true;

	// 32-bit timers
	uint32_t LostModel_timer = 0;
	uint32_t RC_Rate_Timer = 0;
	uint32_t PWM_interval = PWM_PERIOD_WORST;	// Loop period when generating PWM. Initialise with worst case until updated.

	// 16-bit timers
	uint16_t Status_timeout = 0;
	uint16_t UpdateStatus_timer = 0;
	uint16_t Ticker_Count = 0;
	uint16_t RC_Timeout = 0;
	uint16_t Servo_Rate = 0;
	uint16_t Save_TCNT1 = 0;
	uint16_t ticker_16 = 0;
	uint16_t fast_sync_timer = 0;
	
	// Timer incrementers
	uint16_t RC_Rate_TCNT1 = 0;
	uint8_t Status_TCNT2 = 0;
	uint8_t Refresh_TCNT2 = 0;
	uint8_t Lost_TCNT2 = 0;
	uint8_t Ticker_TCNT2 = 0;
	uint8_t Servo_TCNT2 = 0;
	uint8_t ServoRate_TCNT2 = 0;
	uint8_t fast_sync_TCNT2 = 0;

	// Locals
	uint16_t InterruptCounter = 0;
	uint8_t	LMA_minutes = 0;
	uint8_t Status_seconds = 0;
	uint8_t Menu_mode = STATUS_TIMEOUT;
	uint8_t	old_flight = 0;			// Current/old flight profile
	uint8_t	old_alarms = 0;
	uint8_t ServoFlag = 0;
	uint8_t i = 0;
	int16_t PWM_pulses = 3; 
	uint32_t interval = 0;			// IMU interval


	init();							// Do all init tasks

	// Main loop
	while (1)
	{
		// Increment the loop counter
		LoopCount++;

		//************************************************************
		//* Check for interruption of PWM generation
		//* The "JitterFlag" flag was reset just before PWM generation.
		//* Being set here means that an interrupt has occurred.
		//************************************************************

		if (JitterFlag == true)
		{
			InterruptCounter++;
		}
		
		//************************************************************
		//* Once per second events
		//* - Increment Status_seconds
		//* - Do an RC rate resample
		//* - Check the battery voltage
		//************************************************************

		// Count elapsed seconds
		if (Status_timeout > SECOND_TIMER)
		{
			Status_seconds++;
			Status_timeout = 0;

			// Update the interrupt count each second
			InterruptCount = InterruptCounter;
			InterruptCounter = 0;
			
			// Re-measure the frame rate in FAST mode every second
			if (Config.Servo_rate == FAST)
			{
				ResampleRCRate = true;
			}
	
			// Check if Vbat lower than trigger
			if (GetVbat() < Config.PowerTriggerActual)
			{
				General_error |= (1 << LVA_ALARM);	// Set LVA_Alarm flag
			}
			else
			{
				General_error &= ~(1 << LVA_ALARM);	// Clear LVA_Alarm flag
			}

			//************************************************************
			// No signal jump-start for serial receivers
			//
			// If no signal, try resetting the UART once per second.
			// Also, wait long enough to ensure that at least one set of data has arrived.
			//************************************************************
		
			if ((General_error & (1 << NO_SIGNAL)) != 0)
			{
				init_uart();
				_delay_ms(25);
			}

		}

		//************************************************************
		//* State machine for switching between screens safely
		//* Particularly in FAST mode, if anything slows down the loop
		//* time significantly (beeps, LCD updates) the PWM generation
		//* is at risk of corruption. To get around that, entry and exit
		//* from special states must be handled in stages.
		//* In the state machine, once a state changes, the new state 
		//* will be processed in the next loop.
		//************************************************************

		// Assume PWM is OK until through the state machine
		// If the state machine requires PWM to be blocked, 
		// it will set this flag
		PWMOverride = false; 

		switch(Menu_mode) 
		{
			// In IDLE mode, the text "Press for status" is displayed ONCE.
			// If a button is pressed the mode changes to PRESTATUS, where
			// it will wait for the right time to proceed.
			case IDLE:
				// If any button is pressed
				if((PINB & 0xf0) != 0xf0)
				{
					Menu_mode = PRESTATUS;
					// Reset the status screen timeout
					Status_seconds = 0;
					
					// Allow PWM output
					PWMOverride = false;
					
					// When not in idle mode, enable Timer0 interrupts as loop rate 
					// is slow and we need TMR0 to fully measure it.
					// This may cause PWM generation interruption
					TIMSK0 |= (1 << TOIE0);	
				}
				// Idle mode - fast loop rate so don't need TMR0.
				// We don't want TMR0 to interrupt PWM generation.
				else
				{
					TIMSK0 = 0; 		// Disable Timer0 interrupts
					TIFR0 = 1;			// Clear interrupt flag
				}
				break;

			// Waiting to safely enter Status screen
			// If Interrupted or Interrupted_Clone is true, data must have just completed.
			// If Overdue is true, there is no data to interrupt.
			// PWM activity must stop before we attempt to pop up the status screen.
			case PRESTATUS:
				// If interrupted, or if currently "No signal"
				if (Interrupted || Interrupted_Clone || Overdue)
				{
					// Ready to move on
					Menu_mode = STATUS;
							
					// Prevent PWM output
					PWMOverride = true;		
					
					// Clear Interrupted_Clone
					Interrupted_Clone = false;
				}

				break;

			// Status screen first display
			case STATUS:
				// Reset the status screen period
				UpdateStatus_timer = 0;

				// Update status screen
				Display_status();
				
				// Prevent PWM output just after updating the LCD
				PWMOverride = true;

				// Wait for timeout
				Menu_mode = WAITING_TIMEOUT_BD;
				break;

			// Status screen up, but button still down ;)
			// This is designed to stop the menu appearing instead of the status screen
			// as it will stay in this state until the button is released
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
			// but button is back up
			case WAITING_TIMEOUT:
				// In status screen, change back to idle after timing out
				if (Status_seconds >= STATUS_OFF_TIME)
				{
					Menu_mode = PRESTATUS_TIMEOUT;
					
					// Enable PWM output
					PWMOverride = false;
				}

				// Jump to menu if button pressed
				else if(BUTTON1 == 0)
				{
					Menu_mode = MENU;
					
					// Prevent PWM output
					PWMOverride = true;
				}

				// Update status screen four times/sec while waiting to time out
				else if (UpdateStatus_timer > (SECOND_TIMER >> 2))
				{
					Menu_mode = PRESTATUS;

					// Prevent PWM output
					PWMOverride = true;
				}
				
				else
				{
					// Enable PWM output
					PWMOverride = false;					
				}

				break;

			// Attempting to leave Status gracefully while PWM stopped.
			// If Interrupted or Interrupted_Clone is true, data must have just completed.
			// If Overdue is true, there is no data to interrupt.
			// PWM activity must stop before we attempt to pop up the status screen.
			case PRESTATUS_TIMEOUT:
				// If interrupted, or if currently "No signal"
				if (Interrupted || Interrupted_Clone || Overdue)
				{
					// Switch to STATUS_TIMEOUT mode
					Menu_mode = STATUS_TIMEOUT;
				
					// Enable PWM output
					PWMOverride = false;
					
					// Clear Interrupted_Clone
					Interrupted_Clone = false;
				}
				else
				{
					// Prevent PWM output
					PWMOverride = true;
				}
				
				break;

			// In STATUS_TIMEOUT mode, the idle screen is displayed and the mode 
			// changed to POSTSTATUS_TIMEOUT. 
			case STATUS_TIMEOUT:
				// Pop up the Idle screen
				idle_screen();

				// Switch to IDLE mode
				Menu_mode = POSTSTATUS_TIMEOUT;

				// Prevent PWM output
				PWMOverride = true;

				break;

			// In POSTSTATUS_TIMEOUT mode, we wait for a PWM cycle to complete
			// The idle screen has been refreshed and we need to wait.
			case POSTSTATUS_TIMEOUT:
				// If interrupted, or if currently "No signal"
				if (Interrupted || Interrupted_Clone || Overdue)
				{
					// Switch to IDLE mode
					Menu_mode = IDLE;
					
					// Prevent PWM output
					PWMOverride = false;
					
					// Clear Interrupted_Clone
					Interrupted_Clone = false;
				}
				else
				{
					// Enable PWM output
					PWMOverride = true;			
				}
				
				break;

			// In MENU mode, 
			case MENU:
				LVA = 0;	// Make sure buzzer is off :)
				// Start the menu system
				menu_main();
				// Switch back to status screen when leaving menu
				Menu_mode = STATUS;
				// Reset timeout once back in status screen
				Status_seconds = 0;
				// Reset IMU on return from menu
				reset_IMU();
				
				// Prevent PWM output
				PWMOverride = true;
								
				break;

			default:
				break;
		}

		//************************************************************
		//* Alarms
		//************************************************************

		// If overdue, signal RX error message and
		// set failsafe flag. If in independent camstab mode, 
		// don't bother with either.
		if (Overdue && (Config.CamStab == OFF))
		{
			General_error |= (1 << NO_SIGNAL);	// Set NO_SIGNAL bit
			Flight_flags |= (1 << Failsafe);
		}
		else
		{
			General_error &= ~(1 << NO_SIGNAL);	// Clear NO_SIGNAL bit
			Flight_flags &= ~(1 << Failsafe);
		}

		// Lost model alarm
		LostModel_timer += (uint8_t) (TCNT2 - Lost_TCNT2);
		Lost_TCNT2 = TCNT2;

		// Reset LMA count if any RX activity, LMA off, or CamStab (no RC used)
		if ((Flight_flags & (1 << RxActivity)) || (Config.LMA_enable == 0) || (Config.CamStab == ON))
		{														
			LostModel_timer = 0;
			LMA_minutes = 0;
			General_error &= ~(1 << LOST_MODEL); // Clear lost model bit
		}
		
		// Count the minutes
		if (LostModel_timer > LMA_TIMEOUT)
		{
			LMA_minutes++;
			LostModel_timer = 0;
		}

		// Trigger lost model alarm if enabled and due or failsafe
		if ((LMA_minutes >= Config.LMA_enable) && (Config.LMA_enable != 0))	
		{
			General_error |= (1 << LOST_MODEL); // Set lost model bit
		}

		// Beep buzzer if Vbat lower than trigger
		// Vbat is measured in units of 10mV, so a PowerTrigger of 127 equates to 12.7V
		if (GetVbat() < Config.PowerTriggerActual)
		{
			General_error |= (1 << LVA_ALARM); 	// Set low battery bit
		}
		else 
		{
			General_error &= ~(1 << LVA_ALARM); 	// Clear low battery bit
		}

		// Turn on buzzer if in alarm state (BUZZER_ON is oscillating)
		if	(((General_error & (1 << LVA_ALARM)) ||
			  (General_error & (1 << LOST_MODEL)) || 
			  (General_error & (1 << THROTTLE_HIGH)) ||	
			  (General_error & (1 << NO_SIGNAL))) &&
			  (Alarm_flags	 & (1 << BUZZER_ON))) 
		{
			LVA = 1;
		}
		else 
		{
			LVA = 0;
		}

		// All code based on RC inputs is redundant until new RC data is ready,
		// otherwise the same data will be read back each and every time.
		if (Interrupted || Interrupted_Clone)
		{
			//LED1 = ~LED1; //debug
			
			//************************************************************
			//* Get RC data
			//************************************************************

			// Update zeroed RC channel data
			RxGetChannels();

			// Zero RC when in Failsafe
			if (Flight_flags & (1 << Failsafe))
			{
				for (i = 0; i < MAX_RC_CHANNELS; i++)
				{
					RCinputs[i] = 0;
				}
			}

			// Check for throttle reset
			if (MonopolarThrottle < THROTTLEIDLE)
			{
				// Clear throttle high error
				General_error &= ~(1 << THROTTLE_HIGH);

				// Reset I-terms at throttle cut. Using memset saves code space
				memset(&IntegralGyro[ROLL], 0, sizeof(int32_t) * NUMBEROFAXIS);
			}

			//************************************************************
			//* Flight mode selection. Now hard-coded to the following.
			//* 0 to -50 = 1, -50 to +50 = 2 and  >50 = 3
			//************************************************************

			// If CamStab ON, automatically select Profile 2
			if ((RCinputs[Config.FlightChan] > 500) || (Config.CamStab == ON))
			{
				Config.Flight = 2;			// Flight mode 2
			}	
			else if (RCinputs[Config.FlightChan] > -500)
			{
				Config.Flight = 1;			// Flight mode 1
			}
			else
			{
				Config.Flight = 0;			// Flight mode 0
			}

			// When changing flight modes or on first startup
			if ((Config.Flight != old_flight) || (Main_flags & (1 << FirstTimeFlightMode)))
			{
				// Clear first time flag
				Main_flags &= ~(1 << FirstTimeFlightMode);

				// Update travel limits
				UpdateLimits();
	
				// Reset I-terms so that neutral is reset
				// Using memset saves code space
				memset(&IntegralGyro[ROLL], 0, sizeof(int32_t) * NUMBEROFAXIS);

				old_flight = Config.Flight;
			}
			
			// Detect when sticks centered (hands free)
			RC_Deadband();
			
		} // Interrupted
		
		//************************************************************
		//* Autolevel mode selection
		//************************************************************
		//* Primary override:
		//*		Autolevel always OFF if Config.AutoMode = OFF (default)
		//*		Autolevel disabled if Launch_Block = true
		//*		Autolevel always ON if in Advanced failsafe condition
		//************************************************************

		switch(Config.FlightMode[Config.Flight].AutoMode)
		{
			case DISABLED:
				Flight_flags &= ~(1 << AutoLevel);	// De-activate autolevel mode
				break;
			case HANDSFREE:
				if (Flight_flags & (1 << HandsFree))// If hands free
				{
					Flight_flags |= (1 << AutoLevel);// Activate autolevel mode
				}	
				else
				{
					Flight_flags &= ~(1 << AutoLevel); // De-activate autolevel mode
				}
				break;
			case ALWAYSON:
				Flight_flags |= (1 << AutoLevel);	// Activate autolevel mode
				break;
			default:								// Disable by default
				break;
		}

		// Check for advanced Failsafe
		if ((Config.FailsafeType == ADVANCED) && (Flight_flags & (1 << Failsafe)) && (Config.CamStab == OFF))
		{
			Flight_flags |= (1 << AutoLevel);
		}

		//************************************************************
		//* Stability mode selection
		//************************************************************
		//* Primary override:
		//*		Stability enabled if Config.StabMode = ON
		//*		Stability always OFF if Config.StabMode = OFF (default)
		//************************************************************

		switch(Config.FlightMode[Config.Flight].StabMode)
		{
			case DISABLED:
				Flight_flags &= ~(1 << Stability);// De-activate autolevel mode
				break;
			case ALWAYSON:
				Flight_flags |= (1 << Stability);// Activate autolevel mode
				break;
			default:							// Disable by default
				break;
		}

		// Reset I-terms when stabilise is off
		// This is probably superfluous...
		if (!(Flight_flags & (1 << Stability)))
		{
			memset(&IntegralGyro[ROLL], 0, sizeof(int32_t) * NUMBEROFAXIS);
		}

		//************************************************************
		//* Update timers
		//************************************************************

		// Safely get current value of TCNT1
		Save_TCNT1 = TIM16_ReadTCNT1();

		// 32-bit timers (Max. 1718s measurement on T1, 220K seconds on T2)
		// Handle TCNT1-based timer correctly - this actually seems necessary...
		// Work out the current RC rate by measuring between incoming RC packets
		if (Save_TCNT1 < RC_Rate_TCNT1)
		{
			RC_Rate_Timer += (65536 - RC_Rate_TCNT1 + Save_TCNT1);
		}
		else
		{
			RC_Rate_Timer += (Save_TCNT1 - RC_Rate_TCNT1);
		}
		
		RC_Rate_TCNT1 = Save_TCNT1;
		
		// 16-bit timers (Max. 3.35s measurement on T2)
		// All TCNT2 timers increment at 19.531 kHz		

		// Sets the desired SERVO_RATE by flagging ServoTick when PWM due
		Servo_Rate += (uint8_t)(TCNT2 - ServoRate_TCNT2);
		ServoRate_TCNT2 = TCNT2;

		// Signal RC overdue after RC_OVERDUE time (500ms)
		RC_Timeout += (uint8_t)(TCNT2 - Servo_TCNT2);
		Servo_TCNT2 = TCNT2;

		// Update status timeout
		Status_timeout += (uint8_t) (TCNT2 - Status_TCNT2);
		Status_TCNT2 = TCNT2;

		// Status refresh timer
		UpdateStatus_timer += (uint8_t) (TCNT2 - Refresh_TCNT2);
		Refresh_TCNT2 = TCNT2;

		// Timer for audible alarms
		Ticker_Count += (uint8_t) (TCNT2 - Ticker_TCNT2);
		Ticker_TCNT2 = TCNT2;

		//************************************************************
		//* System ticker - based on TCNT2 (19.531kHz)
		//* 
		//* ((Ticker_Count >> 8) &8) 	= 4.77Hz (Disarm and LVA alarms)
		//************************************************************

		if ((Ticker_Count >> 8) &8) 
		{
			Alarm_flags |= (1 << BUZZER_ON);	// 4.77Hz beep
		}
		else 
		{
			Alarm_flags &= ~(1 << BUZZER_ON);
		}
		
		//************************************************************
		//* Manage desired output update rate when limited by
		//* the PWM rate set to "Low"
		//************************************************************

		// Flag update required based on the variable Servo_Match
		if (Servo_Rate > SERVO_RATE_LOW)
		{
			ServoTick = true;	// Slow device is ready for output generation
			Servo_Rate = 0;
		}
		
		//************************************************************
		//* Measure incoming RC rate and flag no signal
		//************************************************************

		// Check to see if the RC input is overdue (500ms)
		if (RC_Timeout > RC_OVERDUE)
		{
			Overdue = true;	// This results in a "No Signal" error

			// As T2 increments at 19.531 kHz, this will wrap after 3.35s.
			// To stop this problem, clip the value at 1s.			
			if (RC_Timeout > SECOND_TIMER)
			{
				RC_Timeout = SECOND_TIMER;
			}
		}
		else
		{
			Overdue = false; // debug
		}
		
		//************************************************************
		//* Read sensors
		//************************************************************

		ReadGyros();
		ReadAcc();

		//************************************************************
		//* Update IMU
		// TMR1 is a 16-bit counter that counts at 2.5MHz. Max interval is 26.2ms
		// TMR0 is an 8-bit counter that counts at 19.531kHz. Max interval is 13.1ms
		// These two are concatenated to create a virtual timer that can measure up to 
		// 256 x 26.2ms = 6.7072s at which point the "period" is 16,768,000, a 24-bit number
		//************************************************************
		
		// Safely get current value of TCNT1
		Save_TCNT1 = TIM16_ReadTCNT1();
		
		// Reset Timer0 count
		TCNT0 = 0;

		// Handle TCNT1 overflow correctly - this actually seems necessary...
		// ticker_16 will hold the most recent amount measured by TCNT1
		// Timer1 (16bit) - run @ 2.5MHz (400ns) - max 26.2ms
		if (Save_TCNT1 < LoopStartTCNT1)
		{
			ticker_16 = (65536 - LoopStartTCNT1) + Save_TCNT1;
		}
		else
		{
			ticker_16 = (Save_TCNT1 - LoopStartTCNT1);
		}
		
		// Store old TCNT for next measurement
		LoopStartTCNT1 = Save_TCNT1;
		
		// Handle both Timer1 under- and over-run cases
		// If TMR0_counter is less than 2, ICNT1 has not overflowed
		if (TMR0_counter < 2)
		{
			interval = ticker_16; // uint16_t
		}
		
		// If TMR0_counter is 2 or more, then TCNT1 has overflowed
		// So we use chunks of TCNT0, counted during the loop interval
		// to work out the exact period.
		// Timer0 (8bit) - run @ 20MHz / 1024 = 19.531kHz or 51.2us - max 13.1ms
		else
		{
			interval = ticker_16 + (TMR0_counter * 32768);
		}

		TMR0_counter = 0;

		//************************************************************
		//* Update attitude, average acc values each loop
		//************************************************************
				
		imu_update(interval);

		//************************************************************
		//* Update I-terms, average gyro values each loop
		//************************************************************

		Sensor_PID(interval);

		//************************************************************
		//* This is where things start getting really tricky... 
		//* Use the Interrupted state to measure the RC rate.
		//* Result in SlowRC state.
		//* 
		//* RCrateMeasured = Gap between two interrupts successfully measured.
		//* FrameRate = Serial frame gap as measured by the ISR.
		//* PWM_interval = Copied from Interval, is the current loop rate.
		//* 
		//************************************************************

		if (Interrupted)
		{
			// Measure incoming RC rate. Threshold is SLOW_RC_RATE.
			// Use RC_Rate_Timer if not in FAST mode.
			if (Config.Servo_rate < FAST)
			{
				if (RC_Rate_Timer > SLOW_RC_RATE)
				{
					SlowRC = true;
				}
				else
				{
					SlowRC = false;
				}
				
				// Reset RC timeout now that Interrupt has been received.
				RC_Timeout = 0;

				// No longer overdue. This will cancel the "No signal" alarm.
				Overdue = false;
						
				// Reset rate timer once data received. Reset to current time.
				RC_Rate_Timer = 0;
				Save_TCNT1 = TIM16_ReadTCNT1();
				RC_Rate_TCNT1 = Save_TCNT1;	
			}
			
			// Use Framerate in FAST mode, but only when NOT skipping frames
			if ((!RCrateMeasured) && (Config.Servo_rate == FAST))
			{
				// In high-speed mode, the RC rate will be unfairly marked as "slow" once measured and interrupt blocking starts.
				// To stop this being a problem, only set SlowRC prior to RCrateMeasured becoming true in this mode
				if (FrameRate > SLOW_RC_RATE)
				{
					SlowRC = true;
				}
				else
				{
					SlowRC = false;
				}	

				// Once the high speed rate has been calculated, signal that PWM is good to go.
				RCrateMeasured = true;
			}

			//***********************************************************************
			//* Work out the high speed mode RC blocking period when requested. 
			//* Only relevant for high speed mode. The slower the PWM rate the fewer
			//* PWM pulses will fit in the serial data gap.
			//***********************************************************************

			if (RCrateMeasured && (Config.Servo_rate == FAST))
			{
				// Slow packets (19.7ms gap). Pulse spans just two input packets.
				// 38.8s available space for S.Bus, 40ms for Satellite and 39.92ms for Xtreme.
				// Each PWM period is about 2.6ms so we need to see how many will fit before the next packet.
				// It is easiest to assume that say 38ms is safe for all formats.

				if (SlowRC)
				{
					PWM_pulses = 4;				// Four pulses will fit if interval faster than 103Hz
				
					if (PWM_interval < 19400)	// 19600 = 7.76ms
					{
						PWM_pulses += 1;		// One more pulse will fit if interval faster than 129Hz
					}
				
					if (PWM_interval < 16166)	// 16333 = 6.46ms
					{
						PWM_pulses += 1;		// One more pulse will fit if interval faster than 154Hz
					}
				
					if (PWM_interval < 13857)	// 14000 = 5.5ms
					{
						PWM_pulses += 1;		// One more pulse will fit if interval faster than 180Hz
					}
				
					if (PWM_interval < 12125)	// 12250 = 4.85ms
					{
						PWM_pulses += 1;		// One more pulse will fit if interval faster than 206Hz
					}
				
					if (PWM_interval < 10700)	// 10888 = 4.3ms
					{
						PWM_pulses += 1;		// One more pulse will fit if interval faster than 232Hz
					}
				
					if (PWM_interval < 9700)	// 9800 = 3.88ms
					{
						PWM_pulses += 1;		// One more pulse will fit if interval faster than 257Hz
					}
				}

				// Fast packets (9ms gap). Pulse spans three input packets.
				// 30ms available space for S.Bus, 31.5ms for Satellite and Xtreme.
				// Each PWM period is about 2.6ms so we need to see how many will fit before the next packet.
				// It is easiest to assume that say 29ms is safe for all formats.

				else
				{
					PWM_pulses = 3;				// Three pulses will fit if interval faster than 103Hz
				
					if (PWM_interval < 18125)	// 18125 = 7.25ms
					{
						PWM_pulses += 1;		// One more pulse will fit if interval faster than 138Hz
					}
				
					if (PWM_interval < 14500)	// 14500 = 5.8ms
					{
						PWM_pulses += 1;		// One more pulse will fit if interval faster than 172Hz
					}
				
					if (PWM_interval < 12083)	// 12083 = 4.83ms
					{
						PWM_pulses += 1;		// One more pulse will fit if interval faster than 206Hz
					}
				
					if (PWM_interval < 10357)	// 10357 = 4.14ms
					{
						PWM_pulses += 1;		// One more pulse will fit if interval faster than 241Hz
					}
				
					if (PWM_interval < 9062)	// 9062 = 3.62ms
					{
						PWM_pulses += 1;		// One more pulse will fit if interval faster than 275Hz
					}
				}
			}
			
			// Rate not measured or re-calibrating or not FAST mode
			// In all these other modes, just output one pulse
			else
			{
				PWM_pulses = 1;
			}

			// Reset RC timeout now that Interrupt has been received.
			RC_Timeout = 0;

			// No longer overdue. This will cancel the "No signal" alarm.
			Overdue = false;
			
			// Reset rate timer once data received. Reset to current time.
			RC_Rate_Timer = 0;
			Save_TCNT1 = TIM16_ReadTCNT1();
			RC_Rate_TCNT1 = Save_TCNT1;

			//************************************************************
			//* Beyond here lies dragons... proceed with caution
			//*
			//* This is the special FAST mode code which allows >250Hz 
			//* output when serial RC formats are used.
			//************************************************************

			// Block RC interrupts if period has been calculated
			// and PWM mode is FAST.
			if ((Config.Servo_rate == FAST) && RCrateMeasured)
			{
				// If it's time to resample the RC rate, do it now
				// so as not to disturb PWM generation.
				// This will result in a double gap with just one PWM.
				if (ResampleRCRate)
				{
					RCrateMeasured = false;		// Force remeasure of RC rate
					PWMBlocked = true;			// Disable Fast-mode PWM generation		
					ResampleRCRate = false;		// Reset resample request
				}

				// If not, block the RC interrupts until we run out of pulses
				// We need to cancel the Interrupted flag but have to make a copy until 
				// the status screen state machine has seen it.
				else
				{
					if (Interrupted)
					{
						Interrupted_Clone = true;	// Hand "Interrupted" baton on to its clone
					}
					
					Interrupted = false;		// Cancel pending interrupts
					Disable_RC_Interrupts();	// Disable RC interrupts
					RCInterruptsON = false;		// Flag it for the rest of the code
					PWMBlocked = false;			// Enable PWM generation	
				}
			}
		} // Interrupted

		//************************************************************
		//* Output PWM to ESCs/Servos where required, 
		//* based on a very specific set of conditions
		//************************************************************

		// Cases where we are ready to output
		if	(
				(Interrupted) ||									// Running at RC rate or
				((Config.Servo_rate == FAST) && (!PWMBlocked)) ||	// Run at full loop rate if allowed or
				(Flight_flags & (1 << Failsafe)) ||					// In failsafe or
				(Config.CamStab == ON)								// when doing independent camstab.
			)
		{

			//******************************************************************
			//* The following code runs once when PWM generation is desired
			//* The execution rates are:
			//* The RC rate unless in FAST mode
			//* High speed in FAST mode
			//******************************************************************

			if (Interrupted)
			{
				Interrupted_Clone = true;	// Hand "Interrupted" baton on to its clone
				Interrupted = false;		// Reset interrupted flag if that was the cause of entry			
			}

			// Decide which outputs fire this time, depending on their device setting (A.Servo, D.Servo, Motor)
			// D.Servo, Motor are always ready, but A.Servo must be limited to Servo_rate, flagged by ServoTick

			ServoFlag = 0;
				
			// For each output, mark the ones that are to fire this time
			for (i = 0; i < MAX_OUTPUTS; i++)
			{
				//Overdue
				// In SYNC mode, with NO SIGNAL, need to keep the outputs at the correct rate (47Hz or RC rate)
				
				
				// Mark bits depending on the selected output type
				if	(
						((Config.Servo_rate == FAST) && (Config.Channel[i].Motor_marker == ASERVO) && ServoTick) ||				// At ServoTick for A.Servo in FAST mode
						((Config.Servo_rate == SYNC) && (Config.Channel[i].Motor_marker == ASERVO) && (!SlowRC) && ServoTick) ||// At ServoTick for A.Servo in SYNC with Fast RC
						((Config.Servo_rate == SYNC) && (Config.Channel[i].Motor_marker == ASERVO) && (SlowRC)) ||				// At RC rate for A.Servo with slow RC

						((Config.Servo_rate >= SYNC) && (Config.Channel[i].Motor_marker > ASERVO) && (!Overdue)) ||				// Always for D.Servo and Motor in SYNC or FAST modes unless Overdue
						((Config.Servo_rate >= SYNC) && (Config.Channel[i].Motor_marker > ASERVO) && (Overdue) && ServoTick) ||	// At ServoTick SYNC or FAST modes when Overdue
						
						((Config.Servo_rate == LOW) && (!SlowRC) && ServoTick) ||												// All outputs at ServoTick in LOW mode with fast RC
						((Config.Servo_rate == LOW) && (SlowRC))																// All outputs at  RC rate in LOW mode with slow RC
					)
				{
					ServoFlag |= (1 << i);
				}
			}
								
			// Reset slow PWM flag if it was just set. It will automatically set again at around 19531/SERVO_RATE_LOW (Hz)
			if (ServoTick)
			{
				ServoTick = false;
				
				// Reset the Servo rate counter here so that it doesn't force an unusually small gap next time
				Servo_Rate = 0;
			}

			// Block PWM generation after last PWM pulse
			if ((PWM_pulses == 1) && (Config.Servo_rate == FAST))
			{
				PWMBlocked = true;					// Block PWM generation on notification of last call
				
				// Refresh PWM_interval with the actual interval when generating PWM
				// if it lies within believable ranges of 120Hz to 300Hz
				// This is located here to make sure the interval measured
				// is during PWM generation cycles
				if (interval < PWM_PERIOD_BEST)		// Faster than 300Hz
				{
					PWM_interval = PWM_PERIOD_BEST;
				}
				else if (interval > PWM_PERIOD_WORST)
				{
					PWM_interval = PWM_PERIOD_WORST; // Slower than 120Hz
				}
				else
				{
					PWM_interval = interval;		// Actual interval
				}
			}
			
			Calculate_PID();						// Calculate PID values
			ProcessMixer();							// Do all the mixer tasks - can be very slow

			// Set motors to idle on loss of signal.
			// Output LOW pulse (1.1ms) for each output that is set to MOTOR
			if (Overdue)
			{
				for (i = 0; i < MAX_OUTPUTS; i++)
				{
					// Check for motor marker
					if (Config.Channel[i].Motor_marker == MOTOR)
					{
						// Set output to maximum pulse width
						ServoOut[i] = MOTOR_0;
					}
				}
			}
			
			// If, for some reason, a higher power has banned PWM output for this cycle, 
			// just fake a PWM interval. The PWM interval is currently 2.3ms, and doesn't vary.
			// This keeps the cycle time more constant.
			if (PWMOverride)
			{
				_delay_us(2300);
			}
			// Otherwise just output PWM normally
			else
			{
				output_servo_ppm(ServoFlag);		// Output servo signal			
			}


			// Decrement PWM pulse sum
			if ((Config.Servo_rate == FAST) && (PWM_pulses > 0))
			{
				PWM_pulses--;
			}
			
			LoopCount = 0;						// Reset loop counter
		}
		
		// In FAST mode and in-between bursts, sync up with the RC so that the time from Interrupt to PWM is constant.
		// This helps tighten up the number of pulses allowable
		else if ((Config.Servo_rate == FAST) && (PWMBlocked == true) && (RCrateMeasured == true) && (RCInterruptsON == true) && (Overdue == false))
		{
			fast_sync_timer = 0;
			
			// Wait here until interrupted or timed out (15ms)
			while ((Interrupted == false) && (fast_sync_timer < FASTSYNCLIMIT))
			{
				fast_sync_timer += (uint8_t)(TCNT2 - fast_sync_TCNT2);
				fast_sync_TCNT2 = TCNT2;
			}
		}
	
		//************************************************************
		//* Enable RC interrupts when ready (RC rate measured and RC interrupts OFF)
		//* and just one PWM remains
		//************************************************************

		if ((PWM_pulses < 1) && RCrateMeasured && !RCInterruptsON && (Config.Servo_rate == FAST))
		{
			init_int();					// Re-enable interrupts
			RCInterruptsON = true;
		}
		
		//************************************************************
		//* Carefully update idle screen if error level changed
		//************************************************************	

		// Only update idle when error state has changed.
		// This prevents the continual updating of the LCD disrupting the FC
		if ((old_alarms != General_error) && (Menu_mode == IDLE))
		{
			// Force safe update of idle screen
			Menu_mode = PRESTATUS_TIMEOUT;
		}
			
		// Save current alarm state into old_alarms
		old_alarms = General_error;
	} // main loop
} // main()

