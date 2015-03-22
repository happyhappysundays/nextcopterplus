 //**************************************************************************
// OpenAero VTOL software for KK2.1 and later boards
// =================================================
// Version: Release V1.1 Beta 12 - March 2015
//
// Some receiver format decoding code from Jim Drew of XPS and the Paparazzi project.
// OpenAero code by David Thompson, included open-source code as per quoted references.
//
// **************************************************************************
// * 						GNU GPL V3 notice
// **************************************************************************
// * Copyright (C) 2015 David Thompson
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
// * tl;dr - all derivative code MUST be released with the source code!
// *
// **************************************************************************
// Version History
// ===============
//
// V1.1	Based on OpenAeroVTOL V1.0 code.
//
// Beta 1	Removed all KK2.0-specific code.
//			Restore the 16-bit indexing of menu items.
//			Add AL(Roll/Pitch) to universal mixers.
//			Added preliminary eeprom upgrade functionality.
//			Added auto-upgrade for V1.0 to V1.1 eeprom settings.
//			Simplified mixer menu and mixer code.
//			Added REVERSE and SCALED_REVERSED to sensor options (though disabled for now)
//			Tidied up eeprom routines, removed unnecessary second argument from write_buffer()
//			Removed unnecessary casts. Various edits to cover the most strict warnings settings.
//			Added high-speed mode. Only works with S.Bus selected. 
//			Quadcopter compile option updated.
//			Analog servo output synced to ServoTick to better regulate slow PWM generation.
// Beta 2	Real-time adjustment of servo limits working again.
//			Mixer SCALE and REV mixed up in menus - fixed.
//			FAST mode frame rate detection updated once per second 
//			Servo settings reset on update from V1.0 for now.
// Beta 3	LVA changed to "OFF, 3.5, 3.6, 3.7, 3.8V" and nominal voltage auto-calculated
//			Battery voltage accuracy improved.
//			Fixed the V1.0 to V1.1 Beta1 eeprom update corruption
// Beta 4	Loop interval integrated into the "PWM_Available_Timer" to counter PWM jitter issues in FAST mode.
//			It should dynamically change to suit any mixer loading and loop cycle.
// Beta 5	Trying new FAST PWM generation method
// Beta 6	Remove test defaults
// Beta 7	Fixed servo travel volumes.Increased FAST mode to ~250Hz.
//			Smoothed A.Servo variation in FAST mode to about 18-23ms.
//			Improved button feel for all data types.
// Beta 8	Fixed missing mixer volume polarity checking
//			Suggested menu layout changes from Ran.
//			Fixes for output glitches in/out of screens
//			V1.1 B8 auto update code. No beeps on initial screens. 
//			LED indicates ARMED. Fixed V1.0 to V1.1 eeprom update code to automatically track
//			Config structure location. Acc LPF default changed to "None".
//			Status screen error messages are now just text.
//			Change in error status no longer jumps from Status screen to Idle.
//			Status timeout restored to 10s. 
// Beta 9	Made level meter show more representative feel for AccLPF.
//			Changed AccLPF settings and text to better match each other.
//			AccLPF settings now identical to MPU6050 settings and are now floating point numbers.
//			Target A.Servo rate changed to 65Hz~70Hz to reduce variation.
// Beta 10	Fix up the upgrade path to the new filters.
//			Fixed bug with new gyro filter. Acc LPF default changed back to 21Hz.
//			Fixed bugs with both filters where they may not have been turned off when set to "None".
//			Quadcopter settings updated. Status screen shows "Quad P" or "Quad X" if preset in use.
//			Software filter settings above 94Hz disabled in high-speed mode. Quadcopter X defaults updated.
//			Removed start-up beep. Menu beeps less annoying.
//			Added LVA settings down to 3.2V. 
// Beta 11	Made I-terms vary with loop period.
//			Added more code to stabilise loop period in high-speed mode.
//			FAST mode now allowable for Satellite and XPS Xtreme RXs.
//			Tweaked menu beeps. Inverted cal audio confirmation.
//			Xtreme support restored.
//			Serial buffer increased to 38 bytes to handle maximum size of Xtreme packets.
//			Fixed previously unknown gyro LPF bug.
//			Removed gyro noise gate in PID loop.
// Beta 12	Added user selectable presets (Manual, QuadX, QuadP).
//			FINALLY made the menu system correctly index beyond 256 items.
//
//***********************************************************
//* Notes
//***********************************************************
//
// Bugs:
//	
//
// To do:	
//		
//			
//	
//			
//
//***********************************************************
//* Includes
//***********************************************************

#include "compiledefs.h"
#include <avr/io.h>
#include <avr/pgmspace.h> 
#include <avr/wdt.h>
#include <avr/interrupt.h>
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

#include "Font_Verdana.h" 			// 8 (text) and 14 (titles) points
#include "Font_WingdingsOE2.h"		// Cursor and markers

//***********************************************************
//* Defines
//***********************************************************

#define	RC_OVERDUE 9765				// Number of T2 cycles before RC will be overdue = 9765 * 1/19531 = 500ms
#define	SLOW_RC_RATE 41667			// Slowest RC rate tolerable for Plan A syncing = 2500000/60 = 16ms
#define	SERVO_RATE_LOW 300			// A.servo rate. 19531/65(Hz) = 300
#define SECOND_TIMER 19531			// Unit of timing for seconds
#define ARM_TIMER_RESET_1 960		// RC position to reset timer for aileron, elevator and rudder
#define ARM_TIMER_RESET_2 50		// RC position to reset timer for throttle
#define TRANSITION_TIMER 195		// Transition timer units (10ms * 100) (1 to 10 = 1s to 10s)
#define ARM_TIMER 19531				// Amount of time the sticks must be held to trigger arm. Currently one second.
#define DISARM_TIMER 58593			// Amount of time the sticks must be held to trigger disarm. Currently three seconds.
#define SBUS_PERIOD	6250			// Period for S.Bus data to be transmitted (no margin) (2.5ms)
#define SBUS_MARGIN	8750			// Period for S.Bus data to be transmitted (+ 1ms margin) (3.5ms)
#define PWM_PERIOD 12500			// Average PWM generation period (5ms)
#define PWM_PERIOD_WORST 20833		// PWM generation period (8.3ms - 120Hz)
#define PWM_PERIOD_BEST 8333		// PWM generation period (3.333ms - 300Hz)

//***********************************************************
//* Code and Data variables
//***********************************************************

// Flight variables
int16_t transition_counter = 0;
uint8_t Transition_state = TRANS_P1;
int16_t	transition = 0; 

// Flags
volatile uint8_t	General_error = 0;
volatile uint8_t	Flight_flags = 0;
volatile uint8_t	Alarm_flags = 0;

// Global buffers
char pBuffer[PBUFFER_SIZE];			// Print buffer (16 bytes)

// Serial buffer
char sBuffer[SBUFFER_SIZE];			// Serial buffer (38 bytes)

// Transition matrix
// Usage: Transition_state = Trans_Matrix[Config.FlightSel][old_flight]
// Config.FlightSel is where you've been asked to go, and old_flight is where you were.
// Transition_state is where you end up :)
const int8_t Trans_Matrix[3][3] PROGMEM = 	
	{
		{TRANSITIONING, TRANS_P1n_to_P1_start, TRANS_P2_to_P1_start},
		{TRANS_P1_to_P1n_start,TRANSITIONING,TRANS_P2_to_P1n_start},
		{TRANS_P1_to_P2_start,TRANS_P1n_to_P2_start,TRANSITIONING}
	};

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
	bool TransitionUpdated = false;
	bool RCrateMeasured = false;
	bool PWMBlocked = false;
	bool RCInterruptsON = false;
	bool ServoTick = false;
	bool ResampleRCRate = false;
	bool PWMOverride = false;
	bool Interrupted_Clone = false;
	bool SlowRC = true;

	// 32-bit timers
	uint32_t Arm_timer = 0;
	uint32_t RC_Rate_Timer = 0;
	uint32_t PWM_interval = PWM_PERIOD_WORST;	// Loop period when generating PWM. Initialise with worst case until updated.
	
	// 16-bit timers
	uint16_t Status_timeout = 0;
	uint16_t UpdateStatus_timer = 0;
	uint16_t Ticker_Count = 0;
	uint16_t RC_Timeout = 0;
	uint16_t Servo_Rate = 0;
	uint16_t Transition_timeout = 0;
	uint16_t Disarm_timer = 0;
	uint16_t Save_TCNT1 = 0;
	uint16_t ticker_16 = 0;

	// Timer incrementers
	uint16_t RC_Rate_TCNT1 = 0;
	uint8_t Transition_TCNT2 = 0;
	uint8_t Status_TCNT2 = 0;
	uint8_t Refresh_TCNT2 = 0;
	uint8_t Disarm_TCNT2 = 0;
	uint8_t Arm_TCNT2 = 0;
	uint8_t Ticker_TCNT2 = 0;
	uint8_t Servo_TCNT2 = 0;
	uint8_t ServoRate_TCNT2 = 0;

	// Locals
	uint16_t InterruptCounter = 0;
	uint8_t	Disarm_seconds = 0;
	uint8_t Status_seconds = 0;
	uint8_t Menu_mode = STATUS_TIMEOUT;
	int8_t	old_flight = 3;			// Old flight profile
	int8_t	old_trans_mode = 0;		// Old transition mode
	int16_t temp1 = 0;
	uint16_t transition_time = 0;
	uint8_t	old_alarms = 0;
	uint8_t ServoFlag = 0;
	uint8_t i = 0;
	int16_t PWM_pulses = 3; 
	uint32_t interval = 0;			// IMU interval
	
	// Do all init tasks
	init();

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
		//* Status menu timing
		//* Increment Status_seconds every second and trigger
		//* a RC rate resample every second
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
		}

		//************************************************************
		//* State machine for switching between screens safely
		//* Particularly in FAST mode, if anything slows down the loop
		//* time significantly (beeps, LCD updates) the PWM generation
		//* is at risk of corruption. To get around that, entry and exit
		//* from special states must be handled in stages.
		//* In the state machine, once a state changes, the new state 
		//* will be process in the next loop.
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
				if (Status_seconds >= 10)
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
					PWMOverride = true; // Debug - not needed yet?
				}

				// Update status screen four times/sec while waiting to time out
				else if (UpdateStatus_timer > (SECOND_TIMER >> 2))
				{
					Menu_mode = PRESTATUS;

					// Prevent PWM output
					PWMOverride = true; // Debug - not needed yet?
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
				// Disarm the FC
				General_error |= (1 << DISARMED);
				LED1 = 0;
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

		// If RC signal is overdue, signal RX error message and disarm
		if (Overdue)
		{
			General_error |= (1 << NO_SIGNAL);		// Set NO_SIGNAL bit
			
			// If FC is set to "armable" and is currently armed, disarm the FC
			if ((Config.ArmMode == ARMABLE) && ((General_error & (1 << DISARMED)) == 0))
			{
				General_error |= (1 << DISARMED);	// Set flags to disarmed
				LED1 = 0;							// Signal that FC is now disarmed
			}
		}
		// RC signal received normally
		else
		{
			General_error &= ~(1 << NO_SIGNAL);		// Clear NO_SIGNAL bit
		}

		// Beep buzzer if Vbat lower than trigger		
		if (GetVbat() < Config.PowerTriggerActual)
		{
			General_error |= (1 << LVA_ALARM);	// Set LVA_Alarm flag
		}
		else 
		{
			General_error &= ~(1 << LVA_ALARM);	// Clear LVA_Alarm flag
		}

		// Turn on buzzer if in alarm state (BUZZER_ON is oscillating)
		if	(
			 (
				(General_error & (1 << LVA_ALARM)) ||		// Low battery
				(General_error & (1 << NO_SIGNAL)) ||		// No signal
				(General_error & (1 << THROTTLE_HIGH))		// Throttle high
			 ) && 
			  (Alarm_flags & (1 << BUZZER_ON))
			) 
		{
			LVA = 1;
		}
		else 
		{
			LVA = 0;
		}

		//************************************************************
		//* Arm/disarm handling
		//************************************************************

		if (Config.ArmMode == ARMABLE)
		{
			// Manual arm/disarm
			// If sticks not at extremes, reset manual arm/disarm timer
			// Sticks down and centered = armed. Down and outside = disarmed
			if (
				((-ARM_TIMER_RESET_1 < RCinputs[AILERON]) && (RCinputs[AILERON] < ARM_TIMER_RESET_1)) ||
				((-ARM_TIMER_RESET_1 < RCinputs[ELEVATOR]) && (RCinputs[ELEVATOR] < ARM_TIMER_RESET_1)) ||
				((-ARM_TIMER_RESET_1 < RCinputs[RUDDER]) && (RCinputs[RUDDER] < ARM_TIMER_RESET_1)) ||
				(ARM_TIMER_RESET_2 < MonopolarThrottle)
			   )
			{
				Arm_timer = 0;
			}

			// If arm timer times out, the sticks must have been at extremes for ARM_TIMER seconds
			// If aileron is at min, arm the FC
			if ((Arm_timer > ARM_TIMER) && (RCinputs[AILERON] < -ARM_TIMER_RESET_1))
			{
				Arm_timer = 0;
				General_error &= ~(1 << DISARMED);		// Set flags to armed (negate disarmed)
				CalibrateGyrosSlow();					// Calibrate gyros
				LED1 = 1;								// Signal that FC is ready
				reset_IMU();							// Reset IMU just in case...
			}
			// Else, disarm the FC after DISARM_TIMER seconds if aileron at max
			else if ((Arm_timer > DISARM_TIMER) && (RCinputs[AILERON] > ARM_TIMER_RESET_1))
			{
				Arm_timer = 0;
				General_error |= (1 << DISARMED);		// Set flags to disarmed
				LED1 = 0;								// Signal that FC is now disarmed
			}

			// Automatic disarm
			// Reset auto-disarm count if any RX activity or set to zero, or when currently disarmed
			if ((Flight_flags & (1 << RxActivity)) || (Config.Disarm_timer == 0) || (General_error & (1 << DISARMED)))
			{														
				Disarm_timer = 0;
				Disarm_seconds = 0;
			}
		
			// Increment disarm timer (seconds) if armed
			if (Disarm_timer > SECOND_TIMER)
			{
				Disarm_seconds++;
				Disarm_timer = 0;
			}

			// Auto-disarm model if timeout enabled and due
			// Don't allow disarms less than 30 seconds. That's just silly...
			if ((Disarm_seconds >= Config.Disarm_timer) && (Config.Disarm_timer >= 30))	
			{
				// Disarm the FC
				General_error |= (1 << DISARMED);		// Set flags to disarmed
				LED1 = 0;								// Signal that FC is now disarmed
			}
		}
		// Arm when ArmMode is OFF
		else 
		{
			General_error &= ~(1 << DISARMED);			// Set flags to armed
			LED1 = 1;
		}

		//************************************************************
		//* Get RC data
		//************************************************************

		// Update zeroed RC channel data
		RxGetChannels();

		// Check for throttle reset
		if (MonopolarThrottle < THROTTLEIDLE)
		{
			// Clear throttle high error
			General_error &= ~(1 << THROTTLE_HIGH);	

			// Reset I-terms at throttle cut. Using memset saves code space
			memset(&IntegralGyro[P1][ROLL], 0, sizeof(int32_t) * 6); 
		}

		//************************************************************
		//* Flight profile / transition state selection
		//*
		//* When transitioning, the flight profile is a moving blend of 
		//* Flight profiles P1 to P2. The transition speed is controlled 
		//* by the Config.TransitionSpeed setting.
		//* The transition will hold at P1n position if directed to.
		//************************************************************

		// P2 transition point hard-coded to 50% above center
		if 	(RCinputs[Config.FlightChan] > 500)
		{
			Config.FlightSel = 2;			// Flight mode 2 (P2)
		}
		// P1.n transition point hard-coded to 50% below center
		else if (RCinputs[Config.FlightChan] > -500)
		{
			Config.FlightSel = 1;			// Flight mode 1 (P1.n)
		}
		// Otherwise the default is P1
		else
		{
			Config.FlightSel = 0;			// Flight mode 0 (P1)
		}

		// Reset update request each loop
		TransitionUpdated = false;

		//************************************************************
		//* Transition state setup/reset
		//*
		//* Set up the correct state for the current setting.
		//* Check for initial startup - the only time that old_flight should be "3".
		//* Also, re-initialise if the transition setting is changed
		//************************************************************

		if ((old_flight == 3) || (old_trans_mode != Config.TransitionSpeed))
		{
			switch(Config.FlightSel)
			{
				case 0:
					Transition_state = TRANS_P1;
					transition_counter = 0;
					break;
				case 1:
					Transition_state = TRANS_P1n;
					transition_counter = Config.Transition_P1n; // Set transition point to the user-selected point
					break;
				case 2:
					Transition_state = TRANS_P2;
					transition_counter = 100;
					break;
				default:
					break;
			}		 
			old_flight = Config.FlightSel;
			old_trans_mode = Config.TransitionSpeed;
		}

		//************************************************************
		//* Transition state handling
		//************************************************************

		// Update timed transition when changing flight modes
		if (Config.FlightSel != old_flight)
		{
			// Flag that update is required if mode changed
			TransitionUpdated = true;
		}

		// Work out transition number when manually transitioning
		// Convert number to percentage (0 to 100%)
		if (Config.TransitionSpeed == 0)
		{
			// Offset RC input to (approx) -250 to 2250
			temp1 = RCinputs[Config.FlightChan] + 1000;

			// Trim lower end to zero (0 to 2250)
			if (temp1 < 0) temp1 = 0;

			// Convert 0 to 2250 to 0 to 125. Divide by 20
			// Round to avoid truncation errors
			transition = (temp1 + 10) / 20;

			// transition now has a range of 0 to 101 for 0 to 2000 input
			// Limit extent of transition value 0 to 100 (101 steps)
			if (transition > 100) transition = 100;
		}
		else
		{
			// transition_counter counts from 0 to 100 (101 steps)
			transition = transition_counter;
		}

		// Always in the TRANSITIONING state when Config.TransitionSpeed is 0
		// This prevents state changes when controlled by a channel
		if (Config.TransitionSpeed == 0)
		{
			Transition_state = TRANSITIONING;
		}

		// Update transition state change when control value or flight mode changes
		if (TransitionUpdated)
		{
			// Update transition state from matrix
			Transition_state = (uint8_t)pgm_read_byte(&Trans_Matrix[Config.FlightSel][old_flight]);
		}

		// Calculate transition time from user's setting
		transition_time = TRANSITION_TIMER * Config.TransitionSpeed;
		
		// Update state, values and transition_counter every Config.TransitionSpeed if not zero.
		if (((Config.TransitionSpeed != 0) && (Transition_timeout > transition_time)) ||
			// Update immediately
			TransitionUpdated)
		{
			Transition_timeout = 0;
			TransitionUpdated = false;

			// Fixed, end-point states
			if (Transition_state == TRANS_P1)
			{
				transition_counter = 0;
			}
			else if (Transition_state == TRANS_P1n)
			{
				transition_counter = Config.Transition_P1n;
			}
			else if (Transition_state == TRANS_P2)
			{
				transition_counter = 100;
			}		

			// Over-ride users requesting silly states
			// If transition_counter is above P1.n but request is P1 to P1.n or 
			// if transition_counter is below P1.n but request is P2 to P1.n...
			if ((Transition_state == TRANS_P1_to_P1n_start) && (transition_counter > Config.Transition_P1n))
			{
				// Reset state to a more appropriate one
				Transition_state = TRANS_P2_to_P1n_start;
			}

			if ((Transition_state == TRANS_P2_to_P1n_start) && (transition_counter < Config.Transition_P1n))
			{
				// Reset state to a more appropriate one
				Transition_state = TRANS_P1_to_P1n_start;
			}

			// Handle timed transition towards P1
			if ((Transition_state == TRANS_P1n_to_P1_start) || (Transition_state == TRANS_P2_to_P1_start))
			{
				transition_counter--;
				if (transition_counter <= 0)
				{
					transition_counter = 0;
					Transition_state = TRANS_P1;
				}
			}

			// Handle timed transition between P1.n and P1
			if (Transition_state == TRANS_P1_to_P1n_start)
			{
				transition_counter++;
				if (transition_counter >= Config.Transition_P1n)
				{
					transition_counter = Config.Transition_P1n;
					Transition_state = TRANS_P1n;
				}
			}			
				
			// Handle timed transition between P1.n and P2
			if (Transition_state == TRANS_P2_to_P1n_start)
			{
				transition_counter--;
				if (transition_counter <= Config.Transition_P1n)
				{
					transition_counter = Config.Transition_P1n;
					Transition_state = TRANS_P1n;
				}
			}

			// Handle timed transition towards P2
			if ((Transition_state == TRANS_P1n_to_P2_start) || (Transition_state == TRANS_P1_to_P2_start))
			{
				transition_counter++;
				if (transition_counter >= 100)
				{
					transition_counter = 100;
					Transition_state = TRANS_P2;
				}
			}

		} // Update transition_counter

		// Zero the I-terms of the opposite state so as to ensure a bump-less transition
		if ((Transition_state == TRANS_P1) || (transition == 0))
		{
			// Clear P2 I-term while fully in P1
			memset(&IntegralGyro[P2][ROLL], 0, sizeof(int32_t) * NUMBEROFAXIS);
		}
		else if ((Transition_state == TRANS_P2) || (transition == 100))
		{
			// Clear P1 I-term while fully in P2
			memset(&IntegralGyro[P1][ROLL], 0, sizeof(int32_t) * NUMBEROFAXIS);
		}
		
		// Save current flight mode
		old_flight = Config.FlightSel;

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

		// Arm timer for timing stick hold
		Arm_timer += (uint8_t)(TCNT2 - Arm_TCNT2); 
		Arm_TCNT2 = TCNT2;

		// 16-bit timers (Max. 3.35s measurement on T2)
		// All TCNT2 timers increment at 19.531 kHz

		// Sets the desired SERVO_RATE by flagging ServoTick when PWM due
		Servo_Rate += (uint8_t)(TCNT2 - ServoRate_TCNT2);
		ServoRate_TCNT2 = TCNT2;
		
		// Signal RC overdue after RC_OVERDUE time (500ms)
		RC_Timeout += (uint8_t)(TCNT2 - Servo_TCNT2);
		Servo_TCNT2 = TCNT2;
		
		// Update transition timer
		Transition_timeout += (uint8_t)(TCNT2 - Transition_TCNT2);
		Transition_TCNT2 = TCNT2;

		// Update status timeout
		Status_timeout += (uint8_t)(TCNT2 - Status_TCNT2);
		Status_TCNT2 = TCNT2;
		
		// Status refresh timer
		UpdateStatus_timer += (uint8_t)(TCNT2 - Refresh_TCNT2);
		Refresh_TCNT2 = TCNT2;

		// Auto-disarm timer
		Disarm_timer += (uint8_t)(TCNT2 - Disarm_TCNT2);
		Disarm_TCNT2 = TCNT2;

		// Timer for audible alarms
		Ticker_Count += (uint8_t)(TCNT2 - Ticker_TCNT2);
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
		//* FrameRate = S.Bus frame gap as measured by the isr.
		//* PWM_interval = Copied from Interval, is the current loop rate.
		//* 
		//* 
		//************************************************************

		if (Interrupted)
		{
			// Measure incoming RC rate. Threshold is SLOW_RC_RATE.
			// Use RC_Rate_Timer is not in FAST mode.
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
			//* PWM pulses will fit in the S.Bus gap.
			//***********************************************************************

			if (RCrateMeasured && (Config.Servo_rate == FAST))
			{
				//
				// Slow packets (19.7ms gap). Pulse spans just two input packets.
				// It may take at worst case 2.6ms before the PWM starts so that too must be subtracted.
				// (2 x 22ms) - 2.6 - 2.5 = 38.9ms available space for S.Bus, 40ms for Satellite and 39.92ms for Xtreme.
				// Each PWM period is about 2.6ms so we need to see how many will fit before the next packet.
				// It is easiest to assume that say 38ms is safe for all formats.
				//
				if (SlowRC)
				{
					PWM_pulses = 4;				// Three pulses will fit if interval faster than 102Hz
				
					if (PWM_interval < 19600)	// 19600 = 7.84ms
					{
						PWM_pulses += 1;		// One more pulse will fit if interval faster than 127Hz
					}
				
					if (PWM_interval < 16333)	// 16333 = 6.53ms
					{
						PWM_pulses += 1;		// One more pulse will fit if interval faster than 153Hz
					}
				
					if (PWM_interval < 14000)	// 14000 = 5.6ms
					{
						PWM_pulses += 1;		// One more pulse will fit if interval faster than 179Hz
					}
				
					if (PWM_interval < 12250)	// 12250 = 4.9ms
					{
						PWM_pulses += 1;		// One more pulse will fit if interval faster than 204Hz
					}
				
					if (PWM_interval < 10888)	// 10888 = 4.35ms
					{
						PWM_pulses += 1;		// One more pulse will fit if interval faster than 230Hz
					}
				
					if (PWM_interval < 9800)	// 9800 = 3.92ms
					{
						PWM_pulses += 1;		// One more pulse will fit if interval faster than 255Hz
					}
				}
				//
				// Fast packets (9ms gap). Pulse spans three input packets.
				// It may take at worst case 2.6ms before the PWM starts so that too must be subtracted.
				// (3 x 11ms) - 2.6 - 2.5 = 27.9ms available space for S.Bus, 29ms for Satellite and 28.9ms for Xtreme.
				// Each PWM period is about 2.6ms so we need to see how many will fit before the next packet.
				// It is easiest to assume that say 27ms is safe for all formats.
				// 
				else
				{
					PWM_pulses = 3;				// Two pulses will fit if interval faster than 101Hz
				
					if (PWM_interval < 18437)	// 18437 = 7.37ms
					{
						PWM_pulses += 1;		// One more pulse will fit if interval faster than 135Hz
					}
				
					if (PWM_interval < 14750)	// 14750 = 5.9ms
					{
						PWM_pulses += 1;		// One more pulse will fit if interval faster than 169Hz
					}
				
					if (PWM_interval < 11886)	// 11886 = 4.75ms
					{
						PWM_pulses += 1;		// One more pulse will fit if interval faster than 210Hz
					}
				
					if (PWM_interval < 10142)	// 10142 = 4.05ms
					{
						PWM_pulses += 1;		// One more pulse will fit if interval faster than 246Hz
					}
				
					if (PWM_interval < 8859)	// 8859 = 3.5ms
					{
						PWM_pulses += 1;		// One more pulse will fit if interval faster than 282Hz
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

			// No longer overdue. This will cancel the "No signal" alarm
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
				(Interrupted) ||											// Run at RC rate
				((Config.Servo_rate == FAST) && (!PWMBlocked))				// Run at full loop rate if allowed
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
				// Mark bits depending on the selected output type
				if	(
						((Config.Servo_rate == FAST) && (Config.Channel[i].Motor_marker == ASERVO) && ServoTick) ||					// At ServoTick for A.Servo in FAST mode
						((Config.Servo_rate == SYNC) && (Config.Channel[i].Motor_marker == ASERVO) && (!SlowRC) && ServoTick) ||	// At ServoTick for A.Servo in SYNC with Fast RC
						((Config.Servo_rate == SYNC) && (Config.Channel[i].Motor_marker == ASERVO) && (SlowRC)) ||					// At RC rate for A.Servo with slow RC
						((Config.Servo_rate >= SYNC) && (Config.Channel[i].Motor_marker > ASERVO)) ||								// Always for D.Servo and Motor in SYNC or FAST modes
						((Config.Servo_rate == LOW) && (!SlowRC) && ServoTick) ||													// All outputs at ServoTick in LOW mode with fast RC
						((Config.Servo_rate == LOW) && (SlowRC))																	// All outputs at  RC rate in LOW mode with slow RC
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
				if ((interval < PWM_PERIOD_WORST) && (interval > PWM_PERIOD_BEST))
				{
					PWM_interval = interval;
				}
				else
				{
					PWM_interval = PWM_PERIOD_WORST; // 120Hz
				}	
			}
			
			Calculate_PID();					// Calculate PID values
			ProcessMixer();						// Do all the mixer tasks - can be very slow
			UpdateServos();						// Transfer Config.Channel[i].value data to ServoOut[i] and check servo limits
			
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
			
			LoopCount = 0;						// Reset loop counter for averaging accVert
		}
		
		// In FAST mode and while remeasuring the RC rate, to keep the loop rate at the approximate PWM rate,
		// just fake a PWM interval. The PWM interval is currently 2.3ms, and doesn't vary, but we have to also 
		// fake the Calculate_PID() and ProcessMixer() times. This keeps the cycle time more constant.
		//else if ((Config.Servo_rate == FAST) && (PWMBlocked)) // denug
		else if (PWMBlocked)
		{
			_delay_us(2600);
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
		
		// Debug
		//LED1 = ~LED1;
		
	} // while loop
} // main()

