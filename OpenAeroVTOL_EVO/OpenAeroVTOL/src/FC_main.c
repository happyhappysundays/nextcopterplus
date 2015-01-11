//**************************************************************************
// OpenAero VTOL software for KK2.1 and later boards
// =================================================
// Version: Release V1.1 Beta 1 - January 2015
//
// Some receiver format decoding code from Jim Drew of XPS and the Paparazzi project
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
//
// Beta 2	Real-time adjustment of servo limits working again.
//
//
//
//***********************************************************
//* Notes
//***********************************************************
//
// Bugs: 
//		V1.0 to V1.1B1 update screws up RC and sensor zeros
//
// Todo: 
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
#define RC_LOW_REJECT_RATE 62500	// 25ms = 62500/2500 = 40Hz 
#define RC_HIGH_REJECT_RATE 20000	// 8ms = 20000/2500000 = 125Hz
#define	SERVO_RATE_LOW 390			// Requested servo rate when in Normal mode. 19531 / 50(Hz) = 390 - 19.97ms
#define SECOND_TIMER 19531			// Unit of timing for seconds
#define ARM_TIMER_RESET_1 960		// RC position to reset timer for aileron, elevator and rudder
#define ARM_TIMER_RESET_2 50		// RC position to reset timer for throttle
#define TRANSITION_TIMER 195		// Transition timer units (10ms * 100) (1 to 10 = 1s to 10s)
#define ARM_TIMER 19531				// Amount of time the sticks must be held to trigger arm. Currently one second.
#define DISARM_TIMER 58593			// Amount of time the sticks must be held to trigger disarm. Currently three seconds.
#define SBUS_PERIOD	8750			// Period for S.Bus data to be transmitted + margin (3.5ms)
#define PWM_PERIOD 12500			// PWM generation period (5ms)

//***********************************************************
//* Code and Data variables
//***********************************************************

// Flight variables
uint32_t interval;					// IMU interval
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
char sBuffer[SBUFFER_SIZE];			// Serial buffer (25 bytes)

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
volatile uint16_t InterruptCount = 0;
volatile uint16_t LoopStartTCNT1 = 0;
volatile bool Overdue = false;
volatile uint8_t	LoopCount = 0;
volatile bool SlowRC = true;

volatile uint32_t RC_Master_Timer = 0; // debug
volatile uint32_t PWM_Available_Timer = 0; // debug
		
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
	bool PWM_Last_Call = false;

	// 32-bit timers
	uint32_t Arm_timer = 0;
	uint32_t RC_Rate_Timer = 0;
//	uint32_t RC_Master_Timer = 0;

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
	uint16_t RC_Interrupts = 0;
	uint8_t	old_alarms = 0;
	uint8_t ServoFlag = 0;
	uint8_t i = 0;
	
	init();							// Do all init tasks

	// Main loop
	while (1)
	{
		// Increment the loop counter
		LoopCount++;
		
		//************************************************************
		//* Check for interruption of PWM generation
		//* The "JitterFlag" flag was reset just before PWM generation
		//************************************************************

		if (JitterFlag == true)
		{
			InterruptCounter++;
		}

		//************************************************************
		//* State machine for switching between screens safely
		//************************************************************

		switch(Menu_mode) 
		{
			// In IDLE mode, the text "Press for status" is displayed ONCE.
			// If a button is pressed the mode changes to STATUS
			case IDLE:
				if((PINB & 0xf0) != 0xf0)
				{
					Menu_mode = STATUS;
					// Reset the status screen timeout
					Status_seconds = 0;
					menu_beep(1);
					
					// When not in idle mode, enable Timer0 interrupts as loop rate 
					// is slow and we need TMR0 to fully measure it.
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

			// Status screen first display
			case STATUS:
				// Reset the status screen period
				UpdateStatus_timer = 0;

				// Update status screen
				Display_status();
				
				// Force code to wait for a new packet
				Interrupted = false;

				// Debug
				PWMBlocked = true;	
				init_int();

				// Wait for timeout
				Menu_mode = WAITING_TIMEOUT_BD;
				break;

			// Status screen up, but button still down ;)
			// This is designed to stop the menu appearing instead of the status screen
			// as it will stay here until the button is released
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
				if (Status_seconds >= 2)
				//if (Status_seconds >= 10) // debug
				{
					Menu_mode = STATUS_TIMEOUT;
				}

				// Jump to menu if button pressed
				else if(BUTTON1 == 0)
				{
					Menu_mode = MENU;
					menu_beep(1);
					
					// Force code to wait for a new packet
					Interrupted = false;
				}

				// Update status screen while waiting to time out
				else if (UpdateStatus_timer > (SECOND_TIMER >> 2))
				{
					Menu_mode = STATUS;
					Disable_RC_Interrupts(); // Debug
				}

				break;

			// In STATUS_TIMEOUT mode, the idle screen is displayed and the mode changed to IDLE
			case STATUS_TIMEOUT:
				// Pop up the Idle screen
				idle_screen();

				// Switch to IDLE mode
				Menu_mode = IDLE;

				// Force code to wait for a new packet
				Interrupted = false;
				PWMBlocked = false;	

				break;

			// In MENU mode, 
			case MENU:
				LVA = 0;	// Make sure buzzer is off :)
				// Disarm the FC
				General_error |= (1 << DISARMED);
				// Start the menu system
				menu_main();
				// Switch back to status screen when leaving menu
				Menu_mode = STATUS;
				// Reset timeout once back in status screen
				Status_seconds = 0;
				// Reset IMU on return from menu
				reset_IMU();
				
				// Force code to wait for a new packet
				Interrupted = false;
								
				break;

			default:
				break;
		}

		//************************************************************
		//* Status menu timing
		//************************************************************

		// Count elapsed seconds
		if (Status_timeout > SECOND_TIMER)
		{
			Status_seconds++;
			Status_timeout = 0;

			// Update the interrupt count each second
			InterruptCount = InterruptCounter;
			InterruptCounter = 0;
		}

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
				menu_beep(1);						// Signal that FC is now disarmed
			}
		}
		else
		{
			General_error &= ~(1 << NO_SIGNAL);	// Clear NO_SIGNAL bit
		}

		// Beep buzzer if Vbat lower than trigger
		// Vbat is measured in units of 10mV, so a PowerTrigger of 127 equates to 12.7V
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
				menu_beep(20);							// Signal that FC is ready
				reset_IMU();							// Reset IMU just in case...
			}
			// Else, disarm the FC after DISARM_TIMER seconds if aileron at max
			else if ((Arm_timer > DISARM_TIMER) && (RCinputs[AILERON] > ARM_TIMER_RESET_1))
			{
				Arm_timer = 0;
				General_error |= (1 << DISARMED);		// Set flags to disarmed
				menu_beep(1);							// Signal that FC is now disarmed
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
			if ((Disarm_seconds >= Config.Disarm_timer) && (Config.Disarm_timer >= 30))	
			{
				// Disarm the FC
				General_error |= (1 << DISARMED);		// Set flags to disarmed
				menu_beep(1);							// Signal that FC is now disarmed
			}
		}
		// Arm when ArmMode is OFF
		else 
		{
			General_error &= ~(1 << DISARMED);			// Set flags to armed
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
		//* The transition will hold at P1n position if directed to
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
		
		// Update state, values and transition_counter every Config.TransitionSpeed if not zero. 195 = 10ms
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

		// Work out the current RC rate by measuring between incoming RC packets
		//RC_Rate_Timer += (Save_TCNT1 - RC_Rate_TCNT1);
		//RC_Rate_TCNT1 = Save_TCNT1;
		
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
		//* Manage desired output update rate when limited by
		//* the PWM rate set to "Low"
		//************************************************************

		// Flag update required based on SERVO_RATE_LOW (50Hz) - 19.97ms
		if (Servo_Rate > SERVO_RATE_LOW)
		{
			ServoTick = true; // Slow device is ready for output generation
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
			interval = ticker_16;
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
				
		// Call IMU with interval
		simple_imu_update(interval);

		//************************************************************
		//* Update I-terms, average gyro values
		//************************************************************

		Sensor_PID();
		
		//************************************************************
		//* Measure incoming RC. Result in SlowRC state and RC_Rate_Timer
		//************************************************************

		if (Interrupted)
		{
			// Measure incoming RC rate. Threshold is 60Hz.
			// In high-speed mode, the RC rate will be unfairly marked as "slow" once measured and interrupt blocking starts.
			// To stop this being a problem, only set SlowRC prior to RCrateMeasured becoming true in this mode
			if (((Config.Servo_rate == FAST) && (!RCrateMeasured)) || (Config.Servo_rate < FAST))
			{
				if (RC_Rate_Timer > SLOW_RC_RATE)
				{
					SlowRC = true;
				}
				else
				{
					SlowRC = false;
				}		
				
				// If RC rate not measured yet keep refreshing RC_Master_Timer		
				// This is only valid for high speed mode. Other modes just use the SlowRC flag.
				// Note that Framerate is only valid for serial data
				RC_Master_Timer = FrameRate; 
			}
			
			// Reset RC timeout
			RC_Timeout = 0;

			// No longer overdue
			Overdue = false;
			
			// Reset rate timer once data received
			RC_Rate_Timer = 0;
							
			// Increment interrupt counter
			RC_Interrupts++;		

			// Block RC interrupts until timeout if period has been calculated
			if ((Config.Servo_rate == FAST) && RCrateMeasured)
			{
				Interrupted = false;		// Cancel pending interruots
				Disable_RC_Interrupts();	// Disable RC interrupts
				RCInterruptsON = false;		// Flag it for the rest of the code
				PWMBlocked = false;			// Enable PWM generation
			}
		}
		
		//************************************************************
		//* Work out the high speed mode RC blocking period once only. Only relevant for high speed mode.
		//* Wait for at least five interrupt cycles before measuring.
		//************************************************************

		if(!RCrateMeasured && (RC_Interrupts > 5))
		{
			// Work out the exact amount of time, at the current loop rate that we must wait before
			// signalling "last call" and re-enabling interrupts

			// RC rate is 22ms
			if (SlowRC)
			{
				// 41ms (8 cycles)
				PWM_Available_Timer = (2 * (RC_Master_Timer + SBUS_PERIOD)) - SBUS_PERIOD - PWM_PERIOD - PWM_PERIOD;
			}

			// RC rate is 11ms
			else
			{
				// 30ms (6 cycles)
				PWM_Available_Timer = (3 * (RC_Master_Timer + SBUS_PERIOD)) - SBUS_PERIOD - PWM_PERIOD - PWM_PERIOD;
			}
			
			// Once the high speed rate has been calculated, signal that PWM is good to go.
			RCrateMeasured = true;			
		}
	
		//************************************************************
		//* Enable RC interrupts when ready (RC rate measured and RC interrupts OFF)
		//* and set PWM last call made
		//************************************************************

		if ((RC_Rate_Timer > PWM_Available_Timer) && RCrateMeasured && !RCInterruptsON)
		{
			// Re-enable interrupts
			init_int();
			RCInterruptsON = true;
			
			// Signal last PWM generation
			PWM_Last_Call = true;
		}

		//************************************************************
		//* Output PWM to ESCs/Servos where required, 
		//* based on a very specific set of conditions
		//************************************************************

		// Cases where we are ready to output
		if	(
				// Interrupted and LOW or SYNC
				((Config.Servo_rate != FAST) && (Interrupted)) ||			// Run at RC rate

				// Every loop in FAST mode unless blocked
				((Config.Servo_rate == FAST) && (!PWMBlocked))				// Run at full loop rate if allowed
			)
		{

			//******************************************************************
			//* The following code runs once when PWM generation is desired
			//* The execution rates are:
			//* The RC rate unless in FAST mode
			//* High speed in FAST mode
			//******************************************************************

			if (Config.Servo_rate != FAST)
			{
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
								
			// Reset slow PWM flag if it was just set. It will automatically set again at around 50Hz
			if (ServoTick)
			{
				ServoTick = false;
			}

			// Block PWM generation after last call called
			if (PWM_Last_Call)
			{
				PWMBlocked = true;				// Block PWM generation on notification of last call
				PWM_Last_Call = false;			// Reset last call flag	
			}
			
			Calculate_PID();					// Calculate PID values
			ProcessMixer();						// Do all the mixer tasks - can be very slow
			UpdateServos();						// Transfer Config.Channel[i].value data to ServoOut[i] and check servo limits
			output_servo_ppm(ServoFlag);		// Output servo signal
			
			LoopCount = 0;						// Reset loop counter for averaging accVert
		}
		
		// Not ready to output PWM, but should clear Interrupted as all code that needs it has seen it already
		// Cases are: FAST mode and PWM blocked, or SLOW, SYNC but not interrupted
		else 
		{
		//	Interrupted = false;					// Reset interrupted flag
		}

		//************************************************************
		//* Carefully update idle screen if error level changed
		//************************************************************	

		// Only update idle when error state has changed.
		// This prevents the continual updating of the LCD disrupting the FC
		if (old_alarms != General_error)
		{
			// Force update of idle screen
			Menu_mode = STATUS_TIMEOUT;
		}
			
		// Save current alarm state into old_alarms
		old_alarms = General_error;
		
	} // while loop
} // main()

