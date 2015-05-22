// **************************************************************************
// OpenAero2Serial software for KK2.0 and KK2.1
// ============================================
// Version 1.0 Alpha 6 - May 2015
//
// Some receiver format decoding code from Jim Drew of XPS and the Papparazzi project
// OpenAero code by David Thompson, included open-source code as per quoted references
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
// * NB: Summary - all derivative code MUST be released with the source code!
// *
// **************************************************************************
// Version History
// ===============
// V1.0		Based on OpenAero2 V1.4 Beta 2
//
// Alpha 1	Removed unneeded files.
//			Added UART TX routines for all main formats.
//			Removed device type.
// Alpha 2	Fixed Spektrum output.
// Alpha 3	Tweaked Satellite binding, fixed Xtreme inputs.
// Alpha 4	Added support for 8+ channels across input types.
//			Fixed unintentional bind mode on power-up for Satellite RXs.
//			Hard-coded Status timeout to 3 seconds.
//			Added LED feedback for failures in all three output formats.
//			Fixed 16-channel Xtreme in being converted to 8-channel out.
//			Fixed S.Bus2 only recognising 1 in 4 packets.
//			Fixed Status mode causing serial corruption.
//			Fixed generation and receipt of bad Spektrum packets. 
//			Correctly pass the number of channels available to generate in all formats. 
//			Use native resolution (+/-1250), not servo resolution (1000~2000)
//			Extra channels now handled with correct sizing that suits the conversion.
//			S.Bus2 end byte processing simplified.
// Alpha 5	Corrected all format conversion factors.
//			Added "OFF" as a failsafe option.
//			Added separate timer for detecting dropped frames.
//			Now correctly calculates frame period.
//			Don't start transmitting until data received and measured.
//			Added binding modes for all DSM2/DSMX 1024/2048 receivers.
//			Button 1 = DSM2 1024/22ms, Button 2 = DSM2 2048/11ms,
//			Button 3 = DSMX 2048/22ms, Button 4 = DSMX 2048/11ms.
//			Added complex code to retransmit Spektrum data in as efficient an order as possible.
//			Moved CPPM input to THR to be consistent with the serial input.
//			Frame rate measurement added for CPPM mode.
//			Handle serial data errors properly
//			Fixed failsafe re-sync for CPPM.
//			Fixed all Spektrum channel regeneration bugs
// Alpha 6	
//
//	
//***********************************************************
//* To do
//***********************************************************
//
//	ToDo:	Test A6 additions.
//			Failsafe modes		BASIC = OK. ADV untested
//		
//			
//	Bugs:
//			
//			
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
#define	OUTPUT_RATE_LOW 434			// Output rate. 19531/45(Hz) = 434, 22ms for Spektrum testing

#define SECOND_TIMER 19531			// Unit of timing for seconds
#define LMA_TIMEOUT 1171860			// Number or T2 cycles before Lost Model alarm sounds (1 minute)
#define STATUS_OFF_TIME 3			// Number of seconds before the status screen disappears
#define FAILSAFE_MARGIN	5000		// Amount of T1 time over the current frame rate before we assume a frame has been dropped (2ms = 5000)

//***********************************************************
//* Code and Data variables
//***********************************************************

// Flags
uint8_t	volatile General_error = 0;
uint8_t	volatile Flight_flags = 0;
uint8_t	volatile Alarm_flags = 0;

// Global buffers
char pBuffer[PBUFFER_SIZE];			// Print buffer (25 bytes)

// Serial buffer
uint8_t sBuffer[SBUFFER_SIZE];			// Serial buffer (38 bytes)

// Misc globals
volatile uint16_t	InterruptCount = 0;
volatile uint16_t	LoopStartTCNT1 = 0;
volatile uint8_t	LoopCount = 0;

volatile uint32_t FrameDrop_Output_Rate = 0;
volatile uint32_t Failsafe_Output_Rate = 0;
volatile uint16_t RC_Timeout = 0;

//************************************************************
//* Main loop
//************************************************************

int main(void)
{
	// Flags
	bool Interrupted_Clone = false;
	bool OutputTick = false;
	bool OverrideOutput = false;

	// 32-bit timers
	uint32_t LostModel_timer = 0;
	
	// 16-bit timers
	uint16_t Status_timeout = 0;
	uint16_t UpdateStatus_timer = 0;
	uint16_t Ticker_Count = 0;
	uint16_t Save_TCNT1 = 0;
	uint16_t ticker_16 = 0;
	uint16_t FrameRate_TCNT1 = 0;
	
	// Timer incrementers
	uint8_t Status_TCNT2 = 0;
	uint8_t Refresh_TCNT2 = 0;
	uint8_t Lost_TCNT2 = 0;
	uint8_t Ticker_TCNT2 = 0;
	uint8_t RC_TCNT2 = 0;

	// Locals
	uint16_t InterruptCounter = 0;
	uint8_t	LMA_minutes = 0;
	uint8_t Status_seconds = 0;
	uint8_t Menu_mode = STATUS_TIMEOUT;
	uint8_t	old_flight = 0;			// Current/old flight profile
	uint8_t i = 0;
	uint32_t interval = 0;			// IMU interval


	init();							// Do all init tasks

	// Main loop
	while (1)
	{
		// Increment the loop counter
		LoopCount++;
		
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
			
			// Check if Vbat lower than trigger
			if (GetVbat() < Config.PowerTriggerActual)
			{
				General_error |= (1 << LVA_ALARM);	// Set LVA_Alarm flag
			}
			else
			{
				General_error &= ~(1 << LVA_ALARM);	// Clear LVA_Alarm flag
			}
		}

		//************************************************************
		//* State machine for switching between screens safely
		//* Particularly in FAST mode, if anything slows down the loop
		//* time significantly (beeps, LCD updates) the output generation
		//* is at risk of corruption. To get around that, entry and exit
		//* from special states must be handled in stages.
		//* In the state machine, once a state changes, the new state 
		//* will be processed in the next loop.
		//************************************************************
		
		// Assume output is OK until through the state machine
		// If the state machine requires output to be blocked,
		// it will set this flag
		OverrideOutput = false;
		
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

					// Prevent output output
					OverrideOutput = true;
					
					// When not in idle mode, enable Timer0 interrupts as loop rate 
					// is slow and we need TMR0 to fully measure it.
					// This may cause output generation interruption
					TIMSK0 |= (1 << TOIE0);	
				}
				// Idle mode - fast loop rate so don't need TMR0.
				// We don't want TMR0 to interrupt output generation.
				else
				{
					TIMSK0 = 0; 		// Disable Timer0 interrupts
					TIFR0 = 1;			// Clear interrupt flag
				}
				break;

			// Waiting to safely enter Status screen
			// If Interrupted or Interrupted_Clone is true, data must have just completed.
			// If failsafe is true, there is no data to interrupt.
			// output activity must stop before we attempt to pop up the status screen.
			case PRESTATUS:
				// If interrupted, or if currently "No signal"
				if (Interrupted || Interrupted_Clone || (Flight_flags & (1 << FailsafeFlag)))
				{
					// Ready to move on
					Menu_mode = STATUS;

					// Prevent output output
					OverrideOutput = true;
					
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

				// Prevent output output just after updating the LCD
				OverrideOutput = true;

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
					
					// Prevent output output just after updating the LCD
					OverrideOutput = true;
				}

				// Jump to menu if button pressed
				else if(BUTTON1 == 0)
				{
					Menu_mode = MENU;
					
					// Prevent output output
					OverrideOutput = true;
				}

				// Update status screen four times/sec while waiting to time out
				else if (UpdateStatus_timer > (SECOND_TIMER >> 2))
				{
					Menu_mode = PRESTATUS;
					
					// Prevent output output
					OverrideOutput = true;
				}

				// Normal case passes through here
				else
				{
					// Enable output output
					OverrideOutput = false;
				}

				break;

			// Attempting to leave Status gracefully while output stopped.
			// If Interrupted or Interrupted_Clone is true, data must have just completed.
			// If failsafe is true, there is no data to interrupt.
			// output activity must stop before we attempt to pop up the status screen.
			case PRESTATUS_TIMEOUT:
				// If interrupted, or if currently "No signal"
				if (Interrupted || Interrupted_Clone || (Flight_flags & (1 << FailsafeFlag)))
				{
					// Switch to STATUS_TIMEOUT mode
					Menu_mode = STATUS_TIMEOUT;
				
					// Enable output output
					OverrideOutput = false;
				
					// Clear Interrupted_Clone
					Interrupted_Clone = false;
				}
				else
				{
					// Prevent output output
					OverrideOutput = true;
				}
								
				break;

			// In STATUS_TIMEOUT mode, the idle screen is displayed and the mode 
			// changed to POSTSTATUS_TIMEOUT. 
			case STATUS_TIMEOUT:

				// Pop up the Idle screen
				idle_screen();

				// Switch to IDLE mode
				Menu_mode = POSTSTATUS_TIMEOUT;

				// Prevent output output
				OverrideOutput = true;

				break;

			// In POSTSTATUS_TIMEOUT mode, we wait for a output cycle to complete
			// The idle screen has been refreshed and we need to wait.
			case POSTSTATUS_TIMEOUT:
				// If interrupted, or if currently "No signal"
				if (Interrupted || Interrupted_Clone || (Flight_flags & (1 << FailsafeFlag)))
				{
					// Switch to IDLE mode
					Menu_mode = IDLE;

					// Prevent output output
					OverrideOutput = false;
					
					// Clear Interrupted_Clone
					Interrupted_Clone = false;
				}
				else
				{
					// Enable output output
					OverrideOutput = true;
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
				
				// Prevent output output
				OverrideOutput = true;
												
				break;

			default:
				break;
		}

		//************************************************************
		//* Alarms
		//************************************************************

		// Lost model alarm
		LostModel_timer += (uint8_t) (TCNT2 - Lost_TCNT2);
		Lost_TCNT2 = TCNT2;

		// Reset LMA count if any RX activity, LMA off
		if ((Flight_flags & (1 << RxActivity)) || (Config.LMA_enable == 0))
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
			  (Flight_flags & (1 << FailsafeFlag))) &&
			  (Alarm_flags & (1 << BUZZER_ON))) 
		{
			LVA = 1;
		}
		else 
		{
			LVA = 0;
		}
			
		//************************************************************
		//* Get RC data
		//************************************************************

		// Update zeroed RC channel data
		RxGetChannels();

		// Zero RC when in Failsafe
		if (Flight_flags & (1 << FailsafeFlag))
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

		if (RCinputs[Config.FlightChan] > 500)
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
		if ((Config.Flight != old_flight) || (Flight_flags & (1 << FirstTimeFlightMode)))
		{
			// Clear first time flag
			Flight_flags &= ~(1 << FirstTimeFlightMode);

			// Update travel limits
			UpdateLimits();
	
			// Reset I-terms so that neutral is reset
			// Using memset saves code space
			memset(&IntegralGyro[ROLL], 0, sizeof(int32_t) * NUMBEROFAXIS);

			old_flight = Config.Flight;
		}
			
		// Detect when sticks centered (hands free)
		RC_Deadband();
		
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
		if ((Config.FailsafeType == ADVANCED) && (Flight_flags & (1 << FailsafeFlag)))
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
		
		// Sets the desired output rate
		FrameDrop_Output_Rate += (uint16_t)(Save_TCNT1 - FrameRate_TCNT1);
		Failsafe_Output_Rate += (uint16_t)(Save_TCNT1 - FrameRate_TCNT1);
		FrameRate_TCNT1 = Save_TCNT1;
				
		// 16-bit timers (Max. 3.35s measurement on T2)
		// All TCNT2 timers increment at 19.531 kHz		

		// Signal RC overdue after RC_OVERDUE time (500ms)
		RC_Timeout += (uint8_t)(TCNT2 - RC_TCNT2);
		RC_TCNT2 = TCNT2;

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
		//* Flag "failsafe" if no input for 500ms
		//* RC_Timeout is reset at the successful completion of a new RC packet
		//* Reset on successful receipt of an RX packet. Both FailsafeFlag and RC_Timeout.
		//************************************************************

		// Check to see if the RC input is overdue (500ms)
		if (RC_Timeout > RC_OVERDUE)
		{
			 Flight_flags |= (1 << FailsafeFlag);	// This results in a failsafe condition
			 
			// As T2 increments at 19.531 kHz, this will wrap after 3.35s.
			// To stop this problem, clip the value at 1s.			
			if (RC_Timeout > SECOND_TIMER)
			{
				RC_Timeout = SECOND_TIMER;
			}
		}

/*
			 // Debug
			 if (Flight_flags & (1 << FailsafeFlag))
			 {
				 LED1 = 1;
			 }
			 else
			 {
				 LED1 = 0;
			 }
*/
		//************************************************************
		//* Manage desired output update rate in failsafe mode or 
		//* when covering dropped frames.
		//* FrameDrop will be set (FAILSAFE_MARGIN * T1 units) 
		//* (5000 / 2500000 = 2ms) after the current frame period.
		//* OutputTick will be set at approximately the frame rate
		//*
		//* FramePeriod is last frame period in T1 units (1/2,500,000) measured from 
		//* start of burst to start of next burst.
		//* FrameDrop_Output_Rate is also in T1 units (1/2,500,000), wraps at 1718s
		//*
		//***********************************************************

		if (Failsafe_Output_Rate > (FramePeriod + FAILSAFE_MARGIN))
		{
			// Flag that the frame has been dropped, but only when NOT in failsafe mode
			// and only if valid reception has started
			if (!(Flight_flags & (1 << FailsafeFlag)) && (Flight_flags & (1 << RxStarted)))
			{
				Flight_flags |= (1 << FrameDrop);
			}
		}

		// Outputs run at approximately the Failsafe_Output_Rate
		if ((FrameDrop_Output_Rate > FramePeriod) && (Flight_flags & (1 << RxStarted)))
		{
			OutputTick = true;
			FrameDrop_Output_Rate = 0; // This makes an asynchronous oscillator
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
		//* Special handling on return from failsafe
		//*
		//* "What I do when a failsafe comes back is set a flag where the
		//* first valid frame is collected, but no data is output yet. I
		//* allow the next valid frame to resume outputting data. You lose
		//* an extra frame's worth of data, but it is better to do this 
		//* (you are in a failsafe condition anyways) than to have jumbled 
		//* frames together." - Jim Drew
		//*
		//************************************************************

		if ((Alarm_flags & (1 << FAILSAFE_ENDED)) && (Interrupted))
		{
			Interrupted = false;
			Alarm_flags &= ~(1 << FAILSAFE_ENDED);
		}

		//************************************************************
		//* Output serial data where required, 
		//* based on a very specific set of conditions
		//************************************************************
		
		// Cases where we are ready to output. These must be exclusive
		if	(
				// Running at RC rate when Interrupted, assuming that RX has ever started
				((Interrupted) && (Flight_flags & (1 << RxStarted))) ||

				// OutputTick for dropped frames once RX started and NOT in failsafe mode
				// This will stop once failsafe flag is set.
				((!Interrupted) && (!(Flight_flags & (1 << FailsafeFlag))) && (Flight_flags & (1 << FrameDrop)) && (OutputTick)) ||

				// OutputTick for failsafe (>500ms) once RX started and if enabled (Failsafe type = FIXED or ADVANCED)
				((!Interrupted) && (Flight_flags & (1 << FailsafeFlag)) && (OutputTick) && (Config.FailsafeType != NOFAILSAFE))
			
				// When failsafe set to OFF, the frame drop detection will cover for 500ms then cease when failsafe activates.
			)
		{

			//******************************************************************
			//* The following code runs once when serial output is desired
			//******************************************************************

			if (Interrupted)
			{
				Interrupted_Clone = true;			// Hand "Interrupted" baton on to its clone
				Interrupted = false;				// Reset interrupted flag if that was the cause of entry			
			}
			
			// Clear output ticker once fired			
			if (OutputTick)
			{
				OutputTick = false;	
			}
			
			Calculate_PID();				// Calculate PID values
			ProcessMixer();					// Do all the mixer tasks - can be very slow
			Process_servos();				// Check for reversal and limits
							
			// Prevent output output is requested
			if (!OverrideOutput)
			{
				// Note that TransmitData() assumes that ServoOut[] is 
				// updated prior to calling as it destroys the contents
				TransmitData();				// Output serial data			
			}

			LoopCount = 0;					// Reset loop counter
		}
		
		// OutputTick relief. When none of the above are true, OutputTick will not be reset, causing two close output
		// generations to happen. To avoid this, reset it here
/*		else if (OutputTick)
		{
			OutputTick = false;
		}
*/
	} // main loop

} // main()

