//**************************************************************************
// OpenAero VTOL software for KK2.1 and later boards
// =================================================
// Version: Release V1.5 Release - September 2016
//
// Some receiver format decoding code from Jim Drew of XPS and the Paparazzi project.
// OpenAero code by David Thompson, included open-source code as per quoted references.
//
// **************************************************************************
// * 						GNU GPL V3 notice
// **************************************************************************
// * Copyright (C) 2016 David Thompson
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
// Beta 13	Updated the presets to Ran's suggestions and added a Tricopter preset.
//			Changed the preset screen to have an "Abort" option. Also flashes a warning.
//			pBuffer increased to 25 bytes.
// Beta 14	Menu wording and functionality updates from Ran.
//			Rudder volume reversed for Quad-X. 
//			Default preset now Quad-X. Added "Blank" preset to zero profile and mixer settings.
// Beta 15	Removed flashing warning to tighten up menu response.
//			Updated calibrate logic to be more robust.
//			Updated the menu defaults to match the new actual defaults. 
//			Updated defaults as per Ran's input.
// Beta 16	Tweaked the PWM burst generation to be more consistent.
//			Fixed mixer bypass logic to speed up calculation.
//			Fixed eeprom upgrade bugs for MPU6050LPF and stick polarities
//			Fixed SCALED polarity errors - oops
//
// V1.2	Based on OpenAeroVTOL V1.1 code.
//
// Beta 1	Changed BIND logic to hopefully remove unbinding issues.
//			Tightened up stick polarity testing.
//			Added persistent error log. Uncomment "ERROR_LOG" in compiledefs.h to use.
// Beta 2	Timeout for high-speed mode wait loop.
//			Motor behaviour on loss of signal changed - now goes to idle where possible.
//			Tweaks to optimise loop speed. Average gyro readings for P-terms.
//			Experimental D-terms for roll/pitch.
// Beta 3	Updated S.Bus and Spektrum data conversion factors.
//			Remove D-terms. Handles serial data errors properly.
//			Fixed unintentional bind mode on power-up for Satellite RXs.
//			Add binding modes for all DSM2/DSMX 1024/2048 receivers.
//			Button 1 = DSM2 1024/22ms, Button 2 = DSM2 2048/11ms,
//			Button 3 = DSMX 2048/22ms, Button 4 = DSMX 2048/11ms.
//			Fixed S.Bus2 only recognising 1 in 4 packets.
//			Experimental vibration test screen.
//			Display jumps immediately to IDLE once armed.
// Beta 4	Updated vibration display calculation with HPF.
// Beta 5	Fixed loss of control in RC Sync mode.
//			Vibration screen persistence reduced
// Beta 6	Changed vibration mode HPF to 50Hz.
// Beta 7	Added Multiplex channel order, SRLX/UDI Mode B serial input format
// Beta 8	Added all possible flat board orientations
//			Added automated board reorientation with transition (trial)
//			Updated eeprom upgrade process to use purely offsets for future sanity.
// Beta 9	Dual P1/P2 calibration (trial)
// Beta 10	Added advanced menu item for tail-sitter requirements (P1 orientation, P1 reference)
// Beta 11	Final update for eeprom upgrade architecture. Note that only V.0->V1.1 
//			and V1.1 to the current Beta are supported. Settings menu bug fixed.
//			gyroADCalt[] created to correctly support the IMU.
// Beta 12	Fixed gyro calibrate issues.
// Beta 13	Fixed accVert signal handling. Added AccVert to sensor display.
//			Increase strength of AL Correct by 4. 
// Beta 14	Change AL correct range of numbers. Range is currently 1-10 default 6. 
//			Text/message typology clean-up. Dynamic references to Yaw AL, AccYaw etc. 
//			Removed Advanced menu. P1 orientation now implicit from P2 via look-up table.
//			Added auto-update from V1.1 B13. 
//			Reset IMU when leaving P1 or P2 for dual orientation modes.
// Beta 15	Coding rationalisation. Text updates and fixes.
//			Automate AL Correct conversion when updating.
//			Updated sensor display screen to include IMU output.
//			Fixes from Ran's testing of Beta 14.
// Beta 16	Suppressed debug log. Removed IMU reset on ARM as it causes a twitch with
//			dual-orientation modes.
// Beta 17	Reversed AL Correct sense. Scale now 1 to 10.
//			Small tweaks to state machine to try and remove arming glitch.
// Beta 18	Made SRXL/UDI a compile option until verified.
//			AL Correct now 2-11 (seconds).
// Beta 19	New attempt at removing PWM glitch on arming. (V1.2 Release candidate)
//			Update settings update so that all older orientations are converted etc.
//
// V1.3		Based on OpenAeroVTOL V1.2 code.
//
// Alpha 1	Added XBUS Mode.B/UDI code. Fixed packet_size bug. 
//			Made checksum checking a compile option. Working on curve code.
// Alpha 2	Cleaned up curves and changed to seven-points.
// Alpha 3	Added another generic curve. Extended universal mixers.
//			Generic curves now have a larger range of selectable inputs.
// Beta 1	Curves now functional. Added settings update from V1.2
//			I/O screen added.
// Beta 2	Tidied up IO screen. Fixes from Ran.
// Beta 3	More input from Ran. Fixing Throttle curve not working.
// Beta 4	Optimised throttle curve mixing. Throttle cut shown on
//			I/O display. Display changed to percentages.
//			Sensors now affect results.
// Beta 5	Fix travel percentage screw-up. Test limits after expansion for servos.
//			Fix missing THR input for curves. Fix for curve issues after selecting NONE.
// Beta 6	AL correct default corrected to 6. Feedback from Ran.
// Beta 7	I/O display input ranges changed to show "100" for "1000" on the RC inputs display
// Beta 8	Added bipolar throttle to selectable universal inputs.
//			Adjusted eeprom upgrade to suit.
// Beta 9	Adjusted eeprom upgrade from V1.2 such that new data is correctly set to defaults.
//			Changed I/O screen format and added transition number. Throttle input is now monopolar
//			throttle scaled 0-100.
// Beta 10	Tweaks to the I/O display.
// Beta 11	More tweaks to the I/O display.
// Beta 12	Correction to V1.2 - V1.3 eeprom update code. Added custom channel order selection.
//			Combine Mode.B/UDI code from Rene_6 branch. 
// Beta 13	Move I/O display to the last position, and combine Mode.B/UDI code 
//			from Rene_11 branch. 
// Beta 14	Added variable out and in-bound transition speeds.
//			Added settings update from V1.3 B13.
// Beta 15	Increased maximum I-rates to 6. Added seven-point offset curves and menu.
//			Added complicated settings update from V1.3 B14. Added settable transition 
//			low and high points. Altered timed transition code to cope with P1, P1.n and P2 
//			settings that are not strictly in the order of P1 < P1.n < P2.
// Beta 16	Rearranged main menu items. Fixed broken orientation menu item.
// Beta 17	FC no longer disarms on loss of signal. S.Bus frame loss bit ignored.
//			Increased scale of roll and pitch trims by 10. Add "Alt. Damp" to universal mixers.
//			New failsafe functionality that continues pulses with signal loss. 
//			Rate matches previous rate. "No signal" trigger dropped from 500ms to 50ms.
//			Motors now correctly set to 1.1ms on loss of signal. 
//			Removed option of disabling CRC check on UDI mode. Updated settings migration.
// Beta 18	Changed universal input AccRoll and AccPitch to Acc X/Z and Acc Y.
//			Fixed I-term rate at throttle idle. Fixed Preset function.
//			Fixed vibration display killing outputs.
// Beta 19	Added bipolar throttle input to generic curves C and D
//			Increase stick rate maximums to 7. Fixed Alt. Damp text in curve inputs.
//			B19 is Release V1.3.
//
// V1.4		Based on OpenAeroVTOL V1.3 code.
//
// Beta 1	Add integrated Z-axis for height damping (vertical velocity damping).
//			Add Z I-term and I-limit, HPF for Z-axis damping. Test only.
// Beta 2	Fixed B17-B19 update detection bug. Zacc I-terms now functional.
//			Added user-selectable switch for Z-Acc LPF.
// Beta 3	Added Ran's 10s decimator. Sped up Z-acc filter.
//			Removed Ran's 10s decimator. Added adjustable % decimator.
// Beta 4	Decimator now correctly in 1/100ths of a percent.
// Beta 5	Added buzzer active/muted toggle if user holds buttons 3 and 4 on power up. 
// Beta 6	Added EEPROM settings handling update for GUI integration.
// Beta 7	Updates to default settings for presets and default buttons.
// Beta 8	Add EEPROM update from V1.3 and V1.4B6. Note that the typedefs offsets were wrong - updated.
//			Added buzzer control to menus.
//			Beta 8 is Release V1.4
//
// V1.5		Based on OpenAeroVTOL V1.4 code.
//
// Beta 1	Add HoTT SUMD serial protocol first attempt.
// Beta 2	Corrected Status screen RX type text.
//			Hopefully fixed SUMD channel offset and freeze bugs.
// Beta 3	Fix for channel order bugs
// Beta 4	Incorporate EEPROM version changes. 
//			SUMD confirmed fully functional
//			Updated firmware signature.
//			Fix for DSMX reception format bug.
// Beta 5	Add experimental 1.0ms pulse burst on all outputs for 0.5s on power-up
// Beta 6	Tweaks to above
// Beta 7	Fixed ServoFlag bug	
// Beta 8	Updated WDT handling. Added BOD handling (experimental)
//			Fixed BORF detection in isolation. Fixed BORF power-up sequence.
//			Manual disarm doesn't update display in real time to "(Disarmed)" - FIXED
// Beta 9	Removed error log.
// Beta 10	Removed BOD and POR code, fixed arming hang bug.
// Beta 11	Improved compatibility with those picky ESCs.
// Beta 12	Re-arranged init to solve power-up issues
// Beta 13	Changed back to the old bind timing method to solve power-up issues
//			Beta 13 is Release V1.5
//
//***********************************************************
//* Notes
//***********************************************************
//
// Bugs:	
//			
// To do:	
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

//#define	RC_OVERDUE 9765				// Number of T2 cycles before RC will be overdue = 9765 * 1/19531 = 500ms
#define	RC_OVERDUE 977				// Number of T2 cycles before RC will be overdue = 977 * 1/19531 = 50ms

#define	SLOW_RC_RATE 41667			// Slowest RC rate tolerable for Plan A syncing = 2500000/60 = 16ms
#define	SERVO_RATE_LOW 300			// A.servo rate. 19531/65(Hz) = 300
#define	SERVO_RATE_HIGH 217			// A.servo rate. 19531/90(Hz) = 217
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
#define FASTSYNCLIMIT 293			// Max time from end of PWM to next interrupt (15ms)

//***********************************************************
//* Code and Data variables
//***********************************************************

// Flight variables
int16_t transition_counter = 0;
uint8_t Transition_state = TRANS_P1;
uint8_t Old_transition_state = TRANS_P1;
int16_t	transition = 0; 
int16_t	old_transition = 0; 

// Flags
volatile uint8_t	General_error = 0;
volatile uint8_t	Flight_flags = 0;
volatile uint8_t	Alarm_flags = 0;

// Global buffers
char pBuffer[PBUFFER_SIZE];			// Print buffer (25 bytes)

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
volatile uint8_t	Servo_TCNT2 = 0;
volatile uint16_t	RC_Timeout = 0;

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
	bool FastServoTick = false;
	bool ResampleRCRate = false;
	bool PWMOverride = false;
	bool Interrupted_Clone = false;
	bool SlowRC = true;
	bool LastLoopOverdue = false;

	// 32-bit timers
	uint32_t Arm_timer = 0;
	uint32_t RC_Rate_Timer = 0;
	uint32_t PWM_interval = PWM_PERIOD_WORST;	// Loop period when generating PWM. Initialise with worst case until updated.
	
	// 16-bit timers
	uint16_t Status_timeout = 0;
	uint16_t UpdateStatus_timer = 0;
	uint16_t Ticker_Count = 0;
	//uint16_t RC_Timeout = 0;
	uint16_t Servo_Rate = 0;
	uint16_t FastServo_Rate = 0;
	uint16_t Transition_timeout = 0;
	uint16_t Disarm_timer = 0;
	uint16_t Save_TCNT1 = 0;
	uint16_t ticker_16 = 0;
	uint16_t fast_sync_timer = 0;

	// Timer incrementers
	uint16_t RC_Rate_TCNT1 = 0;
	uint8_t Transition_TCNT2 = 0;
	uint8_t Status_TCNT2 = 0;
	uint8_t Refresh_TCNT2 = 0;
	uint8_t Disarm_TCNT2 = 0;
	uint8_t Arm_TCNT2 = 0;
	uint8_t Ticker_TCNT2 = 0;
	//uint8_t Servo_TCNT2 = 0;
	uint8_t ServoRate_TCNT2 = 0;
	uint8_t fast_sync_TCNT2 = 0;

	// Locals
	uint16_t InterruptCounter = 0;
	uint8_t	Disarm_seconds = 0;
	uint8_t Status_seconds = 0;
	uint8_t Menu_mode = STATUS_TIMEOUT;
	int8_t	old_flight = 3;			// Old flight profile
	int8_t	old_trans_mode = 0;		// Old transition mode
	uint16_t transition_time = 0;
	uint8_t	old_alarms = 0;
	uint8_t ServoFlag = 0;
	uint8_t i = 0;
	int16_t PWM_pulses = 3; 
	uint32_t interval = 0;			// IMU interval
	uint8_t transition_direction = P2;
	uint16_t j;
	
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
				// If in vibration test mode, stay in Status
				if ((Status_seconds >= 10) && (Config.Vibration == OFF))
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

					// Unblock motors if blocked
					Flight_flags &= ~(1 << ARM_blocker);
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
				
					// Prevent PWM output going into STATUS_TIMEOUT
					PWMOverride = true;
					
					// Clear Interrupted_Clone
					Interrupted_Clone = false;
				}
				else
				{
					// Prevent PWM output until at least when next RC arrives
					PWMOverride = true;
				}
				
				break;

			// In STATUS_TIMEOUT mode, the idle screen is displayed and the mode 
			// changed to POSTSTATUS_TIMEOUT. 
			case STATUS_TIMEOUT:
				// Pop up the Idle screen
				idle_screen();
				
				// Make sure that these are cleared
				Interrupted = false;
				Interrupted_Clone = false;

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
														
					// Unblock motors if blocked
					Flight_flags &= ~(1 << ARM_blocker);
				}
				else
				{
					// Prevent PWM output until at least when next RC arrives
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
				Menu_mode = PRESTATUS;
				// Reset timeout once back in status screen
				Status_seconds = 0;
				// Reset IMU on return from menu
				reset_IMU();
				
				//***********************************************************
				// Experimental PWM output code
				//***********************************************************
					
				cli();									// Disable interrupts
				
				ServoFlag = 0;							// Reset servo flag

				for (i = 0; i < MAX_OUTPUTS; i++)		// For each output
				{
					// Check for motor marker
					if (Config.Channel[i].Motor_marker == MOTOR)
					{
						// Set output to 1ms pulse width
						ServoOut[i] = MOTORMIN;
			
						// Mark motor outputs
						ServoFlag |= (1 << i);
					}
				}

				for (j = 0; j < 249; j++)
				{
					// Pass address of ServoOut array and select only motor outputs
					output_servo_ppm_asm(&ServoOut[0], ServoFlag);
				}
			
				sei();									// Enable interrupts
					
				//***********************************************************			
					
				// Prevent PWM output
				PWMOverride = true;
								
				break;

			default:
				break;
		}

		//************************************************************
		//* Alarms
		//************************************************************

		// If RC signal is overdue, signal RX error message
		if (Overdue)
		{
			General_error |= (1 << NO_SIGNAL);		// Set NO_SIGNAL bit
		}
		// RC signal received normally
		else
		{
			General_error &= ~(1 << NO_SIGNAL);		// Clear NO_SIGNAL bit
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
			// Check buzzer mode first
			if (Config.Buzzer == ON)
			{
				LVA = 1;
			}
		}
		else 
		{
			LVA = 0;
		}

		//************************************************************
		//* Arm/disarm handling
		//************************************************************

		// All cases - reset arm timer
		if (Config.ArmMode == ARMABLE)
		{
			// Manual arm/disarm
			// If sticks not at extremes, reset manual arm/disarm timer
			// Sticks down and inside = armed. Down and outside = disarmed
			if (
				((-ARM_TIMER_RESET_1 < RCinputs[AILERON]) && (RCinputs[AILERON] < ARM_TIMER_RESET_1)) ||
				((-ARM_TIMER_RESET_1 < RCinputs[ELEVATOR]) && (RCinputs[ELEVATOR] < ARM_TIMER_RESET_1)) ||
				((-ARM_TIMER_RESET_1 < RCinputs[RUDDER]) && (RCinputs[RUDDER] < ARM_TIMER_RESET_1)) ||
				(ARM_TIMER_RESET_2 < MonopolarThrottle)
			   )
			{
				Arm_timer = 0;
			}
			
			// If disarmed, arm if sticks held
			if (General_error & (1 << DISARMED))
			{
				// Reset auto-disarm count
				Disarm_timer = 0;
				Disarm_seconds = 0;
								
				// If arm timer times out, the sticks must have been at extremes for ARM_TIMER seconds
				// If aileron is at min, arm the FC
				if ((Arm_timer > ARM_TIMER) && (RCinputs[AILERON] < -ARM_TIMER_RESET_1))
				{
					Arm_timer = 0;
					General_error &= ~(1 << DISARMED);	// Set flags to armed (negate disarmed)
					CalibrateGyrosSlow();					// Calibrate gyros (also saves to eeprom)
					LED1 = 1;								// Signal that FC is ready

					Flight_flags |= (1 << ARM_blocker);		// Block motors for a little while to remove arm glitch
					Servo_Rate = 0;

					//***********************************************************
					// Experimental PWM output code
					//***********************************************************
					
					cli();									// Disable interrupts

					ServoFlag = 0;							// Reset servo flag

					for (i = 0; i < MAX_OUTPUTS; i++)		// For each output
					{
						// Check for motor marker
						if (Config.Channel[i].Motor_marker == MOTOR)
						{
							// Set output to 1ms pulse width
							ServoOut[i] = MOTORMIN;
			
							// Mark motor outputs
							ServoFlag |= (1 << i);
						}
					}

					for (j = 0; j < 249; j++)
					{
						// Pass address of ServoOut array and select only motor outputs
						output_servo_ppm_asm(&ServoOut[0], ServoFlag);
					}

					sei();									// Enable interrupts
									
					//***********************************************************

					// Force Menu to IDLE immediately unless in vibration test mode
					if (Config.Vibration == OFF)
					{
						Menu_mode = PRESTATUS_TIMEOUT;		// Previously IDLE, which was wrong. 
					}
				}
			}
	
			// If armed, disarm if sticks held
			else 
			{
				// Disarm the FC after DISARM_TIMER seconds if aileron at max
				if ((Arm_timer > DISARM_TIMER) && (RCinputs[AILERON] > ARM_TIMER_RESET_1))
				{
					Arm_timer = 0;
					General_error |= (1 << DISARMED);	// Set flags to disarmed
					LED1 = 0;								// Signal that FC is now disarmed
					
					Flight_flags |= (1 << ARM_blocker);		// Block motors for a little while to remove arm glitch
					Servo_Rate = 0;
					
					// Force Menu to IDLE immediately unless in vibration test mode
					if (Config.Vibration == OFF)
					{
						Menu_mode = PRESTATUS_TIMEOUT;	
					}
#ifdef ERROR_LOG
add_log(MANUAL);
#endif			
				}

				// Automatic disarm
				// Reset auto-disarm count if any RX activity or set to zero
				if ((Flight_flags & (1 << RxActivity)) || (Config.Disarm_timer == 0))
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
					General_error |= (1 << DISARMED);	// Set flags to disarmed
					LED1 = 0;								// Signal that FC is now disarmed
#ifdef ERROR_LOG
add_log(TIMER);
#endif
				}
			}
		} // if (Config.ArmMode == ARMABLE)
		
		// Arm when ArmMode is OFF
		else 
		{
			// If disarmed, arm
			if (General_error & (1 << DISARMED))
			{
				General_error &= ~(1 << DISARMED);			// Set flags to armed
			}
			
			LED1 = 1;
		}

		//************************************************************
		//* Get RC data
		//************************************************************

		// Update zeroed RC channel data
		RxGetChannels();

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

		if ((old_flight == 3) || (old_trans_mode != Config.TransitionSpeedOut))
		{
			switch(Config.FlightSel)
			{
				case 0:
					Transition_state = TRANS_P1;
					transition_counter = Config.Transition_P1;
					break;
				case 1:
					Transition_state = TRANS_P1n;
					transition_counter = Config.Transition_P1n; // Set transition point to the user-selected point
					break;
				case 2:
					Transition_state = TRANS_P2;
					transition_counter = Config.Transition_P2;
					break;
				default:
					break;
			}		 
			old_flight = Config.FlightSel;
			old_trans_mode = Config.TransitionSpeedOut;
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
		if (Config.TransitionSpeedOut == 0)
		{
			// Update the transition variable based on the selected RC channel
			UpdateTransition();
		}
		else
		{
			// transition_counter counts from 0 to 100 (101 steps)
			transition = transition_counter;
		}

		// Always in the TRANSITIONING state when Config.TransitionSpeed is 0
		// This prevents state changes when controlled by a channel
		if (Config.TransitionSpeedOut == 0)
		{
			Transition_state = TRANSITIONING;
		}

		// Update transition state change when control value or flight mode changes
		if (TransitionUpdated)
		{
			// Update transition state from matrix
			Transition_state = (uint8_t)pgm_read_byte(&Trans_Matrix[Config.FlightSel][old_flight]);
		}

		// Calculate transition time from user's setting based on the direction of travel
		if (transition_direction == P2)
		{
			transition_time = TRANSITION_TIMER * Config.TransitionSpeedOut; // Outbound transition speed	
		}
		else 
		{
			transition_time = TRANSITION_TIMER * Config.TransitionSpeedIn; // Inbound transition speed		
		}
		
		// Update state, values and transition_counter every Config.TransitionSpeed if not zero.
		if (((Config.TransitionSpeedOut != 0) && (Transition_timeout > transition_time)) ||
			// Update immediately
			TransitionUpdated)
		{
			Transition_timeout = 0;
			TransitionUpdated = false;

			// Fixed, end-point states
			if (Transition_state == TRANS_P1)
			{
				transition_counter = Config.Transition_P1;
			}
			else if (Transition_state == TRANS_P1n)
			{
				transition_counter = Config.Transition_P1n;
			}
			else if (Transition_state == TRANS_P2)
			{
				transition_counter = Config.Transition_P2;
			}		

			// Over-ride users requesting silly states
			// If transition_counter is above P1.n but request is P1 to P1.n or 
			// if transition_counter is below P1.n but request is P2 to P1.n...
/*			if ((Transition_state == TRANS_P1_to_P1n_start) && (transition_counter > Config.Transition_P1n))
			{
				// Reset state to a more appropriate one
				Transition_state = TRANS_P2_to_P1n_start;
			}

			if ((Transition_state == TRANS_P2_to_P1n_start) && (transition_counter < Config.Transition_P1n))
			{
				// Reset state to a more appropriate one
				Transition_state = TRANS_P1_to_P1n_start;
			}
*/
			// Handle timed transition towards P1
			if ((Transition_state == TRANS_P1n_to_P1_start) || (Transition_state == TRANS_P2_to_P1_start))
			{
				if (transition_counter > Config.Transition_P1)
				{
					transition_counter--;
					
					// Check end point
					if (transition_counter <= Config.Transition_P1)
					{
						transition_counter = Config.Transition_P1;
						Transition_state = TRANS_P1;
					}
				}
				else
				{
					transition_counter++;
			
					// Check end point
					if (transition_counter >= Config.Transition_P1)
					{
						transition_counter = Config.Transition_P1;
						Transition_state = TRANS_P1;
					}
				}
				
				transition_direction = P1;
			}

			// Handle timed transition between P1 and P1.n
			if (Transition_state == TRANS_P1_to_P1n_start)
			{
				if (transition_counter > Config.Transition_P1n)
				{
					transition_counter--;
					
					// Check end point
					if (transition_counter <= Config.Transition_P1n)
					{
						transition_counter = Config.Transition_P1n;
						Transition_state = TRANS_P1n;
					}
				}
				else
				{
					transition_counter++;
					
					// Check end point
					if (transition_counter >= Config.Transition_P1n)
					{
						transition_counter = Config.Transition_P1n;
						Transition_state = TRANS_P1n;
					}
				}

				transition_direction = P2;
			}			
				
			// Handle timed transition between P2 and P1.n
			if (Transition_state == TRANS_P2_to_P1n_start)
			{
				if (transition_counter > Config.Transition_P1n)
				{
					transition_counter--;
					
					// Check end point
					if (transition_counter <= Config.Transition_P1n)
					{
						transition_counter = Config.Transition_P1n;
						Transition_state = TRANS_P1n;
					}
				}
				else
				{
					transition_counter++;
					
					// Check end point
					if (transition_counter >= Config.Transition_P1n)
					{
						transition_counter = Config.Transition_P1n;
						Transition_state = TRANS_P1n;
					}
				}

				transition_direction = P1;
			}

			// Handle timed transition towards P2
			if ((Transition_state == TRANS_P1n_to_P2_start) || (Transition_state == TRANS_P1_to_P2_start))
			{
				if (transition_counter > Config.Transition_P2)
				{
					transition_counter--;
					
					// Check end point
					if (transition_counter <= Config.Transition_P2)
					{
						transition_counter = Config.Transition_P2;
						Transition_state = TRANS_P2;
					}
				}
				else
				{
					transition_counter++;
					
					// Check end point
					if (transition_counter >= Config.Transition_P2)
					{
						transition_counter = Config.Transition_P2;
						Transition_state = TRANS_P2;
					}
				}

				transition_direction = P2;
			}

		} // Update transition_counter

		// Zero the I-terms of the opposite state so as to ensure a bump-less transition
		if ((Transition_state == TRANS_P1) || (transition == Config.Transition_P1))
		{
			// Clear P2 I-term while fully in P1
			memset(&IntegralGyro[P2][ROLL], 0, sizeof(int32_t) * NUMBEROFAXIS);
			IntegralAccVertf[P2] = 0.0;
		}
		else if ((Transition_state == TRANS_P2) || (transition == Config.Transition_P2))
		{
			// Clear P1 I-term while fully in P2
			memset(&IntegralGyro[P1][ROLL], 0, sizeof(int32_t) * NUMBEROFAXIS);
			IntegralAccVertf[P1] = 0.0;
		}
		
		//**********************************************************************
		//* Reset the IMU when using two orientations and just leaving P1 or P2
		//**********************************************************************
		
		if (Config.P1_Reference != NO_ORIENT)
		{
			// If Config.FlightSel has changed (switch based) and TransitionSpeed not set to zero, the transition state will change.
			if ((Config.TransitionSpeedOut != 0) && (Transition_state != Old_transition_state) && ((Old_transition_state == TRANS_P1) || (Old_transition_state == TRANS_P2)))
			{
				reset_IMU();
			}
			
			// If TransitionSpeed = 0, the state is always TRANSITIONING so we can't use the old/new state changes.
			// If user is using a knob or TX-slowed switch, TransitionSpeed will be 0.
			else if (
						(Config.TransitionSpeedOut == 0) &&														// Manual transition mode and...
						(((old_transition == Config.Transition_P1) && (transition > Config.Transition_P1)) ||	// Was in P1 or P2
						((old_transition == Config.Transition_P2) && (transition < Config.Transition_P2)))		// Is not somewhere in-between.
					)
			{
				reset_IMU();
			}
		}
		
		// Save current flight mode
		old_flight = Config.FlightSel;
		
		// Save old transtion state;
		Old_transition_state = Transition_state;
		
		// Save last transition value
		old_transition = transition;
				
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
		FastServo_Rate += (uint8_t)(TCNT2 - ServoRate_TCNT2);
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
		
		if (FastServo_Rate > SERVO_RATE_HIGH)
		{
			FastServoTick = true;	// Slow device is ready for output generation
			FastServo_Rate = 0;
		}
		
		//************************************************************
		//* Measure incoming RC rate and flag no signal
		//************************************************************

		// Check to see if the RC input is overdue (50ms)
		if (RC_Timeout > RC_OVERDUE)
		{
#ifdef ERROR_LOG
			// Log the no signal event if previously NOT overdue, armable and armed
			// This makes sure we only get one log per event
			if ((!Overdue) && (Config.ArmMode == ARMABLE) && ((General_error & (1 << DISARMED)) == 0))
			{
				add_log(NOSIGNAL);
			}
#endif			
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

		// Check for throttle reset
		if (MonopolarThrottle < THROTTLEIDLE)  // THROTTLEIDLE = 50
		{
			// Clear throttle high error
			General_error &= ~(1 << THROTTLE_HIGH);

			// Reset I-terms at throttle cut. Using memset saves code space
			memset(&IntegralGyro[P1][ROLL], 0, sizeof(int32_t) * 6);
			IntegralAccVertf[P1] = 0.0;
			IntegralAccVertf[P2] = 0.0;
		}	
	
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
				((Config.Servo_rate == FAST) && (!PWMBlocked)) ||			// Run at full loop rate if allowed
				(Overdue)													// On loss of RX signal
			)
		{

			//******************************************************************
			//* The following code runs once when PWM generation is desired
			//* The execution rates are:
			//* The RC rate unless in FAST mode
			//* High speed (loop rate) in FAST mode
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
						// Frame rates regardless of signal presence/absence
						(
						((Config.Servo_rate == FAST) && (Config.Channel[i].Motor_marker == ASERVO) && ServoTick) ||					// At ServoTick for A.Servo in FAST mode
						((Config.Servo_rate == FAST) && (Config.Channel[i].Motor_marker > ASERVO))									// Always for D.Servo and Motor in FAST modes when signal present
						)
						
						||
						
						// Frame rates when signal present
						(
						(!Overdue) &&
						(
						((Config.Servo_rate == SYNC) && (Config.Channel[i].Motor_marker == ASERVO) && (!SlowRC) && ServoTick) ||	// At ServoTick for A.Servo in SYNC with Fast RC
						((Config.Servo_rate == SYNC) && (Config.Channel[i].Motor_marker == ASERVO) && (SlowRC)) ||					// At RC rate for A.Servo with slow RC when signal present
						((Config.Servo_rate == SYNC) && (Config.Channel[i].Motor_marker > ASERVO)) ||								// At RC rate for D.Servo and Motor when signal present

						((Config.Servo_rate == LOW) && (!SlowRC) && ServoTick) ||													// All outputs at ServoTick in LOW mode with fast RC
						((Config.Servo_rate == LOW) && (SlowRC))																	// All outputs at  RC rate in LOW mode with slow RC when signal present
						)
						)
						
						||
						
						// Rates when no signal
						(
						(Overdue) &&
						(
						((Config.Servo_rate == SYNC) && (Config.Channel[i].Motor_marker == ASERVO) && ServoTick) ||					// At ServoTick for A.Servo when no signal present
						((Config.Servo_rate == SYNC) && (Config.Channel[i].Motor_marker > ASERVO) && (!SlowRC) && FastServoTick) ||	// At FastServoTick for A.Servo in SYNC with Fast RC
						((Config.Servo_rate == SYNC) && (Config.Channel[i].Motor_marker > ASERVO) && (SlowRC) && ServoTick) ||		// At ServoTick for A.Servo in SYNC with slow RC
	
						((Config.Servo_rate == LOW) && ServoTick)																	// All outputs at ServoTick in LOW mode
						)
						)
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

			if (FastServoTick)
			{
				FastServoTick = false;
				
				// Reset the Servo rate counter here so that it doesn't force an unusually small gap next time
				FastServo_Rate = 0;
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
			UpdateServos();							// Transfer Config.Channel[i].value data to ServoOut[i] and check servo limits. 
													// Note that values are now at system levels (were centered around zero, now centered around 3750).				

			// Set motors to idle on loss of signal.
			// Output LOW pulse (1.1ms) for each output that is set to MOTOR
			if (Overdue)
			{
				for (i = 0; i < MAX_OUTPUTS; i++)
				{
					// Check for motor marker
					if (Config.Channel[i].Motor_marker == MOTOR)
					{
						// Set output to motor idle pulse width
						ServoOut[i] = MOTOR_0_SYSTEM;
					}
				}
			}
		
			// Note: This is probably pointless as it's too late to save the PWM just mangled
			// Has overdue become false this loop?
			if (LastLoopOverdue && !Overdue)
			{
				Flight_flags |= (1 << ARM_blocker);		// Block motors for a little while to remove glitch
			}
		
			// Save Overdue status of current loop
			LastLoopOverdue = Overdue;
			
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
			
			LoopCount = 0;						// Reset loop counter for averaging sensors
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
		
	} // while loop
} // main()

