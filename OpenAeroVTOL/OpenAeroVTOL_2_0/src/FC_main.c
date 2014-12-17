//**************************************************************************
// OpenAero VTOL software for KK2.0 & KK2.1
// ========================================
// Version: V1.0 Release - December 2014
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
// VTOL 	Based on OpenAero2 V1.2 Beta 8 code
// Alpha 1	Added transition code
//			Added basic height dampening code (untested)
// Alpha 2	Fixed bug where the output mixer switcher channel was screwing up the gyro signal distribution.
// Alpha 3	Fixed all the missing "const" that annoy some compilers. 
// Alpha 4	Fixed partial transition and offset bugs. Also fixed case where power up state is wrong. 
// Beta 0	Changed flying wing mixing to the output mixers.
//			Removed Source Mix setting that confused everyone. Also removed RC input Source B mixing
//			as this is best done in the output mixers now. Dynamic gain effect reversed. Now max input
//			is maximum stability. Decoupled stick and gyro for P gain.
//			Factory reset now enterable by pressing just the middle two buttons. 
//			Removed output switcher and added per-output offset.
//			Added the ability to mix any RC source into the outputs.
// Beta 1 	Added arming
// Beta 2	Reversed arming and reduced the stick extremes required.
//			Changed default to ARMED. Fixed up some defaults.
//			Fixed OUT1 first input mixer bug.
// Beta 3 	Merged Channel and Output mixers. Removed presets.
//			Transition is default mode. Removed Profile 3 setting.
// Beta 4 	Completely changed mixers, removed unwanted features.
//			Now customised for transitioning. 
// Beta 5	Memory reduction via sensor switches
//			Expanded transition to all eight channels.	
//			Updated status display to show more transition info.
//			Output mixer menus for OUT1 to OUT8 now contain both P1 and P2 configuration.
// Beta 6	Added motor marking and new safety features for arming, trims, limits and low-throttle.
//			Fixed LVA setting and display. Removed failsafe functionality.
//			Fixed menu position behaviour when moving about main and sub menus.
//			Fixed flakey initial transition behaviour when changing transition settings
//			PID values for both profiles now calculated on the fly and only transitioned in the mixer.
//			Increased transition steps to 100 and interval to 10ms. I-terms reset at throttle cut.
//			Added experimental three-point offset handling. Removed redundant servo trim.
// Beta 7 	Re-added sensor reversing. Very code-size inefficient.
// Beta 8 	Changed display text for safety to "Armed", "Armable". Rearranged mixer items.
//			Rationalised version number. Changed motor marker to "Motor/Servo".
//			Changed servo rate to "Normal/fast". P1 to P2 transition point now hard-coded to 20% above center
//			Changed manual transition range to +/-100% (was closer to +/-125%). LMA time now "Disarm time".
//			LMA/Disarm timeout also switches to disarmed state if armable.
//			Height damping removed, Z-axis delta now included in mixer list.
//			Loop speed optimisations in the mixer. PWM sync logic simplified.
// Beta 9	Disarm timer now functions in 0 to 127 seconds and will auto-disarm if armable and not set to zero.
//			"No signal" state also disarms if set to "armable". Rearranged General menu. Fixed transition bug.
//			Disarming state sets all outputs marked as "Motor" to minimum output.
//			Acc Z Delta now working. Missing REV for P2 sensors restored.
// Beta 10	Added support for KK2.1 board. Logo displays for K2.1 boards.
//			Most tunable items changed fom 5 to 1 increment.
//			Added dedicated throttle sources to mixers with throttle curves.
//			Throttle now referenced from zero and has its own Config.ThrottleMinOffset for referencing. 
//			Rearranged mixer menus. Simplified includes.
// Beta 11	Fixed throttle high bug. Axis lock stick rate now calibrated to 10, 20, 40, 80, 160, 320 and 640 deg/sec.
//			Contrast recovered from saved setting correctly.
// Beta 12	Status menu timer fixed at 10 seconds. Fixed low battery visual message not showing.
//			Fixed SINE throttle curve reversed when P2 less than P1.
// Beta 13	Servo limits corrected to accurately move to the stated position.
//			Divides rounded correctly, not truncated. 3-point offset scale corrected.
//			Added dedicated aileron, elevator, rudder inputs to all mixer channels.
//			Sensor options changed to (OFF/ON/Scaled). Scaled sensors scale to their associated axis input volume.
//			Normal sources no longer have aileron, elevator, rudder options. Tidied up CPPM code.
//			Loop speed optimisations. Added basic mixer defaults for primary channels on OUT1, OUT5, OUT6, OUT7, OUT8 
//			Fixed the balance meter display for all orientations. Battery voltage calculation faster and more accurate.
//			Arm/disarm trigger readjusted to suite monopolar throttle and reduced to three seconds.
//			Disarm timer will ignore values below 30 seconds. 
// Beta 14	IMU update. Pitch response works the same as roll in autolevel when inverted.
//			Fixed balance meter response in KK2.1. Added P1n point concept to transition based on 3-point switch
//			P1.n trigger point changed to -20%.
// Beta 15	Fixed case where users can request invalid automated transition states.
//			Removed XPS Xtreme RX support to save space. 
// Beta 16	Fixed throttle trigger for arm/disarm. Fixed Config.Disarm_timer > 30 means "30" also ignored.
// Beta 17	Reinstated gyro D-terms. Changed arm time to 1 second, disarm still 3 seconds.
//			RX activity detection is now only valid if throttle below idle. This should save in-flight auto disarms.
//			Vbat now displays correctly for both KK2.0 and KK2.1. S.Bus timing tweaked.
//			RC noise threshold changed from +/-20 to +/-5. Improved throttle curve response.
// Beta 18	Fixed throttle minimum offset. Fixed slight rounding error when P2 throttle vol is 80% and P1 is 0%
// Beta 19	Throttle separated into Monopolar and Bipolar versions. Source list now includes Throttle, Aileron, 
//			Elevator and Rudder again as a result. Changed Acc delta Z to an Acc P function. 
//			Fixed acc calibration issues. Removed Free RAM display to save space.
//			Fixed Acc Z initial offset for KK2.1. Added SQRTSINE curve for KK2.1
//			Updated sine table data for 101 unique values.
// Beta 20	Reduced Acc-Z gain by 32 and improved noise response.
//			Fixed broken throttle high alarm
// Beta 21	Fixed I-term constraint calculation bug. Acc-Z code tweaked.
//			Stick input into I-term reversed for Yaw and Pitch.
// Beta 22	Fixed small bug where Acc LPF would not turn off when set to minimum.
//			Acc-Z now sourced from AccSmooth so is filtered according to ACC LPF
// Beta 23	Balance meter uses AccSmooth, so now shows an approximation of the Acc LPF. 
//			Going into any menu disarms the system. Min/Max travel and gyro limits now incrementable by 1.
// Beta 24	Changed lock rate back to Beta 15 values and reset range to accommodate.
// Beta 25	Tweaked transition switch trigger points. Better rounding in Acc LPF calculation.
//			Tidied up IMU multiplies to make sure there were no casting gotchas.
//			Doubled the strength of the Acc Z signal. Saved code space in the CPPM section thanks to Edgar.
//			Source C removed. Source A/B can now also select R/P/Y gyros and R/P accs.
//			Added Yaw gyro trim to flight profiles. 
// Beta 26	Added SQRTSINE curve for KK2.0 also. Source A/B accs now from accSmooth.
// Beta 27	Halved the accSmooth feedback. Added a per-channel 3-point transition curve.
//			Dynamic gain removed. Fixed broken D-term.
// Beta 28	Removed experimental 3-point transition curve. Removed the outputs as a selectable source.
//			Added some more presets to help new users. Made the profile state display more informative.
// Beta 29	Changed presets to cover OUT1 to 4 instead of 1, 5, 6, 7 and 8.
//			Acc trim functionality restored. Main menu rearranged. Profile items rearranged. 
//			Output mixer items rearranged.
// Beta 30	Renamed and rearranged offset items. 
//			Fixed broken Yaw trim that resulted in axis gyro data issues.
//			Increased yaw trim effectiveness to about +/-50% throw.
// Beta 31	Increased Acc LPF range to 127.
//			Gyro calibrate on initialisation waits for stability.
//			Arming now does a full recalibrate.
//			Add two experimental test settings - Acc Gate and Acc Slew
//			RX modes changed to CPPM, PWM, S.Bus and Satellite.
//			Add new menu item for selecting PWM sync source.
//			Now all five PWM inputs can be selectable as a sync source.
//			Screen contrast updated before logo - logo prettier.
//			Lock rate default changed to 3.
//			Magic jitter counter updated every second in PWM mode on Status screen.
// Beta 32	Fixed jitter meter. Experimenting with different gyro and acc sensitivity.
//			Currently 500 deg/sec and 4G fullscale.
//			Fixed MPU6050 setup codes. Now actually correct!
//			Added user-settable chip LPF setting for KK2.1 version.
// Beta 33	Optimised gyro slow cal starting point for each board
//			Added airspeed sensor display on sensor screen if AIRSPEED compile option set (KK2.1 only)
//			Contrast always set correctly on power-up.
//			Tweaked the G limits of the IMU to MultiWii 2.3 levels.
//			Changed the Scaled gyro response to Scaled x5 for all except Acc-Z.
//			Edge-clip bug fixed in fillcircle() so level meter works properly. 
//			Added PitchUp board orientation. CF factor goes down to 1.
//			Big change to IMU. Loop time is now calculated at the source and passed to getEstimatedAttitude().
//			The case where local G is outside 0.85 to 1.15 now returns angle numbers of the same size as within.
//			Fixed the over-writing of the inverted acc calibration on power-up.
// Beta 34	Moved roll/pitch trim to the PID loop as ineffective in mixer.
//			Renamed angle-based terms in menus to "Level".
//			Improved autolevel angle resolution from 1 degree to 0.01 degree.
//			MPU6050 LPF fixed at 5Hz.
//			Acc calibration values saved so that redoing normal acc does not reset the inv cal.
//			Normalised the whole sensor polarity system.
//			Simplified inverted behaviour of IMU
// Beta 35	Changes to inverted behaviour of IMU. Menu text tweaks. Ported to Studio 6 solution.
//			Made the default menu item values the same as the factory defaults in all cases.
//			Simplified submenu cursor position code and saved some RAM
// Beta 36	Final changes to IMU to correct glitch when rotating 360 degrees
// Beta 37	Compiler fix for KK2.1 builds
// Beta 38	Minimum CPPM pulse filter changed from 500us to 300us to cover FrSky 27ms firmware.
//			Added measures to stop false extra channel data breaking CPPM reception.
//			Now a "no signal" event resets the CPPM max channel number calculation.
// Beta 39	Fixed a CPPM mis-detection bug. Gyro full scale resolution changed to 2000 deg/s
//			Display wizard improved so that bad RX selection doesn't read as stick inputs.
//			Changed to a vector-based IMU from KK2 assembler code.
//			GLCD update speed improved.
// Beta 40	Type casting tweaks in IMU. Changed MPU6050 gyro/acc LPF default to 256Hz.
//			Re-added user adjustable LPF. 
// Beta 41	Fixed KK2.1 gyro handling so that the stick rate is unaffected.
//			Added compile option for Servo travel rescaled to 0.875~2.125 (150%) for an indicated 125%
//			Added ESC calibration routine.
// Beta 42	Expanded servo pulses only affect outputs marked as "Servo".
//			Trim/limit settings also now work with expanded pulse widths.
//			D-terms no longer multiplied by 16 as this results in too coarse behaviour.
//			Power up with no radio connected no longer causes menu entry and arming issues.
// Beta 43	Fixed min throttle bug. Fixed output_servo_ppm_asm3() so that it can operate up to 2.3ms
//			and so support the new wider pulse widths.
// Beta 44	Increased the maximum automatic transition time to 20 seconds.
//			Reset the IMU at init and gyro cal. Changed gyro slow calibration method. "Hold steady" on all boards now.
//			Board does a software reset if cal fails at startup.
// Beta 45	Errors now show on the Idle screen
//			Fixed broken slow gyro calibrate on KK2.0
// Beta 46	Individual lock rates for RPY in both profiles. Stick rate values reversed. Default is now 1.
//			More internal IMU regs added to reset. AccLPF values reversed. Default is now 120.
//			High-speed PWM output for S.Bus and Satellite modes. Removed D-terms again.
//			Basic bump-less I-term handling added. CF factor now "AL correct" and numbers reversed.
//			High-speed PWM now locked to incoming RC. Fixed broken slow gyro calibrate on KK2.0
// Beta 47	Gyro slow calibrate sped up for KK2.0 boards. Gyro cal on sensor screen changed to fast calibrate.
//			High speed mode architecture altered to fix timer disruption.
//			Checked that gyro behaviour same as B37 on KK2.0.
//			Added gyro data log build option for KK2.1. Increased loop rate by rebalancing the code.
// Beta 48	Removed high-speed mode as it is unlikely to work well for everyone. Added gyro-averaging code.
// Beta 49	Added Gyro LPF and menu item to replace gyro averaging code. This seems to work better than B48.
// Beta 50	Changed default MPU LPF to 21Hz instead of 5Hz. Now increments/decrements properly.
//			Changed SW LPFs to have fixed, text-based values -  5Hz, 10Hz, 21Hz, 32Hz, 44Hz, 74Hz, None.
//			Note that the 74Hz option is possibly not useful and may be removed later to save space.
//			Removed WIDE_PULSES build option as this is now the norm.
//			Added Acro mode. I-terms and AL reset when sticks move past 5%.
//			Added Progressive P-gain mode. Settable from 0 to 100%. At 50%, P gain reduced to zero at max stick.
//			At 100%, P gain reduced to zero at 50% stick.
// Beta 51  Changed LCD_Display_Text() and do_menu_item() to index more than 256 items
//			Added very complicated auto-trim feature to allow independent mechanical trims for P1 and P2
//			to be set in flight. I-term neutral stick position resets after two seconds of no stick input
//			if the Auto-trim feature is in use. S.Bus2 packets now supported for KK2.1+, thanks to OpenPilot code example.
//			Sped up EEPROM saves, removed LED flash. Tweaked menus to suit different feel. Code size improvements.
//			Auto-trim EEPROM save moved to throttle cut at end of flight. Dramatically reduced LCD startup time.
//			Improved code to stabilise FC when pressing calibrate button.
//			Move eeprom save to throttle cut.
// Beta 52	Now has "Quadcopter" compile option to assist rapid setup and testing.
//			Fixed MPU6050 filter setting being forgotten on power-up.
//			Restored missing logo. Sped up GLCD reset. Fixed orientation bugs.
//			Improved default AccZeroNormZ setting. Fixed throttle offset change when volumes both zero.
//			KK2 Mini compatibility tweaks. Change LCD initialisation and limit available contrast.
//			Added ADVANCED compile option to hide/expose the new features (Acro, Progressive P and Auto-trim)
// Beta 53  Maximum automated transition time now 40 seconds.
//			Factory defaults for lock rates now correctly set to 1.
// Beta 54	Revert to B46-era IMU. Read eeProm back into config structure.
//			Made text-based values increment more slowly. LED flashes to show successful acc calibrations
//			I-rates now default to 2. Checked that all defaults are the same as factory settings.
//			Fixed pbuffer overrun which caused text to be centered off-screen.
//			Removed advanced features to simplify debugging.
//			Wound back LCD_Display_Text() indexing to B46, ServoOut[] no longer volatile
// Beta 55	Bugfix release for IMU that could no longer handle tumbling.
//			Had to reinstate the use of TMR0 to count the over-runs of TMR1 when measuring the loop interval.
// Beta 55.1
//			Still buggy for weird AL on existing the menu. Trying new loop interval handling.
// Beta 55.2
//			Bug fixed. Minor tweaks. Limit the instantaneous change in angle of the IMU to +/-15 degrees.
//			Removed read-back of eeProm into config structure.
//			Updated the default eeprom settings. Scrimped a few bytes to squeeze it into the KK2.0.
// V1.0		Released.
//
//***********************************************************
//* Notes
//***********************************************************
//
// Bugs: Not any more... I hope.
//	
//
// Todo: Nada!
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

//***********************************************************
//* Fonts
//***********************************************************

#include "Font_Verdana.h" 			// 8 (text) and 14 (titles) points
#include "Font_WingdingsOE2.h"		// Cursor and markers

//***********************************************************
//* Defines
//***********************************************************

#define	RC_OVERDUE 9765				// Number of T2 cycles before RC will be overdue = 9765 * 1/19531 = 500ms
#define	SLOW_RC_RATE 41667			// Slowest RC rate tolerable for Plan A syncing = 2500000/60 = 41667
#define	SERVO_RATE_LOW 390			// Requested servo rate when in Normal mode. 19531 / 50(Hz) = 390
#define SECOND_TIMER 19531			// Unit of timing for seconds
#define ARM_TIMER_RESET_1 960		// RC position to reset timer for aileron, elevator and rudder
#define ARM_TIMER_RESET_2 50		// RC position to reset timer for throttle
#define TRANSITION_TIMER 195		// Transition timer units (10ms * 100) (1 to 10 = 1s to 10s)
#define ARM_TIMER 19531				// Amount of time the sticks must be held to trigger arm. Currently one second.
#define DISARM_TIMER 58593			// Amount of time the sticks must be held to trigger disarm. Currently three seconds.

//***********************************************************
//* Code and Data variables
//***********************************************************

// Flight variables
uint32_t interval;					// IMU interval
int16_t transition_counter = 0;
uint8_t Transition_state = TRANS_P1;
int16_t	transition = 0; 

// Flags
uint8_t	General_error = 0;
uint8_t	Flight_flags = 0;
uint8_t	Alarm_flags = 0;
uint8_t	old_alarms = 0;

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
uint16_t InterruptCount = 0;
uint16_t LoopStartTCNT1 = 0;
bool Overdue = false;
uint8_t	LoopCount = 0;
	
//************************************************************
//* Main loop
//************************************************************

int main(void)
{
	bool ServoTick = false;
	bool SlowRC = false;
	bool TransitionUpdated = false;

	// 32-bit timers
	uint32_t Arm_timer = 0;
	uint32_t RC_Rate_Timer = 0;

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
					
					// Enable Timer0 interrupts as loop rate is slow
					// and we need TMR0 to fully measure it.
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
				// Force resync on next RC packet
				Interrupted = false;	
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
				if (Status_seconds >= 10)
				{
					Menu_mode = STATUS_TIMEOUT;
				}

				// Jump to menu if button pressed
				else if(BUTTON1 == 0)
				{
					Menu_mode = MENU;
					menu_beep(1);
					// Force resync on next RC packet
					Interrupted = false;
				}

				// Update status screen while waiting to time out
				else if (UpdateStatus_timer > (SECOND_TIMER >> 2))
				{
					Menu_mode = STATUS;
				}
				break;

			// In STATUS_TIMEOUT mode, the idle screen is displayed and the mode changed to IDLE
			case STATUS_TIMEOUT:
				// Pop up the Idle screen
				idle_screen();
				// Force resync on next RC packet
				Interrupted = false;
				// Switch to IDLE mode
				Menu_mode = IDLE;
				break;

			// In MENU mode, 
			case MENU:
				LVA = 0;	// Make sure buzzer is off :)
				// Disarm the FC
				General_error |= (1 << DISARMED);
				// Start the menu system
				menu_main();
				
				// Force resync on next RC packet
				Interrupted = false;
				// Switch back to status screen when leaving menu
				Menu_mode = STATUS;
				// Reset timeout once back in status screen
				Status_seconds = 0;
				// Reset IMU on return from menu
				reset_IMU();
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

			// Display the interrupt count each second
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

		// If RC signals overdue, signal RX error message and disarm
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

		// Update state, values and transition_counter every Config.TransitionSpeed if not zero. 195 = 10ms
		if (((Config.TransitionSpeed != 0) && (Transition_timeout > (TRANSITION_TIMER * Config.TransitionSpeed))) ||
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
		RC_Rate_Timer += (Save_TCNT1 - RC_Rate_TCNT1);
		RC_Rate_TCNT1 = Save_TCNT1;

		// Arm timer for timing stick hold
		Arm_timer += (uint8_t) (TCNT2 - Arm_TCNT2); 
		Arm_TCNT2 = TCNT2;

		// 16-bit timers (Max. 3.35s measurement on T2)
		// All TCNT2 timers increment at 19.531 kHz

		// Sets the desired SERVO_RATE by flagging ServoTick when PWM due
		Servo_Rate += (uint8_t) (TCNT2 - ServoRate_TCNT2);
		ServoRate_TCNT2 = TCNT2;
		
		// Signal RC overdue after RC_OVERDUE time (500ms)
		RC_Timeout += (uint8_t) (TCNT2 - Servo_TCNT2);
		Servo_TCNT2 = TCNT2;
		
		// Update transition timer
		Transition_timeout += (uint8_t) (TCNT2 - Transition_TCNT2);
		Transition_TCNT2 = TCNT2;

		// Update status timeout
		Status_timeout += (uint8_t) (TCNT2 - Status_TCNT2);
		Status_TCNT2 = TCNT2;
		
		// Status refresh timer
		UpdateStatus_timer += (uint8_t) (TCNT2 - Refresh_TCNT2);
		Refresh_TCNT2 = TCNT2;

		// Auto-disarm timer
		Disarm_timer += (uint8_t) (TCNT2 - Disarm_TCNT2);
		Disarm_TCNT2 = TCNT2;

		// Timer for audible alarms
		Ticker_Count += (uint8_t) (TCNT2 - Ticker_TCNT2);
		Ticker_TCNT2 = TCNT2;

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
		//* Measure incoming RC rate and flag no signal
		//************************************************************

		// Check to see if the RC input is overdue (500ms)
		if (RC_Timeout > RC_OVERDUE)
		{
			Overdue = true;	// This results in a "No Signal" error
		}

		if (Interrupted)
		{
			RC_Timeout = 0;					// Reset RC timeout
			Overdue = false;				// No longer overdue			
			
			// Measure incoming RC rate. Threshold is 60Hz.
			if (RC_Rate_Timer > SLOW_RC_RATE)
			{
				SlowRC = true;
			}
			else
			{
				SlowRC = false;
			}

			RC_Rate_Timer = 0;
		}

		//************************************************************
		//* Manage desired output update rate when limited by
		//* the PWM rate set to "Low"
		//************************************************************

		// Flag update required based on SERVO_RATE_LOW (50Hz)
		if (Servo_Rate > SERVO_RATE_LOW)
		{
			ServoTick = true; // Slow device is ready for output generation
			Servo_Rate = 0;
		}
					
		//************************************************************
		//* Output PWM to ESCs/Servos where required, 
		//* based on a very specific set of conditions
		//************************************************************

		// Cases where we are ready to output
		if	(
				// Interrupted and LOW or SYNC
				Interrupted &&												// Only when interrupted (RC receive completed)
				(
					(SlowRC && (Config.Servo_rate == LOW)) || 				// Plan A (Run as fast as the incoming RC if slow RC detected with LOW selected)
					(ServoTick && !SlowRC && (Config.Servo_rate == LOW)) ||	// Plan B (Run no faster than the preset rate (ServoTick) if fast RC detected with LOW selected)
					(Config.Servo_rate >= SYNC)								// Plan C (Run as fast as the incoming RC if in SYNC or MAX modes)
				)
			)
		{
			Interrupted = false;				// Reset interrupted flag
			ServoTick = false;					// Reset output requested flag
			Servo_Rate = 0;						// Reset servo rate timer
			
			Calculate_PID();					// Calculate PID values
			ProcessMixer();						// Do all the mixer tasks - can be very slow
			UpdateServos();						// Transfer Config.Channel[i].value data to ServoOut[i] and check servo limits
			output_servo_ppm();					// Output servo signal
			
			LoopCount = 0;						// Reset loop counter for averaging accVert
		}

		// Not ready for output, so cancel the current interrupt and wait for the next one
		else
		{
			Interrupted = false;				// Reset interrupted flag
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

