// **************************************************************************
// OpenAero software for KK2.0
// ===========================
// Version 2.00 Alpha 4
// Inspired by KKmulticopter
// Contains trace elements of assembly code by Rolf R Bakke, and C code by Mike Barton
// OpenAero code by David Thompson, included open-source code as per quoted references
// Includes PID and Auto-level functions inspired by the open-sourced MultiWii project
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
// * along with this program.If not, see <http://www.gnu.org/licenses/>.
// * 
// * NB: Summary - all derivative code MUST be released with the source code!
// *
// **************************************************************************
// Version History
// ===============
// V2.00a	Based on OpenAero V1.13 Beta 8 code
//			Initial code base.
// Alpha 1	First release to alpha testers
// Alpha 2  Fixed channel number bug, added second aileron to default mixer
// Alpha 3	Enabled autolevel and acc settings in mixer menus, gyro reversing works.
//			Contrast updates visually during modification. Status menu updated.
// Alpha 4	Battery auto-calculates LVA setting. Restore to default instead of clear (not working)
//
//***********************************************************
//* To do
//***********************************************************
//
// Add two acc channels to each so that FW can also do autolevel
// Expo
// RC mixing menu
// Camera stabilisation (tilt/pan and gimbal)
// Advanced PWM/CPPM settings (RC settings menu)
// Differential
//
//***********************************************************
//* Flight configurations (Servo number)
//***********************************************************

/*
Standard mode

			 X <--  M1 (Throttle - CPPM mode)
             |
     M6 -----+----- M7 Ailerons (Second aileron channel)
             |
             |
        M5 --+--    Elevator
             |
             M8     Rudder


Flying Wing

		 	X <-- 	M1 (Throttle - CPPM mode)
         __/^\__
        /       \
      /           \
     |______|______|
     |_____/|\_____|

        M6     M7 (Elevons/Flaperons)
       Left   Right

            M8
          Rudder

Camera Gimbal (if enabled)

 M2 Pitch (Tilt)
 M3 Yaw	(Pan)
 M4 Roll (Roll - only for 3-axis gimbals)

*/

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

//***********************************************************
//* Fonts
//***********************************************************

#include "..\inc\Font_Verdana.h" 		// 8 (text), 14 (titles), and 22 (numbers to edit) points
#include "..\inc\Font_WingdingsOE2.h"	// Cursor and markers

//***********************************************************
//* Defines
//***********************************************************

#define	SERVO_OVERDUE 9765			// Number of T2 cycles before servo will be overdue = 9765 * 1/19531 = 500ms
#define	SERVO_RATE 390				// Requested servo rate when in failsafe mode. 19531 / 50(Hz) = 390
#define LMA_TIMEOUT 1171860			// Number or T2 cycles before Lost Model alarm sounds (1 minute)
#define FS_TIMEOUT 19531			// Number or T2 cycles before Failsafe setting engages (1 second)
#define	PWM_DELAY 250				// Number of 8us blocks to wait between "Interrupted" and starting the PWM pulses 250 = 2ms
#define	LOWER	3000				// Lower third of travel
#define	MIDDLE	3500				// Mid third of travel
#define	UPPER	4000				// Upper third of travel
#define REFRESH_TIMEOUT 39060		// Amount of time to wait after last RX activity before refreshing LCD (2 seconds)

//***********************************************************
//* Code and Data variables
//***********************************************************

// Flight variables
uint8_t flight_mode;				// Global flight mode flag
uint16_t cycletime;					// Loop time in microseconds

// Misc
bool AutoLevel;						// AutoLevel = 1
bool Stability;						// Stability = 1
bool Failsafe;
bool RefreshStatus;
char pBuffer[16];					// Print buffer

//************************************************************
// Main loop
//************************************************************

int main(void)
{
	bool Overdue = false;
	bool ServoTick = false;
	// Alarms
	bool 	BUZZER_ON = false;
	bool 	LED_ON = false;
	bool 	Model_lost = false;			// Model lost flag
	bool 	LMA_Alarm = false;			// Lost model alarm active
	bool 	LVA_Alarm = false;			// Low voltage alarm active
	bool	menu_mode = false;

	// Timers
//	uint32_t Change_UpdateStatus = 0;
	uint32_t Change_LostModel = 0;
	uint32_t Ticker_Count = 0;
	uint16_t Servo_Timeout = 0;
	uint16_t Servo_Rate = 0;

//	uint8_t Refresh_TCNT2 = 0;
	uint8_t Lost_TCNT2 = 0;
	uint8_t Ticker_TCNT2 = 0;
	uint8_t Servo_TCNT2 = 0;
	uint8_t ServoRate_TCNT2 = 0;

	uint16_t LoopStartTCNT1 = 0;

	init();									// Do all init tasks
	AccInit();								// Clear avg buffer, set indexes
	LED1 = 1;								// Switch on Red LED whenever running

//************************************************************
// Test code - start
//************************************************************

while (0) 
{
	while(BUTTON1 != 0)
	{
		if (BUTTON4 == 0)
		{
			_delay_ms(500);
			CalibrateAcc();
			CalibrateGyros();
		}

		ReadGyros();
		//ReadAcc();

		Calculate_PID();
		//get_raw_gyros();

		mugui_lcd_puts("Roll:",(prog_uchar*)Verdana8,30,0);
		mugui_lcd_puts("Pitch:",(prog_uchar*)Verdana8,30,10);
		mugui_lcd_puts("Yaw:",(prog_uchar*)Verdana8,30,20);
		mugui_lcd_puts("rADC:",(prog_uchar*)Verdana8,30,30);
		mugui_lcd_puts("pADC:",(prog_uchar*)Verdana8,30,40);
		mugui_lcd_puts("yADC:",(prog_uchar*)Verdana8,30,50);


		mugui_lcd_puts(itoa(PID_Gyros[ROLL],pBuffer,10),(prog_uchar*)Verdana8,80,0); 
		mugui_lcd_puts(itoa(PID_Gyros[PITCH],pBuffer,10),(prog_uchar*)Verdana8,80,10);
		mugui_lcd_puts(itoa(PID_Gyros[YAW],pBuffer,10),(prog_uchar*)Verdana8,80,20);
		/*mugui_lcd_puts(itoa(Config.Roll.P_mult,pBuffer,10),(prog_uchar*)Verdana8,80,30);
		mugui_lcd_puts(itoa(Config.Pitch.P_mult,pBuffer,10),(prog_uchar*)Verdana8,80,40);
		mugui_lcd_puts(itoa(Config.Yaw.P_mult,pBuffer,10),(prog_uchar*)Verdana8,80,50);*/
		mugui_lcd_puts(itoa(gyroADC[ROLL],pBuffer,10),(prog_uchar*)Verdana8,80,30);
		mugui_lcd_puts(itoa(gyroADC[PITCH],pBuffer,10),(prog_uchar*)Verdana8,80,40);
		mugui_lcd_puts(itoa(gyroADC[YAW],pBuffer,10),(prog_uchar*)Verdana8,80,50);

		// Update buffer
		write_buffer(buffer);
		clear_buffer(buffer);
		_delay_ms(100);
	}
}

//************************************************************
// RX input to Servos test code - OK
//************************************************************

while(0)
{
	ServoOut[0] = 2250;
	ServoOut[1] = 2500;
	ServoOut[2] = 3000;
	ServoOut[3] = 3750;
	ServoOut[4] = 4000;
	ServoOut[5] = 4375;
	ServoOut[6] = 5000;
	ServoOut[7] = 5500;

	RC_Lock = true;					// Fudge lock
	Failsafe = true;
	output_servo_ppm();				// Output servo signal
	_delay_ms(20);
}

//************************************************************
// Test code - end
//************************************************************

	Display_status();

	//************************************************************
	//* Main loop
	//************************************************************
	while (1)
	{
		// Display menu on request
		if((BUTTON2 == 0) || (BUTTON3 == 0))
		{
			menu_mode = true;
			menu_main();
			Display_status();
			menu_mode = false;
		}

		//************************************************************
		//* Status menu refreshing
		//************************************************************

		// Refresh status window on manual request
		if(BUTTON1 == 0)
		{
			Display_status();
		}
/*
		// Update status provided no RX activity for X seconds
		Change_UpdateStatus += (uint8_t) (TCNT2 - Refresh_TCNT2);
		Refresh_TCNT2 = TCNT2;

		// Reset count if any RX activity
		if (RxActivity)	
		{														
			Change_UpdateStatus = 0;
			RefreshStatus = false;			
		}
		// Wait for Xs then allow status refresh
		if (Change_UpdateStatus > REFRESH_TIMEOUT)	
		{
			RefreshStatus = true;
		}

		if (BUZZER_ON && RefreshStatus) 
		{
			Display_status();
		}
*/
		//************************************************************
		//* Reconfigure interrupts if menu changed
		//************************************************************

		// Reconfigure interrupts for PWM/CPPM modes
		if (Config.RxMode != CPPM_MODE)
		{
			PCMSK1 |= (1 << PCINT8);			// PB0 (Aux pin change mask)
			PCMSK3 |= (1 << PCINT24);			// PD0 (Throttle pin change mask)
			EIMSK  |= (1 << INT0);				// Enable INT0 (Elevator input)
			EIMSK  |= (1 << INT1);				// Enable INT1 (Aileron input)
			EIMSK  |= (1 << INT2);				// Enable INT2 (Rudder/CPPM input)
		}
		else // CPPM mode
		{
			PCMSK1 = 0;							// Disable AUX
			PCMSK3 = 0;							// Disable THR
			EIMSK = 0;							// Disable external interrupts
			EIMSK  |= (1 << INT2);				// Enable INT2 (Rudder/CPPM input)
		}

		//************************************************************
		//* System ticker - based on TCNT2 (19.531kHz)
		//* 
		//* (Ticker_Count &128) 	 	= 152.6Hz
		//* ((Ticker_Count >> 8) &2) 	= 38.1Hz
		//* ((Ticker_Count >> 8) &8) 	= 4.77Hz (LMA and LVA alarms)
		//* ((Ticker_Count >> 8) &32)	= 0.59Hz (LED mode alarms)
		//* 
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

		if ((Ticker_Count >> 8) &64) 
		{
			LED_ON = true;		// 0.59Hz flash
		}
		else 
		{
			LED_ON = false;
		}

		//************************************************************
		//* Alarms
		//************************************************************

		// Lost model alarm
		Change_LostModel += (uint8_t) (TCNT2 - Lost_TCNT2);
		Lost_TCNT2 = TCNT2;

		// Reset count if any RX activity
		if (RxActivity)	
		{														
			Change_LostModel = 0;
			Model_lost = false;			
		}
		// Wait for 60s then trigger lost model alarm
		if (Change_LostModel > LMA_TIMEOUT)	
		{
			Model_lost = true;
		}

		if (BUZZER_ON && Model_lost) 
		{
			LMA_Alarm = true;	// Turn on buzzer
		}
		else 
		{
			LMA_Alarm = false;				// Otherwise turn off buzzer
		}

		// Low-voltage alarm (LVA)
		GetVbat();							// Check battery

		if ((Config.Modes &16) > 0)			// Buzzer mode
		{
			// Beep buzzer if Vbat lower than trigger
			if ((vBat < Config.PowerTrigger) && BUZZER_ON) 
			{
				LVA_Alarm = true;
			}
			else 
			{
				LVA_Alarm = false;			// Otherwise turn off buzzer
			}
		}
		else 								// LED mode
		{	// Flash LEDs if Vbat lower than trigger
			if ((vBat < Config.PowerTrigger) && LED_ON) 
			{
				LVA_Alarm = false;	
			}
			else 
			{
				LVA_Alarm = true;			// Otherwise leave LEDs on
			}
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
		//* Autolevel mode selection
		//************************************************************
		//* Primary override:
		//*		Autolevel enabled if Config.AutoMode = 0N
		//*		Autolevel always OFF if Config.AutoMode = OFF (default)
		//*
		//* Three switchable modes:
		//*		1. Disabled by Config.AutoMode = OFF
		//*		2. Enabled by "AutoChan" channel number
		//*		3. Enabled by upper 33% throw of "ThreePos" channel number
		//************************************************************

		// Update zeroed RC channel data
		RxGetChannels();

		switch(Config.AutoMode)
		{
			case DISABLED:
				AutoLevel = false;				// De-activate autolevel mode
				break;
			case AUTOCHAN:
				//if (RxChannel[AUX1] > 3000)
				if (RxChannel[Config.AutoChan] > MIDDLE)
				{
					AutoLevel = true;			// Activate autolevel mode
				}	
				else
				{
					AutoLevel = false;			// De-activate autolevel mode
				}	
				break;
			case THREEPOS:
				if (RxChannel[Config.AutoChan] > UPPER)
				{
					AutoLevel = true;			// Activate autolevel mode
				}	
				else
				{
					AutoLevel = false;			// De-activate autolevel mode
				}	
				break;
			case ALWAYSON:
				AutoLevel = true;			// Activate autolevel mode
				break;
			default:							// Disable by default
				AutoLevel = false;				// De-activate autolevel mode
				break;
		}

		if (AutoLevel)
		{
			flight_mode |= 1;
		}
		else
		{
			flight_mode &= 0xfe;
		}

		//************************************************************
		//* Stability mode selection
		//************************************************************
		//* Primary override:
		//*		Stability enabled if Config.StabMode = ON
		//*		Stability always OFF if Config.StabMode = OFF (default)
		//*
		//* Three switchable modes:
		//*		1. Disabled by Config.StabMode = OFF
		//*		2. Enabled by "StabChan" channel number
		//*		3. Enabled by middle or higher throw of "ThreePos" channel number
		//************************************************************

		switch(Config.StabMode)
		{
			case DISABLED:
				Stability = false;				// De-activate autolevel mode
				break;
			case STABCHAN:
			case THREEPOS:
				//if (RxChannel[GEAR] > 3000)
				if (RxChannel[Config.StabChan] > MIDDLE)
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
				Stability = false;				// De-activate autolevel mode
				break;
		}

		if (Stability)
		{
			flight_mode |= 8;
		}
		else
		{
			flight_mode &= 0xf7;
			// Reset I-terms when stabilise is off
			IntegralaPitch = 0;	 
			IntegralaRoll = 0;
		}

		// Read gyros when required
		if (Stability || AutoLevel)
		{
			ReadGyros();		// Read sensors
		}

		// Autolevel mode ON
		if (AutoLevel) 
		{
			LED1 = 1; //debug
			ReadAcc();			// Only read Accs if in AutoLevel mode
			AvgAcc();			// Average acc readings
		}
		else
		{
			LED1 = 0; //debug
		}

		// Calculate PID
		Calculate_PID();

		// Remove RC noise when sticks centered
		RC_Deadband();

		// Calculate mix
		ProcessMixer();

		// Transfer Config.Channel[i].value data to servos
		UpdateServos();

		//************************************************************
		//* Process servos, failsafe mode
		//************************************************************

		// Servo_Timeout increments at 19.531 kHz, in loop cycle chunks
		Servo_Timeout += (uint8_t) (TCNT2 - Servo_TCNT2);
		Servo_TCNT2 = TCNT2;
		if (Servo_Timeout > SERVO_OVERDUE)
		{
			Overdue = true;
		}

		// Servo_Rate increments at 19.531 kHz, in loop cycle chunks
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

		// Set failsafe positions when RC lock lost
		if (Failsafe)
		{
			uint8_t i;
			for (i = 0; i < MAX_OUTPUTS; i++)
			{
				ServoOut[i] = Config.Channel[i].Failsafe;
			}
		}

		// Ensure that output_servo_ppm() is synchronised to the RC interrupts
		if (Interrupted)
		//if (Interrupted && !RefreshStatus)
		{
			Interrupted = false;			// Reset interrupted flag

			Failsafe = false;				// Cancel failsafe
			Servo_Timeout = 0;				// Reset servo failsafe timeout
			Overdue = false;				// And no longer overdue...

			if(Config.RxMode == CPPM_MODE)
			{
				uint8_t i;
				// Short delay to ensure no residual interrupt activity from ISR
				TIFR0 &= ~(1 << TOV0);			// Clear overflow
				TCNT0 = 0;						// Reset counter
				for (i=0;i<PWM_DELAY;i++)		// PWM_DELAY * 8us = 1ms
				{
					while (TCNT0 < 160);		// 1/20MHz * 64 = 8us
					TCNT0 -= 160;
				}
			}
			output_servo_ppm();				// Output servo signal
		}
		// If in failsafe, just output unsynchronised
		else if (Failsafe && Overdue && ServoTick)
		//else if ((Failsafe && Overdue && ServoTick) || (Overdue && ServoTick && RefreshStatus))
		{
			ServoTick = false;				// Reset servo update ticker
			Servo_Rate = 0;					// Reset servo rate timer

			output_servo_ppm();				// Output servo signal
		}

		// Measure the current loop rate
		cycletime = TCNT1 - LoopStartTCNT1;	// Update cycle time
		LoopStartTCNT1 = TCNT1;				// Measure period of loop from here

	} // main loop
} // main()

