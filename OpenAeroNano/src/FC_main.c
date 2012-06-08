// **************************************************************************
// OpenAero software
// =================
// Version 1.10a
// Inspired by KKmulticopter
// Based on assembly code by Rolf R Bakke, and C code by Mike Barton
//
// Includes PID and Auto-level functions inspired by the open-sourced MultiWiiproject
// Compatible with KK boards fitted with X and Y accelerometers 
// on Roll/Pitch pot inputs. LCD board or GUI required for setup of PID constants.
//
// Tested only on Atmega168P boards (KK+)
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
//
// Stick Centering
// ---------------
// Set Yaw Gain pot to max
// Set Tx trims to centre
// Power on
// LED flashes 3 times
// Restore gain pot
// Restart
//
// 
// **************************************************************************
// Version History
// ===============
// V1.0a	Based on OpenAero V1.10a code
//			Initial code base.

//
//***********************************************************
//* To do
//***********************************************************
//

//
//***********************************************************
//* Flight configurations (Servo number)
//***********************************************************

/*
Standard mode

			 X <-- THR (Throttle - CPPM mode)
             |
  M3/M4 -----+----- Aileron
             |
             |
    M5/M6 ---+---   Elevator
             |
           M1/M2    Rudder


Flying Wing - Assumes mixing done in the transmitter

		 	X <-- THR (Throttle - CPPM mode)
         __/^\__
        /       \
      /           \
     |______|______|
     |_____/|\_____|

      M3/M4   M5/M6 (Elevons/Flaperons)
       Left   Right

          M1/M2
          Rudder
*/

//***********************************************************
//* Includes
//***********************************************************

#include <avr/io.h>
#include <stdlib.h>
#include <stdbool.h>
#include <util/delay.h>
#include "..\inc\io_cfg.h"
#include "..\inc\eeprom.h"
#include "..\inc\adc.h"
#include "..\inc\rc.h"
#include "..\inc\servos.h"
#include "..\inc\pots.h"
#include "..\inc\gyros.h"
#include "..\inc\init.h"
#include "..\inc\isr.h"

//***********************************************************
//* Defines
//***********************************************************
// Sets the rate of the main loop (Normally 500Hz)
#define LOOP_RATE 500				// in Hz
#define LOOP_INTERVAL (1000000 / LOOP_RATE ) // 3,333

// Defines output rate to the servo (Normally 50Hz)
#define SERVO_RATE 50				// in Hz

//***********************************************************
//* Code and Data variables
//***********************************************************

// Axis variables
int32_t Roll;						// Temp axis values. Seems we need 32 bits here.
int32_t Pitch;
int32_t Yaw;

// PID variables
int32_t P_term_gRoll;				// Calculated P-terms (gyro) for each axis
int32_t P_term_gPitch;

// Misc
uint16_t uber_loop_count;			// Used to count main loops for everything else :)
uint16_t LoopStartTCNT1, LoopElapsedTCNT1, LoopCurrentTCNT1;
uint16_t loop_padding, servo_skip;
uint8_t rxin;

//************************************************************
// Main loop
//************************************************************

int main(void)
{
	init();									// Do all init tasks

//************************************************************
// Test code - start
//************************************************************
if (0) 
{
	// Test new servo code
	while(1)
	{
		RxGetChannels();
		ServoOut1 = RxChannel3;
		ServoOut2 = 1000;
		ServoOut3 = 1200;
		ServoOut4 = 1500;
		ServoOut5 = 1500;
		ServoOut6 = RxInAux + 1200;
		Throttle = RxChannel3;
		while (Interrupted == false){};
		Interrupted = false;
		//_delay_ms(30);
		output_servo_ppm();	
	}
}

//************************************************************
// Test code - end
//************************************************************


	LED = 1;								// Switch on LED whenever on

	// Main loop
	while (1)
	{
		rxin = UDR0;						// Flush RX buffer


		RxGetChannels();
		// Reverse primary servo outputs as required
		if(Config.YawServo) {
			ServoOut1 = Config.RxChannel4ZeroOffset - RxInYaw; 
		}
		else {
			ServoOut1 = RxChannel4;
		}
		if(Config.RollServo) {
			ServoOut3 = Config.RxChannel1ZeroOffset - RxInRoll;
		}
		else {
			ServoOut3 = RxChannel1;
		}
		if(Config.PitchServo) {
			ServoOut5 = Config.RxChannel2ZeroOffset - RxInPitch;
		}
		else {
			ServoOut5 = RxChannel2;
		}

		ServoOut2 = Config.RxChannel4ZeroOffset - RxInYaw;
		ServoOut4 = Config.RxChannel1ZeroOffset - RxInRoll;
		ServoOut6 = Config.RxChannel2ZeroOffset - RxInPitch;

		Throttle = RxChannel3;

		// Stability mode OFF
		#ifdef CPPM_MODE
		if (RxChannel7 > 1600)				// AUX 1 over-rides stability
		#else
		if (RxInAux > 0)					// RxInAux (formerly throttle) over-rides stability
		#endif
		{
			// Do nothing
		}

		// Stability mode ON
		else
		{
			ReadGainValues();
			ReadGyros();

			//***********************************************************************
			//                --- Calculate roll gyro output ---
			//***********************************************************************

			Roll = gyroADC[ROLL];

			// Gyro PID terms
			P_term_gRoll = Roll * GainInADC[ROLL];				// Multiply P-term (Max gain of 256)
			P_term_gRoll = P_term_gRoll * 3;					// Multiply by 3, so max effective gain is 768
			Roll = P_term_gRoll >> 7;							// Divide by 128 to rescale values back to normal

			//--- (Add)Adjust roll gyro output to Servos
			#ifdef STANDARD
			ServoOut3 -= Roll;
			ServoOut4 += Roll;
			#elif defined(FWING)
			ServoOut3 += Roll;
			ServoOut4 -= Roll;
			ServoOut5 += Roll;
			ServoOut6 -= Roll;
			#else
			#error No configuration defined !!!!
			#endif

			//***********************************************************************
			//                --- Calculate pitch gyro output ---
			//***********************************************************************

			Pitch = gyroADC[PITCH];

			// Gyro PID terms
			P_term_gPitch = Pitch * GainInADC[PITCH];			// Multiply P-term (Max gain of 256)
			P_term_gPitch = P_term_gPitch * 3;					// Multiply by 3, so max effective gain is 768
			Pitch = P_term_gPitch >> 7;							// Divide by 128 to rescale values back to normal

			//--- (Add)Adjust pitch gyro output to Servos
			#ifdef STANDARD
			ServoOut5 -= Pitch;
			ServoOut6 += Pitch;
			#elif defined(FWING)
			ServoOut3 -= Pitch;
			ServoOut4 += Pitch;
			ServoOut5 += Pitch;
			ServoOut6 -= Pitch;
			#else
			#error No configuration defined !!!!
			#endif

			//***********************************************************************
			//                 --- Calculate yaw gyro output ---
			//***********************************************************************

			Yaw = gyroADC[YAW];
			Yaw *= GainInADC[YAW];								// Multiply P-term (Max gain of 768)
			Yaw = Yaw * 3;	
			Yaw = Yaw >> 7;										// Divide by 128 to rescale values back to normal

			//--- (Add)Adjust yaw gyro output to servos
			#if (defined(STANDARD) || defined(FWING))
			ServoOut1 -= Yaw;
			ServoOut2 += Yaw;
			#else
			#error No configuration defined !!!!
			#endif

		} // AUX 1 over-rides stability

		// Loop governor is here so that output_servo_ppm() is only called every 20ms
		// and the loop is regulated to LOOP_RATE Hz.
		// Also, handle the odd case where the TCNT1 rolls over and TCNT1 < LoopStartTCNT1
		LoopCurrentTCNT1 = TCNT1;
		if (LoopCurrentTCNT1 > LoopStartTCNT1) LoopElapsedTCNT1 = LoopCurrentTCNT1 - LoopStartTCNT1;
		else LoopElapsedTCNT1 = (0xffff - LoopStartTCNT1) + LoopCurrentTCNT1;

		// If loop period less than LOOP_INTERVAL, pad it out. (NB: blocking code)
		loop_padding = (LOOP_INTERVAL - LoopElapsedTCNT1) / 8;

		if (loop_padding > 0)
		{
			TIFR0 &= ~(1 << TOV0);		// Clear overflow
			TCNT0 = 0;					// Reset counter
			for (int i=0;i<loop_padding;i++)
			{
				while (TCNT0 < 64);		// 8MHz * 64 = 8us
				TCNT0 -= 64;
			}
		}

		// If SERVO_RATE due, update servos, otherwise keep looping
		if (servo_skip >= (LOOP_RATE/SERVO_RATE))
		{
			Interrupted = false;
			servo_skip = 0;
			while (Interrupted == false){};	// Wait here for any interrupts to complete
			Interrupted = false;
			output_servo_ppm();			// Output servo signal
		}

		// Measure period of loop from here
		LoopStartTCNT1 = TCNT1;
		uber_loop_count++;
		servo_skip++;
	} // main loop
} // main()

