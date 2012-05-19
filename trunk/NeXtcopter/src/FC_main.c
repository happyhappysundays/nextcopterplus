// **************************************************************************
// NeXtcopter software
// ===================
// Version 1.0a
// Inspired by KKmulticopter
// Based on assembly code by Rolf R Bakke, and C code by Mike Barton
//
// Includes PID functions inspired by the open-sourced MultiWiiproject
//
// Compatible with ATmega168-based KKcontroller boards
// Tested on a Blackboard 5.5 and a KK PLus 5.5d
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
// Set Pitch Gain pot to zero
// Set Tx trims to centre
// Power on
// LED flashes 3 times, then one long flash when data saved
// Restore gain pot
// Restart

// ESC throttle calibration
// ------------------------
// Set Yaw Gain pot to zero
// Put throttle stick to full
// Power on
// LED flashes 3 times
// Wait for motor signal (double bleep)
// Set throttle to zero
// Wait for motor confirm signal
// Power off
// Restore gain pot
//
// Clear all settings (gyro & stick centering)
// -------------------------------------------
// Set Roll, Pitch & Yaw Gain pots to zero
// Power on
// LED flashes twice, then one long flash when data saved
// Power off
// Restore gain pots
//
// **************************************************************************
// Version History
// ===============
// V1.0 	Includes PID loop control on all axis.
//			Removed most experimental and pointless code, 
//			Realigned with Rolf's V4.7 code.
//			Introduced GNU GPL open-source protection.
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

//***********************************************************
//* Compilation options
//***********************************************************

//#define QUAD_COPTER		// Choose this for + config
#define QUAD_X_COPTER		// Choose this for X config

#define STICK_MODES			// Stick sentitivity option 
							// NB: Increases code size

//***********************************************************
//* Defines
//***********************************************************

// Max Collective
// Limits the maximum stick collective (range 160->200 200=Off)
// This allows gyros to stabilise better when full throttle applied
#define MAX_COLLECTIVE 190
// Motor idle speed. If your motors stall during some maneuvers, increase.
#define MOTOR_IDLE 15				// Minimum motor speed allowed.

// PID constants
#define ITERM_LIMIT_RP 150			// Max I-term sum for Roll/Pitch axis
#define ITERM_LIMIT 500				// Max I-term sum for Yaw axis (Heading hold)
#define DEAD_BAND 2					// Centre region of RC input where no activity is processed
#define YAW_LIMIT 50				// Limit yaw contribution to avoid saturation
#define YAW_ITERM 110				// Fixed yaw I-term

// STICKSCALE constants
#define NORMAL_ROLL_STICK_SCALE		2	// Experimental values
#define NORMAL_PITCH_STICK_SCALE	2	// Increasing value by 1 halves stick sensitivity
#define NORMAL_YAW_STICK_SCALE		2	// Reducing value by 1 doubles stick sensitivity
#define ACRO_ROLL_STICK_SCALE		1
#define ACRO_PITCH_STICK_SCALE		1
#define ACRO_YAW_STICK_SCALE		1
#define UFO_YAW_STICK_SCALE			0

// Stick Arming
// If you cannot either arm or disarm, lower this value
#define STICKARM_POINT 60			// Defines how far stick must be moved

//***********************************************************
//* Code and Data variables
//***********************************************************

uint8_t RollStickScale;				// Stick scale variables
uint8_t PitchStickScale;
uint8_t YawStickScale;

int32_t IntegralPitch = 0;			// PID I-terms for each axis
int32_t IntegralRoll = 0;
int32_t IntegralYaw = 0;
int16_t currentGyroError[3] = {0,0,0}; // Used with lastGyroError to keep track of D-Terms in PID calculations
int16_t lastGyroError[3] = {0,0,0}; 
int16_t DifferentialGyro = 0;		// Holds difference between last two gyro errors (angular acceleration)

bool AcroMode;
uint8_t loop_count = 0;				// Used to count main loops

//************************************************************
// Main loop
//************************************************************

int main(void)
{
	static uint16_t Change_Arming=0;
	static uint8_t Arming_TCNT2=0;
	int16_t RollGyro;
	int32_t Roll;
	int16_t PitchGyro;
	int32_t Pitch;
	int16_t YawGyro;
	int32_t Yaw;
	int32_t I_term_Roll = 0;
	int32_t I_term_Pitch = 0;
	int32_t I_term_Yaw = 0;

	init();

	while (1)
	{
		RxGetChannels();

		if (RxInCollective == 0) {
			// Check for stick arming (Timer2 @ 8MHz/1024 = 7812.5KHz)
			// Arm: yaw right (>60), dis-arm: yaw left (<-60)
			Change_Arming += (uint8_t) (TCNT2 - Arming_TCNT2);
			Arming_TCNT2 = TCNT2;

			if (Armed) {
				if (RxInYaw<STICKARM_POINT) 	Change_Arming = 0;		// Reset count
			} else {
				if (RxInYaw>-STICKARM_POINT) 	Change_Arming = 0;		// Reset count
			}

			// Reset I-term sums at rest.
			// Read gain pots here to save repowering the FC
		    ReadGainValues();
			IntegralPitch = 0;	 
			IntegralRoll = 0;
			IntegralYaw = 0;

			// 1 sec, about 8000. (8000 * 1/7812Hz = 1.024s)
			if (Change_Arming > 8000)
			{
				Armed = ! Armed;
				LED = 0;
				if (Armed) {
					output_motor_high = false;	// Reset 1st time flag
					CalibrateGyros();
					IntegralPitch = 0;	 
					IntegralRoll = 0;
					IntegralYaw = 0;


					#ifdef STICK_MODES
					// Stick selectable flight modes
					uint8_t nBlink = 1;

					if (RxInPitch > STICKARM_POINT) // ACRO
					{
						AcroMode = true;
						RollStickScale = ACRO_ROLL_STICK_SCALE;
						PitchStickScale = ACRO_PITCH_STICK_SCALE;
						YawStickScale = ACRO_YAW_STICK_SCALE;
						nBlink = 5;
		 			}
			 		else if (RxInPitch < -STICKARM_POINT) // UFO
			 		{
						AcroMode = false;
						RollStickScale = NORMAL_ROLL_STICK_SCALE;
						PitchStickScale = NORMAL_PITCH_STICK_SCALE;
						YawStickScale = UFO_YAW_STICK_SCALE;
						nBlink = 3;
			 		}
			 		else // NORMAL
			 		{
						AcroMode = false;
						RollStickScale = NORMAL_ROLL_STICK_SCALE;
						PitchStickScale = NORMAL_PITCH_STICK_SCALE;
						YawStickScale = NORMAL_YAW_STICK_SCALE;
						nBlink = 1;
			 		}

			 		// Flash LED to indicate flight mode
					for (uint8_t i=0;i<nBlink;i++)
						{
							LED = 1;
							_delay_ms(200);
							LED = 0;
							_delay_ms(200);
						}
					#endif //(STICK_MODES)

					LED = 1; // Light LED to indicate armed.

				} // if (Armed)
				else if (output_motor_high) {
					output_motor_ppm();			// Turn off
				}
			} // if (Change_Arming)
		} // if (RxInCollective == 0)


		//--- Read sensors ---
		ReadGyros();

		//--- Start mixing by setting collective to motor input 1,2,3 and 4 ---
		if (RxInCollective > MAX_COLLECTIVE) RxInCollective = MAX_COLLECTIVE;
		MotorOut1 = RxInCollective;
		MotorOut2 = RxInCollective;
		MotorOut3 = RxInCollective;
		MotorOut4 = RxInCollective; 

		//***********************************************************************
		//                --- Calculate roll gyro output ---
		// NB: IF YOU CHANGE THIS CODE, YOU MUST REMOVE PROPS BEFORE TESTING !!!
		//***********************************************************************

		RxInRoll = RxInRoll >> RollStickScale;		// Reduce RxInRoll rate per flying mode

		RollGyro = gyroADC[ROLL];	
		Roll = RxInRoll + RollGyro;	

		IntegralRoll += Roll;						// I-term (32-bit)
		if (IntegralRoll > ITERM_LIMIT_RP) IntegralRoll = ITERM_LIMIT_RP;
		else if (IntegralRoll < -ITERM_LIMIT_RP) IntegralRoll = -ITERM_LIMIT_RP;// Anti wind-up

		Roll *= GainIn[ROLL];						// Multiply P-term (Max gain of 128)
		Roll = Roll * 3;							// Multiply by 3, so max effective gain is 384

		I_term_Roll = IntegralRoll * GainIn[PITCH];	// Multiply I-term (Max gain of 128)
		I_term_Roll = I_term_Roll >> 3;				// Divide by 8, so max effective gain is 16

		currentGyroError[ROLL] = gyroADC[ROLL];		// D-term. No multiplier for now. Experimental
		DifferentialGyro = currentGyroError[ROLL] - lastGyroError[ROLL];
		lastGyroError[ROLL] = currentGyroError[ROLL];	
	
		Roll = Roll + I_term_Roll + DifferentialGyro;// P + I + D
		Roll = Roll >> 7;							// Divide by 128 to rescale values back to normal

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

		RxInPitch = RxInPitch >> PitchStickScale;	// Reduce RxInPitch rate per flying mode

		PitchGyro = gyroADC[PITCH];				 
		Pitch = RxInPitch + PitchGyro;

		IntegralPitch += Pitch;						// I-term (32-bit)
		if (IntegralPitch > ITERM_LIMIT_RP) IntegralPitch = ITERM_LIMIT_RP;
		else if (IntegralPitch < -ITERM_LIMIT_RP) IntegralPitch = -ITERM_LIMIT_RP;// Anti wind-up

		Pitch *= GainIn[ROLL];						// Multiply P-term (Max gain of 128)
		Pitch = Pitch * 3;							// Multiply by 3, so max effective gain is 384

		I_term_Pitch = IntegralPitch * GainIn[PITCH];// Multiply I-term (Max gain of 128)
		I_term_Pitch = I_term_Pitch >> 3;			// Divide by 8, so max effective gain is 16

		currentGyroError[PITCH] = gyroADC[PITCH];	// D-term. No multiplier for now. Experimental
		DifferentialGyro = currentGyroError[PITCH] - lastGyroError[PITCH];
		lastGyroError[PITCH] = currentGyroError[PITCH];	
	
		Pitch = Pitch + I_term_Pitch + DifferentialGyro; // P + I + D
		Pitch = Pitch >> 7;							// Divide by 128 to rescale values back to normal

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

		RxInYaw = RxInYaw >> YawStickScale;		// Reduce RxInYaw rate per flying mode

		YawGyro = gyroADC[YAW];					
		Yaw = RxInYaw + YawGyro;

		IntegralYaw += Yaw;						// I-term (32-bit)
		if (IntegralYaw > ITERM_LIMIT) IntegralYaw = ITERM_LIMIT;
		else if (IntegralYaw < -ITERM_LIMIT) IntegralYaw = -ITERM_LIMIT;// Anti wind-up (Experiment with value)

		Yaw *= GainIn[YAW];						// Multiply P-term (Max gain of 128 x 3)
		Yaw *= 3;

		I_term_Yaw = IntegralYaw * YAW_ITERM;	// Multiply IntegralPitch by fixed amount
		I_term_Yaw = I_term_Yaw >> 3;			// Divide by 8, so max effective gain is 16

		currentGyroError[YAW] = gyroADC[YAW];	// D-term. No multiplier for now. Experimental
		DifferentialGyro = currentGyroError[YAW] - lastGyroError[YAW];
		lastGyroError[YAW] = currentGyroError[YAW];	
	
		Yaw = Yaw + I_term_Yaw + DifferentialGyro;// P + I + D

		Yaw = Yaw >> 7;							// Divide by 128 to rescale values back to normal
												// P=2.7, I=0.0047, D=0.0078

		if (Yaw > YAW_LIMIT) Yaw = YAW_LIMIT;		
		else if (Yaw < -YAW_LIMIT) Yaw = -YAW_LIMIT; // Apply YAW limit to PID calculation

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

		if (RxInCollective < 1 || !Armed || !GyroCalibrated)	// Turn off motors if collective below 1% 
		{														// or if gyros not calibrated
			MotorOut1 = 0;
			MotorOut2 = 0;
			MotorOut3 = 0;
			MotorOut4 = 0; 
		}

		if (Armed) output_motor_ppm();			// Output ESC signal
	}
}

