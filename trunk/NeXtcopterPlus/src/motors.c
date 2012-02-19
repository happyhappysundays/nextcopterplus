//***********************************************************
//* motors.c
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include <avr/io.h>
#include <stdbool.h>
#include "..\inc\io_cfg.h"
#include "..\inc\init.h"
#include "..\inc\main.h"
#include "..\inc\lcd.h"

//************************************************************
// Prototypes
//************************************************************

void output_motor_ppm(void);

//************************************************************
// Defines
//************************************************************

// Defines output rate to ESC/Servo (Max is approx 495Hz)
#define ESC_RATE 495	// in Hz
#define PWM_LOW_PULSE_INTERVAL (1000000 / ESC_RATE ) // 2020
#define BASE_PULSE (1120 / 8) // 1120us / 8us groups

//************************************************************
// Code
//************************************************************

int16_t	PWM_Low_Pulse_Interval = PWM_LOW_PULSE_INTERVAL;

int16_t MotorOut1;		// Motor speed variables
int16_t MotorOut2;
int16_t MotorOut3;
int16_t MotorOut4;
int16_t MotorOut5;
int16_t MotorOut6;

void output_motor_ppm(void)
{
	uint8_t i;
	static uint16_t MotorStartTCNT1, ElapsedTCNT1, CurrentTCNT1;
	uint8_t m1,m2,m3,m4,m5,m6;

	// Only enable motors when armed or not connected to the GUI
	if (!Armed || GUIconnected) return; 

	// Make sure we have spent enough time between pulses
	// Also, handle the odd case where the TCNT1 rolls over and TCNT1 < MotorStartTCNT1
	CurrentTCNT1 = TCNT1;
	if (CurrentTCNT1 > MotorStartTCNT1) ElapsedTCNT1 = CurrentTCNT1 - MotorStartTCNT1;
	else ElapsedTCNT1 = (0xffff - MotorStartTCNT1) + CurrentTCNT1;

	// If period less than 1/ESC_RATE, pad it out.
	PWM_Low_Pulse_Interval = (PWM_LOW_PULSE_INTERVAL - ElapsedTCNT1) / 8;

	if (PWM_Low_Pulse_Interval > 0)
	{
		TIFR0 &= ~(1 << TOV0);		// Clear overflow
		TCNT0 = 0;					// Reset counter
		for (i=0;i<PWM_Low_Pulse_Interval;i++)
		{
			while (TCNT0 < 64);		// 8MHz * 64 = 8us
			TCNT0 -= 64;
		}
	}

	// Set motor limits (0 -> 200)
	if ( MotorOut1 < 0 ) m1 = 0;
	else if ( MotorOut1 > 200 ) m1 = 200;
	else m1 = MotorOut1;
	
	if ( MotorOut2 < 0 ) m2 = 0;
	else if ( MotorOut2 > 200 ) m2 = 200;
	else m2 = MotorOut2;

	if ( MotorOut3 < 0 ) m3 = 0;
	else if ( MotorOut3 > 200 ) m3 = 200;
	else m3 = MotorOut3;

	if ( MotorOut4 < 0 ) m4 = 0;
	else if ( MotorOut4 > 200 ) m4 = 200;
	else m4 = MotorOut4;

	if ( MotorOut5 < 0 ) m5 = 0;
	else if ( MotorOut5 > 200 ) m5 = 200;
	else m5 = MotorOut5;

	if ( MotorOut6 < 0 ) m6 = 0;
	else if ( MotorOut6 > 200 ) m6 = 200;
	else m6 = MotorOut6;

	// T0 = 8 bit @ 8MHz, so 1 count per 125ns, max of 32us
	// T1 = 16 bit @ 1MHz, so 1 count per us, max of 65,539us or 65.5ms
	// CurrentTCNT1 = TCNT1; // Snapshot

	// Minimum pulse we want to make is 1ms, max is 2ms
	// So to start, let's make the 1ms base pulse.
	// First, we switch on the motor outputs
	M1 = 1;
	M2 = 1;
	M3 = 1;
	M4 = 1;
	M5 = 1;
	M6 = 1;

	// Measure period of ESC rate from here
	MotorStartTCNT1 = TCNT1;

	// Create the base pulse of 1120us
	TIFR0 &= ~(1 << TOV0);		// Clear overflow
	TCNT0 = 0;					// Reset counter

	for (i=0;i<BASE_PULSE;i++)	// BASE_PULSE * 8us = 1ms
	{
		while (TCNT0 < 64);		// 8MHz * 64 = 8us
		TCNT0 -= 64;
	}

	// Now switch off the pulses as required
	// 1120us to 1920us = 800us / 4us = 200 steps
	// Motors 0->200, 1120->1920 us
	for (i=0;i<220;i++)			// 220 gives a max of 2000us (1120 + (220 * 4us)) - TWEAK THIS
	{
		while (TCNT0 < 32);		// 8MHz * 32 = 4us
		TCNT0 -= 32;

		if (i==m1) M1 = 0;
		if (i==m2) M2 = 0;
		if (i==m3) M3 = 0;
		if (i==m4) M4 = 0;
		if (i==m5) M5 = 0;
		if (i==m6) M6 = 0;
	} 
	// Pulse done, now back to waiting about...
}
