//***********************************************************
//* Servos.c
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
#include <stdlib.h> //debug

//************************************************************
// Prototypes
//************************************************************

void output_servo_ppm(void);

//************************************************************
// Defines
//************************************************************

// Defines output rate to the servo (Normally 50Hz)
#define SERVO_RATE 50	// in Hz
#define PWM_LOW_PULSE_INTERVAL (1000000 / SERVO_RATE ) // 20,000
//#define BASE_PULSE (904 / 8) // 904us / 8us = 113 * 8us groups
#define BASE_PULSE (512 / 8)

#define MIN_PULSE 904
#define MAX_PULSE 2200

//************************************************************
// Code
//************************************************************

int16_t	PWM_Low_Pulse_Interval = PWM_LOW_PULSE_INTERVAL;

int16_t ServoOut1;
int16_t ServoOut2;
int16_t ServoOut3;
int16_t ServoOut4;
int16_t ServoOut5;
int16_t ServoOut6;

void output_servo_ppm(void)
{
	uint16_t i;
	static uint16_t ServoStartTCNT1, ElapsedTCNT1, CurrentTCNT1;
	uint16_t m1,m2,m3,m4,m5,m6;

	// Make sure we have spent enough time between pulses
	// Also, handle the odd case where the TCNT1 rolls over and TCNT1 < ServoStartTCNT1
	CurrentTCNT1 = TCNT1;
	if (CurrentTCNT1 > ServoStartTCNT1) ElapsedTCNT1 = CurrentTCNT1 - ServoStartTCNT1;
	else ElapsedTCNT1 = (0xffff - ServoStartTCNT1) + CurrentTCNT1;

	// If period less than 1/SERVO_RATE, pad it out. (NB: blocking code)
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

	// Set Servo limits (MIN_PULSE -> MAX_PULSE)
	if ( ServoOut1 < MIN_PULSE ) m1 = MIN_PULSE;
	else if ( ServoOut1 > MAX_PULSE ) m1 = MAX_PULSE;
	else m1 = ServoOut1;
	
	if ( ServoOut2 < MIN_PULSE ) m2 = MIN_PULSE;
	else if ( ServoOut2 > MAX_PULSE ) m2 = MAX_PULSE;
	else m2 = ServoOut2;

	if ( ServoOut3 < MIN_PULSE ) m3 = MIN_PULSE;
	else if ( ServoOut3 > MAX_PULSE ) m3 = MAX_PULSE;
	else m3 = ServoOut3;

	if ( ServoOut4 < MIN_PULSE ) m4 = MIN_PULSE;
	else if ( ServoOut4 > MAX_PULSE ) m4 = MAX_PULSE;
	else m4 = ServoOut4;

	if ( ServoOut5 < MIN_PULSE ) m5 = MIN_PULSE;
	else if ( ServoOut5 > MAX_PULSE ) m5 = MAX_PULSE;
	else m5 = ServoOut5;

	if ( ServoOut6 < MIN_PULSE ) m6 = MIN_PULSE;
	else if ( ServoOut6 > MAX_PULSE ) m6 = MAX_PULSE;
	else m6 = ServoOut6;

	// T0 = 8 bit @ 8MHz, so 1 count per 125ns, max of 32us
	// T1 = 16 bit @ 1MHz, so 1 count per us, max of 65,539us or 65.5ms

	// Minimum pulse we want to make is MIN_PULSE, max is MAX_PULSE
	// So to start, let's make the base pulse.
	// First, we switch on the servo outputs
	M1 = 1;
	M2 = 1;
	M3 = 1;
	M4 = 1;
	M5 = 1;
	M6 = 1;

	// Measure period of servo rate from here to the start of the next pulse
	ServoStartTCNT1 = TCNT1;

	// Create the base pulse
	TIFR0 &= ~(1 << TOV0);		// Clear overflow
	TCNT0 = 0;					// Reset counter
	for (i=0;i<BASE_PULSE;i++)	// BASE_PULSE * 8us = 1ms
	{
		while (TCNT0 < 64);		// 8MHz * 64 = 8us
		TCNT0 -= 64;
	}

	// Now switch off the pulses as required
	TCNT0 = 0;
	for (i=904;i<2200;i+=4)		// Tweak this
	{
		while (TCNT0 < 8);		// 8MHz * 8 = 1us + overhead
		TCNT0 -= 8;				// Tweak until a 2000 pulse is close to 2ms long

		if (i>m1) M1 = 0;
		if (i>m2) M2 = 0;
		if (i>m3) M3 = 0;
		if (i>m4) M4 = 0;
		if (i>m5) M5 = 0;
		if (i>m6) M6 = 0;
	} 
	// Pulse done, now back to waiting about...
}
