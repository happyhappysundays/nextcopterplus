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

//************************************************************
// Prototypes
//************************************************************

void output_motor_ppm(void);

//************************************************************
// Defines
//************************************************************

// Defines output rate to ESC/Servo (Max is approx 495Hz)
#define ESC_RATE 495	// in Hz
#define PWM_LOW_PULSE_INTERVAL ((1000000 / ESC_RATE) - 2000)/10

//************************************************************
// Code
//************************************************************

uint16_t	PWM_Low_Pulse_Interval = PWM_LOW_PULSE_INTERVAL;
bool output_motor_high = false;

int16_t MotorOut1;		// Motor speed variables
int16_t MotorOut2;
int16_t MotorOut3;
int16_t MotorOut4;

void output_motor_ppm(void)
{
	uint8_t i;
	static uint16_t MotorStartTCNT1, ElapsedTCNT1;
	static uint16_t PWM_Low_Count;
	int8_t num_of_8us;	

	// Fixed value, as time elapsed is take care of in for loop
	// pulse with in 8 microseconds at 0 value, 1120 gives 1120 - 1920 range
	#define MOTOR_ADJUST 145 // 140 // (1120/8) 

	// If ESC's are high, we need to turn them off
	if (output_motor_high)
	{
		uint8_t m1,m2,m3,m4;

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

		// Now calculate the time already passed that Motors were HIGH
		ElapsedTCNT1 = (TCNT1 - MotorStartTCNT1);

		// Convert into 8us intervals
		num_of_8us = (ElapsedTCNT1 >> 3) + 1;
		
		// Keep signal on for correct time
		// MotorOutX = 100 -> 200
		// Pulse len =   1 -> 2ms

		// Start output timer
		TIFR0 &= ~(1 << TOV0);		// Clear overflow
		TCNT0 = 0;					// Reset counter

		for (i=num_of_8us;i<MOTOR_ADJUST;i++)
		{
			while (TCNT0 < 64);		// 6us @ 8MHz = 64 // 10 @ 1MHz = 10us
				TCNT0 -= 64;
		}

		for (i=0;i<220;i++)			// Motors 0->200, 1120->1920 us
		{
			while (TCNT0 < 32);		// 4us @ 8MHz = 32 // 10 @ 1MHz = 10us
				TCNT0 -= 32;

			if (i==m1) M1 = 0;
			if (i==m2) M2 = 0;
			if (i==m3) M3 = 0;
			if (i==m4) M4 = 0;
		}

		//Now wait low signal interval
		PWM_Low_Count = PWM_Low_Pulse_Interval - 1;

		TIFR0 &= ~(1 << TOV0);		// Clear overflow
		TCNT0 = 0;					// Reset counter

		while (PWM_Low_Count--)
		{
			while (TCNT0 < 80);		// 20 @ 2MHz = 10us
			TCNT0 -= 80;
		}
	} //(output_motor_high)

	if (! Armed) return;

	// Log PWM signal HIGH	
	MotorStartTCNT1 = TCNT1;
	output_motor_high = true;

	// Turn on pins
	M1 = 1;
	M2 = 1;
	M3 = 1;
	M4 = 1;
}
