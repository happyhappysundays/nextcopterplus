//***********************************************************
//* Servos.c
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include <avr/interrupt.h>
#include <avr/io.h>
#include "..\inc\io_cfg.h"
#include "..\inc\main.h"

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
#define BASE_PULSE (864 / 8)

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
int16_t Throttle;

void output_servo_ppm(void)
{
	uint16_t i;
	static uint16_t ServoStartTCNT1;

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
	#ifdef CPPM_MODE
	THR = 1;
	#endif

	// Measure period of servo rate from here to the start of the next pulse
	ServoStartTCNT1 = TCNT1;

	// Create the base pulse
	cli();
	TIFR0 &= ~(1 << TOV0);		// Clear overflow
	TCNT0 = 0;					// Reset counter
	for (i=0;i<BASE_PULSE;i++)	// BASE_PULSE * 8us = 1ms
	{
		while (TCNT0 < 64);		// 8MHz * 64 = 8us
		TCNT0 -= 64;
	}

	// Now switch off the pulses as required
	TCNT0 = 0;
	for (i=904;i<2200;i+=5)		// Tweak this
	{
		while (TCNT0 < 8);		// 8MHz * 8 = 1us + overhead
		TCNT0 -= 8;				// Tweak until a 2000 pulse is close to 2ms long

		if (i>ServoOut1) M1 = 0;
		if (i>ServoOut2) M2 = 0;
		if (i>ServoOut3) M3 = 0;
		if (i>ServoOut4) M4 = 0;
		if (i>ServoOut5) M5 = 0;
		if (i>ServoOut6) M6 = 0;
		#ifdef CPPM_MODE
		if (i>Throttle) THR = 0;
		#endif
	} 
	sei();
	// Pulse done, now back to waiting about...
}
