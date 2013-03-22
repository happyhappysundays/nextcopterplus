//***********************************************************
//* Servos.c
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdbool.h>
#include "..\inc\io_cfg.h"
#include "..\inc\main.h"
#include "..\inc\isr.h"

//************************************************************
// Prototypes
//************************************************************

void output_servo_ppm(void);
void output_servo_ppm_asm(volatile int16_t	*ServoOut);

//************************************************************
// Code
//************************************************************

volatile int16_t ServoOut[MAX_OUTPUTS]; // Hands off my servos!

void output_servo_ppm(void)
{
	int32_t temp;
	uint8_t i;

	// Scale servo from 2500~5000 to 1000~2000
	for (i = 0; i < MAX_OUTPUTS; i++)
	{
		temp = ((ServoOut[i] << 2) / 10); 
		ServoOut[i] = temp;
	}

	// Suppress outputs during throttle high error
	if((General_error & (1 << THROTTLE_HIGH)) == 0)
	{
		// Create the output pulses if in sync with RC inputs
		if (RC_Lock) 
		{
			output_servo_ppm_asm(&ServoOut[0]);
		}
		else if (Failsafe || (Config.CamStab == ON)) // Unsynchronised so need to disable interrupts
		{
			cli();
			output_servo_ppm_asm(&ServoOut[0]);
			sei();
		}
	}
}
