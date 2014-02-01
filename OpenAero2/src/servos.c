//***********************************************************
//* Servos.c
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdbool.h>
#include "io_cfg.h"
#include "main.h"
#include "isr.h"

//************************************************************
// Prototypes
//************************************************************

void output_servo_ppm(void);
void output_servo_ppm_asm(volatile uint16_t	*ServoOut);

//************************************************************
// Code
//************************************************************

volatile uint16_t ServoOut[MAX_OUTPUTS]; // Hands off my servos!

void output_servo_ppm(void)
{
	uint32_t temp;
	uint8_t i;

	// Scale servo from 2500~5000 to 1000~2000
	for (i = 0; i < MAX_OUTPUTS; i++)
	{
		temp = ServoOut[i];					// Promote to 32 bits
		temp = ((temp << 2) + 5) / 10; 		// Round and convert
		ServoOut[i] = (uint16_t)temp;
	}

	// Suppress outputs during throttle high error
	if((General_error & (1 << THROTTLE_HIGH)) == 0)
	{
		// Create unsynchronised output pulses if needed
		if ((Flight_flags & (1 << Failsafe)) || (Config.CamStab == ON))
		{
			cli();
			output_servo_ppm_asm(&ServoOut[0]);
			sei();
		}
		// Create synchronised output pulses if in sync with RC inputs
		else
		{
			// Pass address of ServoOut array
			output_servo_ppm_asm(&ServoOut[0]);
		}
	}
}
