//***********************************************************
//* Servos.c
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdbool.h>
#include "typedefs.h"
#include "io_cfg.h"
#include "main.h"
#include "isr.h"
#include "rc.h"

//************************************************************
// Prototypes
//************************************************************

void output_servo_ppm(uint8_t ServoFlag);
void output_servo_ppm_asm(volatile uint16_t *ServoOut, uint8_t ServoFlag);

//************************************************************
// Code
//************************************************************

volatile uint16_t ServoOut[MAX_OUTPUTS];

void output_servo_ppm(uint8_t ServoFlag)
{
	int16_t temp = 0;
	uint8_t i = 0;
	
	// Formerly in UpdateServos()
	// Update servos from the mixer Config.Channel[i].P1_value data, add offsets and enforce travel limits
	for (i = 0; i < MAX_OUTPUTS; i++)
	{
		// Servo reverse and trim
		temp = Config.Channel[i].value;

		// Reverse this channel
		if (Config.Servo_reverse[i] == ON)
		{
			temp = -temp;
		}

		// Add offset value to restore to system compatible value
		temp += 3750;

		// Enforce min, max travel limits
		if (temp > Config.Limits[i].maximum)
		{
			temp = Config.Limits[i].maximum;
		}

		else if (temp < Config.Limits[i].minimum)
		{
			temp = Config.Limits[i].minimum;
		}

		// Check for motor marker and ignore if set
		if (Config.Channel[i].Motor_marker != MOTOR)
		{
			// Scale servo from 2500~5000 to 875~2125
			temp = ((temp - 3750) >> 1) + 1500;
		}
		else
		{
			// Scale motor from 2500~5000 to 1000~2000
			temp = ((temp << 2) + 5) / 10; 	// Round and convert
		}
		
		// Save back to servo array
		ServoOut[i] = temp;
	}

	// Suppress outputs during throttle high error -  debug (might disable outputs in failsafe)
	if((General_error & (1 << THROTTLE_HIGH)) == 0)
	{
		// Create unsynchronised output pulses if needed for camstab or failsafe
		if ((Flight_flags & (1 << Failsafe)) || (Config.CamStab == ON))
		{
			cli();
			output_servo_ppm_asm(&ServoOut[0], ServoFlag);
			sei();
		}
		
		// Create synchronised output pulses if in sync with RC inputs
		else
		{
			// Reset JitterFlag immediately before PWM generation
			JitterFlag = false;

			// We now care about interrupts
			JitterGate = true;

			// Pass address of ServoOut array
			output_servo_ppm_asm(&ServoOut[0], ServoFlag);

			// We no longer care about interrupts
			JitterGate = false;
		}
	}
}
