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

void Process_servos(void);

//************************************************************
// Code
//************************************************************

volatile uint16_t ServoOut[MAX_OUTPUTS];

void Process_servos(void)
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
		// +/-1250 --> 2500~5000
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
		
		// Save back to servo array
		ServoOut[i] = temp;
	}
}
