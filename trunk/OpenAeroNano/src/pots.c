//***********************************************************
//* pots.c
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include <avr/io.h>
#include <stdbool.h>
#include "..\inc\io_cfg.h"
#include "..\inc\adc.h"

//************************************************************
// Prototypes
//************************************************************

void ReadGainValues(void);

//************************************************************
// Code
//************************************************************

uint16_t	GainInADC[3];	// Pot readings
uint16_t	vBat;			// Battery voltage

void ReadGainValues(void)
{
	read_adc(ROLL_POT);		// Read roll pot value 0 - 256
	GainInADC[ROLL] = ADCW >> 2;

	read_adc(PITCH_POT);	// Read pitch pot value 0 - 256
	GainInADC[PITCH] = ADCW >> 2;

	read_adc(YAW_POT);		// Read yaw pot value 0 - 256
	GainInADC[YAW] = ADCW >> 2;
}
