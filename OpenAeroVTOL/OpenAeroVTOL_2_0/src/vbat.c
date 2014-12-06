//***********************************************************
//* vbat.c - Now looks after all analog sensors
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include "compiledefs.h"
#include <avr/io.h>
#include <stdlib.h>
#include <util/delay.h>
#include "io_cfg.h"
#include "adc.h"
#include "eeprom.h"

//************************************************************
// Prototypes
//************************************************************

uint16_t GetVbat(void);	

//************************************************************
// Code
//************************************************************

uint16_t GetVbat(void)				// Get battery voltage (VBAT on ADC3)
{
	uint16_t	vBat;				// Battery voltage
		
	read_adc(AIN_VBAT0);				// Multiplication factor = (Display volts / 1024) / (Vbat / 11 / Vref)

	// For Vref = 2.45V, Multiplication factor = 2.632
	// For Vref = 2.305V, Multiplication factor = approx 2.5
	// An input voltage of 10V will results in a value of 999.
	// This means that the number represents units of 10mV.

	vBat = ADCW;

#ifdef KK21
	// Multiply by 2.500
	// 2 + 1/2
	vBat = (vBat << 1) + (vBat >> 1); // Multiply by 2.500

#else
	// Multiply by 2.633
	// 2 + 1/2 + 1/8 + 1/128 :)
	vBat = (vBat << 1) + (vBat >> 1) + (vBat >> 3) + (vBat >> 7); // Multiply by 2.633
#endif

	return vBat;
}