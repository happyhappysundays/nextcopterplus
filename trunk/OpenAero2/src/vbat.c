//***********************************************************
//* vbat.c
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include <avr/io.h>
#include <stdlib.h>
#include <util/delay.h>
#include "..\inc\io_cfg.h"
#include "..\inc\adc.h"

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
		
	read_adc(AIN_VBAT);				// Multiplication factor = (Display volts / 1024) / (Vbat / 11 / Vref)
	vBat = ((ADCW * 21) >> 3);		// For Vref = 2.45V, factor = 2.632 (21/8 = 2.625)

	return vBat;
}

