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
void GetVbat(void);	

//************************************************************
// Code
//************************************************************

uint16_t	GainInADC[3];	// Pot readings
uint16_t	vBat;			// Battery voltage

void ReadGainValues(void)
{
	read_adc(YAW_POT);							// Read yaw gain ADC5 
	GainInADC[YAW] = ADCW;
}

void GetVbat(void)								// Get battery voltage (VBAT on ADC6)
{	
	read_adc(VBAT);								// Normal Vreg is 1.64/1024 = about 1.6mV per bit
												// MEMS module Vreg is 2.72V = about 2.65mV per bit
#ifndef MEMS_MODULE
	vBat = ADCW * 7 / 4;						// Vbat/11/1.6mV * 7/4 = Voltage (decimal)
#else
	vBat = ADCW * 3;							// Vbat/11/2.72mV * 3  = Voltage (decimal)
#endif
}
