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
int8_t		GainIn[3];		// Resulting gain
uint16_t	vBat;			// Battery voltage

void ReadGainValues(void)
{
	read_adc(YAW_POT);							// Read yaw gain ADC5 
	GainInADC[YAW] = ADCW;
	GainIn[YAW] = GainInADC[YAW] >> 3;			// 1024 / 8 = 0~128 range
}

void GetVbat(void)								// Get battery voltage
{	
	read_adc(VBAT);								// Read VBAT on ADC6 
												// 1024 = 1.63V
												// 1.63/1024 = about 1.6mV per bit
	vBat = ADCW * 7 / 4;						// 10V reads as 571 * 1.75 = 1000
												// NB: This will have to be adjusted if the ADC ref is adjusted away from 1.64V.
}
