//***********************************************************
//* acc.c
//*
//* This fuction populates following vars:
//* accADC[] holds the raw ADC values
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include <avr/io.h>
#include <stdbool.h>
#include <util/delay.h>
#include "..\inc\io_cfg.h"
#include "..\inc\pots.h"
#include "..\inc\adc.h"
#include "..\inc\init.h"
#include "..\inc\eeprom.h"

//************************************************************
// Prototypes
//************************************************************

void ReadAcc(void);
void CalibrateAcc(void);

//************************************************************
// Code
//************************************************************

int16_t accADC[2];				// Holds Acc ADC values
int16_t accZero[2];				// Used for calibrating Accs on ground

bool AccCalibrated;

void ReadAcc()					// At rest range is approx 300 - 700
{
	int16_t acc;

	read_adc(ROLL_POT);			// Read roll acc ADC3
	acc = ADCW;
	acc -= Config.AccRollZero;	// Remove offset from acc output
	accADC[ROLL] = acc;

	read_adc(PITCH_POT);		// Read pitch acc ADC4
	acc = ADCW;
	acc -= Config.AccPitchZero;	// Remove offset from acc output
	accADC[PITCH] = -acc;		// Reverse pitch response
}

void CalibrateAcc(void)
{
	uint8_t i;

	accZero[ROLL] 	= 0;		// Get average zero value (over 32 readings)			
	accZero[PITCH]	= 0;	

	for (i=0;i<32;i++)
	{
		read_adc(ROLL_POT);		// Read roll acc ADC3
		accADC[ROLL] = ADCW;

		read_adc(PITCH_POT);	// Read pitch acc ADC4
		accADC[PITCH] = ADCW;
	
		accZero[ROLL]  += accADC[ROLL];						
		accZero[PITCH] += accADC[PITCH];	

		_delay_ms(10);			// Get a better acc average over time
	}

	accZero[ROLL] = (accZero[ROLL]>> 5);						
	accZero[PITCH]= (accZero[PITCH] >> 5);
	AccCalibrated = true;

	Config.AccRollZero 	= accZero[ROLL]; // Save to eeProm
	Config.AccPitchZero = accZero[PITCH];

	Save_Config_to_EEPROM();

	LED1 = !LED1;
	_delay_ms(500);
	LED1 = !LED1;
}
