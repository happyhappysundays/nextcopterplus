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
#include "..\inc\adc.h"
#include "..\inc\eeprom.h"

//************************************************************
// Prototypes
//************************************************************

void ReadAcc(void);
void CalibrateAcc(void);
void get_raw_accs(void);

//************************************************************
// Code
//************************************************************

int16_t accADC[3];				// Holds Acc ADC values
int16_t accZero[3];				// Used for calibrating Accs on ground

void ReadAcc()					// At rest range is approx 300 - 700?
{
	get_raw_accs();				// Updates accADC[]

	// Remove offsets from acc outputs
	accADC[ROLL] -= Config.AccRollZero;
	accADC[PITCH] -= Config.AccPitchZero;
	accADC[YAW] -= Config.AccZedZero;
}

void CalibrateAcc(void)
{
	uint8_t i;

	accZero[PITCH] 	= 0;		// Get average zero value (over 32 readings)			
	accZero[ROLL]	= 0;
	accZero[YAW]	= 0;	

	for (i=0;i<32;i++)
	{
		get_raw_accs();			// Updates gyroADC[]

		accZero[ROLL] += accADC[ROLL];
		accZero[PITCH] += accADC[PITCH];						
		accZero[YAW] += accADC[YAW];		

		_delay_ms(10);			// Get a better acc average over time
	}

	accZero[PITCH] = (accZero[PITCH] >> 5);						
	accZero[ROLL] = (accZero[ROLL] 	>> 5);
	accZero[YAW] = (accZero[YAW] 	>> 5);

	Config.AccPitchZero = accZero[PITCH];
	Config.AccRollZero 	= accZero[ROLL];
	Config.AccZedZero = accZero[YAW];

	Save_Config_to_EEPROM();
}

void get_raw_accs(void)
{
	read_adc(AIN_X_ACC);		// Read X acc ADC5 (Pitch)
	accADC[PITCH] = ADCW;

	read_adc(AIN_Y_ACC);		// Read pitch gyro ADC4 (Pitch)
	accADC[ROLL] = ADCW;

	read_adc(AIN_Z_ACC);		// Read yaw gyro ADC2 (Yaw)
	accADC[YAW] = ADCW;
}
