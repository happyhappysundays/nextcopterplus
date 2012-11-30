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
void CalibrateInvAcc(void);
void get_raw_accs(void);

//************************************************************
// Defines
//************************************************************

#define Z_UNCAL_DEFAULT 643 	// Uncalibrated default

//************************************************************
// Code
//************************************************************

uint16_t accADC[3];				// Holds Acc ADC values
uint16_t accZero[3];			// Used for calibrating Accs on ground
bool	inv_cal_done;
bool	normal_cal_done;

void ReadAcc()					// At rest range is approx 300 - 700?
{
	get_raw_accs();				// Updates accADC[]

	// Remove offsets from acc outputs
	accADC[ROLL] -= Config.AccRollZero;
	accADC[PITCH] -= Config.AccPitchZero;

	// Only update official Z cal if it has been done (properly)
	if (inv_cal_done)
	{
		accADC[YAW] -= Config.AccZedZero;
	}
	else
	{
		accADC[YAW] -= Z_UNCAL_DEFAULT;
	}
}

void CalibrateAcc(void)
{
	uint8_t i;

	accZero[PITCH] 	= 0;		
	accZero[ROLL]	= 0;
	accZero[YAW]	= 0;	

	// Get average zero value (over 32 readings)
	for (i=0;i<32;i++)
	{
		get_raw_accs();			// Updates accADC[]

		accZero[ROLL] += accADC[ROLL];
		accZero[PITCH] += accADC[PITCH];						
		accZero[YAW] += accADC[YAW];		

		_delay_ms(10);			// Get a better acc average over time
	}

	Config.AccPitchZero = (accZero[PITCH] >> 5);						
	Config.AccRollZero = (accZero[ROLL] >> 5);
	Config.AccZedZero = (accZero[YAW] 	>> 5);

	normal_cal_done = true;
	inv_cal_done = false;

	Save_Config_to_EEPROM();
}

void CalibrateInvAcc(void)
{
	uint8_t i;
	int16_t temp;

	// Only update the cal value if preceeded by a normal calibration
	if (normal_cal_done)
	{
		accZero[YAW] = 0;

		// Get average zero value (over 32 readings)
		for (i=0;i<32;i++)
		{
			get_raw_accs();					// Updates gyroADC[]
			accZero[YAW] += accADC[YAW];		
			_delay_ms(10);					// Get a better acc average over time
		}

		accZero[YAW] = (accZero[YAW] >> 5);	// Inverted zero point

		// Test if board is actually inverted
		if (accZero[YAW] < Z_UNCAL_DEFAULT)
		{
			// Reset zero to halfway between min and max Z
			temp = ((Config.AccZedZero - accZero[YAW]) >> 1);
			Config.AccZedZero -= temp;

			inv_cal_done = true;
			normal_cal_done = false;

			Save_Config_to_EEPROM();
		}
	}
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
