//***********************************************************
//* acc.c
//*
//* This fuction populates following vars:
//* accADC[] holds the raw ADC values
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include <avr/pgmspace.h>
#include <avr/io.h>
#include <stdbool.h>
#include <util/delay.h>
#include "..\inc\io_cfg.h"
#include "..\inc\adc.h"
#include "..\inc\eeprom.h"
#include "..\inc\main.h"

//************************************************************
// Prototypes
//************************************************************

void ReadAcc(void);
void CalibrateAcc(int8_t type);
void get_raw_accs(void);

//************************************************************
// Code
//************************************************************

int16_t accADC[3];				// Holds Acc ADC values
int16_t tempaccZero = 0;		// Holds Z acc in between normal and inverted cals

// Uncalibrated default values of Z middle per orientation
int16_t UncalDef[5] PROGMEM = {640, 615, 640, 640, 640}; // 764-515, 488-743, 515-764, 764-515, 764-515

// Polarity handling table
int8_t Acc_Pol[5][3] PROGMEM =  // ROLL, PITCH, YAW
{
	{1,1,1},		// Forward
	{1,-1,-1},		// Vertical
	{-1,1,-1},		// Upside down
	{-1,-1,1},		// Aft
	{1,-1,1},		// Sideways
};

void ReadAcc()					// At rest range is approx 300 - 700
{
	uint8_t i;

	get_raw_accs();				// Updates accADC[]

	for (i=0;i<3;i++)			// For all axis
	{
		// Remove offsets from acc outputs
		accADC[i] -= Config.AccZero[i];

		// Change polarity as per orientation mode
		accADC[i] *= (int8_t)pgm_read_byte(&Acc_Pol[Config.Orientation][i]);
	}

	// Use default inverse calibration value if not done yet
	if (!(Main_flags & (1 << inv_cal_done)))
	{
		Config.AccZero[YAW] = (int16_t)pgm_read_word(&UncalDef[Config.Orientation]);
	}
}

void CalibrateAcc(int8_t type)
{
	uint8_t i;
	int16_t accZero[3] = {0,0,0};	// Used for calibrating Accs on ground
	int16_t temp;
	int16_t accZeroYaw = 0;

	// Calibrate acc
	if (type == NORMAL)
	{
		// Get average zero value (over 32 readings)
		for (i=0;i<32;i++)
		{
			get_raw_accs();			// Updates accADC[]

			accZero[ROLL] += accADC[ROLL];
			accZero[PITCH] += accADC[PITCH];						
			accZero[YAW] += accADC[YAW];		

			_delay_ms(10);			// Get a better acc average over time
		}

		for (i=0;i<3;i++)			// For all axis
		{
			Config.AccZero[i] = (accZero[i] >> 5);
		}

		tempaccZero = Config.AccZero[YAW];

		Main_flags |= (1 << normal_cal_done);
		Main_flags &= ~(1 << inv_cal_done);

		Save_Config_to_EEPROM();
	}

	else
	// Calibrate inverted acc
	{
		// Only update the cal value if preceeded by a normal calibration
		if (Main_flags & (1 << normal_cal_done))
		{
			// Get average zero value (over 32 readings)
			for (i=0;i<32;i++)
			{
				get_raw_accs();					// Updates gyroADC[]
				accZeroYaw += accADC[YAW];		
				_delay_ms(10);					// Get a better acc average over time
			}

			accZeroYaw = (accZeroYaw >> 5);	// Inverted zero point

			// Test if board is actually inverted
			if (((pgm_read_byte(&Acc_Pol[Config.Orientation][YAW]) == 1) && (accZeroYaw < pgm_read_word(&UncalDef[Config.Orientation]))) || // Horizontal and 
			    ((pgm_read_byte(&Acc_Pol[Config.Orientation][YAW]) == -1) && (accZeroYaw > pgm_read_word(&UncalDef[Config.Orientation]))))  // Vertical and Upside down
			{
				// Reset zero to halfway between min and max Z
				temp = ((tempaccZero - accZeroYaw) >> 1);
				Config.AccZero[YAW] = tempaccZero - temp;

				Main_flags |= (1 << inv_cal_done);
				Main_flags &= ~(1 << normal_cal_done);

				Save_Config_to_EEPROM();
			}
		}
	}
}

void get_raw_accs(void)
{
	read_adc(AIN_X_ACC);		// Read X acc ADC5 (Pitch)
	accADC[PITCH] = ADCW;

	read_adc(AIN_Y_ACC);		// Read Y acc ADC4 (Roll)
	accADC[ROLL] = ADCW;

	read_adc(AIN_Z_ACC);		// Read Z acc ADC2 (inverted)
	accADC[YAW] = ADCW;
}
