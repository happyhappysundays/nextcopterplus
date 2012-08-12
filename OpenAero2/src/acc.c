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
#include "..\inc\vbat.h"
#include "..\inc\adc.h"
#include "..\inc\init.h"
#include "..\inc\eeprom.h"

//************************************************************
// Prototypes
//************************************************************

void ReadAcc(void);
void AvgAcc(void);
void CalibrateAcc(void);
void AccInit(void);

//************************************************************
// Defines
//************************************************************

#define AVGLENGTH 5			// Accelerometer filter buffer length + 1 (16 new + 1 old)

//************************************************************
// Code
//************************************************************
int16_t AvgRoll;
int16_t AvgPitch;
int8_t  OldIndex;
int16_t accADC[3];					// Holds Acc ADC values
int16_t accZero[3];					// Used for calibrating Accs on ground

int16_t AvgAccRoll[AVGLENGTH];		// Circular buffer of historical accelerometer roll readings
int16_t AvgAccPitch[AVGLENGTH];		// Circular buffer of historical accelerometer pitch readings

bool AccCalibrated;

void AccInit(void)				
{
	OldIndex = 1;
	for (int i = 0; i < AVGLENGTH; i++)
	{
		AvgAccRoll[i] = 0;					// Initialise all elements of the averaging arrays
		AvgAccPitch[i] = 0;
	}
}

void ReadAcc()					// At rest range is approx 300 - 700?
{
	int16_t acc;

	read_adc(X_ACC);			// Read X acc ADC5
	acc = ADCW;
	acc -= Config.AccRollZero;	// Remove offset from acc output
	accADC[X] = -acc;			// Reverse pitch response

	read_adc(Y_ACC);			// Read Y acc ADC6
	acc = ADCW;
	acc -= Config.AccPitchZero;	// Remove offset from acc output
	accADC[Y] = -acc;			// Reverse pitch response

	read_adc(Z_ACC);			// Read Z acc ADC7
	acc = ADCW;
	acc -= Config.AccZedZero;	// Remove offset from acc output
	accADC[Z] = acc;			// 
}


void AvgAcc(void)
{
	static int8_t  AvgIndex;
	static int32_t AvgRollSum;	
	static int32_t AvgPitchSum;

	// Average accelerometer readings properly to create a genuine low-pass filter
	// Note that this has exactly the same effect as a complementary filter but with vastly less overhead.
	AvgAccRoll[AvgIndex] = accADC[Y];
	AvgAccPitch[AvgIndex] = accADC[X];	// Add new values into buffer

	//Calculate rolling average with latest 32 values
	AvgRollSum = AvgRollSum + accADC[Y] - AvgAccRoll[OldIndex]; // Add new value to sum, subtract oldest
	AvgPitchSum = AvgPitchSum + accADC[X] - AvgAccPitch[OldIndex];

	AvgRoll = ((AvgRollSum >> 2) - Config.AccRollZeroTrim); // Divide by 8 to get rolling average then adjust for acc trim
	AvgPitch = ((AvgPitchSum >> 2) - Config.AccPitchZeroTrim);

	AvgIndex ++;
	if (AvgIndex >= AVGLENGTH) AvgIndex = 0; // Wrap both indexes properly to create circular buffer
	OldIndex = AvgIndex + 1;
	if (OldIndex >= AVGLENGTH) OldIndex = 0;
}


void CalibrateAcc(void)
{
	uint8_t i;

	accZero[X] 	= 0;					// Get average zero value (over 32 readings)			
	accZero[Y]	= 0;
	accZero[Z]	= 0;	

	for (i=0;i<32;i++)
	{
		read_adc(X_ACC);
		accADC[X] = ADCW;

		read_adc(Y_ACC);
		accADC[Y] = ADCW;

		read_adc(Z_ACC);
		accADC[Z] = ADCW;
			
		accZero[X] += accADC[X];						
		accZero[Y] += accADC[Y];
		accZero[Z] += accADC[Z];		

		_delay_ms(10);					// Get a better acc average over time
	}

	accZero[X] = (accZero[X] >> 5);						
	accZero[Y] = (accZero[Y] >> 5);
	accZero[Z] = (accZero[Z] >> 5);

	Config.AccRollZero 	= accZero[X]; 	// Save to eeProm
	Config.AccPitchZero = accZero[Y];
	Config.AccZedZero = accZero[Z];

	Save_Config_to_EEPROM();
	AccCalibrated = true;
}


