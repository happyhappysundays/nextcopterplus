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

int16_t RollGyroLimit; 		// Ensure no overflow in gyro/gain multiplication 
int16_t PitchGyroLimit;
int16_t YawGyroLimit;

uint16_t	GainInADC[3];	// Pot readings
int8_t		GainIn[3];		// Resulting gain

void ReadGainValues(void)
{
	read_adc(ROLL_POT);							// Read roll gain ADC3
	GainInADC[ROLL] = ADCW;
	GainIn[ROLL] = GainInADC[ROLL] >> 3;		// 1024 / 8 = 0~128 range

	read_adc(PITCH_POT);						// Read pitch gain ADC4
	GainInADC[PITCH] = ADCW;
	GainIn[PITCH] = GainInADC[PITCH] >> 3;		// 1024 / 8 = 0~128 range

	read_adc(YAW_POT);							// Read yaw gain ADC5 
	GainInADC[YAW] = ADCW;
	GainIn[YAW] = GainInADC[YAW] >> 3;			// 1024 / 8 = 0~128 range
}
