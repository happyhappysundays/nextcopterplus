//***********************************************************
//* gyros.c
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include <avr/io.h>
#include <stdbool.h>
#include <util/delay.h>
#include "..\inc\io_cfg.h"
#include "..\inc\adc.h"

//************************************************************
// Prototypes
//************************************************************

void ReadGyros(void);
void CalibrateGyros(void);
void get_raw_gyros(void);

//************************************************************
// Code
//************************************************************

int16_t gyroADC[3];						// Holds Gyro ADCs
int16_t gyroZero[3];					// Used for calibrating Gyros on ground

void ReadGyros(void)					// Conventional orientation
{
	get_raw_gyros();					// Updates gyroADC[]

	// Remove offsets from gyro outputs
	gyroADC[ROLL] -= gyroZero[ROLL];
	gyroADC[PITCH] -= gyroZero[PITCH];
	gyroADC[YAW] -= gyroZero[YAW];
}

void CalibrateGyros(void)
{
	uint8_t i;

	gyroZero[ROLL] 	= 0;						
	gyroZero[PITCH] = 0;	
	gyroZero[YAW] 	= 0;

	for (i=0;i<32;i++)					// Calculate average over 32 reads
	{
		get_raw_gyros();				// Updates gyroADC[]

		gyroZero[ROLL] 	+= gyroADC[ROLL];						
		gyroZero[PITCH] += gyroADC[PITCH];	
		gyroZero[YAW] 	+= gyroADC[YAW];

		_delay_ms(10);					// Get a better gyro average over time
	}

	gyroZero[ROLL] 	= (gyroZero[ROLL] 	>> 5);	//Divide by 32				
	gyroZero[PITCH] = (gyroZero[PITCH] 	>> 5);
	gyroZero[YAW] 	= (gyroZero[YAW]	>> 5);
}

void get_raw_gyros(void)
{
	read_adc(AIN_Y_GYRO);				// Read roll gyro ADC1 (Roll)
	gyroADC[ROLL] = ADCW;

	read_adc(AIN_X_GYRO);				// Read pitch gyro ADC4 (Pitch)
	gyroADC[PITCH] = ADCW;

	read_adc(AIN_Z_GYRO);				// Read yaw gyro ADC2 (Yaw)
	gyroADC[YAW] = -ADCW;				// Reverse gyro sense
}
