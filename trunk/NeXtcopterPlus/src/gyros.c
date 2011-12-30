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
#include "..\inc\pots.h"
#include "..\inc\adc.h"
#include "..\inc\init.h"

//************************************************************
// Prototypes
//************************************************************

void ReadGyros(void);
void CalibrateGyros(void);

//************************************************************
// Code
//************************************************************

bool	GyroCalibrated;
int16_t gyroADC[3];						// Holds Gyro ADCs
int16_t gyroZero[3];					// Used for calibrating Gyros on ground


void ReadGyros(void)
{
	int16_t gyro;

	read_adc(ROLL_GYRO);				// Read roll gyro ADC2
	gyro = ADCW;
	gyro -= gyroZero[ROLL]; 			// Remove offset from gyro output
#ifndef MEMS_MODULE
	gyroADC[ROLL] = -gyro;				// Reverse gyro on KK boards
#else
	gyroADC[ROLL] = gyro;				// Normal gyro on MEMS module
#endif

	read_adc(PITCH_GYRO);				// Read pitch gyro ADC1
	gyro = ADCW;
	gyro -= gyroZero[PITCH]; 			// Remove offset from gyro output
#ifndef MEMS_MODULE
	gyroADC[PITCH] = -gyro;				// Reverse gyro on KK boards
#else
	gyroADC[PITCH] = gyro;				// Normal gyro on MEMS module
#endif

	read_adc(YAW_GYRO);					// Read yaw gyro ADC0
	gyro = ADCW;
	gyro -= gyroZero[YAW]; 				// Remove offset from gyro output
	gyroADC[YAW] = gyro;				// Normal gyro on all boards
	
}

void CalibrateGyros(void)
{
	uint8_t i;

	gyroZero[ROLL] 	= 0;						
	gyroZero[PITCH] = 0;	
	gyroZero[YAW] 	= 0;

	for (i=0;i<32;i++)					// Calculate average over 32 reads
	{
		read_adc(ROLL_GYRO);			// Read roll gyro ADC2
		gyroADC[ROLL] = ADCW;
		read_adc(PITCH_GYRO);			// Read pitch gyro ADC1
		gyroADC[PITCH] = ADCW;
		read_adc(YAW_GYRO);				// Read yaw gyro ADC0
		gyroADC[YAW] = ADCW;

		gyroZero[ROLL] 	+= gyroADC[ROLL];						
		gyroZero[PITCH] += gyroADC[PITCH];	
		gyroZero[YAW] 	+= gyroADC[YAW];

		_delay_ms(10);					// Get a better gyro average over time
	}

	gyroZero[ROLL] 	= (gyroZero[ROLL] >> 5);	//Divide by 32				
	gyroZero[PITCH] = (gyroZero[PITCH] >> 5);
	gyroZero[YAW] 	= (gyroZero[YAW]>> 5);

	GyroCalibrated = true;
}
