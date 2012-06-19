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
#include "..\inc\i2c.h"

//************************************************************
// Prototypes
//************************************************************

void ReadGyros(void);
void CalibrateGyros(void);
void get_raw_gyros(void);
void init_i2c_gyros(void);

//************************************************************
// Code
//************************************************************

bool	GyroCalibrated;
int16_t gyroADC[3];						// Holds Gyro ADCs
int16_t gyroZero[3];					// Used for calibrating Gyros on ground

void ReadGyros(void)					// Conventional orientation
{
	get_raw_gyros();					// Updates gyroADC[]

	// Remove offsets from gyro outputs
	gyroADC[ROLL] -= gyroZero[ROLL];
	gyroADC[PITCH] -= gyroZero[PITCH];
	gyroADC[YAW] -= gyroZero[YAW];

	// Reverse gyros if requested
	gyroADC[ROLL] = (Config.RollGyro?-1:1) * gyroADC[ROLL];
	gyroADC[PITCH] = (Config.PitchGyro?-1:1) * gyroADC[PITCH];
	gyroADC[YAW] = (Config.YawGyro?-1:1) * gyroADC[YAW];

	// XMODE allows the board to be mounted square instead of with the arrow forwards
#ifdef XMODE
	int16_t	temp = gyroADC[ROLL];
	gyroADC[ROLL] = temp - gyroADC[PITCH];
	gyroADC[PITCH] = temp + gyroADC[PITCH];
#endif
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

	gyroZero[ROLL] 	= (gyroZero[ROLL] >> 5);	//Divide by 32				
	gyroZero[PITCH] = (gyroZero[PITCH] >> 5);
	gyroZero[YAW] 	= (gyroZero[YAW]>> 5);

	GyroCalibrated = true;
}

void get_raw_gyros(void)
{
// For standard KK and MEMS gyros, just read the ADC input
#ifndef N6_MODE
	read_adc(ROLL_GYRO);			// Read roll gyro ADC2
	gyroADC[ROLL] = ADCW;
	read_adc(PITCH_GYRO);			// Read pitch gyro ADC1
	gyroADC[PITCH] = ADCW;
	read_adc(YAW_GYRO);				// Read yaw gyro ADC0
	gyroADC[YAW] = ADCW;

// For i86/N6 boards, use the i2c data from the L3G4200D
#else
	readI2CbyteArray(L3G4200D_ADDRESS,0xA8,(uint8_t *)gyroADC,2*3);
	// L3G4200D gyro will return values over 200x larger than the Murata gyros + analog ADC code
	// Also note that the gyro array axis order (enumeration) is changed for N6 in io_cfg.h
	gyroADC[PITCH] = gyroADC[PITCH] >> 6;	// Seems like a waste of sensitivity, 
	gyroADC[ROLL] = gyroADC[ROLL] >> 6;		// but at least there should be no noise left...
	gyroADC[YAW] = gyroADC[YAW] >> 6;
#endif
}

#ifdef N6_MODE
void init_i2c_gyros(void)
{
	writeI2Cbyte(L3G4200D_ADDRESS, 0x20, 0x8F); // 400Hz ODR, 20Hz cut-off
	writeI2Cbyte(L3G4200D_ADDRESS, 0x23, 0x20); // 500dps
	writeI2Cbyte(L3G4200D_ADDRESS, 0x24, 0x02);
}
#endif
