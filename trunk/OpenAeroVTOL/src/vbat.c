//***********************************************************
//* vbat.c - Now looks after all analog sensors
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include "compiledefs.h"
#include <avr/io.h>
#include <stdlib.h>
#include <util/delay.h>
#include "io_cfg.h"
#include "adc.h"
#include "eeprom.h"

//************************************************************
// Prototypes
//************************************************************

uint16_t GetVbat(void);	

#ifdef AIRSPEED
uint16_t GetAirspeed(void);
void CalibrateAirspeed(void);
#endif

//************************************************************
// Code
//************************************************************

uint16_t GetVbat(void)				// Get battery voltage (VBAT on ADC3)
{
	uint16_t	vBat;				// Battery voltage
		
	read_adc(AIN_VBAT0);				// Multiplication factor = (Display volts / 1024) / (Vbat / 11 / Vref)

	// For Vref = 2.45V, Multiplication factor = 2.632
	// For Vref = 2.305V, Multiplication factor = approx 2.5
	// An input voltage of 10V will results in a value of 999.
	// This means that the number represents units of 10mV.

	vBat = ADCW;

#ifdef KK21
	// Multiply by 2.500
	// 2 + 1/2
	vBat = (vBat << 1) + (vBat >> 1); // Multiply by 2.500

#else
	// Multiply by 2.633
	// 2 + 1/2 + 1/8 + 1/128 :)
	vBat = (vBat << 1) + (vBat >> 1) + (vBat >> 3) + (vBat >> 7); // Multiply by 2.633
#endif

	return vBat;
}

//************************************************************
//* Airspeed sensor
//*
//* 1V/kPa differential. 10 m/s = 64.615kPa
//* Lowest resolution (lsb) is 2.305 / 1024 = 2.25mV = 2.25Pa = 1.8m/s = 6.7 km/h
//* Highest resolution is 2.305 kPa = 59.7m/s = 215 km/h
//*
//************************************************************

#ifdef AIRSPEED
uint16_t GetAirspeed(void)			// Get airspeed (ADC6)
{
	uint16_t	AirSpeed;
		
	// Read analog airspeed input
	read_adc(AIN_PITOT);				
	AirSpeed = ADCW;

	// Multiplication factor = 2305 / 1024 = 2.2509766
	AirSpeed = (AirSpeed << 1) + (AirSpeed >> 2) + (AirSpeed >> 10); // Multiply by 2.2509766

	return AirSpeed;
}

void CalibrateAirspeed(void)
{
	uint16_t	temp = 0;
	int8_t		i = 0;

	// Get average of 32 reads
	for (i=0; i<32; i++)
	{
		temp += GetAirspeed();		// Get airspeed sensor reading

		_delay_ms(10);				// Get a better acc average over time
	}

	// Divide by 32
	temp = (temp >> 5);
	Config.AirspeedZero = temp; 
}
#endif
