//***********************************************************
//* adc.c
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include "..\inc\compiledefs.h"
#include <avr/io.h>
#include <avr/pgmspace.h>
#include "..\inc\io_cfg.h"

//************************************************************
// Prototypes
//************************************************************

void Init_ADC(void);
void read_adc(uint8_t channel);

//***********************************************************
// Remap analog sensors for vertical and sideways modes
//***********************************************************
//											Vcc			Roll		Yaw			Vbat	Pitch		PitchA		RollA		ZA
//									{AIN_VCC = 0, AIN_Y_GYRO, AIN_Z_GYRO, AIN_VBAT, AIN_X_GYRO, AIN_X_ACC, AIN_Y_ACC, AIN_Z_ACC}; // Normal/UD definition
const int8_t ADCseqVert[8] PROGMEM = 	{AIN_VCC, AIN_X_GYRO, AIN_Y_GYRO, AIN_VBAT, AIN_Z_GYRO, AIN_Y_ACC, AIN_Z_ACC, AIN_X_ACC}; // Vertical
const int8_t ADCseqSide[8] PROGMEM = 	{AIN_VCC, AIN_X_GYRO, AIN_Z_GYRO, AIN_VBAT, AIN_Y_GYRO, AIN_Y_ACC, AIN_X_ACC, AIN_Z_ACC}; // Sideways

//***********************************************************
// ROLL, PITCH, YAW mapping for vertical and sideways modes
//***********************************************************
#ifdef KK21
const int8_t ACC_RPY_Order[5][3] PROGMEM = 
{
	{PITCH, ROLL, YAW}, // Normal
	{YAW, PITCH, ROLL}, // Vertical
	{PITCH, ROLL, YAW},	// Upside down
	{PITCH, ROLL, YAW},	// Aft
	{ROLL, PITCH, YAW}	// Sideways
};
const int8_t Gyro_RPY_Order[5][3] PROGMEM = 
{
	{ROLL, PITCH, YAW}, // Normal
	{PITCH, YAW, ROLL}, // Vertical
	{ROLL, PITCH, YAW},	// Upside down
	{ROLL, PITCH, YAW},	// Aft
	{PITCH, ROLL, YAW}	// Sideways
};
#endif

//***********************************************************
// ADC subroutines
//***********************************************************

void Init_ADC(void)
{
	DIDR0 	= 0b11111111;					// Digital Input Disable Register - ADC0~7 Digital Input Disable
	ADCSRB 	= 0b00000000; 					// ADC Control and Status Register B - ADTS2:0
}

void read_adc(uint8_t channel)
{
	if (Config.Orientation == VERTICAL)
	{
		ADMUX = (int8_t)pgm_read_byte(&ADCseqVert[channel]);	// Set channel swapped as per ADCseqVert[]
	}
	
	else if (Config.Orientation == SIDEWAYS)
	{
		ADMUX = (int8_t)pgm_read_byte(&ADCseqSide[channel]);	// Set channel swapped as per ADCseqSide[]
	}

	else
	{
		ADMUX = channel;					// Set channel - use Aref as reference
	}

	ADCSRA 	= 0b11000110;					// ADEN, ADSC, ADPS1,2
	while (ADCSRA & (1 << ADSC));			// Wait to complete. Result is in ADCW
}
