//***********************************************************
//* adc.c
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include <avr/io.h>
#include "..\inc\io_cfg.h"

//************************************************************
// Prototypes
//************************************************************

void Init_ADC(void);
void read_adc(uint8_t channel);

//***********************************************************
// Vertical mode:	Pitch gyro used for Yaw
//				 	Yaw gyro used for Pitch 
//					Z_ACC used for X_ACC
//					X_ACC used for Z_ACC	
//***********************************************************

int8_t ADCseqVert[8]  = {VCC, ROLL_GYRO, PITCH_GYRO, VBAT, YAW_GYRO, Y_ACC, Z_ACC, X_ACC}; // Vertical

void Init_ADC(void)
{
	DIDR0 	= 0b11111111;					// Digital Input Disable Register - ADC0~7 Digital Input Disable
	ADCSRB 	= 0b00000000; 					// ADC Control and Status Register B - ADTS2:0
}

void read_adc(uint8_t channel)
{

ADMUX = channel; //debug (horiz only)

/*	if (Config.Orientation == VERTICAL)
	{
		ADMUX = ADCseqVert[channel];		// Set channel swapped as per ADCseqVert[]
	}
	else
	{
		ADMUX = channel;					// Set channel - use Aref as reference
	}
*/
	ADCSRA 	= 0b11000110;					// ADEN, ADSC, ADPS1,2
	while (ADCSRA & (1 << ADSC));			// Wait to complete
}
