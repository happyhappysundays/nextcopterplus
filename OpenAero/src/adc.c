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

//************************************************************
// Code
//************************************************************

void Init_ADC(void)
{
#ifndef N6_MODE
	DIDR0 	= 0b00111111;					// Digital Input Disable Register - ADC0~5 Digital Input Disable
#else
	DIDR0 	= 0b00000111;					// Digital Input Disable Register - ADC0~2 Digital Input Disable
#endif
	ADCSRB 	= 0b00000000; 					// ADC Control and Status Register B - ADTS2:0
}

void read_adc(uint8_t channel)
{
#ifndef N6_MODE
	ADMUX 	= channel;						// Set channel - use Aref as reference
#else
	ADMUX 	= (channel | (1 << REFS0));		// Use AVCC as reference
#endif
	ADCSRA 	= 0b11000110;					// ADEN, ADSC, ADPS1,2
	while (ADCSRA & (1 << ADSC));			// Wait to complete
}
