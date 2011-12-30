//***********************************************************
//* adc.c
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include <avr/io.h>

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
	DIDR0 	= 0b00111111;					// Digital Input Disable Register - ADC50 Digital Input Disable
	ADCSRB 	= 0b00000000; 					// ADC Control and Status Register B - ADTS2:0
}

void read_adc(uint8_t channel)
{
	ADMUX 	= channel;						// Set channel
	ADCSRA 	= 0b11000110;					// ADEN, ADSC, ADPS1,2
	while (ADCSRA & (1 << ADSC));			// Wait to complete
}
