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

//***********************************************************
// ADC subroutines
//***********************************************************

void Init_ADC(void)
{
	// Digital Input Disable Register - ADC0~7 Digital Input Disable
	DIDR0 	= (1<<ADC0D)|(1<<ADC1D)|(1<<ADC2D)|(1<<ADC3D)|(1<<ADC4D)|(1<<ADC5D)|(1<<ADC6D)|(1<<ADC7D);
	
	// ADC Control and Status Register B - ADTS2:0
	ADCSRB 	= 0x00;
}

void read_adc(uint8_t channel)
{
	ADMUX	= channel;
	
	// ADEN, ADSC, ADPS1,2
	ADCSRA 	= (1<<ADEN)|(1<<ADSC)|(1<<ADPS1)|(1<<ADPS2);

	// Wait to complete. Result is in ADCW
	while (ADCSRA & (1 << ADSC));
}


