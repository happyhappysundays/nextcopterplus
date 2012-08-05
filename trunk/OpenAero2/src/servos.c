//***********************************************************
//* Servos.c
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include <avr/interrupt.h>
#include <avr/io.h>
#include "..\inc\io_cfg.h"
#include "..\inc\main.h"
#include "..\inc\isr.h"

//************************************************************
// Prototypes
//************************************************************

void output_servo_ppm(void);
void output_servo_ppm_asm1(uint16_t ServoOut1, uint16_t ServoOut2, uint16_t ServoOut3, uint16_t ServoOut4);
void output_servo_ppm_asm2(uint16_t ServoOut5, uint16_t ServoOut6, uint16_t ServoOut7, uint16_t ServoOut8);

//************************************************************
// Code
//************************************************************
uint16_t ServoOut[MAX_OUTPUTS];

void output_servo_ppm(void)
{
	uint32_t temp;
	uint16_t i;

	// Scale servo from 2500~5000 to 1000~2000
	for (i = 0; i < MAX_OUTPUTS; i++)
	{
		temp = ((ServoOut[i] << 2) / 10); 
		ServoOut[i] = temp;
	}

	// Create the output pulses if in sync with RC inputs
	if (RC_Lock) {
		output_servo_ppm_asm1(ServoOut[0], ServoOut[1], ServoOut[2], ServoOut[3]);
		output_servo_ppm_asm2(ServoOut[4], ServoOut[5], ServoOut[6], ServoOut[7]);
	}
	else if (Failsafe) // Unsynchronised so need to disable interrupts
	{
		cli();
		output_servo_ppm_asm1(ServoOut[0], ServoOut[1], ServoOut[2], ServoOut[3]);
		output_servo_ppm_asm2(ServoOut[4], ServoOut[5], ServoOut[6], ServoOut[7]);
		sei();
	}
}
