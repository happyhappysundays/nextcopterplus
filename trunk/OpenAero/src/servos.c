//***********************************************************
//* Servos.c
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdbool.h>
#include "..\inc\io_cfg.h"
#include "..\inc\init.h"
#include "..\inc\main.h"
#include "..\inc\isr.h"

//************************************************************
// Prototypes
//************************************************************

void output_servo_ppm(void);
#ifdef THREE_CHANNEL
void output_servo_ppm_asm(uint16_t ServoOut1, uint16_t ServoOut2, uint16_t ServoOut4);
#else
void output_servo_ppm_asm(uint8_t mot1, uint8_t mot2, uint8_t mot3, uint8_t mot4, uint8_t mot5, uint8_t mot6);
#endif

//************************************************************
// Code
//************************************************************
uint16_t ServoOut1;
uint16_t ServoOut2;
uint16_t ServoOut4;
uint16_t ServoOut5;
uint16_t ServoOut6;
uint16_t Throttle;

uint8_t	Servo1;
uint8_t	Servo2;
uint8_t	Servo4;
uint8_t	Servo5;
uint8_t	Servo6;
uint8_t	Servo_thr;

void output_servo_ppm(void)
{

#ifdef THREE_CHANNEL
	ServoOut1 = ServoOut1 >> 1; // Scale servo from 1000~2000 to 500~1000
	ServoOut2 = ServoOut2 >> 1;
	ServoOut4 = ServoOut4 >> 1;
	//
	// Create the output pulses if in sync with RC inputs
	if (RC_Lock) {
		output_servo_ppm_asm(ServoOut1, ServoOut2, ServoOut4);
	}
	else if (Failsafe) // Unsynchronised so need to disable interrupts
	{
		cli();
		output_servo_ppm_asm(ServoOut1, ServoOut2, ServoOut4);
		sei();
	}
#else
	Servo1 = ((ServoOut1 - 1000) >> 2); // Scale servo from 1000~2000 to 0~250
	Servo2 = ((ServoOut2 - 1000) >> 2);
	Servo4 = ((ServoOut4 - 1000) >> 2);
	Servo5 = ((ServoOut5 - 1000) >> 2);
	Servo6 = ((ServoOut6 - 1000) >> 2);
	Servo_thr = ((Throttle  - 1000) >> 2);

	//
	// Debug: Force values to positive (should not be needed as already limited by FC_main.c)
	//
	if (Servo1 <= 0) Servo1 = 1;
	if (Servo2 <= 0) Servo2 = 1;
	if (Servo4 <= 0) Servo4 = 1;
	if (Servo5 <= 0) Servo5 = 1;
	if (Servo6 <= 0) Servo6 = 1;
	if (Servo_thr <= 0) Servo_thr = 1;
	//
	// Create the output pulses if in sync with RC inputs
	if (RC_Lock) {
		output_servo_ppm_asm(Servo1, Servo2, Servo4, Servo5, Servo6, Servo_thr);
	}
	else if (Failsafe) // Unsynchronised so need to disable interrupts
	{
		cli();
		output_servo_ppm_asm(Servo1, Servo2, Servo4, Servo5, Servo6, Servo_thr);
		sei();
	}
#endif

}
