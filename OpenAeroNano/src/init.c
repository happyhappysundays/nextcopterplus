//***********************************************************
//* init.c
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include <avr/io.h>
#include <stdbool.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "..\inc\eeprom.h"
#include "..\inc\io_cfg.h"
#include "..\inc\isr.h"
#include "..\inc\rc.h"
#include "..\inc\main.h"
#include "..\inc\adc.h"
#include "..\inc\pots.h"
#include "..\inc\servos.h"
#include "..\inc\gyros.h"

//************************************************************
// Prototypes
//************************************************************

void init(void);
void CenterSticks(void);

//************************************************************
// Defines
//************************************************************

#define UC_ADC_MAX 1023				// Used to invert ADC reading. Do not change.

//************************************************************
// Code
//************************************************************

CONFIG_STRUCT Config;			// eeProm data configuration

void init(void)
{
//	uint8_t i;

	MCUCR |= (1<<PUD);			// Pull-up Disable

	RX_ROLL_DIR		= INPUT;
	RX_PITCH_DIR	= INPUT;
#ifdef CPPM_MODE
	THR_DIR			= OUTPUT;	// In CPPM mode the THR pin is an output
#else
	RX_COLL_DIR		= INPUT;	// In non-CPPM mode the THR pin is an input
#endif
	RX_YAW_DIR		= INPUT;

	GYRO_YAW_DIR	= INPUT;
	GYRO_PITCH_DIR	= INPUT;
	GYRO_ROLL_DIR	= INPUT;
	GAIN_YAW_DIR	= INPUT;
	GAIN_PITCH_DIR	= INPUT;
	GAIN_ROLL_DIR	= INPUT;

	M1_DIR			= OUTPUT;
	M2_DIR			= OUTPUT;
	M3_DIR			= OUTPUT;
	M4_DIR			= OUTPUT;
	M5_DIR			= OUTPUT;
	M6_DIR			= OUTPUT;

	LED_DIR			= OUTPUT;

	LED 		= 0;
	RX_ROLL 	= 0;
	RX_PITCH 	= 0;
#ifndef CPPM_MODE 
	RX_COLL 	= 0;
#endif
	RX_YAW		= 0;

// Conditional builds for interrupt and pin function setup
#ifdef CPPM_MODE 
	// External interrupts INT0 (CPPM input)
	EICRA = (1 << ISC01);				// Falling edge of INT0
	EIMSK = (1 << INT0);				// Enable INT0
	EIFR |= (1 << INTF0);				// Clear INT0 interrupt flags

#else // Non-CPPM mode
	// Pin change interrupt enables PCINT0 and PCINT2 (Yaw, Roll)
	PCICR |= (1 << PCIE0);				// PCINT0  to PCINT7  (PCINT0 group)		
	PCICR |= (1 << PCIE2);				// PCINT16 to PCINT23 (PCINT2 group)
	PCMSK0 |= (1 << PCINT7);			// PB7 (Rudder/Yaw pin change mask)
	PCMSK2 |= (1 << PCINT17);			// PD1 (Aileron/Roll pin change mask)

	// External interrupts INT0 and INT1 (Pitch, Collective)
	EICRA = (1 << ISC00) | (1 << ISC10);// Any change INT0, INT1 
	EIMSK = (1 << INT0) | (1 << INT1);	// External Interrupt Mask Register - enable INT0 and INT1
	EIFR |= (1 << INTF0) | (1 << INTF1);// Clear both INT0 and INT1 interrupt flags
#endif


	// Timer0 (8bit) - run @ 8MHz
	// Used to control ESC/servo pulse length
	TCCR0A = 0;							// Normal operation
	TCCR0B = (1 << CS00);				// Clk/0
	TIMSK0 = 0; 						// No interrupts

	// Timer1 (16bit) - run @ 1Mhz
	// Used to measure Rx Signals & control ESC/servo output rate
	TCCR1A = 0;
	TCCR1B = (1 << CS11);

	// Timer2 8bit - run @ 8MHz / 1024 = 7812.5KHz
	// Used to time arm/disarm intervals
	TCCR2A = 0;	
	TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20);	// /1024
	TIMSK2 = 0;
	TIFR2 = 0;
	TCNT2 = 0;							// Reset counter

	Initial_EEPROM_Config_Load();		// Loads config at start-up 

	Init_ADC();

	GyroCalibrated = true;

	RxChannelsUpdatedFlag = 0;

	RxChannel1 = Config.RxChannel1ZeroOffset;	// Prime the channels 1520;
	RxChannel2 = Config.RxChannel2ZeroOffset;	// 1520;
	RxChannel3 = Config.RxChannel3ZeroOffset;	// 1120;
	RxChannel4 = Config.RxChannel4ZeroOffset;	// 1520;

	CalibrateGyros();

	// Flash LED
	LED = 1;
	_delay_ms(150);
	LED = 0;

	sei();						// Enable global Interrupts 

	// 1.5 second delay
	_delay_ms(1500);

	ReadGainValues();

	// Config Modes (at startup)

	// Stick Centering
	if (GainInADC[YAW] > 240)	// More than 95%
	{
		CenterSticks();
		while(1); 				// Loop forever
	}

	// Servo reversing
	if (GainInADC[YAW] < 15)	// less than 5% 
	{
		// flash LED 5 times
		for (int i=0;i<5;i++)
		{
			LED = 1;
			_delay_ms(100);
			LED = 0;
			_delay_ms(100);
		}

		while(1)
		{
			RxGetChannels();

			if (RxInRoll < -200) {			// Normal(left)
				Config.RollServo = NORMAL;
				Save_Config_to_EEPROM();
				LED = 1;
			} if (RxInRoll > 200) {			// Reverse(right)
				Config.RollServo = REVERSED;
				Save_Config_to_EEPROM();
				LED = 1;
			} else if (RxInPitch < -200) {	// Normal(up)
				Config.PitchServo = NORMAL;
				Save_Config_to_EEPROM();
				LED = 1;
			} else if (RxInPitch > 200) {	// Reverse(down)
				Config.PitchServo = REVERSED;
				Save_Config_to_EEPROM();
				LED = 1;
			} else if (RxInYaw < -200) {	// Normal(left)
				Config.YawServo = NORMAL;
				Save_Config_to_EEPROM();
				LED = 1;
			} else if (RxInYaw > 200) {		// Reverse(right)
				Config.YawServo = REVERSED;
				Save_Config_to_EEPROM();
				LED = 1;
			}

			_delay_ms(50);
			LED = 0;
		}
	} //if (GainInADC[YAW] > 15)

	// Gyro direction reversing
	if (GainInADC[ROLL] < 15)	// less than 5% 
	{
		// flash LED 3 times
		for (int i=0;i<3;i++)
		{
			LED = 1;
			_delay_ms(100);
			LED = 0;
			_delay_ms(100);
		}

		while(1)
		{
			RxGetChannels();

			if (RxInRoll < -200) {			// Normal(left)
				Config.RollGyro = NORMAL;
				Save_Config_to_EEPROM();
				LED = 1;
			} if (RxInRoll > 200) {			// Reverse(right)
				Config.RollGyro = REVERSED;
				Save_Config_to_EEPROM();
				LED = 1;
			} else if (RxInPitch < -200) {	// Normal(up)
				Config.PitchGyro = NORMAL;
				Save_Config_to_EEPROM();
				LED = 1;
			} else if (RxInPitch > 200) {	// Reverse(down)
				Config.PitchGyro = REVERSED;
				Save_Config_to_EEPROM();
				LED = 1;
			} else if (RxInYaw < -200) {	// Normal(left)
				Config.YawGyro = NORMAL;
				Save_Config_to_EEPROM();
				LED = 1;
			} else if (RxInYaw > 200) {		// Reverse(right)
				Config.YawGyro = REVERSED;
				Save_Config_to_EEPROM();
				LED = 1;
			}

			_delay_ms(50);
			LED = 0;
		}
	} //if (GainInADC[ROLL] < 70)

} // init()

// Center sticks on request from GUI
// Probably a better place for this subroutine than init.c but...
void CenterSticks(void)		
{
	uint8_t i;

	// Flash LED 3 times
	for (i=0;i<3;i++)
	{
		LED = !LED;
		_delay_ms(150);
		LED = !LED;
		_delay_ms(150);
	}

	uint16_t RxChannel1ZeroOffset = 0;
	uint16_t RxChannel2ZeroOffset = 0;
	uint16_t RxChannel4ZeroOffset = 0;
	
	for (i=0;i<8;i++)
	{
		do
		{
		RxChannelsUpdatedFlag = false;
		RxChannel1ZeroOffset += RxChannel1;
		RxChannel2ZeroOffset += RxChannel2;
		RxChannel4ZeroOffset += RxChannel4;
		}
		while (RxChannelsUpdatedFlag);

		_delay_ms(100);
	}

	Config.RxChannel1ZeroOffset = RxChannel1ZeroOffset >> 3; // Divide by 8
	Config.RxChannel2ZeroOffset = RxChannel2ZeroOffset >> 3;
	Config.RxChannel3ZeroOffset = 1500;						 // Cheat for CH4 - we just need a guaranteed switch here
	Config.RxChannel4ZeroOffset = RxChannel4ZeroOffset >> 3;

	Save_Config_to_EEPROM();

	LED = !LED;
	_delay_ms(500);
	LED = !LED;
}
