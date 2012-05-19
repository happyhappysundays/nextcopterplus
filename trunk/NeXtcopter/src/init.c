//***********************************************************
//* init.c
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include <avr/io.h>
#include <avr/eeprom.h>
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
#include "..\inc\motors.h"
#include "..\inc\Gyros.h"

//************************************************************
// Prototypes
//************************************************************

void init(void);

//************************************************************
// Defines
//************************************************************

#define UC_ADC_MAX 1023			// Used to invert ADC reading. Do not change.

//************************************************************
// Code
//************************************************************

CONFIG_STRUCT Config;			// eeProm data configuration
bool	Armed;

void init(void)
{
	uint16_t i;

	MCUCR |= (1<<PUD);			// Pull-up Disable

	RX_ROLL_DIR		= INPUT;
	RX_PITCH_DIR	= INPUT;
	RX_COLL_DIR		= INPUT;
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
	LED_DIR			= OUTPUT;
	USART_TX_DIR	= OUTPUT;

	LED 		= 0;
	RX_ROLL 	= 0;
	RX_PITCH 	= 0;
	RX_COLL 	= 0;
	RX_YAW	= 0;

	// Pin change interrupt enables
	PCICR |= (1 << PCIE0);				// PCINT07		
	PCICR |= (1 << PCIE2);				// PCINT1623

	// Pin change masks
	PCMSK0 |= (1 << PCINT7);			// PB7
	PCMSK2 |= (1 << PCINT17);			// PD1
	// External interrupts
	EICRA = (1 << ISC00) | (1 << ISC10);// Any change INT0, INT1
	EIMSK = (1 << INT0) | (1 << INT1);	// External Interrupt Mask Register
	EIFR |= (1 << INTF0) | (1 << INTF1);

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
	TCCR2A = 0;	
	TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20);	// /1024
	TIMSK2 = 0;
	TIFR2 = 0;
	TCNT2 = 0;							// Reset counter

	Initial_EEPROM_Config_Load();		// Loads config at start-up 

	Init_ADC();

	GyroCalibrated = false;
	Armed = false;
	IntegralPitch = 0;	 
	IntegralRoll = 0;
	IntegralYaw = 0;

	RxChannelsUpdatedFlag = 0;

	RxChannel1 = Config.RxChannel1ZeroOffset;	// Prime the channels 1520;
	RxChannel2 = Config.RxChannel2ZeroOffset;	// 1520;
	RxChannel3 = Config.RxChannel3ZeroOffset;	// 1120;
	RxChannel4 = Config.RxChannel4ZeroOffset;	// 1520;

	// Flash LED
	LED = 1;
	_delay_ms(150);
	LED = 0;

	sei();								// Enable global Interrupts 

	// 2 second delay
	_delay_ms(1500);

	ReadGainValues();
	ReadGainValues();					// Just because KK's code does?

	// Config Modes (at startup)

	// Clear config
	if (GainInADC[PITCH] < (UC_ADC_MAX*5)/100 && 
			GainInADC[ROLL] < (UC_ADC_MAX*5)/100 &&
			GainInADC[YAW]< (UC_ADC_MAX*5)/100 )
	{
		// Flash LED twice
		for (i=0;i<2;i++)
		{
			LED = 1;
			_delay_ms(25);
			LED = 0;
			_delay_ms(25);
		}

		Set_EEPROM_Default_Config();

		LED = 1;	// Long flash
		_delay_ms(500);
		LED = 0;

		while (1); // Loop forever
	} 

	// Stick Centering
	if (GainInADC[PITCH] < (UC_ADC_MAX*5)/100)	// Less than 5%
	{
		// Flash LED 3 times
		for (i=0;i<3;i++)
		{
			LED = 1;
			_delay_ms(25);
			LED = 0;
			_delay_ms(25);
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
		Config.RxChannel3ZeroOffset = 1120;
		Config.RxChannel4ZeroOffset = RxChannel4ZeroOffset >> 3;

		// Store gyro direction to EEPROM
		Save_Config_to_EEPROM();

		LED = 1;
		_delay_ms(500);
		LED = 0;

		while(1); // Loop forever
	}

	// Placeholder for future setup features
	if (GainInADC[ROLL] < (UC_ADC_MAX*5)/100)	// Less than 5% (5/100) * 1023 = 51 
	{
		// Gyro reversing removed
	}

	// ESC throttle calibration
	if (GainInADC[YAW] < (UC_ADC_MAX*5)/100)	// Less than 5%
	{
		// Flash LED 3 times
		for (i=0;i<3;i++)
		{
			LED = 1;
			_delay_ms(25);
			LED = 0;
			_delay_ms(25);
		}

		Armed = true;	// Override so that output_motor_pwm() won't quit early

		PWM_Low_Pulse_Interval = ((1000000UL / 50) - 2000)/10;	// Set to 50Hz

		while (1)	// Loop forever
		{
			RxGetChannels();
			MotorOut1 = RxInCollective;
			MotorOut2 = RxInCollective;
			MotorOut3 = RxInCollective;
			MotorOut4 = RxInCollective;
			output_motor_ppm();	// This regulates rate at which we output signals
		}
	}
} // init()
