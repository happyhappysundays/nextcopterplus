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
#include "..\inc\acc.h"
#include "..\inc\uart.h"

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
bool	Armed;

void init(void)
{
	uint8_t i;

	MCUCR |= (1<<PUD);			// Pull-up Disable

	RX_ROLL_DIR		= INPUT;
	RX_PITCH_DIR	= INPUT;
	RX_COLL_DIR		= INPUT;

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

	LED_DIR			= OUTPUT;
	LCD_TX_DIR		= OUTPUT;
	LVA_DIR			= OUTPUT;

	LVA 		= 0; // LVA alarm OFF

	LED 		= 0;
	RX_ROLL 	= 0;
	RX_PITCH 	= 0;
	RX_COLL 	= 0;

// Conditional builds for interrupt and pin function setup
#ifdef CPPM_MODE 
	M6_DIR	= OUTPUT;					// M6 is motor output only
	// Proximity module only available in CPPM mode
	#ifdef PROX_MODULE
	PING_DIR = OUTPUT;					// PB7 is PING output in Proximity mode
	ECHO_DIR = INPUT;					// PD3 is ECHO input in Proximity mode
	PING = 0;
	ECHO = 0;							// Disable pull-ups

	// External interrupts INT0 (CPPM input) and INT1 (Proximity module pulse)
	EICRA = (1 << ISC01) | (1 << ISC10);// Falling edge of INT0, both edges of INT1
	EIMSK = (1 << INT0) | (1 << INT1);	// Enable INT0 and INT1
	EIFR |= (1 << INTF0) | (1 << INTF1);// Clear both INT0 and INT1 interrupt flags
	#else 
	PING_DIR = INPUT;					// Set PING_DIR to input
	ECHO_DIR = INPUT;					// Set ECHO_DIR to input

	// External interrupts INT0 (CPPM input)
	EICRA = (1 << ISC01);				// Falling edge of INT0
	EIMSK = (1 << INT0);				// Enable INT0
	EIFR |= (1 << INTF0);				// Clear INT0 interrupt flags
	#endif

#else // Non-CPPM mode
	RX_YAW_DIR	= INPUT;				// PB7 is rudder input in non-CPPM mode
	RX_YAW		= 0;					// Disable pull-ups
	// Pin change interrupt enables PCINT0 and PCINT2 (Yaw, Roll)
	PCICR |= (1 << PCIE0);				// PCINT0  to PCINT7  (PCINT0 group)		
	PCICR |= (1 << PCIE2);				// PCINT16 to PCINT23 (PCINT2 group)
	PCMSK0 |= (1 << PCINT7);			// PB7 (Rudder/Yaw pin change mask)
	PCMSK2 |= (1 << PCINT17);			// PD1 (Aileron/Roll pin change mask)

	// Pin change interrupt enables PCINT2 (Rx_Aux)
	#if defined(HEXA_COPTER) || defined(HEXA_X_COPTER)
	M6_DIR	= OUTPUT;					// M6 is motor output only
	#else								// When M6 is free, use as RX_AUX input (CH5)
	PCMSK2 |= (1 << PCINT21);			// PD5 (RX_AUX pin change mask)
	M6_DIR	= INPUT;
	RX_AUX	= INPUT;
	RX_AUX	= 0;						// Disable pull-ups
	#endif

	// External interrupts INT0 and INT1 (Pitch, Collective)
	EICRA = (1 << ISC00) | (1 << ISC10);	// Any change INT0, INT1 
	EIMSK = (1 << INT0) | (1 << INT1);		// External Interrupt Mask Register - enable INT0 and INT1
	EIFR |= (1 << INTF0) | (1 << INTF1);	// Clear both INT0 and INT1 interrupt flags
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
	init_uart();						// Enable UART with stored values

	GyroCalibrated = true;
	AccCalibrated = false;
	Armed = false;
	IntegralgPitch = 0;	 
	IntegralgRoll = 0;
	IntegralaPitch = 0;	 
	IntegralaRoll = 0;
	IntegralYaw = 0;
	AutoLevel = false;

	RxChannelsUpdatedFlag = 0;

	RxChannel1 = Config.RxChannel1ZeroOffset;	// Prime the channels 1520;
	RxChannel2 = Config.RxChannel2ZeroOffset;	// 1520;
	RxChannel3 = Config.RxChannel3ZeroOffset;	// 1120;
	RxChannel4 = Config.RxChannel4ZeroOffset;	// 1520;
	RxChannel5 = 1500;

	RollPitchRate = Config.RollPitchRate;		// Preset User-set stick sensitivity
	Yawrate = Config.Yawrate;

	CalibrateGyros();

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
	// Placeholder - no longer supported


	// Stick Centering
	if (GainInADC[YAW] > 970)	// More than 95%
	{
		CenterSticks();
		while(1); // Loop forever
	}

	// Was gyro direction reversing
	// Placeholder - no longer supported

	// ESC throttle calibration
	if (GainInADC[YAW] < (UC_ADC_MAX*5)/100)	// Less than 5%
	{
		// Flash LED 5 times
		for (i=0;i<5;i++)
		{
			LED = 1;
			_delay_ms(150);
			LED = 0;
			_delay_ms(150);
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
			MotorOut5 = RxInCollective;
			MotorOut6 = RxInCollective;
			output_motor_ppm();	// This regulates rate at which we output signals
		}
	}
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
	Config.RxChannel3ZeroOffset = 1120;
	Config.RxChannel4ZeroOffset = RxChannel4ZeroOffset >> 3;

	Save_Config_to_EEPROM();

	LED = !LED;
	_delay_ms(500);
	LED = !LED;
}
