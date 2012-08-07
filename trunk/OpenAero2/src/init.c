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
#include "..\inc\vbat.h"
#include "..\inc\servos.h"
#include "..\inc\gyros.h"
#include "..\inc\acc.h"
#include "..\inc\uart.h"
#include "..\inc\mixer.h"
#include "..\inc\glcd_driver.h"
#include "..\inc\mugui.h"
#include <avr/pgmspace.h>
#include "..\inc\glcd_menu.h"
#include "..\inc\pid.h"
#include "..\inc\menu_ext.h"

//************************************************************
// Defines
//************************************************************

#define GYROS_STABLE 30

//************************************************************
// Prototypes
//************************************************************

void init(void);

//************************************************************
// Code
//************************************************************

CONFIG_STRUCT Config;			// eeProm data configuration

void init(void)
{

	//***********************************************************
	// I/O setup
	//***********************************************************

	MOTOR_DIR		= 0xFF;		// Port C - Quickly set M1 to M8 motors to output
	MOTORS			= 0;		// Hold all PWM outputs low to stop glitches

	CPPM_DIR		= INPUT;	// Port B, 2

	GYRO_YAW_DIR	= INPUT;	// Port A
	GYRO_PITCH_DIR	= INPUT;
	GYRO_ROLL_DIR	= INPUT;
	ACC_X_DIR		= INPUT;
	ACC_Y_DIR		= INPUT;
	ACC_Z_DIR		= INPUT;
	VCC_DIR			= INPUT;
	VBAT_DIR		= INPUT;

	RX_AUX_DIR		= INPUT;
	LVA_DIR			= OUTPUT;	// Port B
	RX_YAW_DIR		= INPUT;
	LED1_DIR		= OUTPUT;
	BUTTON4_DIR		= INPUT;
	BUTTON3_DIR		= INPUT;
	BUTTON2_DIR		= INPUT;
	BUTTON1_DIR		= INPUT;

	RX_COLL_DIR		= INPUT;	// Port D
	LCD_SI_DIR		= OUTPUT;
	RX_PITCH_DIR	= INPUT;
	RX_ROLL_DIR		= INPUT;
	LCD_SCL_DIR		= OUTPUT;
	LCD_CSI_DIR		= OUTPUT;
	LCD_RES_DIR		= OUTPUT;
	LCD_A0_DIR		= OUTPUT;

	// Preset I/O pins
	LED1 			= 0;		// LED1 off
	LVA 			= 0; 		// LVA alarm OFF
	LCD_CSI			= 1;
	LCD_SCL			= 1;
	LCD_RES			= 1;

	// Set/clear pull-ups (1 = set, 0 = clear)
	RX_ROLL 	= 1;
	RX_PITCH 	= 1;
	RX_YAW		= 1;
	RX_COLL 	= 1;
	RX_AUX		= 1;
	BUTTON1		= 1;
	BUTTON2		= 1;
	BUTTON3		= 1;
	BUTTON4		= 1;
	CPPM		= 1;

	//***********************************************************
	// Timers
	//***********************************************************
	// Timer0 (8bit) - run @ 20MHz (50ns) - max 12.8us
	// Fast timer for small, precise interval timing
	TCCR0A = 0;							// Normal operation
	TCCR0B = (1 << CS00);				// Clk / 1 = 20MHz = 50ns
	TIMSK0 = 0; 						// No interrupts

	// Timer1 (16bit) - run @ 2.5MHz (400ns) - max 26.2ms
	// Used to measure Rx Signals & control ESC/servo output rate
	TCCR1A = 0;
	TCCR1B = (1 << CS11);				// Clk/8 = 2.5MHz

	// Timer2 8bit - run @ 20MHz / 1024 = 19.531kHz or 51.2us - max 13.1ms
	// Used to time arm/disarm intervals
	TCCR2A = 0;	
	TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20);	// Clk/1024 = 19.531kHz
	TIMSK2 = 0;
	TIFR2 = 0;
	TCNT2 = 0;							// Reset counter

	//***********************************************************
	// Interrupts and pin function setup
	//***********************************************************
	// Pin change interrupt enables PCINT1, PCINT2 and PCINT3 (Throttle, Aux and CPPM input)

	PCICR  |= (1 << PCIE1);				// PCINT8  to PCINT15 (PCINT1 group - AUX)
	//PCICR  |= (1 << PCIE2);				// PCINT16 to PCINT23 (PCINT2 group - M8/CPPM)	
	PCICR  |= (1 << PCIE3);				// PCINT24 to PCINT31 (PCINT3 group - THR)
	PCMSK1 |= (1 << PCINT8);			// PB0 (Aux pin change mask)
	//PCMSK2 |= (1 << PCINT23);			// PC7 (M8 CPPM input)
	PCMSK3 |= (1 << PCINT24);			// PD0 (Throttle pin change mask)
	PCIFR  |= (1 << PCIF0);				// Clear PCIF0 interrupt flag 
	PCIFR  |= (1 << PCIF1);				// Clear PCIF1 interrupt flag 
	PCIFR  |= (1 << PCIF2);				// Clear PCIF2 interrupt flag 
	PCIFR  |= (1 << PCIF3);				// Clear PCIF3 interrupt flag 

	// External interrupts INT0 (Elevator) and INT1 (Aileron) and INT2 (Rudder)

	EIMSK |= (1 << INT0);				// Enable INT0 (Elevator input)
	EIMSK |= (1 << INT1);				// Enable INT1 (Aileron input)
	EIMSK |= (1 << INT2);				// Enable INT2 (Rudder/CPPM input)
	EICRA |= (1 << ISC00);				// Any change INT0
	EICRA |= (1 << ISC10);				// Any change INT1
	EICRA |= (1 << ISC20);				// Any change INT2
	EIFR  |= (1 << INTF0); 				// Clear INT0 interrupt flag (Elevator)
	EIFR  |= (1 << INTF1); 				// Clear INT1 interrupt flag (Aileron)
	EIFR  |= (1 << INTF2); 				// Clear INT2 interrupt flag (Rudder/CPPM)

	//***********************************************************

	RC_Lock = false;					// Preset important flags
	Failsafe = false;
	GyroCalibrated = false;
	AccCalibrated = false;
	AutoLevel = false;
	Stability = false;

	Initial_EEPROM_Config_Load();		// Loads config at start-up 
	Init_ADC();
	//init_uart();						// Initialise UART

	// Flash LED
	LED1 = 1;
	_delay_ms(150);
	LED1 = 0;

	// Beep
	menu_beep(1);

	// Initialise the GLCD
	st7565_init();
	st7565_command(CMD_DISPLAY_ON); 		// Check (AF)
	st7565_command(CMD_SET_ALLPTS_NORMAL);	// Check (A4)
	st7565_set_brightness(0x26);
	clear_screen();
	write_logo_buffer(buffer);				// Display logo
	_delay_ms(1000);
	clear_buffer(buffer);					// Clear
	write_buffer(buffer);
	st7565_command(CMD_SET_COM_NORMAL); 	// For text

	// Display OpenAero text
/*	LCD_Display_Text(0,(prog_uchar*)Verdana14,14,24);
	write_buffer(buffer);
	_delay_ms(1000);*/
	clear_buffer(buffer);					// Clear

	// Reset I-terms
	IntegralaPitch = 0;	 
	IntegralaRoll = 0;

	// Calibrate gyros, hopefully after motion minimised
	CalibrateGyros();			


	//***********************************************************
	//* Reload eeprom settings if all buttons are pressed 
	//***********************************************************

	if ((PINB & 0xf0) == 0)
	{

		LCD_Display_Text(1,(prog_uchar*)Verdana14,15,10);
		LCD_Display_Text(2,(prog_uchar*)Verdana14,31,30);

		write_buffer(buffer);
		clear_buffer(buffer);				// Clear
		_delay_ms(1000);
		Set_EEPROM_Default_Config();
		Save_Config_to_EEPROM();
		write_buffer(buffer);
	}

	//***********************************************************

	sei();									// Enable global Interrupts 

	// Check to see that gyros are stable
	ReadGyros();

	if ((gyroADC[ROLL] > GYROS_STABLE) || (gyroADC[ROLL] < -GYROS_STABLE) ||
	 	(gyroADC[PITCH] > GYROS_STABLE) || (gyroADC[PITCH] < -GYROS_STABLE) ||
		(gyroADC[YAW] > GYROS_STABLE) || (gyroADC[YAW] < -GYROS_STABLE))
	{
		General_error |= (1 << SENSOR_ERROR); 	// Set sensor error bit
	}

	// Check to see that throttle is low if in CPPM mode
	_delay_ms(100);
	if ((Config.RxMode == CPPM_MODE) && RC_Lock)
	{
		RxGetChannels();
		if (RCinputs[THROTTLE] > 100)
		{
			General_error |= (1 << THROTTLE_HIGH); 	// Set throttle high error bit
		}
	}

} // init()
