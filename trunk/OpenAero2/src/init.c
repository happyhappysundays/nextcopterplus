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
#include "..\inc\mixer.h"
#include "..\inc\glcd_driver.h"
#include "..\inc\mugui.h"
#include <avr/pgmspace.h>
#include "..\inc\glcd_menu.h"
#include "..\inc\pid.h"
#include "..\inc\menu_ext.h"
#include "..\inc\imu.h"
#include "..\inc\uart.h"

extern void StackPaint(void) __attribute__ ((naked)) __attribute__ ((section (".init1")));

//************************************************************
// Defines
//************************************************************

#define GYROS_STABLE 30

//************************************************************
// Prototypes
//************************************************************

void init(void);
void init_int(void);

//************************************************************
// Code
//************************************************************

CONFIG_STRUCT Config;			// eeProm data configuration

void init(void)
{
	//***********************************************************
	// I/O setup
	//***********************************************************
	// Set port directions
	DDRA		= 0x00;		// Port A
	DDRB		= 0x0A;		// Port B
	DDRC		= 0xFF;		// Port C
	DDRD		= 0xF2;		// Port D

	MOTORS		= 0;		// Hold all PWM outputs low to stop glitches

	// Preset I/O pins
	LED1 		= 0;		// LED1 off
	LVA 		= 0; 		// LVA alarm OFF
	LCD_CSI		= 1;		// GLCD control bits
	LCD_SCL		= 1;
	LCD_RES		= 1;

	// Set/clear pull-ups (1 = set, 0 = clear)
	PINB		= 0xF5;		// Set PB pull-ups
	PIND		= 0x0C;		// Set PD pull-ups (Don't pull up RX yet)

	//***********************************************************
	// Spektrum receiver binding
	//***********************************************************

	_delay_ms(63);				// Pause while satellite wakes up	
								// and pull-ups have time to rise.
								// Tweak until bind pulses about 68ms after power-up

	// Bind as slave if ONLY button 1 pressed
	if ((PINB & 0xf0) == 0x70)
	{
		DDRD		= 0xF3;		// Switch PD0 to output
		bind_slave();
	}

	// Bind as master if ONLY button 4 pressed
	if ((PINB & 0xf0) == 0xE0)
	{
		DDRD		= 0xF3;		// Switch PD0 to output
		bind_master();
	}

	DDRD		= 0xF2;			// Reset Port D directions

	// Set/clear pull-ups (1 = set, 0 = clear)
	PIND		= 0x0D;			// Set PD pull-ups (now pull up RX as well)

	//***********************************************************
	// Timers
	//***********************************************************
	// Timer0 (8bit) - run @ 20MHz (50ns) - max 12.8us
	// Fast timer for small, precise interval timing
	TCCR0A = 0;								// Normal operation
	TCCR0B = (1 << CS00);					// Clk / 1 = 20MHz = 50ns
	TIMSK0 = 0; 							// No interrupts

	// Timer1 (16bit) - run @ 2.5MHz (400ns) - max 26.2ms
	// Used to measure Rx Signals & control ESC/servo output rate
	TCCR1A = 0;
	TCCR1B = (1 << CS11);					// Clk/8 = 2.5MHz

	// Timer2 8bit - run @ 20MHz / 1024 = 19.531kHz or 51.2us - max 13.1ms
	// Used to time arm/disarm intervals
	TCCR2A = 0;	
	TCCR2B = 0x07;							// Clk/1024 = 19.531kHz
	TIMSK2 = 0;
	TIFR2 = 0;
	TCNT2 = 0;								// Reset counter

	//***********************************************************
	// Interrupts and pin function setup
	//***********************************************************

	// Pin change interrupt enables PCINT1, PCINT2 and PCINT3 (Throttle, Aux and CPPM input)
	PCICR  = 0x0A;							// PCINT8  to PCINT15 (PCINT1 group - AUX)
											// PCINT24 to PCINT31 (PCINT3 group - THR)
	PCIFR  = 0x0F;							// Clear PCIF0 interrupt flag 
											// Clear PCIF1 interrupt flag 
											// Clear PCIF2 interrupt flag 
											// Clear PCIF3 interrupt flag 

	// External interrupts INT0 (Elevator) and INT1 (Aileron) and INT2 (Rudder)
	EICRA = 0x15;							// Any change INT0
											// Any change INT1
											// Any change INT2
	EIFR  = 0x07; 							// Clear INT0 interrupt flag (Elevator)
											// Clear INT1 interrupt flag (Aileron)
											// Clear INT2 interrupt flag (Rudder/CPPM)

	//***********************************************************
	// Start up
	//***********************************************************

	// Preset important flags
	RC_Lock = false;						
	Flight_flags &= ~(1 << Failsafe);
	Flight_flags &= ~(1 << AutoLevel);
	Flight_flags &= ~(1 << Stability);
	Main_flags |= (1 << FirstTimeIMU);

	// Initialise the GLCD
	st7565_init();
	st7565_command(CMD_DISPLAY_ON);
	st7565_command(CMD_SET_ALLPTS_NORMAL);
	st7565_set_brightness(0x26);
	st7565_command(CMD_SET_COM_NORMAL); 	// For text
	clear_buffer(buffer);
	write_buffer(buffer);

	// This delay prevents the GLCD flashing up a ghost image of old data
	_delay_ms(300);	

	// Reload default eeprom settings if all buttons are pressed 
	if ((PINB & 0xf0) == 0)
	{
		// Display reset message
		LCD_Display_Text(1,(prog_uchar*)Verdana14,40,25);
		write_buffer(buffer);
		clear_buffer(buffer);

		Set_EEPROM_Default_Config();
		Save_Config_to_EEPROM();
	}
	// Load "Config" global data structure
	else
	{
		Initial_EEPROM_Config_Load();			
	}

	// Display "Hold steady" message
	LCD_Display_Text(2,(prog_uchar*)Verdana14,18,25);
	write_buffer(buffer);
	clear_buffer(buffer);
		
	// Do startup tasks
	UpdateLimits();							// Update travel limts	
	UpdateIMUvalues();						// Update IMU factors
	Init_ADC();
	init_int();								// Intialise interrupts based on RC input mode

	// Reset I-terms (possibly unnecessary)
	IntegralGyro[ROLL] = 0;	
	IntegralGyro[PITCH] = 0;
	IntegralGyro[YAW] = 0;

	// Initialise UART
	init_uart();

	// Pause to allow model to become stable
	_delay_ms(1000);

	// Calibrate gyros, hopefully after motion minimised
	CalibrateGyros();	

	// Check to see that gyros are stable
	ReadGyros();

	if ((gyroADC[ROLL] > GYROS_STABLE) || (gyroADC[ROLL] < -GYROS_STABLE) ||
	 	(gyroADC[PITCH] > GYROS_STABLE) || (gyroADC[PITCH] < -GYROS_STABLE) ||
		(gyroADC[YAW] > GYROS_STABLE) || (gyroADC[YAW] < -GYROS_STABLE))
	{
		General_error |= (1 << SENSOR_ERROR); 	// Set sensor error bit
	}

	// Check to see that throttle is low if in CPPM mode if RC detected
	// Don't bother if in CamStab mode
	_delay_ms(100);
	if ((Config.RxMode == CPPM_MODE) && RC_Lock && (Config.CamStab == OFF))
	{
		RxGetChannels();
		if (RCinputs[THROTTLE] > 300)
		{
			General_error |= (1 << THROTTLE_HIGH); 	// Set throttle high error bit
		}
	}

	// Flash LED
	LED1 = 1;
	_delay_ms(150);
	LED1 = 0;

	// Beep that all sensors have been handled
	menu_beep(1);

} // init()


void init_int(void)
{
	cli();	// Disable interrupts

	switch (Config.RxMode)
	{
		case CPPM_MODE:
			PCMSK1 = 0;							// Disable AUX
			PCMSK3 = 0;							// Disable THR
			EIMSK = 0x04;						// Enable INT2 (Rudder/CPPM input)
			UCSR0B &= ~(1 << RXCIE0);			// Disable serial interrupt
			break;

		case PWM1:
		case PWM2:
		case PWM3:
			PCMSK1 |= (1 << PCINT8);			// PB0 (Aux pin change mask)
			PCMSK3 |= (1 << PCINT24);			// PD0 (Throttle pin change mask)
			EIMSK  = 0x07;						// Enable INT0, 1 and 2 
			UCSR0B &= ~(1 << RXCIE0);			// Disable serial interrupt
			break;

		case XTREME:
		case SBUS:
		case SPEKTRUM:
			// Disable PWM input interrupts
			PCMSK1 = 0;							// Disable AUX
			PCMSK3 = 0;							// Disable THR
			EIMSK  = 0;							// Disable INT0, 1 and 2 

			// Enable serial interrupt
			UCSR0B |= (1 << RXCIE0);
			break;

		default:
			break;	
	}	

	sei(); // Re-enable interrupts

} // init_int
