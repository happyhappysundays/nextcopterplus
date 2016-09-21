//***********************************************************
//* init.c
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include "compiledefs.h"
#include <avr/io.h>
#include <avr/eeprom.h>
#include <stdbool.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>
#include "eeprom.h"
#include "io_cfg.h"
#include "isr.h"
#include "rc.h"
#include "main.h"
#include "adc.h"
#include "vbat.h"
#include "servos.h"
#include "gyros.h"
#include "acc.h"
#include "mixer.h"
#include "glcd_driver.h"
#include "mugui.h"
#include <avr/pgmspace.h>
#include "glcd_menu.h"
#include "pid.h"
#include "menu_ext.h"
#include "imu.h"
#include "uart.h"
#include "i2cmaster.h"
#include "i2c.h"
#include "MPU6050.h"
#include "glcd_driver.h"
#include <avr/wdt.h>

//************************************************************
// Prototypes
//************************************************************

void init(void);

// WDT reset prototype. Placed before main() in code to prevent wdt re-firing
// Also allows inspection of the MCUSR after a reset
void wdt_init(void) __attribute__((naked)) __attribute__((section(".init3")));

void wdt_init(void)
{
	MCUSR = 0;
	wdt_disable();
	return;
}

//************************************************************
// Code
//************************************************************

CONFIG_STRUCT Config;			// eeProm data configuration
uint16_t SystemVoltage = 0;		// Initial voltage measured.

void init(void)
{
	uint8_t i;
	uint16_t j;
	bool	updated;
	uint8_t ServoFlag = 0;
	
	//***********************************************************
	// I/O setup
	//***********************************************************
	// Set port directions
	DDRA		= 0x30;		// Port A
	DDRB		= 0x0A;		// Port B
	DDRC		= 0xFC;		// Port C
	DDRD		= 0xF2;		// Port D

	// Hold all PWM outputs low to stop glitches
	// M5 and M6 are on PortA for KK2.1
	MOTORS		= 0;
	M5			= 0;
	M6			= 0;

	// Preset I/O pins
	LED1 		= 0;		// LED1 off
	LVA 		= 0; 		// LVA alarm OFF
	LCD_SCL		= 1;		// GLCD clock high

	// Set/clear pull-ups (1 = set, 0 = clear)
	PINB		= 0xF5;		// Set PB pull-ups
	PIND		= 0x0C;		// Set PD pull-ups (Don't pull up RX yet)

	//***********************************************************
	// Spektrum receiver binding. Must be done immediately on power-up
	// 
	// 3 low pulses: DSM2 1024/22ms
	// 5 low pulses: DSM2 2048/11ms
	// 7 low pulses: DSMX 1024/22ms
	// 9 low pulses: DSMX 2048/11ms
	//***********************************************************

	PIND	= 0x0C;			// Release RX pull up on PD0
	_delay_ms(63);			// Pause while satellite wakes up
							// and pull-ups have time to rise.
							// Tweak until bind pulses about 68ms after power-up
				
	// Bind as master if any single button pressed.
	// NB: Have to wait until the button pull-ups rise before testing for a button press.
	// Button 1
	if ((PINB & 0xf0) == 0x70)
	{
		DDRD	= 0xF3;		// Switch PD0 to output
		bind_master(3);
		
	}
	// Button 2	
	if ((PINB & 0xf0) == 0xb0)
	{
		DDRD	= 0xF3;		// Switch PD0 to output
		bind_master(5);
	}
	// Button 3	
	if ((PINB & 0xf0) == 0xd0)
	{
		DDRD	= 0xF3;		// Switch PD0 to output
		bind_master(7);
	}
	
	// Button 4
	if ((PINB & 0xf0) == 0xE0)
	{
		DDRD	= 0xF3;		// Switch PD0 to output
		bind_master(9);
	}
	
	DDRD	= 0xF2;			// Reset Port D directions
	PIND	= 0x0D;			// Set PD pull-ups (now pull up RX as well)

	//***********************************************************
	// Timers
	//***********************************************************

	// Timer0 (8bit) - run @ 20MHz / 1024 = 19.531kHz or 51.2us - max 13.1ms
	// Slow timer to extend Timer 1
	TCCR0A = 0;								// Normal operation
	TCCR0B = 0x05;							// Clk / 1024 = 19.531kHz or 51.2us - max 13.1ms
	TIMSK0 |= (1 << TOIE0);					// Enable interrupts
	TCNT0 = 0;								// Reset counter
	
	// Timer1 (16bit) - run @ 2.5MHz (400ns) - max 26.2ms
	// Used to measure Rx Signals & control ESC/servo output rate
	TCCR1A = 0;
	TCCR1B |= (1 << CS11);					// Clk/8 = 2.5MHz

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

	// Pin change interrupt enables PCINT1, PCINT2 and PCINT3 (Throttle, AUX and CPPM input)
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
	Interrupted = false;						

	// Load EEPROM settings
	updated = Initial_EEPROM_Config_Load(); // Config now contains valid values

	//***********************************************************
	// RX channel defaults for when no RC connected
	// Not doing this can result in the FC trying (unsuccessfully) to arm
	// and makes entry into the menus very hard
	//***********************************************************

	for (i = 0; i < MAX_RC_CHANNELS; i++)
	{
		RxChannel[i] = 3750;
	}
	
	RxChannel[THROTTLE] = 2500; // Min throttle

	//***********************************************************
	// GLCD initialisation
	//***********************************************************

	// Initialise the GLCD
	st7565_init();

	// Make sure the LCD is blank without clearing buffer (and so no logo)
	clear_screen();

	//***********************************************************
	// ESC calibration
	//***********************************************************
	
	// Calibrate ESCs if ONLY buttons 1 and 4 pressed
	if ((BUTTON1 == 0) && (BUTTON4 == 0))
	{
		// Display calibrating message
		st7565_command(CMD_SET_COM_NORMAL); 	// For text (not for logo)
		clear_buffer(buffer);
		LCD_Display_Text(59,(const unsigned char*)Verdana14,10,25);
		write_buffer(buffer);
		clear_buffer(buffer);
				
		// For each output
		for (i = 0; i < MAX_OUTPUTS; i++)
		{
			// Check for motor marker
			if (Config.Channel[i].Motor_marker == MOTOR)
			{
				// Set output to maximum pulse width
				ServoOut[i] = MOTOR_100;
			}
			else
			{
				ServoOut[i] = SERVO_CENTER;
			}
		}
					
		// Output HIGH pulse (1.9ms) until buttons released
		while ((PINB & 0xf0) == 0x60)
		{
			// Pass address of ServoOut array and select all outputs
			output_servo_ppm_asm(&ServoOut[0], 0xFF);

			// Loop rate = 20ms (50Hz)
			_delay_ms(20);			
		}

		// Output LOW pulse (1.1ms) after buttons released
		// For each output
		for (i = 0; i < MAX_OUTPUTS; i++)
		{
			// Check for motor marker
			if (Config.Channel[i].Motor_marker == MOTOR)
			{
				// Set output to maximum pulse width
				ServoOut[i] = MOTOR_0;
			}
		}		

		// Loop forever here
		while(1)
		{
			// Pass address of ServoOut array and select all outputs
			output_servo_ppm_asm(&ServoOut[0], 0xFF);

			// Loop rate = 20ms (50Hz)
			_delay_ms(20);			
		}
	}

	//***********************************************************
	// Reset EEPROM settings
	//***********************************************************

	// This delay prevents the GLCD flashing up a ghost image of old data
	_delay_ms(300); // Turns out this only happens with programmer attached...

	// Reload default eeprom settings if middle two buttons are pressed
	if ((BUTTON2 == 0) && (BUTTON3 == 0))
	{
		// Display reset message
		st7565_command(CMD_SET_COM_NORMAL); 	// For text (not for logo)
		clear_buffer(buffer);
		LCD_Display_Text(262,(const unsigned char*)Verdana14,40,25); // "Reset"
		write_buffer(buffer);
		clear_buffer(buffer);
		
		// Reset EEPROM settings
		Set_EEPROM_Default_Config();

		// Save settings
		Save_Config_to_EEPROM();

		// Set contrast to the default value
		st7565_set_brightness(Config.Contrast);

		_delay_ms(500);		// Save is now too fast to show the "Reset" text long enough
	}

	// Display message in place of logo when updating eeprom structure
	if (updated)
	{
		st7565_command(CMD_SET_COM_NORMAL); 	// For text (not for logo)
		clear_buffer(buffer);
		LCD_Display_Text(259,(const unsigned char*)Verdana14,30,13); // "Updating"
		LCD_Display_Text(260,(const unsigned char*)Verdana14,33,37); // "settings"
		write_buffer(buffer);
		clear_buffer(buffer);
		_delay_ms(1000);
	}
	else
	{
		// Write logo from buffer
		write_buffer(buffer);
		_delay_ms(1000);
	}
	
	clear_buffer(buffer);
	write_buffer(buffer);
		
	st7565_init(); // Seems necessary for KK2 mini

	//***********************************************************
	// i2c init
	//***********************************************************	

	i2c_init();
	init_i2c_gyros();
	init_i2c_accs();

	//***********************************************************
	// Remaining init tasks
	//***********************************************************

	// Display "Hold steady" message
	clear_buffer(buffer);
	st7565_command(CMD_SET_COM_NORMAL); 	// For text (not for logo)
	LCD_Display_Text(263,(const unsigned char*)Verdana14,18,25);	// "Hold steady"
	write_buffer(buffer);
	clear_buffer(buffer);
		
	// Do startup tasks
	Init_ADC();
	init_int();								// Initialise interrupts based on RC input mode
	init_uart();							// Initialise UART

	// Initial gyro calibration
	if (!CalibrateGyrosSlow())
	{
		clear_buffer(buffer);
		LCD_Display_Text(61,(const unsigned char*)Verdana14,25,25); // "Cal. failed"
		write_buffer(buffer);
		_delay_ms(1000);
			
		// Reset
		cli();
		wdt_enable(WDTO_15MS);				// Watchdog on, 15ms
		while(1);							// Wait for reboot
	}

	// Update voltage detection
	SystemVoltage = GetVbat();				// Check power-up battery voltage
	UpdateLimits();							// Update travel and trigger limits

	// Disarm on start-up if Armed setting is ARMABLE
	if (Config.ArmMode == ARMABLE)
	{
		General_error |= (1 << DISARMED); 	// Set disarmed bit
	}

	// Check to see that throttle is low if RC detected
	if (Interrupted)
	{
		RxGetChannels();
		if (MonopolarThrottle > THROTTLEIDLE) // THROTTLEIDLE = 50
		{
			General_error |= (1 << THROTTLE_HIGH); 	// Set throttle high error bit
		}
	}

	// Reset IMU
	reset_IMU();

	// Beep that init is complete
	// Check buzzer mode first
	if (Config.Buzzer == ON)
	{
		LVA = 1;
		_delay_ms(25);
		LVA = 0;
	}

	//***********************************************************
	// Experimental PWM output code
	//***********************************************************

	cli();									// Disable interrupts

	ServoFlag = 0;							// Reset servo flag

	// For each output
	for (i = 0; i < MAX_OUTPUTS; i++)
	{
		// Check for motor marker
		if (Config.Channel[i].Motor_marker == MOTOR)
		{
			// Set output to 1ms pulse width
			ServoOut[i] = MOTORMIN;
			
			// Mark motor outputs
			ServoFlag |= (1 << i);
		}
	}

	for (j = 0; j < 249; j++)
	{
		// Pass address of ServoOut array and select only motor outputs
		output_servo_ppm_asm(&ServoOut[0], ServoFlag);
	}
	
	sei();									// Enable interrupts
	
	//***********************************************************	

	#ifdef ERROR_LOG
	// If restart, log it as such
	add_log(REBOOT);
	#endif
	
} // init()

