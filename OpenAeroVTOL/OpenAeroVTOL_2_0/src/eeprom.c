//***********************************************************
//* eeprom.c
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include "compiledefs.h"
#include <string.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <stdbool.h>
#include "io_cfg.h"
#include <util/delay.h>
#include "mixer.h"
#include "menu_ext.h"
#include "MPU6050.h"

//************************************************************
// Prototypes
//************************************************************

void Initial_EEPROM_Config_Load(void);
void Save_Config_to_EEPROM(void);
void Set_EEPROM_Default_Config(void);
void eeprom_write_byte_changed( uint8_t * addr, uint8_t value );
void eeprom_write_block_changes( const uint8_t * src, void * dest, uint16_t size );

//************************************************************
// Defines
//************************************************************

#define EEPROM_DATA_START_POS 0	// Make sure Rolf's signature is over-written for safety
#define MAGIC_NUMBER 0x35		// eePROM signature - change for each eePROM structure change 
								// to force factory reset. 0x34 = Beta 54.2+

//************************************************************
// Code
//************************************************************

const uint8_t	JR[MAX_RC_CHANNELS] PROGMEM 	= {0,1,2,3,4,5,6,7}; 	// JR/Spektrum channel sequence (TAERG123)
const uint8_t	FUTABA[MAX_RC_CHANNELS] PROGMEM = {1,2,0,3,4,5,6,7}; 	// Futaba channel sequence (AETRGF12)

void Set_EEPROM_Default_Config(void)
{
	uint8_t i;
	
	// Clear entire Config space first
	memset(&Config.setup,0,(sizeof(Config)));

	// Set magic number / setup byte
	Config.setup = MAGIC_NUMBER;

	// Misc settings
	Config.RxMode = PWM;				// Default to PWM
	Config.PWM_Sync = GEAR;
	Config.Acc_LPF = 2;					// Acc LPF around 21Hz (5, 10, 21, 32, 44, 74, None)
	Config.Gyro_LPF = 6;				// Gyro LPF off "None" (5, 10, 21, 32, 44, 74, None)
	Config.CF_factor = 7;
	Config.FlightChan = GEAR;			// Channel GEAR switches flight mode by default
	Config.Disarm_timer = 30;			// Default to 30 seconds
	Config.Transition_P1n = 50;			// Set P1.n point to 50%

	// Servo defaults
	for (i = 0; i < MAX_RC_CHANNELS; i++)
	{
		Config.ChannelOrder[i] = pgm_read_byte(&JR[i]);
		Config.RxChannelZeroOffset[i] = 3750;
	}
		
	// Monopolar throttle is a special case. Set to -100% or -1000
	Config.RxChannelZeroOffset[THROTTLE] = 2750;

	// Preset mixers to safe values
	for (i = 0; i < MAX_OUTPUTS; i++)
	{
		Config.Channel[i].P1n_position	= 50;
		Config.Channel[i].P1_source_a 	= NOMIX;
		Config.Channel[i].P1_source_b 	= NOMIX;
		Config.Channel[i].P2_source_a 	= NOMIX;
		Config.Channel[i].P2_source_b 	= NOMIX;
		Config.min_travel[i] = -100;
		Config.max_travel[i] = 100;
	}

	// Preset simple mixing for primary channels - all models
	Config.Channel[OUT1].P1_throttle_volume = 100;
	Config.Channel[OUT2].P1_aileron_volume = 100;
	Config.Channel[OUT3].P1_elevator_volume = 100;
	Config.Channel[OUT4].P1_rudder_volume = 100;
	
	// Set up profile 1
	Config.FlightMode[P1].Roll_P_mult = 60;			// PID defaults
	Config.FlightMode[P1].A_Roll_P_mult = 5;
	Config.FlightMode[P1].Pitch_P_mult = 60;
	Config.FlightMode[P1].A_Pitch_P_mult = 5;
	Config.FlightMode[P1].Yaw_P_mult = 80;
	
	// KK2.1-specific setup
#ifdef KK21
	Config.Channel[OUT1].P2_throttle_volume = 100;
	Config.Channel[OUT2].P2_aileron_volume = 100;
	Config.Channel[OUT3].P2_elevator_volume = 100;
	Config.Channel[OUT4].P2_rudder_volume = 100;

	// Preset basic axis gyros in P2
	Config.Channel[OUT2].P2_sensors |= (1 << RollGyro);
	Config.Channel[OUT3].P2_sensors |= (1 << PitchGyro);
	Config.Channel[OUT4].P2_sensors |= (1 << YawGyro);

	Config.FlightMode[P1].Roll_I_mult = 40;
	Config.FlightMode[P1].Roll_limit = 10;
	Config.FlightMode[P1].Pitch_I_mult = 40;
	Config.FlightMode[P1].Pitch_limit = 10;
	Config.FlightMode[P1].Roll_Rate = 2;
	Config.FlightMode[P1].Pitch_Rate = 2;
	Config.FlightMode[P1].Yaw_Rate = 1;
	
	// Set up profile 2
	Config.FlightMode[P2].Roll_Rate = 2;
	Config.FlightMode[P2].Pitch_Rate = 2;
	Config.FlightMode[P2].Yaw_Rate = 1;
	
	// Set default sensor LPF
	Config.MPU6050_LPF = 2;				// 6 - 2 = 4. MPU6050's internal LPF. Values are 0x06 = 5Hz, (5)10Hz, (4)21Hz*, (3)44Hz, (2)94Hz, (1)184Hz LPF, (0)260Hz

	// Preset AccZeroNormZ for KK2.1
	Config.AccZeroNormZ		= 128;	

	// KK2.0-specific setup
#else
	// Preset acc zeros
	for (i = 0; i < NUMBEROFAXIS; i++)
	{
		Config.AccZero[i]	= 625;		// Analog inputs on KK2.0 average about 625.
	}
	
	// Preset AccZeroNormZ for KK2.0
	Config.AccZeroNormZ		= 765;
#endif

#ifdef KK2Mini
	Config.Contrast = 30;				// Contrast (KK2 Mini)
#else
	Config.Contrast = 36;				// Contrast (Everything else)
#endif	
}

void Save_Config_to_EEPROM(void)
{
	// Write to eeProm
	cli();
	eeprom_write_block_changes((const void*) &Config, (void*) EEPROM_DATA_START_POS, sizeof(CONFIG_STRUCT));	
	sei();
}

void eeprom_write_byte_changed( uint8_t * addr, uint8_t value )
{ 
	if(eeprom_read_byte(addr) != value)
	{
		eeprom_write_byte( addr, value );
	}
}

void eeprom_write_block_changes( const uint8_t * src, void * dest, uint16_t size )
{ 
	uint16_t len;

	for(len=0;len<size;len++)
	{
		eeprom_write_byte_changed( dest, *src );
		src++;
		dest++;
	}
}

void Initial_EEPROM_Config_Load(void)
{
	// Load last settings from EEPROM
	if(eeprom_read_byte((uint8_t*) EEPROM_DATA_START_POS )!= MAGIC_NUMBER)
	{
		Set_EEPROM_Default_Config();
		// Write to eeProm
		Save_Config_to_EEPROM();
	} 
	else 
	{
		// Read eeProm
		eeprom_read_block(&Config, (void*) EEPROM_DATA_START_POS, sizeof(CONFIG_STRUCT)); 
	}
}

