//***********************************************************
//* eeprom.c
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <stdbool.h>
#include "..\inc\io_cfg.h"
#include "..\inc\init.h"
#include "..\inc\main.h"

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

#define EEPROM_DATA_START_POS 10

//************************************************************
// Code
//************************************************************

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
	if(eeprom_read_byte((uint8_t*) EEPROM_DATA_START_POS )!=0x47)
	{
		Config.setup = 0x47;
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

void Set_EEPROM_Default_Config(void)
{
	Config.RxChannel1ZeroOffset	= 1520;	// RC input stick centers
	Config.RxChannel2ZeroOffset	= 1520;
	Config.RxChannel3ZeroOffset	= 1520;
	Config.RxChannel4ZeroOffset	= 1520;
	Config.RxChannel5ZeroOffset = 1520;
	//
	Config.AccRollZeroTrim	= 127;		// User-set ACC trim (127-127 = 0)
	Config.AccPitchZeroTrim	= 127;
	//
	Config.AccRollZero	= 578;			// Acc calibration defaults
	Config.AccPitchZero	= 585;
	//
	Config.P_mult_roll = 60;			// PID defaults
	Config.I_mult_roll = 0;
	Config.D_mult_roll = 0;
	Config.P_mult_pitch = 60;
	Config.I_mult_pitch = 0;
	Config.D_mult_pitch = 0;
	Config.P_mult_yaw = 60;
	Config.I_mult_yaw = 0;
	Config.D_mult_yaw = 0;
	Config.P_mult_glevel = 60;
	Config.D_mult_glevel = 0;
	Config.P_mult_alevel = 100;
	Config.I_mult_alevel = 0;
	//
	Config.StabMode = 0;				// Stability mode for non-CPPM RX. 0 = stability disables with THR input **, 1 = stability always on
	Config.ALMode = 1;					// Autolevel mode for non-CPPM RX. 0 = Autolevel enables with THR input, 1 = Autolevel always off **
	Config.PowerTrigger = 1080; 		// 7.33V for 2S, 10.8V for 3S are good values here

//Config.PowerTrigger = 0; 

	#if (defined (ACCELEROMETER) || defined(MEMS_MODULE))
	Config.Modes = 20; 					// LVA mode (1) = buzzer (bit 4), 
										// Stability mode (0) (bit 3),
										// Pot mode (1) = inactive (bit 2) eeprom mode
										// (bit 1) (0) unused
										// Autolevel (0) = inactive (bit 0);
	#else
	Config.Modes = 16; 					// LVA mode (1) = buzzer (bit 4), 
										// Stability mode (0) (bit 3),
										// Pot mode (0) = active (bit 2)
										// (bit 1) (0) unused
										// Autolevel (0) = inactive (bit 0);
	#endif

	Config.AutoTuneRX = 25;				// Default best fit for 19,200
	Config.AutoTuneTX = 51;				// Default best fit for 19,200
	Config.RollGyro = 0;
	Config.PitchGyro = 0;				// Default to normal (not reversed)
	Config.YawGyro = 0;
	Config.RollServo = 0;
	Config.PitchServo = 0;				// Default to normal (not reversed)
	Config.YawServo = 0;
	Config.StabChan = 8;				// Channel 8 switches stability by default

	// Failsafe positions
	Config.Failsafe_1 = 1520;			// Aileron & Left flaperon
	Config.Failsafe_2 = 1520;			// Elevator
	Config.Failsafe_3 = 900;			// Throttle
	Config.Failsafe_4 = 1520;			// Rudder
	Config.Failsafe_5 = 1520;			// Right flaperon
	Config.Failsafe_6 = 1520;			// Spare
}

void Save_Config_to_EEPROM(void)
{
	// Write to eeProm
	cli();
	eeprom_write_block_changes((const void*) &Config, (void*) EEPROM_DATA_START_POS, sizeof(CONFIG_STRUCT));	
	sei();
}
