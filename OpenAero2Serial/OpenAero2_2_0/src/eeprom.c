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
void eeprom_write_block_changes(uint8_t *src, uint8_t *dest, uint16_t size);

//************************************************************
// Defines
//************************************************************

#define EEPROM_DATA_START_POS 0	// Make sure Rolf's signature is over-written for safety
#define MAGIC_NUMBER 0x51		// eePROM signature - change for each eePROM structure change 0x50 = V1.0 a3+
								// to force factory reset

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
	
	// Servo and mixer defaults
	for (i = 0; i < MAX_OUTPUTS; i++)
	{	
		Config.min_travel[i] = -125;
		Config.max_travel[i] = 125;	
		Config.Channel[i].output_b = NOMIX;
		Config.Channel[i].output_c = NOMIX;
	}
	
	// RC channel defaults
	for (i = 0; i < MAX_RC_CHANNELS; i++)
	{
		Config.ChannelOrder[i] = pgm_read_byte(&JR[i]);
		Config.RxChannelZeroOffset[i] = 3750;
	}
	
	// Monopolar throttle is a special case. Set to -100% or -1000
	Config.RxChannelZeroOffset[THROTTLE] = 2750;

	Config.Failsafe[0] = -100;			// Throttle should failsafe to minimum
	//
	get_preset_mix(AEROPLANE_MIX);		// Load AEROPLANE default mix
	//
	Config.RxModeIn = SBUS;				// Default to S.Bus
	Config.RxModeOut = SBUS;			// Default to S.Bus
	Config.TxSeq = JRSEQ;

#ifdef KK21
	Config.AccZero[ROLL] 	= 0;		// Acc calibration defaults for KK2.1
	Config.AccZero[PITCH]	= 0;
	Config.AccZero[YAW]		= 0;
	Config.AccZeroNormZ		= 128;
#else
	Config.AccZero[ROLL] 	= 621;		// Acc calibration defaults for KK2.0
	Config.AccZero[PITCH]	= 623;
	Config.AccZero[YAW]		= 643; 		// 643 is the center
	Config.AccZeroNormZ		= 765;
#endif
	
	// Set up flight modes for all three profiles
	Config.FlightMode[1].StabMode = ALWAYSON;
	Config.FlightMode[2].StabMode = ALWAYSON;
	Config.FlightMode[2].AutoMode = ALWAYSON;

	// Set up all three profiles the same initially
	for (i = 0; i < FLIGHT_MODES; i++)
	{
		Config.FlightMode[i].Roll.P_mult = 80;			// PID defaults		
		Config.FlightMode[i].Roll.I_mult = 50;	
		Config.FlightMode[i].Pitch.P_mult = 80;
		Config.FlightMode[i].Pitch.I_mult = 50;
		Config.FlightMode[i].Yaw.P_mult = 80;
		Config.FlightMode[i].Yaw.I_mult = 80;
		Config.FlightMode[i].A_Roll_P_mult = 20;
		Config.FlightMode[i].A_Pitch_P_mult = 20;
	}

	Config.Acc_LPF = HZ21;
#ifdef KK21
	Config.MPU6050_LPF = HZ21;			// 21Hz
#endif
	Config.CF_factor = 7;
	Config.DynGainSrc = NOCHAN;
	Config.DynGain = 100;
	Config.FlightChan = GEAR;			// Channel GEAR switches flight mode by default
	Config.FlapChan = NOCHAN;			// This is to make sure that flaperons are handled correctly when disabled
	Config.Orientation = HORIZONTAL;	// Board orientation
#ifdef KK2Mini
	Config.Contrast = 30;				// Contrast (KK2 Mini)
#else
	Config.Contrast = 36;				// Contrast (Everything else)
#endif
	Config.LMA_enable = 0;				// Default to off
	Config.Stick_Lock_rate = 3;
	Config.Deadband = 2;				// RC deadband = 2%
	Config.FailsafeThrottle = -100;		// Throttle position in failsafe
}

void Save_Config_to_EEPROM(void)
{
	// Write to eeProm
	cli();
	eeprom_write_block_changes((uint8_t*)&Config, (uint8_t*)EEPROM_DATA_START_POS, sizeof(CONFIG_STRUCT));
	sei();
}

// src is the address in RAM
// dest is the address in eeprom (hence const)
void eeprom_write_block_changes(uint8_t *src, uint8_t *dest, uint16_t size)
{
	uint16_t len;
	uint8_t value;

	for (len=0; len < size; len++)
	{
		// Get value at src
		value = *src;
		
		// Write the value at src to dest
		eeprom_write_byte_changed(dest, value);
		src++;
		dest++;
	}
}

// addr is the address in eeprom
// value is the value to be written
void eeprom_write_byte_changed(uint8_t *addr, uint8_t value)
{
	if (eeprom_read_byte(addr) != value)
	{
		// void eeprom_write_byte (uint8_t *__p, uint8_t __value);
		eeprom_write_byte(addr, value);
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

