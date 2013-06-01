//***********************************************************
//* eeprom.c
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include <string.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <stdbool.h>
#include "..\inc\io_cfg.h"
#include <util/delay.h>
#include "..\inc\mixer.h"
#include "..\inc\menu_ext.h"

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
#define MAGIC_NUMBER 0x11		// eePROM signature - change for each eePROM structure change 0x11 = V1.2b7
								// to force factory reset

//************************************************************
// Code
//************************************************************

uint8_t	JR[MAX_RC_CHANNELS] PROGMEM 	= {0,1,2,3,4,5,6,7}; 	// JR/Spektrum channel sequence (TAERG123)
uint8_t	FUTABA[MAX_RC_CHANNELS] PROGMEM = {1,2,0,3,4,5,6,7}; 	// Futaba channel sequence (AETRGF12)
uint8_t	SATELLITE[MAX_RC_CHANNELS] PROGMEM = {5,1,2,3,4,0,6,7}; // Spektrum satellite channel sequence (FAERGT12)

void Set_EEPROM_Default_Config(void)
{
	uint8_t i;
	
	// Clear entire Config space first
	memset(&Config.setup,0,(sizeof(Config)));

	// Set magic number / setup byte
	Config.setup = MAGIC_NUMBER;

	for (i = 0; i < MAX_RC_CHANNELS; i++)
	{
		Config.ChannelOrder[i] = pgm_read_byte(&JR[i]);
		Config.RxChannelZeroOffset[i] = 3750;
	}
	// Servo defaults
	for (i = 0; i < MAX_RC_CHANNELS; i++)
	{
		//Config.Servo_reverse[i] = NORMAL;
		//Config.Offset[i] = 0;
		Config.min_travel[i] = -100;
		Config.max_travel[i] = 100;
		//Config.Failsafe[i] = 0;
	}
	Config.Failsafe[0] = -100;			// Throttle should failsafe to minimum
	//
	get_preset_mix(AEROPLANE_MIX);		// Load AEROPLANE default mix
	//
	Config.RxMode = PWM1;				// Default to PWM1
	Config.TxSeq = JRSEQ;
	Config.AccZero[ROLL] 	= 621;		// Acc calibration defaults
	Config.AccZero[PITCH]	= 623;
	Config.AccZero[YAW]		= 643; 		// 643 is the centre, 520 is inverted
	
	// Flight modes
	Config.FlightMode[0].Profilelimit = -100;			// Trigger setting

	Config.FlightMode[1].Profilelimit = 0;	
	Config.FlightMode[1].StabMode = ALWAYSON;
//	Config.FlightMode[1].Yaw_type = LOCK; // debug for axis lock testing
//	Config.FlightMode[1].Yaw_limit = 125; // debug

	Config.FlightMode[2].Profilelimit = 80;	
	Config.FlightMode[2].StabMode = ALWAYSON;
	Config.FlightMode[2].AutoMode = ALWAYSON;
//	Config.FlightMode[2].Yaw_type = LOCK; // debug
//	Config.FlightMode[2].Yaw_limit = 125; // debug

	// Set up all three profiles the same initially
	for (i = 0; i < 3; i++)
	{
		Config.FlightMode[i].Roll.P_mult = 80;			// PID defaults		
		Config.FlightMode[i].Roll.I_mult = 50;	
		Config.FlightMode[i].Pitch.P_mult = 80;
		Config.FlightMode[i].Pitch.I_mult = 50;
		Config.FlightMode[i].Yaw.P_mult = 80;
		Config.FlightMode[i].Yaw.I_mult = 80;
		Config.FlightMode[i].A_Roll_P_mult = 60;
		Config.FlightMode[i].A_Pitch_P_mult = 60;
	}

	Config.Acc_LPF = 8;
	Config.CF_factor = 30;
	Config.DynGainSrc = NOCHAN;
	Config.DynGain = 100;
	Config.IMUType = 1;					// Advanced IMU ON
	Config.BatteryType = LIPO;
	Config.MinVoltage = 83;				// 83 * 4 = 332
	Config.MaxVoltage = 105;			// 105 * 4 = 420
	Config.FlightChan = GEAR;			// Channel GEAR switches flight mode by default
	Config.FlapChan = NOCHAN;			// This is to make sure that flaperons are handled correctly when disabled
	Config.LaunchDelay = 10;
	Config.Orientation = HORIZONTAL;	// Horizontal / vertical
	Config.Contrast = 38;				// Contrast
	Config.Status_timer = 10;			// Refresh timeout
	Config.LMA_enable = 3;				// Default to 3 minutes
	Config.Servo_rate = LOW;			// Default to LOW (50Hz)
	Config.Stick_Lock_rate = 2;
	Config.Deadband = 2;				// RC deadband = 2%
	Config.FailsafeThrottle = -100;		// Throttle position in failsafe
}

void Save_Config_to_EEPROM(void)
{
	// Write to eeProm
	cli();
	eeprom_write_block_changes((const void*) &Config, (void*) EEPROM_DATA_START_POS, sizeof(CONFIG_STRUCT));	
	sei();
	menu_beep(1);
	LED1 = !LED1;
	_delay_ms(500);
	LED1 = !LED1;
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
		Config.setup = MAGIC_NUMBER;
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

