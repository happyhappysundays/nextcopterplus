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

//************************************************************
// Prototypes
//************************************************************

void Initial_EEPROM_Config_Load(void);
void Save_Config_to_EEPROM(void);
void Set_EEPROM_Default_Config(void);
void eeprom_write_byte_changed( uint8_t * addr, uint8_t value );
void eeprom_write_block_changes( const uint8_t * src, void * dest, size_t size );

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

void eeprom_write_block_changes( const uint8_t * src, void * dest, size_t size )
{ 
	size_t len;

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
	} else {
		// Read eeProm
		eeprom_read_block(&Config, (void*) EEPROM_DATA_START_POS, sizeof(CONFIG_STRUCT)); 
	}
}

void Set_EEPROM_Default_Config(void)
{
	Config.RxChannel1ZeroOffset	= 1520;
	Config.RxChannel2ZeroOffset	= 1520;
	Config.RxChannel3ZeroOffset	= 1120;
	Config.RxChannel4ZeroOffset	= 1520;
}

void Save_Config_to_EEPROM(void)
{
	// Write to eeProm
	cli();
	eeprom_write_block_changes( (const void*) &Config, (void*) EEPROM_DATA_START_POS, sizeof(CONFIG_STRUCT));	
	sei();
}
