/*********************************************************************
 * eeprom.h
 ********************************************************************/

//***********************************************************
//* Externals
//***********************************************************

extern void Initial_EEPROM_Config_Load(void);
extern void Save_Config_to_EEPROM(void);
extern void Set_EEPROM_Default_Config(void);
extern void eeprom_write_byte_changed( uint8_t * addr, uint8_t value );
extern void eeprom_write_block_changes( const uint8_t * src, void * dest, size_t size );
