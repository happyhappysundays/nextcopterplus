/*********************************************************************
 * eeprom.h
 ********************************************************************/

//***********************************************************
//* Externals
//***********************************************************

extern void Initial_EEPROM_Config_Load(void);
extern void Save_Config_to_EEPROM(void);
extern void Set_EEPROM_Default_Config(void);

extern uint8_t JR[];
extern uint8_t FUTABA[];
extern uint8_t SATELLITE[];
