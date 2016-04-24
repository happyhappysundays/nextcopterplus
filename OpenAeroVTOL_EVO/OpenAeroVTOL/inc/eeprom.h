/*********************************************************************
 * eeprom.h
 ********************************************************************/

//***********************************************************
//* Externals
//***********************************************************

extern bool Initial_EEPROM_Config_Load(void);
extern void Save_Config_to_EEPROM(void);
extern void Set_EEPROM_Default_Config(void);
extern void Load_eeprom_preset(uint8_t preset);

extern const int8_t JR[];
extern const int8_t FUTABA[];
extern const int8_t MPX[];
extern const int8_t P1_Orientation_LUT[];

