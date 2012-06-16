/*********************************************************************
 * i2c.h
 ********************************************************************/

//***********************************************************
//* Externals
//***********************************************************

extern uint8_t checkI2CDeviceIsResponding(uint8_t addr);
extern uint8_t readI2Cbyte(uint8_t address, uint8_t location);
extern void writeI2Cbyte(uint8_t address, uint8_t location, uint8_t value);
extern void readI2CbyteArray(uint8_t address, uint8_t location, uint8_t *array,uint8_t size);
