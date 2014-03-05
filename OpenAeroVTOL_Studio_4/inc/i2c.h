/*********************************************************************
 * i2c.h
 ********************************************************************/

//***********************************************************
//* Externals
//***********************************************************

extern void writeI2Cbyte(uint8_t address, uint8_t location, uint8_t value);
extern void readI2CbyteArray(uint8_t address, uint8_t location, uint8_t *array,uint8_t size);
extern void init_i2c_gyros(void);
extern void init_i2c_accs(void);


