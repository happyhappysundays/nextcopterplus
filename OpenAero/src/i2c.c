//***********************************************************
//* i2c.c
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include <avr/io.h>
#include "..\inc\io_cfg.h"
#include "..\inc\i2cmaster.h"

//************************************************************
// Prototypes
//************************************************************

uint8_t checkI2CDeviceIsResponding(uint8_t addr);
uint8_t readI2Cbyte(uint8_t address, uint8_t location);
void writeI2Cbyte(uint8_t address, uint8_t location, uint8_t value);
void readI2CbyteArray(uint8_t address, uint8_t location, uint8_t *array,uint8_t size);

#ifdef N6_MODE
//************************************************************
// i86/N6 i2c gyro-specifc code
//************************************************************

uint8_t checkI2CDeviceIsResponding(uint8_t addr)
{
    uint8_t value = i2c_start(addr+I2C_WRITE); 		// Write to device
    i2c_stop(); 									// Don't send byte to write
    return (value==0);  							// if start command is successful, then a device has responded
}

uint8_t readI2Cbyte(uint8_t address, uint8_t location)
{
	uint8_t value;
	i2c_start_wait(address+I2C_WRITE);				// Set up device address 
    i2c_write(location);							// Set up register address 
	i2c_rep_start(address+I2C_READ);
    value = i2c_readNak(); 							// Read back one byte
    i2c_stop();
    return value;
}

void writeI2Cbyte(uint8_t address, uint8_t location, uint8_t value)
{
    i2c_start_wait(address+I2C_WRITE);				// Set up device address 
    i2c_write(location);							// Set up register address 
    i2c_write(value); 								// Write byte
    i2c_stop();
}

void readI2CbyteArray(uint8_t address, uint8_t location, uint8_t *array,uint8_t size)
{
    i2c_start_wait(address+I2C_WRITE);
    i2c_write(location);							// Set up register address 
    i2c_rep_start(address+I2C_READ);
	int i=0;
	while(i < size)
	{
		if ((i+1)!=size)
			array[i]=i2c_readAck();
		else array[i]=i2c_readNak(); 				// Read without ACK on last byte
		i++;
	}
    i2c_stop();
}

#endif
