/*********************************************************************
 * typedefs.h
 ********************************************************************/

#ifndef TYPE_DEFS_H_
#define TYPE_DEFS_H_

/*********************************************************************
 * Type definitions
 ********************************************************************/

#define INPUT 	0
#define OUTPUT 	1

typedef enum _BOOL { FALSE = 0, TRUE } BOOL;

typedef struct
{
	uint8_t	setup;					// Byte to identify if already setup

	uint16_t RxChannel1ZeroOffset;  // Zero offsets
	uint16_t RxChannel2ZeroOffset;
	uint16_t RxChannel3ZeroOffset;	// Currently throttle is fixed
	uint16_t RxChannel4ZeroOffset;
} CONFIG_STRUCT;

// The following code courtesy of: stu_san on AVR Freaks

typedef struct
{
  unsigned int bit0:1;
  unsigned int bit1:1;
  unsigned int bit2:1;
  unsigned int bit3:1;
  unsigned int bit4:1;
  unsigned int bit5:1;
  unsigned int bit6:1;
  unsigned int bit7:1;
} _io_reg; 

#define REGISTER_BIT(rg,bt) ((volatile _io_reg*)&rg)->bit##bt

#endif
