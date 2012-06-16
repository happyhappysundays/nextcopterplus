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
	uint8_t	setup;						// Byte to identify if already setup
	uint16_t	RxChannel1ZeroOffset;	// Zero offsets (stick centering)
	uint16_t	RxChannel2ZeroOffset;
	uint16_t	RxChannel3ZeroOffset;	// Currently offset is fixed at 1520
	uint16_t	RxChannel4ZeroOffset;
	uint16_t	RxChannel5ZeroOffset;
	uint8_t		AccRollZeroTrim;		// User-set ACC trim (0~255 -> +/-127 @ GUI)
	uint8_t		AccPitchZeroTrim;
	uint16_t	AccRollZero;			// Acc calibration results
	uint16_t	AccPitchZero;
	uint8_t		P_mult_roll;			// PID gain settings
	uint8_t		I_mult_roll;
	uint8_t		D_mult_roll;
	uint8_t		P_mult_pitch;
	uint8_t		I_mult_pitch;
	uint8_t		D_mult_pitch;
	uint8_t		P_mult_yaw;
	uint8_t		I_mult_yaw;
	uint8_t		D_mult_yaw;
	uint8_t		P_mult_glevel;			// Autolevel gyro P-term
	uint8_t		D_mult_glevel;			// Autolevel gyro D-term
	uint8_t		P_mult_alevel;			// Autolevel acc P-term
	uint8_t		I_mult_alevel;			// Autolevel acc I-term
	//
	uint8_t		StabMode;
	uint8_t		ALMode;
	uint16_t	PowerTrigger;
	uint8_t		Modes;
	uint16_t	AutoTuneRX;				// Auto-tuned rate for UART RX
	uint16_t	AutoTuneTX;				// Auto-tuned rate for UART TX
	uint8_t		RollGyro;
	uint8_t		PitchGyro;
	uint8_t		YawGyro;
	uint8_t		RollServo;
	uint8_t		PitchServo;
	uint8_t		YawServo;
	uint8_t		StabChan;				// CPPM channel number to select stability mode
	//
	uint16_t	Failsafe_1;				// Failsafe positions
	uint16_t	Failsafe_2;
	uint16_t	Failsafe_3;
	uint16_t	Failsafe_4;
	uint16_t	Failsafe_5;
	uint16_t	Failsafe_6;

} CONFIG_STRUCT;

typedef struct
{
  int16_t lower;						// Lower limit for menu item
  int16_t upper;						// Upper limit for menu item
} menu_range_t; 


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
