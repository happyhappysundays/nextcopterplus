/*********************************************************************
 * io_cfg.h
 ********************************************************************/

#ifndef IO_CFG_H
#define IO_CFG_H

//***********************************************************
//* Includes
//***********************************************************

#include "typedefs.h"
#include "compiledefs.h"

//***********************************************************
//* Random externals
//***********************************************************

extern CONFIG_STRUCT Config;

//***********************************************************
//* Project defines
//***********************************************************

#define VERSION 13		// OpenAero version number
#define MENUITEMS 28	// Number of LCD menu items

//***********************************************************
//* L3G4200D address and ID for i86/N6 board
//***********************************************************
#define L3G4200D_ADDRESS	0xD2
#define L3G4200D_WHOAMI		0xD3

#ifndef N6_MODE 		// Standard KK board I/O configuration
//***********************************************************
//* Pin definitions - KK
//***********************************************************

#define RX_ROLL			REGISTER_BIT(PIND,1)
#define RX_PITCH		REGISTER_BIT(PIND,2)
#define RX_COLL			REGISTER_BIT(PIND,3)
#define RX_YAW			REGISTER_BIT(PINB,7)
#define RX_ROLL_DIR 	REGISTER_BIT(DDRD,1)
#define RX_PITCH_DIR 	REGISTER_BIT(DDRD,2)
#define RX_COLL_DIR 	REGISTER_BIT(DDRD,3)
#define RX_YAW_DIR 		REGISTER_BIT(DDRB,7)
#define RX_AIL2			REGISTER_BIT(PINB,0)

#define	GYRO_ROLL 		REGISTER_BIT(PINC,2)
#define GYRO_PITCH 		REGISTER_BIT(PINC,1)
#define GYRO_YAW 		REGISTER_BIT(PINC,0)
#define	GYRO_ROLL_DIR 	REGISTER_BIT(DDRC,2)
#define GYRO_PITCH_DIR 	REGISTER_BIT(DDRC,1)
#define GYRO_YAW_DIR	REGISTER_BIT(DDRC,0)

#define	GAIN_ROLL 		REGISTER_BIT(PINC,3)
#define GAIN_PITCH 		REGISTER_BIT(PINC,4)
#define GAIN_YAW 		REGISTER_BIT(PINC,5)
#define	GAIN_ROLL_DIR 	REGISTER_BIT(DDRC,3)
#define GAIN_PITCH_DIR 	REGISTER_BIT(DDRC,4)
#define GAIN_YAW_DIR	REGISTER_BIT(DDRC,5)

#define M1				REGISTER_BIT(PORTB,2)
#define M2				REGISTER_BIT(PORTB,1)
#define M3				REGISTER_BIT(PORTB,0)
#define M4				REGISTER_BIT(PORTD,7)
#define M5				REGISTER_BIT(PORTD,6)
#define M6				REGISTER_BIT(PORTD,5)
#define M1_DIR 			REGISTER_BIT(DDRB,2)
#define M2_DIR 			REGISTER_BIT(DDRB,1)
#define M3_DIR 			REGISTER_BIT(DDRB,0)
#define M4_DIR 			REGISTER_BIT(DDRD,7)
#define M5_DIR 			REGISTER_BIT(DDRD,6)
#define M6_DIR 			REGISTER_BIT(DDRD,5)

#define THR				REGISTER_BIT(PORTD,3)	// THR output for throttle
#define THR_DIR 		REGISTER_BIT(DDRD,3)

#define LED1 			REGISTER_BIT(PORTB,6)	// Red LED (write to either LED mapped to the one LED)
#define LED1_DIR 		REGISTER_BIT(DDRB,6)
#define LED2 			REGISTER_BIT(PORTB,6)	// Red LED
#define LED2_DIR 		REGISTER_BIT(DDRB,6)

#define TX 				REGISTER_BIT(PORTB,4)
#define TX_DIR			REGISTER_BIT(DDRB,4)

#define LVA				REGISTER_BIT(PORTD,4)	// LVA output
#define LVA_DIR			REGISTER_BIT(DDRD,4)

#define ICP				REGISTER_BIT(PORTB,0)	// ICP CPPM input (formerly M3)
#define ICP_DIR 		REGISTER_BIT(DDRB,0)

#define FS				REGISTER_BIT(PIND,5)	// Failsafe set input (formerly M6)
#define FS_DIR 			REGISTER_BIT(DDRD,5)

#else // N6_MODE 
//***********************************************************
//* Pin definitions - Eagle N6/HobbyKing i86
//***********************************************************

#define RX_ROLL			REGISTER_BIT(PIND,1)
#define RX_PITCH		REGISTER_BIT(PIND,2)
#define RX_COLL			REGISTER_BIT(PIND,3)
#define RX_YAW			REGISTER_BIT(PINB,7)
#define RX_ROLL_DIR 	REGISTER_BIT(DDRD,1)
#define RX_PITCH_DIR 	REGISTER_BIT(DDRD,2)
#define RX_COLL_DIR 	REGISTER_BIT(DDRD,3)
#define RX_YAW_DIR 		REGISTER_BIT(DDRB,7)

#define	GAIN_ROLL 		REGISTER_BIT(PINC,0)
#define GAIN_PITCH 		REGISTER_BIT(PINC,1)
#define GAIN_YAW 		REGISTER_BIT(PINC,2)
#define	GAIN_ROLL_DIR 	REGISTER_BIT(DDRC,0)
#define GAIN_PITCH_DIR 	REGISTER_BIT(DDRC,1)
#define GAIN_YAW_DIR	REGISTER_BIT(DDRC,2)

#define M1				REGISTER_BIT(PORTB,0)
#define M2				REGISTER_BIT(PORTB,1)
#define M3				REGISTER_BIT(PORTB,2)
#define M4				REGISTER_BIT(PORTB,3)
#define M5				REGISTER_BIT(PORTB,4)
#define M6				REGISTER_BIT(PORTB,5)
#define M1_DIR 			REGISTER_BIT(DDRB,0)
#define M2_DIR 			REGISTER_BIT(DDRB,1)
#define M3_DIR 			REGISTER_BIT(DDRB,2)
#define M4_DIR 			REGISTER_BIT(DDRB,3)
#define M5_DIR 			REGISTER_BIT(DDRB,4)
#define M6_DIR 			REGISTER_BIT(DDRB,5)

#define THR				REGISTER_BIT(PORTD,3)	// THR output for throttle
#define THR_DIR 		REGISTER_BIT(DDRD,3)

#define TX 				REGISTER_BIT(PORTB,4)	// MISO output
#define TX_DIR			REGISTER_BIT(DDRB,4)

#define LVA				REGISTER_BIT(PORTC,3)	// LVA output (Blue LED)
#define LVA_DIR			REGISTER_BIT(DDRC,3)

#define ICP				REGISTER_BIT(PORTB,0)	// ICP CPPM input (formerly M1)
#define ICP_DIR 		REGISTER_BIT(DDRB,0)

#define FS				REGISTER_BIT(PIND,4)	// Failsafe set input (SW1)
#define FS_DIR 			REGISTER_BIT(DDRD,4)

#define RX_MODE			REGISTER_BIT(PIND,5)	// RX modes set input (SW2)
#define RX_MODE_DIR 	REGISTER_BIT(DDRD,5)

#define LED1 			REGISTER_BIT(PORTC,3)	// Blue LED
#define LED1_DIR 		REGISTER_BIT(DDRC,3)
#define LED2 			REGISTER_BIT(PORTD,0)	// Red LED
#define LED2_DIR 		REGISTER_BIT(DDRD,0)

#define SW_1 			REGISTER_BIT(PIND,4)	// 4-way switch inputs
#define SW_2 			REGISTER_BIT(PIND,5)
#define SW_3 			REGISTER_BIT(PIND,6)
#define SW_4 			REGISTER_BIT(PIND,7)
#define SW_1_DIR 		REGISTER_BIT(DDRD,4)
#define SW_2_DIR 		REGISTER_BIT(DDRD,5)
#define SW_3_DIR 		REGISTER_BIT(DDRD,6)
#define SW_4_DIR 		REGISTER_BIT(DDRD,7)

#endif //N6_MODE 

//***********************************************************
// Gyro enumeration
//***********************************************************
#ifndef N6_MODE
enum GyroArrayIndex { ROLL = 0, PITCH, YAW};
#else // Swap Roll and pitch gyros for N6
enum GyroArrayIndex { PITCH = 0, ROLL, YAW};
#endif

enum GyroDirection {GYRO_NORMAL = 0, GYRO_REVERSED};

//***********************************************************
// Vertical mode:	Pitch gyro used for Yaw
//					Roll gyro used for Roll
//				 	Yaw gyro used for Pitch 
//***********************************************************
#ifdef N6_MODE
	#ifdef VERTICAL
	enum ADCInputs {YAW_POT = 0, ROLL_POT, PITCH_POT, DUMMY3, DUMMY4, DUMMY5, VBAT };
	#else
	enum ADCInputs {PITCH_POT = 0, ROLL_POT, YAW_POT, DUMMY3, DUMMY4, DUMMY5, VBAT };
	#endif
#else
	#ifdef VERTICAL // Thanks to wargh from RCG
	enum ADCInputs {PITCH_GYRO = 0, YAW_GYRO, ROLL_GYRO, ROLL_POT, PITCH_POT, YAW_POT, VBAT };
	#else
	enum ADCInputs {YAW_GYRO = 0, PITCH_GYRO, ROLL_GYRO, ROLL_POT, PITCH_POT, YAW_POT, VBAT };
	#endif
#endif // N6_MODE

#endif //IO_CFG_H
