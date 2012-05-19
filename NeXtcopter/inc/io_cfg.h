/*********************************************************************
 * io_cfg.h
 ********************************************************************/

#ifndef IO_CFG_H
#define IO_CFG_H

//***********************************************************
//* Includes
//***********************************************************

#include "typedefs.h"

//***********************************************************
//* Pin definitions
//***********************************************************

#define RX_ROLL			REGISTER_BIT(PIND,1)
#define RX_PITCH		REGISTER_BIT(PIND,2)
#define RX_COLL			REGISTER_BIT(PIND,3)
#define RX_YAW			REGISTER_BIT(PINB,7)
#define RX_ROLL_DIR 	REGISTER_BIT(DDRD,1)
#define RX_PITCH_DIR 	REGISTER_BIT(DDRD,2)
#define RX_COLL_DIR 	REGISTER_BIT(DDRD,3)
#define RX_YAW_DIR 		REGISTER_BIT(DDRB,7)

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

#define LED 			REGISTER_BIT(PORTB,6)
#define LED_DIR 		REGISTER_BIT(DDRB,6)

#define USART_TX 		REGISTER_BIT(PORTB,4)
#define USART_TX_DIR	REGISTER_BIT(DDRB,4)


enum GyroDirection { GYRO_NORMAL = 0, GYRO_REVERSED };
enum GyroArrayIndex { ROLL = 0, PITCH, YAW };
enum ADCInputs {YAW_GYRO = 0, PITCH_GYRO, ROLL_GYRO, ROLL_POT, PITCH_POT, YAW_POT };

#endif //IO_CFG_H
