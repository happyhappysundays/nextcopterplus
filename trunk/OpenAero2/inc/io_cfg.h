
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
//* Random externals
//***********************************************************

extern CONFIG_STRUCT Config;

//***********************************************************
//* Pin definitions - KK2.0
//***********************************************************

// RX inputs
#define RX_ROLL			REGISTER_BIT(PIND,3)
#define RX_PITCH		REGISTER_BIT(PIND,2)
#define RX_COLL			REGISTER_BIT(PIND,0)
#define RX_YAW			REGISTER_BIT(PINB,2)
#define RX_AUX			REGISTER_BIT(PINB,0)
#define RX_ROLL_DIR 	REGISTER_BIT(DDRD,3)
#define RX_PITCH_DIR 	REGISTER_BIT(DDRD,2)
#define RX_COLL_DIR 	REGISTER_BIT(DDRD,0)
#define RX_YAW_DIR 		REGISTER_BIT(DDRB,2)
#define RX_AUX_DIR 		REGISTER_BIT(DDRB,0)

// Gyro inputs
#define	GYRO_ROLL 		REGISTER_BIT(PINA,4)
#define GYRO_PITCH 		REGISTER_BIT(PINA,1)
#define GYRO_YAW 		REGISTER_BIT(PINA,2)
#define	GYRO_ROLL_DIR 	REGISTER_BIT(DDRA,4)
#define GYRO_PITCH_DIR 	REGISTER_BIT(DDRA,1)
#define GYRO_YAW_DIR	REGISTER_BIT(DDRA,2)

// ACC inputs
#define	ACC_X 			REGISTER_BIT(PINA,5)
#define ACC_Y 			REGISTER_BIT(PINA,6)
#define ACC_Z 			REGISTER_BIT(PINA,7)
#define	ACC_X_DIR 		REGISTER_BIT(DDRA,5)
#define ACC_Y_DIR 		REGISTER_BIT(DDRA,6)
#define ACC_Z_DIR		REGISTER_BIT(DDRA,7)

// Misc analog inputs
#define	VCC_PIN			REGISTER_BIT(PINA,0)
#define VBAT_PIN		REGISTER_BIT(PINA,3)
#define VCC_DIR 		REGISTER_BIT(DDRA,0)
#define VBAT_DIR		REGISTER_BIT(DDRA,3)

// Motor outputs
#define M1				REGISTER_BIT(PORTC,6)
#define M2				REGISTER_BIT(PORTC,4)
#define M3				REGISTER_BIT(PORTC,2)
#define M4				REGISTER_BIT(PORTC,3)
#define M5				REGISTER_BIT(PORTC,1)
#define M6				REGISTER_BIT(PORTC,0)
#define M7				REGISTER_BIT(PORTC,5)
#define M8				REGISTER_BIT(PORTC,7)
#define M1_DIR 			REGISTER_BIT(DDRC,6)
#define M2_DIR 			REGISTER_BIT(DDRC,4)
#define M3_DIR 			REGISTER_BIT(DDRC,2)
#define M4_DIR 			REGISTER_BIT(DDRC,3)
#define M5_DIR 			REGISTER_BIT(DDRC,1)
#define M6_DIR 			REGISTER_BIT(DDRC,0)
#define M7_DIR 			REGISTER_BIT(DDRC,5)
#define M8_DIR 			REGISTER_BIT(DDRC,7)
#define MOTORS			PORTC
#define MOTOR_DIR		DDRC

// LCD module
#define LCD_SI			REGISTER_BIT(PORTD,1)
#define LCD_SI_DIR		REGISTER_BIT(DDRD,1)
#define LCD_SCL			REGISTER_BIT(PORTD,4)
#define LCD_SCL_DIR		REGISTER_BIT(DDRD,4)
#define LCD_CSI			REGISTER_BIT(PORTD,5)
#define LCD_CSI_DIR		REGISTER_BIT(DDRD,5)
#define LCD_RES			REGISTER_BIT(PORTD,6)
#define LCD_RES_DIR		REGISTER_BIT(DDRD,6)
#define LCD_A0			REGISTER_BIT(PORTD,7)
#define LCD_A0_DIR		REGISTER_BIT(DDRD,7)

// Misc IO
#define LED1 			REGISTER_BIT(PORTB,3)	// Red LED
#define LED1_DIR 		REGISTER_BIT(DDRB,3)
#define LVA				REGISTER_BIT(PORTB,1)	// LVA output (buzzer)
#define LVA_DIR			REGISTER_BIT(DDRB,1)
#define BUTTON1			REGISTER_BIT(PINB,7)	// Button 1
#define BUTTON1_DIR 	REGISTER_BIT(DDRB,7)
#define BUTTON2			REGISTER_BIT(PINB,6)	// Button 2
#define BUTTON2_DIR 	REGISTER_BIT(DDRB,6)
#define BUTTON3			REGISTER_BIT(PINB,5)	// Button 3
#define BUTTON3_DIR 	REGISTER_BIT(DDRB,5)
#define BUTTON4			REGISTER_BIT(PINB,4)	// Button 4
#define BUTTON4_DIR 	REGISTER_BIT(DDRB,4)

// I/O clones
#define CPPM			REGISTER_BIT(PINB,2)	// Same physical port as RUDDER input
#define CPPM_DIR 		REGISTER_BIT(DDRB,2)
#define TX 				REGISTER_BIT(PORTB,0)	// UART output (AUX input)
#define TX_DIR			REGISTER_BIT(DDRB,0)

//***********************************************************
// Enumeration
//***********************************************************
enum RPYArrayIndex 	{ROLL = 0, PITCH, YAW, NO_GYRO};
enum RX_Modes		{CPPM_MODE = 0, PWM1, PWM2, PWM3, XTREME, SBUS, SPEKTRUM};
enum RX_Sequ		{JRSEQ = 0, FUTABASEQ, SATSEQ};
enum MIX_Modes		{AEROPLANE = 0, FWING, CAMSTAB};
enum Polarity 		{NORMAL = 0, REVERSED};
enum RCinputs 		{CH1 = 0, CH2, CH3, CH4, CH5, CH6, CH7, CH8, CH9, CH10, CH11, CH12, UNUSED};
enum RCchannels 	{THROTTLE = 0, AILERON, ELEVATOR, RUDDER, GEAR, AUX1, AUX2, AUX3, NOCHAN};
enum SwitchModes	{DISABLED = 0, ALWAYSON, HANDSFREE};
enum Availability	{OFF = 0, ON, REV};
enum BatteryType	{LIPO = 0, NIMH};
enum Orientation	{HORIZONTAL = 0, VERTICAL, UPSIDEDOWN, AFT, SIDEWAYS};
enum ADCInputs 		{AIN_VCC = 0, AIN_Y_GYRO, AIN_Z_GYRO, AIN_VBAT, AIN_X_GYRO, AIN_X_ACC, AIN_Y_ACC, AIN_Z_ACC};
enum Global_Status	{IDLE = 0, REQ_STATUS, WAITING_STATUS, STATUS, WAITING_TIMEOUT, WAITING_TIMEOUT_BD, STATUS_TIMEOUT, MENU};
enum Servo_rate		{LOW = 0, HIGH};
enum Gyro_type		{RATE = 0, LOCK};
enum Failsafes		{SIMPLE = 0, ADVANCED};

//***********************************************************
// Flags
//***********************************************************

enum GlobalError	{NO_ERROR = 0, LOW_BATT, THROTTLE_HIGH, NO_SIGNAL, SENSOR_ERROR, LOST_MODEL};
enum FlightFlags	{AutoLevel = 0, Stability, Failsafe, RxActivity, HandsFree, Launch_Mode, Launch_Block, Model_lost};
enum MainFlags		{inv_cal_done = 0, normal_cal_done, Refresh_safe, FirstTimeIMU, Overdue, ServoTick};
enum AlarmFlags		{BUZZER_ON = 0, LVA_Alarm, SIG_Alarm};


#endif //IO_CFG_H
