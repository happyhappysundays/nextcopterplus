
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

// Misc analog inputs
#define VBAT_PIN		REGISTER_BIT(PINA,3)
#define VBAT_DIR		REGISTER_BIT(DDRA,3)

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
#define CPPM			REGISTER_BIT(PIND,0)	// Same physical port as THR input
#define CPPM_DIR 		REGISTER_BIT(DDRD,0)

//***********************************************************
// Enumeration
//***********************************************************

enum RPYArrayIndex 	{ROLL = 0, PITCH, YAW};
enum RX_Modes		{CPPM_MODE = 0, SBUS, SPEKTRUM, XTREME};
enum RX_Sequ		{JRSEQ = 0, FUTABASEQ};
enum MIX_Modes		{AEROPLANE = 0, FWING, MANUAL};
enum Polarity 		{NORMAL = 0, REVERSED};
enum MixSource 		{CH1 = 0, CH2, CH3, CH4, CH5, CH6, CH7, CH8, UNUSED};
enum RCchannels 	{THROTTLE = 0, AILERON, ELEVATOR, RUDDER, GEAR, AUX1, AUX2, AUX3, NOCHAN};
enum SwitchModes	{DISABLED = 0, ALWAYSON, HANDSFREE};
enum Availability	{OFF = 0, ON, REV};
enum Orientation	{HORIZONTAL = 0, VERTICAL, UPSIDEDOWN, AFT, SIDEWAYS, PITCHUP};
enum KK21ADCInputs 	{AIN_VCC1 = 0, AIN_ADC1, AIN_ADC2, AIN_VBAT1, AIN_ADC4, AIN_ADC5, AIN_PITOT, AIN_ADC7};
enum KK20ADCInputs 	{AIN_VCC0 = 0, AIN_Y_GYRO, AIN_Z_GYRO, AIN_VBAT0, AIN_X_GYRO, AIN_X_ACC, AIN_Y_ACC, AIN_Z_ACC};
enum Global_Status	{IDLE = 0, REQ_STATUS, WAITING_STATUS, PRESTATUS, STATUS, WAITING_TIMEOUT, WAITING_TIMEOUT_BD, PRESTATUS_TIMEOUT, STATUS_TIMEOUT, POSTSTATUS_TIMEOUT, MENU};
enum Servo_rate		{LOW = 0, SYNC, FAST};
enum Gyro_type		{RATE = 0, LOCK};
enum Failsafes		{NOFAILSAFE = 0, SIMPLE, ADVANCED};
enum Devices		{ASERVO = 0, DSERVO, MOTOR}; 
enum Filters		{HZ5 = 0, HZ10, HZ21, HZ44, HZ94, HZ184, HZ260, NOFILTER};
enum Sources 		{SRC1 = 0, SRC2, SRC3, SRC4, SRC5, SRC6, SRC7, SRC8, SRC9, SRC10, SRC11, SRC12, SRC13, SRC14, SRC15, NOMIX};

//***********************************************************
// Flags
//***********************************************************

enum GeneralError	{NO_ERROR = 0, THROTTLE_HIGH, LOST_MODEL, LVA_ALARM, BUZZER_ON, FAILSAFE_ENDED}; // 0-4 
enum FlightFlags	{AutoLevel = 0, Stability, FailsafeFlag, RxActivity, HandsFree, FrameDrop, RxStarted, FirstTimeFlightMode}; // Volatile flags 0-7
enum NVMainFlags	{inv_cal_done = 0, normal_cal_done}; // Located in the non-volatile Config.Main_Flags byte Bits:0-1

#endif //IO_CFG_H
