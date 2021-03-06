
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
//* Pin definitions
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

// Motor outputs
#define M1				REGISTER_BIT(PORTC,6)
#define M2				REGISTER_BIT(PORTC,4)
#define M3				REGISTER_BIT(PORTC,2)
#define M4				REGISTER_BIT(PORTC,3)
#define M5				REGISTER_BIT(PORTA,4)
#define M6				REGISTER_BIT(PORTA,5)
#define M7				REGISTER_BIT(PORTC,5)
#define M8				REGISTER_BIT(PORTC,7)
#define M1_DIR 			REGISTER_BIT(DDRC,6)
#define M2_DIR 			REGISTER_BIT(DDRC,4)
#define M3_DIR 			REGISTER_BIT(DDRC,2)
#define M4_DIR 			REGISTER_BIT(DDRC,3)
#define M5_DIR 			REGISTER_BIT(DDRA,4)
#define M6_DIR 			REGISTER_BIT(DDRA,5)
#define M7_DIR 			REGISTER_BIT(DDRC,5)
#define M8_DIR 			REGISTER_BIT(DDRC,7)

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
#define MOTORS			PORTC

// I/O clones
#define CPPM			REGISTER_BIT(PINB,2)	// Same physical port as RUDDER input
#define CPPM_DIR 		REGISTER_BIT(DDRB,2)

//***********************************************************
// Enumeration
//***********************************************************

enum Orientation	{UP_BACK = 0, UP_LEFT, UP_FRONT, UP_RIGHT,
					BACK_DOWN, BACK_LEFT, BACK_UP, BACK_RIGHT,
					DOWN_BACK, DOWN_RIGHT, DOWN_FRONT, DOWN_LEFT,
					FRONT_DOWN, FRONT_RIGHT, FRONT_UP, FRONT_LEFT,
					LEFT_DOWN, LEFT_FRONT, LEFT_UP, LEFT_BACK,
					RIGHT_DOWN, RIGHT_BACK, RIGHT_UP, RIGHT_FRONT
					};
enum Old_Orientation	{HORIZONTAL = 0, VERTICAL, UPSIDEDOWN, AFT, SIDEWAYS, PITCHUP};
enum RPYArrayIndex 	{ROLL = 0, PITCH, YAW, ZED};
enum RX_Modes		{CPPM_MODE = 0, PWM, SBUS, SPEKTRUM, XTREME, MODEB, SUMD};
enum RX_Sequ		{JRSEQ = 0, FUTABASEQ, MPXSEQ, CUSTOM};
enum Polarity 		{NORMAL = 0, REVERSED};
enum KKoutputs 		{OUT1 = 0, OUT2, OUT3, OUT4, OUT5, OUT6, OUT7, OUT8};
enum RCchannels 	{THROTTLE = 0, AILERON, ELEVATOR, RUDDER, GEAR, AUX1, AUX2, AUX3, NOCHAN};
enum Availability	{OFF = 0, ON, SCALE, REVERSE, REVERSESCALE};
enum KK21ADCInputs 	{AIN_VCC1 = 0, AIN_ADC1, AIN_ADC2, AIN_VBAT1, AIN_ADC4, AIN_ADC5, AIN_PITOT, AIN_ADC7};
enum Global_Status	{IDLE = 0, REQ_STATUS, WAITING_STATUS, PRESTATUS, STATUS, WAITING_TIMEOUT, WAITING_TIMEOUT_BD, PRESTATUS_TIMEOUT, STATUS_TIMEOUT, POSTSTATUS_TIMEOUT, MENU};
enum Servo_rate		{LOW = 0, SYNC, FAST};
enum TransitState	{TRANS_P1 = 0, TRANS_P1_to_P1n_start, TRANS_P1n_to_P1_start, TRANS_P1_to_P2_start, TRANS_P1n, TRANSITIONING, TRANS_P2_to_P1_start, TRANS_P1n_to_P2_start, TRANS_P2_to_P1n_start, TRANS_P2};
//					THROTTLE, CURVE A, CURVE B, COLLECTIVE, THROTTLE, AILERON, ELEVATOR, RUDDER, GEAR, AUX1, AUX2, AUX3, ROLLGYRO, PITCHGYO, YAWGYRO, ACCSMOOTH, PITCHSMOOTH, ROLLACC, PITCHACC, AccZ, NONE
enum Sources 		{SRC1,		SRC2,	SRC3,	SRC4,		SRC5,		SRC6,	SRC7,	SRC8,	SRC9,	SRC10, SRC11, SRC12, SRC13, SRC14, SRC15, SRC16, SRC17, SRC18, SRC19, SRC20, NOMIX};
enum Profiles		{P1 = 0, P2};
enum Safety			{ARMED = 0, ARMABLE}; 
enum Devices		{ASERVO = 0, DSERVO, MOTOR}; 
enum Curve			{LINEAR = 0, SINE, SQRTSINE}; 
enum Filters		{HZ5 = 0, HZ10, HZ21, HZ44, HZ94, HZ184, HZ260, NOFILTER};
enum Presets		{QUADX = 0, QUADP, TRICOPTER, BLANK, OPTIONS};
enum Frames			{BASIC = 0, EDIT, ABORT, LOG, CURVE, OFFSET};
enum Errors			{NOERR = 0, REBOOT, MANUAL, NOSIGNAL, TIMER};
enum Reference		{NO_ORIENT = 0, EARTH, MODEL};
enum Curves			{P1_THR_CURVE = 0, P2_THR_CURVE, P1_COLL_CURVE, P2_COLL_CURVE, GEN_CURVE_C, GEN_CURVE_D};
enum Psych			{MONOPOLAR = 0, BIPOLAR};

//***********************************************************
// Flags
//***********************************************************

enum GlobalError	{THROTTLE_HIGH = 0, NO_SIGNAL, DISARMED, LVA_ALARM, BUZZER_ON};
enum FlightFlags	{RxActivity = 0, ARM_blocker};
enum MainFlags		{inv_cal_done_P1 = 0, normal_cal_done_P1, inv_cal_done_P2, normal_cal_done_P2};
enum SensorFlags	{RollGyro = 0, PitchGyro, YawGyro, RollAcc, PitchAcc, ZDeltaAcc, MotorMarker};
enum ScaleFlags		{RollScale = 0, PitchScale, YawScale, AccRollScale, AccPitchScale, AccZScale};
enum ReverseFlags	{RollReverse = 0, PitchReverse, YawReverse, AccRollReverse, AccPitchReverse, AccZReverse};

#endif //IO_CFG_H
