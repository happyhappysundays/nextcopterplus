/*********************************************************************
 * typedefs.h
 ********************************************************************/
#include "compiledefs.h"

#ifndef TYPE_DEFS_H_
#define TYPE_DEFS_H_

/*********************************************************************
 * Type definitions
 ********************************************************************/

#define MAX_RC_CHANNELS 8				// Maximum input channels from RX
#define MAX_OUTPUTS 8					// Maximum output channels
#define MAX_ZGAIN 500					// Maximum amount of Z-based height dampening
#define	FLIGHT_MODES 3					// Number of flight profiles
#define NUMBEROFAXIS 3					// Number of axis (Roll, Pitch, Yaw)
#define NUMBEROFORIENTS 6				// Number board orientations
#define	THROTTLEIDLE 50					// Throttle value below which is considered idle

#define MOTOR_100 1900					// PWM value to produce a 1.9ms throttle pulse regardless of pulse width mode
#define MOTOR_0	1100					// PWM value to produce a 1.1ms throttle pulse regardless of pulse width mode
#define	MOTORMIN 1000					// Minimum throttle value. 1000 or 1.1ms

typedef struct
{
	uint16_t	x;
	uint16_t	y;
} mugui_size16_t;

typedef struct
{
	int16_t	minimum;
	int16_t	maximum;
	int16_t	failsafe;
	int16_t	trim;
} servo_limits_t;

// Channel mixer definition
typedef struct
{
	int16_t		value;					// Current value of this channel

	// Mixer menu (14)
	int8_t		Motor_marker;			// Motor/Servo marker
	int8_t		source_a;				// Source A RC input for calculation
	int8_t		source_a_volume;		// Percentage of source to pass on
	int8_t		source_b;				// Optional source B RC input for calculation
	int8_t		source_b_volume;		// Percentage of source to pass on
	int8_t		roll_gyro;				// Use roll gyro
	int8_t		pitch_gyro;				// Use pitch gyro
	int8_t		yaw_gyro;				// Use yaw gyro
	int8_t		roll_acc;				// Use roll acc
	int8_t		pitch_acc;				// Use pitch acc
	int8_t		output_b;				// Channel B for calculation
	int8_t		output_b_volume;		// Percentage of output to use
	int8_t		output_c;				// Channel C for calculation
	int8_t		output_c_volume;		// Percentage of output to use
} channel_t;

// PID type
typedef struct
{
	int8_t	P_mult;
	int8_t	I_mult;
	int8_t	D_mult;
} PID_mult_t;

// Flight_control type (22)
typedef struct
{
	int8_t		StabMode;				// Stability on/off
	int8_t		AutoMode;				// Autolevel on/off

	int8_t		Roll_type;				// Gyro type (rate mode, axis lock, AVCS simulate)
	PID_mult_t	Roll;					// Gyro PID settings [P,I,D]
	int8_t		Roll_limit;				// I-term limits (0 to 125%)
	int8_t		A_Roll_P_mult;			// Acc gain settings
	int8_t		AccRollZeroTrim;		// User-set ACC trim (+/-127)

	int8_t		Pitch_type;
	PID_mult_t	Pitch;
	int8_t		Pitch_limit;
	int8_t		A_Pitch_P_mult;
	int8_t		AccPitchZeroTrim;

	int8_t		Yaw_type;
	PID_mult_t	Yaw;
	int8_t		Yaw_limit;
	int8_t		Yaw_trim;

} flight_control_t;

// Settings structure
typedef struct
{
	uint8_t	setup;						// Byte to identify if already setup

	// Menu adjustable items
	// RC settings
	uint8_t		ChannelOrder[MAX_RC_CHANNELS];	// Assign channel numbers to hard-coded channel order
										// OpenAero2 uses Thr, Ail, Ele, Rud, Gear, Aux1, Aux2, Aux3
										// THROTTLE will always return the correct data for the assigned throttle channel
										// AILERON will always return the correct data for the assigned aileron channel
										// ELEVATOR will always return the correct data for the assigned elevator channel
										// RUDDER will always return the correct data for the assigned rudder channel
	// Servo travel limits
	servo_limits_t	Limits[MAX_OUTPUTS];	// Actual, respanned travel limits to save recalculation each loop

	// RC items (11)
	int8_t		RxMode;					// PWM, CPPM or serial types
	int8_t		PWM_Sync;				// Channel to sync to in PWM mode
	int8_t		TxSeq;					// Channel order of transmitter (JR/Futaba etc)
	int8_t		FlightChan;				// Channel number to select flight mode
	int8_t		FlapChan;				// Channel number for second aileron input
	int8_t		DynGainSrc;				// Dynamic gain source channel
	int8_t		DynGain;				// Dynamic gain attenuation (0% to 100%)
	int8_t		Differential;			// Aileron differential
	int8_t		flapspeed;				// Flap deploy speed
	int8_t		Stick_Lock_rate;		// Axis lock mode stick rate
	int8_t		Deadband;				// RC deadband (%)

	// Failsafe items (5)
	int8_t		FailsafeType;			// Simple or Advanced (Autolevel)
	int8_t		FailsafeThrottle;		// Throttle position in failsafe (-125 to 125%)
	int8_t		FailsafeElevator;		// Elevator trim in failsafe (-125 to 125%)
	int8_t		FailsafeAileron;		// Aileron trim in failsafe (-125 to 125%)
	int8_t		FailsafeRudder;			// Rudder trim in failsafe (-125 to 125%)
	
	// Flight mode settings
	flight_control_t FlightMode[FLIGHT_MODES];		// Flight control settings (3)

	// Servo travel limts
	int32_t		Raw_I_Limits[NUMBEROFAXIS];			// Actual, unspanned I-term out limits to save recalculation each loop
	int32_t		Raw_I_Constrain[NUMBEROFAXIS];		// Actual, unspanned I-term in limits to save recalculation each loop

	// Triggers
	int16_t		Launchtrigger;			// Actual, unspanned launch trigger
	uint8_t		HandsFreetrigger;		// Actual, unspanned hands-free trigger
	int16_t		PowerTriggerActual;		// LVA alarm * 10;
	
	// Limits
	int16_t		DeadbandLimit;			// Actual deadband limit

	//Dynamic gain divisor
	int16_t		DynGainDiv;				// Precalculated dynamic gain variable

	// General items (11)
	int8_t		MixMode;				// Aeroplane/Flying Wing/Camstab
	int8_t		Orientation;			// Horizontal / vertical / upside-down / (others)
	int8_t		Contrast;				// Contrast setting
	int8_t		Servo_rate;				// Servo rate for camstab (Low = 50Hz, High = 200Hz)
	int8_t		CamStab;				// Camstab. Removes dependence on RC input.
	int8_t		AutoCenter;				// Yaw heading hold auto centering ***
	int8_t		PowerTrigger;			// LVA cell voltage (0 to 5 for  = 3.5V to 3.9V)
	int8_t		LMA_enable;				// LMA setting
	int8_t		CF_factor;				// Gyro/Acc Complementary Filter mix
	int8_t		Acc_LPF;				// LPF for accelerometers
	int8_t		MPU6050_LPF;			// MPU6050's internal LPF. Values are 0x06 = 5Hz, (5)10Hz, (4)21Hz, (3)44Hz, (2)94Hz, (1)184Hz LPF, (0)260Hz

	// Non-menu items 
	// Input channel configuration
	channel_t	Channel[MAX_OUTPUTS];	// Channel mixing data	

	// Servo menu
	int8_t		Servo_reverse[MAX_OUTPUTS];	// Reversal of output channel
	int8_t		Offset[MAX_OUTPUTS];		// Trim/neutral position (-125 to 125%)
	int8_t		min_travel[MAX_OUTPUTS];	// Minimum output value (-125 to 125%)
	int8_t		max_travel[MAX_OUTPUTS];	// Maximum output value (-125 to 125%)
	int8_t		Failsafe[MAX_OUTPUTS];		// Failsafe position (-125 to 125%)

	// RC inputs
	uint16_t 	RxChannelZeroOffset[MAX_RC_CHANNELS];	// RC channel offsets for actual radio channels

	// Acc zeros
	uint16_t	AccZero[NUMBEROFAXIS];	// Acc calibration results
	int16_t		AccZeroNormZ;			// Acc-Z zero for normal Z values
	int16_t		AccZeroInvZ;			// Acc-Z zero for inverted Z values
	int16_t		AccZeroDiff;			// Difference between normal and inverted Acc-Z zeros

	// Gyro zeros
	int16_t		gyroZero[NUMBEROFAXIS];

	// Flight mode
	int8_t		Flight;					// Flight mode in use
	
	// Adjusted trims
	int16_t		Rolltrim[FLIGHT_MODES];	// User set trims * 100
	int16_t		Pitchtrim[FLIGHT_MODES];

	// Sticky flags
	uint8_t		Main_flags;				// Non-volatile flags

	// Misc
	int8_t		AileronPol;				// Aileron RC input polarity
	int8_t		SecAileronPol;			// Second aileron RC input polarity
	int8_t		ElevatorPol;			// Elevator RC input polarity
	int8_t		RudderPol;				// Rudder RC input polarity
			
} CONFIG_STRUCT;

typedef struct
{
	int8_t lower;						// Lower limit for menu item
	int8_t upper;						// Upper limit for menu item
	uint8_t increment;					// Increment for menu item
	uint8_t style;						// 0 = numeral, 1 = text, 2 = numeric * 4
	int8_t default_value;				// Default value for this item
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
