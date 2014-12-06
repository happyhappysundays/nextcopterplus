/*********************************************************************
 * typedefs.h
 ********************************************************************/

#include "compiledefs.h"

#ifndef TYPE_DEFS_H_
#define TYPE_DEFS_H_

/*********************************************************************
 * Defines
 ********************************************************************/

#define MAX_RC_CHANNELS 8				// Maximum input channels from RX
#define MAX_OUTPUTS 8					// Maximum output channels
#define MAX_ZGAIN 500					// Maximum amount of Z-based height dampening
#define	FLIGHT_MODES 2					// Number of flight profiles
#define NUMBEROFAXIS 3					// Number of axis (Roll, Pitch, Yaw)
#define NUMBEROFORIENTS 6				// Number board orientations
#define	THROTTLEIDLE 50					// Throttle value below which is considered idle

#define MOTOR_100 1900					// PWM value to produce a 1.9ms throttle pulse regardless of pulse width mode
#define MOTOR_0	1100					// PWM value to produce a 1.1ms throttle pulse regardless of pulse width mode
#define	MOTORMIN 1000					// PWM value for throttle cut. 1000 or 1.0ms regardless of pulse width mode

#define SERVO_CENTER 1500				// Servo center position. 1500us
#define	THROTTLEMIN 1000				// Minimum throttle input offset value. 3750-1000 = 2750 or 1.1ms.
										// Not to be confused with MOTORMIN which is a PWM value.
#define THROTTLEOFFSET 1250				// Mixer offset needed to reduce the output center to MOTORMIN

/*********************************************************************
 * Type definitions
 ********************************************************************/

typedef struct
{
	uint16_t	x;
	uint16_t	y;
} mugui_size16_t;

typedef struct
{
	int16_t	minimum;
	int16_t	maximum;
} servo_limits_t;

// Channel mixer definition
typedef struct
{
	int16_t		P1_value;				// Current value of this channel at P1
	int16_t		P2_value;				// Current value of this channel at P2
	
	// Mixer menu (25 bytes, 34 items)
	int8_t		P1_offset;				// P1 Offset for this output
	int8_t		P1n_position;			// Position of P1.n offset for this output
	int8_t		P1n_offset;				// P1.n Offset for this output
	int8_t		P2_offset;				// P2 Offset for this output
	//
	int8_t		P1_throttle_volume;		// Percentage of throttle to use in P1
	int8_t		P2_throttle_volume;		// Percentage of throttle to use in P2
	int8_t		Throttle_curve;			// Throttle transition curve (Linear, Sine)

	int8_t		P1_aileron_volume;		// Percentage of aileron to use in P1
	int8_t		P2_aileron_volume;		// Percentage of aileron to use in P2
	int8_t		P1_elevator_volume;		// Percentage of elevator to use in P1
	int8_t		P2_elevator_volume;		// Percentage of elevator to use in P2
	int8_t		P1_rudder_volume;		// Percentage of rudder to use in P1
	int8_t		P2_rudder_volume;		// Percentage of rudder to use in P2

	int8_t		P1_sensors;				// Sensor switches (6), motor marker (1)
	int8_t		P2_sensors;				// Sensor switches (6)
	int8_t		P1_scale;				// P1 sensor scale flags (6)
	int8_t		P2_scale;				// P2 sensor scale flags (6)

	int8_t		P1_source_a;			// Source A for calculation
	int8_t		P1_source_a_volume;		// Percentage of source to use
	int8_t		P2_source_a;			// Source A for calculation
	int8_t		P2_source_a_volume;		// Percentage of source to use
	int8_t		P1_source_b;			// Source B for calculation
	int8_t		P1_source_b_volume;		// Percentage of source to use
	int8_t		P2_source_b;			// Source B for calculation
	int8_t		P2_source_b_volume;		// Percentage of source to use

} channel_t;

// Flight_control type (18/20)
typedef struct
{
	int8_t		Roll_P_mult;			// Roll PI
	int8_t		Roll_I_mult;
	int8_t		Roll_limit;				// I-term limits (0 to 125%)
	int8_t		Roll_Rate;				// 0 to 4, 1 (Default)
	int8_t		A_Roll_P_mult;			// Acc gain settings
	int8_t		AccRollZeroTrim;		// User-set ACC trim (+/-127)

	int8_t		Pitch_P_mult;			// Pitch PI
	int8_t		Pitch_I_mult;
	int8_t		Pitch_limit;			// I-term limits (0 to 125%)
	int8_t		Pitch_Rate;				// 0 to 4, 1 (Default)
	int8_t		A_Pitch_P_mult;
	int8_t		AccPitchZeroTrim;

	int8_t		Yaw_P_mult;				// Yaw PI
	int8_t		Yaw_I_mult;
	int8_t		Yaw_limit;				// I-term limits (0 to 125%)
	int8_t		Yaw_Rate;				// 0 to 4, 1 (Default)
	int8_t		Yaw_trim;

	int8_t		A_Zed_P_mult;

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
	servo_limits_t	Limits[MAX_OUTPUTS];// Actual, respanned travel limits to save recalculation each loop

	// RC items (9)
	int8_t		RxMode;					// PWM, CPPM or serial types
	int8_t		PWM_Sync;				// Channel to sync to in PWM mode
	int8_t		TxSeq;					// Channel order of transmitter (JR/Futaba etc)
	int8_t		FlightChan;				// Channel number to select flight mode
	int8_t		AileronPol;				// Aileron RC input polarity
	int8_t		ElevatorPol;			// Elevator RC input polarity
	int8_t		RudderPol;				// Rudder RC input polarity
	int8_t		TransitionSpeed;		// Transition speed/channel 0 = tied to channel, 1 to 10 seconds.
	int8_t		Transition_P1n;			// Transition SFF point as a percentage -100% to 100%

	// Flight mode settings
	flight_control_t FlightMode[FLIGHT_MODES];	// Flight control settings

	// Servo travel limts
	int32_t		Raw_I_Limits[FLIGHT_MODES][NUMBEROFAXIS];		// Actual, unspanned I-term output limits to save recalculation each loop
	int32_t		Raw_I_Constrain[FLIGHT_MODES][NUMBEROFAXIS];	// Actual, unspanned I-term input limits to save recalculation each loop

	// Triggers
	int16_t		PowerTriggerActual;		// LVA alarm * 10;

	// General items (9/10)
	int8_t		Orientation;			// Horizontal / vertical / upside-down / (others)
	int8_t		Contrast;				// Contrast setting
	int8_t		ArmMode;				// Arming mode on/off
	int8_t		Disarm_timer;			// Auto-disarm setting
	int8_t		PowerTrigger;			// LVA voltage (0 to 127 = 0 to 12.7V)
	int8_t		Servo_rate;				// Servo rate for camstab (Low = ~50Hz, High = ~300Hz)
	int8_t		Acc_LPF;				// LPF for accelerometers
	int8_t		Gyro_LPF;				// LPF for gyros
	int8_t		CF_factor;				// Autolevel correction rate
#ifdef KK21
	int8_t		MPU6050_LPF;			// MPU6050's internal LPF. Values are 0x06 = 5Hz, (5)10Hz, (4)21Hz, (3)44Hz, (2)94Hz, (1)184Hz LPF, (0)260Hz
#endif

	// Channel configuration
	channel_t	Channel[MAX_OUTPUTS];	// Channel mixing data	

	// Servo menu
	int8_t		Servo_reverse[MAX_OUTPUTS];	// Reversal of output channel
	int8_t		min_travel[MAX_OUTPUTS];	// Minimum output value (-125 to 125)
	int8_t		max_travel[MAX_OUTPUTS];	// Maximum output value (-125 to 125)

	// RC inputs
	uint16_t 	RxChannelZeroOffset[MAX_RC_CHANNELS];	// RC channel offsets for actual radio channels

	// Acc zeros
	int16_t		AccZero[NUMBEROFAXIS];	// Acc calibration results. Note: Acc-Z zero centered on 1G (about +124)
	int16_t		AccZeroNormZ;			// Acc-Z zero for normal Z values
	int16_t		AccZeroInvZ;			// Acc-Z zero for inverted Z values
	int16_t		AccZeroDiff;			// Difference between normal and inverted Acc-Z zeros

	// Gyro zeros
	int16_t		gyroZero[NUMBEROFAXIS];

	// Airspeed zero
	int16_t		AirspeedZero;			// Zero airspeed sensor offset

	// Flight mode
	int8_t		FlightSel;				// User set flight mode

	// Adjusted trims
	int16_t		Rolltrim[FLIGHT_MODES];	// User set trims * 100
	int16_t		Pitchtrim[FLIGHT_MODES];

	// Sticky flags
	uint8_t		Main_flags;				// Non-volatile flags

} CONFIG_STRUCT;

typedef struct
{
	int8_t lower;						// Lower limit for menu item
	int8_t upper;						// Upper limit for menu item
	uint8_t increment;					// Increment for menu item
	uint8_t style;						// 0 = numeral, 1 = text
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
