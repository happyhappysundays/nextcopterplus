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
#define	FLIGHT_MODES 2					// Number of flight profiles
#define NUMBEROFAXIS 3					// Number of axis (Roll, Pitch, Yaw)
#define NUMBEROFCURVES 6				// Number of curves available
#define NUMBEROFPOINTS 7				// Number of points on a curve
#define NUMBEROFORIENTS 24				// Number board orientations
#define NUMBEROFSOURCES 21				// Number of universal input sources

#define	THROTTLEIDLE 50					// Throttle value below which is considered idle
#define MOTOR_100 1900					// PWM value to produce a 1.9ms throttle pulse regardless of pulse width mode
#define MOTOR_0	1100					// PWM value to produce a 1.1ms throttle pulse regardless of pulse width mode
#define MOTOR_0_SYSTEM 2750				// System value to produce a 1.1ms throttle pulse regardless of pulse width mode
#define	MOTORMIN 1000					// PWM value for throttle cut. 1000 or 1.0ms regardless of pulse width mode

#define SERVO_CENTER 1500				// Servo center position. 1500us
#define SYSTEM_CENTER 3750				// System center position. 1500us
#define	THROTTLEMIN 1000				// Minimum throttle input offset value. 3750-1000 = 2750 or 1.1ms.
										// Not to be confused with MOTORMIN which is a PWM value.
#define THROTTLEOFFSET 1250				// Mixer offset needed to reduce the output center to MOTORMIN

#define LOGLENGTH	20					// Log length for debugging

/*********************************************************************
 * Type definitions
 ********************************************************************/

// Curves (7)
typedef struct
{
	int8_t		Point1;					// First point in the curve
	int8_t		Point2;
	int8_t		Point3;
	int8_t		Point4;
	int8_t		Point5;
	int8_t		Point6;	
	int8_t		Point7;					// Last point in the curve
	int8_t		channel;				// Associated channel
} curve_t;

// Servo limits (4)
typedef struct
{
	int16_t		minimum;
	int16_t		maximum;
} servo_limits_t;

// Flight_control type (20)
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
	int8_t		A_Zed_P_mult;			// Vertical velocity damping
	int8_t		A_Zed_I_mult;
	int8_t		A_Zed_limit;

} flight_control_t;

// Channel mixer definition 34 bytes
typedef struct
{
	int16_t		P1_value;				// Current value of this channel at P1
	int16_t		P2_value;				// Current value of this channel at P2
	
	// Mixer menu (30 bytes, 30 items)
	int8_t		Motor_marker;			// Motor/Servo marker

	int8_t		P1_throttle_volume;		// Percentage of throttle to use in P1
	int8_t		P2_throttle_volume;		// Percentage of throttle to use in P2
	int8_t		Throttle_curve;			// Throttle transition curve (Linear, Sine)

	int8_t		P1_aileron_volume;		// Percentage of aileron to use in P1
	int8_t		P2_aileron_volume;		// Percentage of aileron to use in P2
	int8_t		P1_elevator_volume;		// Percentage of elevator to use in P1
	int8_t		P2_elevator_volume;		// Percentage of elevator to use in P2
	int8_t		P1_rudder_volume;		// Percentage of rudder to use in P1
	int8_t		P2_rudder_volume;		// Percentage of rudder to use in P2

	int8_t		P1_Roll_gyro;			// P1 roll_gyro (OFF/ON/REV/SCALED/REVSCALED)
	int8_t		P2_Roll_gyro;			// P2 roll_gyro
	int8_t		P1_Pitch_gyro;			// P1 pitch_gyro
	int8_t		P2_Pitch_gyro;			// P2 pitch_gyro
	int8_t		P1_Yaw_gyro;			// P1 yaw_gyro
	int8_t		P2_Yaw_gyro;			// P2 yaw_gyro
	int8_t		P1_Roll_acc;			// P1 roll_acc
	int8_t		P2_Roll_acc;			// P2 roll_acc
	int8_t		P1_Pitch_acc;			// P1 pitch_acc
	int8_t		P2_Pitch_acc;			// P2 pitch_acc
	int8_t		P1_Z_delta_acc;			// P1 Z_delta_acc
	int8_t		P2_Z_delta_acc;			// P2 Z_delta_acc

	int8_t		P1_source_a;			// Source A for calculation
	int8_t		P1_source_a_volume;		// Percentage of source to use
	int8_t		P2_source_a;			// Source A for calculation
	int8_t		P2_source_a_volume;		// Percentage of source to use
	int8_t		P1_source_b;			// Source B for calculation
	int8_t		P1_source_b_volume;		// Percentage of source to use
	int8_t		P2_source_b;			// Source B for calculation
	int8_t		P2_source_b_volume;		// Percentage of source to use

} channel_t;

// Config settings structure (size)[offset]
typedef struct
{
	// Signature (1)[0]
	uint8_t		setup;					// Byte to identify if already setup

	// Menu adjustable items
	// RC settings (8)[1]
	int8_t		ChannelOrder[MAX_RC_CHANNELS];	// Assign channel numbers to hard-coded channel order
										// OpenAero2 uses Thr, Ail, Ele, Rud, Gear, Aux1, Aux2, Aux3
										// THROTTLE will always return the correct data for the assigned throttle channel
										// AILERON will always return the correct data for the assigned aileron channel
										// ELEVATOR will always return the correct data for the assigned elevator channel
										// RUDDER will always return the correct data for the assigned rudder channel
	// Servo travel limits (32)[9]
	servo_limits_t	Limits[MAX_OUTPUTS];// Actual, respanned travel limits to save recalculation each loop

	// RC items (12)[41]
	int8_t		RxMode;					// PWM, CPPM or serial types
	int8_t		Servo_rate;				// PWM rate for (Low = ~50Hz, RCSync = as per RX, High = ~200Hz)
	int8_t		PWM_Sync;				// Channel to sync to in PWM mode
	int8_t		TxSeq;					// Channel order of transmitter (JR/Futaba etc)
	int8_t		FlightChan;				// Channel number to select flight mode
	int8_t		TransitionSpeedOut;		// Outbound transition speed/channel 0 = tied to channel, 1 to 40 seconds.
	int8_t		TransitionSpeedIn;		// Inbound transition  speed/channel 0 = tied to channel, 1 to 40 seconds.
	int8_t		Transition_P1;			// Transition point as a percentage 0% to 99%
	int8_t		Transition_P1n;			// Transition point as a percentage 1% to 99%
	int8_t		Transition_P2;			// Transition point as a percentage 1% to 100%
	int8_t		Vibration;				// Vibration test mode on/off
	int8_t		AccVertFilter;			// Acc Z filter for I-terms in 1/100%
	
	// Flight mode settings (40)[53]
	flight_control_t FlightMode[FLIGHT_MODES];	// Flight control settings

	// Servo travel limits (32)[93]
	int32_t		Raw_I_Limits[FLIGHT_MODES][NUMBEROFAXIS+1];		// Actual, unspanned I-term output limits to save recalculation each loop (RPY + Z) 4 x 2 x 4 = 32
	
	// Servo constraints (32)[125]
	int32_t		Raw_I_Constrain[FLIGHT_MODES][NUMBEROFAXIS+1];	// Actual, unspanned I-term input limits to save recalculation each loop (RPY + Z) 4 x 2 x 4 = 32

	// Triggers (2)[157]
	uint16_t	PowerTriggerActual;		// LVA alarm * 10;

	// General items (12)[159]
	int8_t		Orientation_P2;			// P2 orientation
	int8_t		P1_Reference;			// Hover plane of reference	(NO, EARTH, VERT_AP)
	int8_t		Contrast;				// Contrast setting
	int8_t		ArmMode;				// Arming mode on/off
	int8_t		Disarm_timer;			// Auto-disarm setting
	int8_t		PowerTrigger;			// LVA cell voltage (0 to 5 for  = 3.5V to 3.9V)
	int8_t		MPU6050_LPF;			// MPU6050's internal LPF. Values are 0x06 = 5Hz, (5)10Hz, (4)21Hz, (3)44Hz, (2)94Hz, (1)184Hz LPF, (0)260Hz
	int8_t		Acc_LPF;				// LPF for accelerometers
	int8_t		Gyro_LPF;				// LPF for gyros
	int8_t		CF_factor;				// Autolevel correction rate
	int8_t		Preset;					// Mixer preset
	int8_t		Buzzer;					// Buzzer control ON/OFF
	
	// Channel configuration (272)[171]
	channel_t	Channel[MAX_OUTPUTS];	// Channel mixing data	

	// Servo menu (24)[443]
	int8_t		Servo_reverse[MAX_OUTPUTS];	// Reversal of output channel
	int8_t		min_travel[MAX_OUTPUTS];	// Minimum output value (-125 to 125)
	int8_t		max_travel[MAX_OUTPUTS];	// Maximum output value (-125 to 125)

	// RC inputs (16)[467]
	uint16_t 	RxChannelZeroOffset[MAX_RC_CHANNELS];	// RC channel offsets for actual radio channels

	// P1 Acc zeros (12)[483]
	int16_t		AccZero_P1[NUMBEROFAXIS];	// P1 Acc calibration results. Note: Acc-Z zero centered on 1G (about +124)
	int16_t		AccZeroNormZ_P1;			// Acc-Z zero for normal Z values
	int16_t		AccZeroInvZ_P1;				// Acc-Z zero for inverted Z values
	int16_t		AccZeroDiff_P1;				// Difference between normal and inverted Acc-Z zeros

	// Gyro zeros (6)[495]
	int16_t		gyroZero_P1[NUMBEROFAXIS];		// NB. These are now for P1 only

	// Airspeed zero (2)[501]
	int16_t		AirspeedZero;			// Zero airspeed sensor offset

	// Flight mode (1)[503]
	int8_t		FlightSel;				// User set flight mode

	// Adjusted trims (8)[504]
	int16_t		Rolltrim[FLIGHT_MODES];	// User set trims * 100
	int16_t		Pitchtrim[FLIGHT_MODES];

	// Sticky flags (1)[512]
	uint8_t		Main_flags;				// Non-volatile flags

	// Misc (2)[513]
	int8_t		RudderPol;				// Rudder RC input polarity (V1.1 stops here...)
	int8_t		AileronPol;				// Aileron RC input polarity
		
	// Error log (21)[515]
	int8_t		log_pointer;
	int8_t		Log[LOGLENGTH];
	
	// P2 Acc zeros (12)[536]
	int16_t		AccZero_P2[NUMBEROFAXIS];	// P2 Acc calibration results. Note: Acc-Z zero centered on 1G (about +124)
	int16_t		AccZeroNormZ_P2;			// Acc-Z zero for normal Z values
	int16_t		AccZeroInvZ_P2;				// Acc-Z zero for inverted Z values
	int16_t		AccZeroDiff_P2;				// Difference between normal and inverted Acc-Z zeros
	
	// P2 Gyro zeros (6)[548]
	int16_t		gyroZero_P2[NUMBEROFAXIS];		// NB. These are for P2 only

	// Advanced items (1) [554]
	int8_t		Orientation_P1;			// P1 orientation
	
	// Curves (48) [555]
	curve_t		Curve[NUMBEROFCURVES];
	
	// Custom channel order (8) [603]
	int8_t		CustomChannelOrder[MAX_RC_CHANNELS];
	
	// Output offsets (64) [611]
	curve_t		Offsets[MAX_OUTPUTS];
	
	// Misc (1)[675]
	int8_t		ElevatorPol;			// Elevator RC input polarity

	// [676]


		
} CONFIG_STRUCT;

// Misc structures
typedef struct
{
	int8_t lower;						// Lower limit for menu item
	int8_t upper;						// Upper limit for menu item
	uint8_t increment;					// Increment for menu item
	uint8_t style;						// 0 = numeral, 1 = text
	int8_t default_value;				// Default value for this item
} menu_range_t; 

typedef struct
{
	uint16_t	x;
	uint16_t	y;
} mugui_size16_t;

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
