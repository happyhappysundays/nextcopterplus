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
#define MAX_RC_SOURCES 9				// Maximum input channels including non-RX channels
#define MAX_RC_CHANNELS 8				// Maximum input channels from RX
#define PSUEDO_OUTPUTS 12				// Total number of mixer channels
#define MAX_OUTPUTS 8					// Maximum output channels
#define MIN_OUTPUTS 4					// Minimum output channels
#define NUM_MIXERS 2

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

	// Input mix menu (10)
	int8_t		source_a;				// Source A RC input for calculation
	int8_t		source_a_volume;		// Percentage of source to pass on
	int8_t		source_b;				// Optional source B RC input for calculation
	int8_t		source_b_volume;		// Percentage of source to pass on
	int8_t		source_mix;				// Source RC included in stabilised modes
	int8_t		roll_gyro;				// Use roll gyro
	int8_t		pitch_gyro;				// Use pitch gyro
	int8_t		yaw_gyro;				// Use yaw gyro
	int8_t		roll_acc;				// Use roll acc
	int8_t		pitch_acc;				// Use pitch acc

	// Ouput mix menu (7)
	int8_t		switcher;				// Switch input to activate this mixer
	int8_t		output_b;				// Channel B for calculation
	int8_t		output_b_volume;		// Percentage of output to use
	int8_t		output_c;				// Channel C for calculation
	int8_t		output_c_volume;		// Percentage of output to use
	int8_t		output_d;				// Channel D for calculation
	int8_t		output_d_volume;		// Percentage of output to use
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
	int8_t		Profilelimit;			// Flight mode switch point (-125% to 125%)
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
	// Servo travel limts
	servo_limits_t	Limits[MAX_OUTPUTS];	// Actual, respanned travel limits to save recalculation each loop

	// RC items (14)
	int8_t		RxMode;					// PWM or CPPM mode
	int8_t		TxSeq;					// Channel order of transmitter (JR/Futaba etc)
	int8_t		FlightChan;				// Channel number to select flight mode
	int8_t		FlapChan;				// Channel number for second aileron input
	int8_t		DynGainSrc;				// Dynamic gain source channel
	int8_t		DynGain;				// Dynamic gain attenuation (0% to 100%)
	int8_t		AileronPol;				// Aileron RC input polarity
	int8_t		SecAileronPol;			// Second aileron RC input polarity
	int8_t		ElevatorPol;			// Elevator RC input polarity
	int8_t		RudderPol;				// Rudder RC input polarity
	int8_t		Differential;			// Aileron differential
	int8_t		flapspeed;				// Flap deploy speed
	int8_t		Stick_Lock_rate;		// Axis lock mode stick rate
	int8_t		Deadband;				// RC deadband (%)

	// Failsafe items (5)
	int8_t		FailsafeType;			// Simple or Advanced (Autolevel)
	int8_t		FailsafeThrottle;		// Throttle position in failsafe
	int8_t		FailsafeElevator;		// Elevator trim in failsafe
	int8_t		FailsafeAileron;		// Aileron trim in failsafe
	int8_t		FailsafeRudder;			// Rudder trim in failsafe
	
	// Flight mode settings
	flight_control_t FlightMode[3];		// Flight control settings (3)

	// Servo travel limts
	int32_t		Raw_I_Limits[3];		// Actual, unspanned I-term out limits to save recalculation each loop
	int32_t		Raw_I_Constrain[3];		// Actual, unspanned I-term in limits to save recalculation each loop

	// Triggers
	int16_t		Autotrigger1;			// Actual, unspanned flight mode 0 trigger to save recalculation each loop
	int16_t		Autotrigger2;			// Actual, unspanned flight mode 1 
	int16_t		Autotrigger3;			// Actual, unspanned flight mode 2
	int16_t		Launchtrigger;			// Actual, unspanned launch trigger
	uint8_t		HandsFreetrigger;		// Actual, unspanned hands-free trigger
	
	// Limits
	int16_t		DeadbandLimit;			// Actual deadband limit

	//Dynamic gain divisor
	int16_t		DynGainDiv;				// Precalculated dynamic gain variable

	// Battery settings (5)
	int8_t		BatteryType;			// LiPo, NiMh
	int8_t		BatteryCells;			// Number of cells (0~12)	
	int8_t		PowerTrigger;			// Trip voltage (0 to 127 = 0 to 25.4V)
	int8_t		MaxVoltage;				// Maximum cell voltage in charged state (0 to 127 = 0 to 508mV)
	int8_t		MinVoltage;				// Minimum cell voltage in discharge state

	// General items (14)
	int8_t		MixMode;				// Aeroplane/Flying Wing/Camstab
	int8_t		Orientation;			// Horizontal / vertical / upside-down / (others)
	int8_t		Contrast;				// Contrast setting
	int8_t		Status_timer;			// Status screen timer
	int8_t		LMA_enable;				// LMA setting
	int8_t		CamStab;				// Camstab. Removes dependence on RC input.
	int8_t		Servo_rate;				// Servo rate for camstab (Low = 50Hz, High = 200Hz?)
	int8_t		AutoCenter;				// Yaw heading hold auto centering ***
	int8_t		Acc_LPF;				// LPF for accelerometers
	int8_t		CF_factor;				// Gyro/Acc Complementary Filter mix
	int8_t		IMUType;				// IMU style (old/new)
	int8_t		LaunchMode;				// Launch mode on/off
	int8_t		LaunchThrPos;			// Launch mode throttle position
	int8_t		LaunchDelay;			// Launch mode delay time

	// Non-menu items 
	// Input channel configuration
	channel_t	Channel[PSUEDO_OUTPUTS];	// Channel mixing data	

	// Servo menu
	int8_t		Servo_reverse[MAX_OUTPUTS];	// Reversal of output channel
	int8_t		Offset[MAX_OUTPUTS];		// Trim/neutral position (-125 to 125)
	int8_t		min_travel[MAX_OUTPUTS];	// Minimum output value (-125 to 125)
	int8_t		max_travel[MAX_OUTPUTS];	// Maximum output value (-125 to 125)
	int8_t		Failsafe[MAX_OUTPUTS];		// Failsafe position (-125 to 125)

	// RC inputs
	uint16_t 	RxChannelZeroOffset[MAX_RC_CHANNELS];	// RC channel offsets for actual radio channels

	// Acc
	uint16_t	AccZero[3];				// Acc calibration results
	
	// Menu channel number
	int8_t		MenuChannel;			// Current M1 to M8 channel

	// Flight mode
	int8_t		Flight;					// Flight mode in use

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
