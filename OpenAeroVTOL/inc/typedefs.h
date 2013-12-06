/*********************************************************************
 * typedefs.h
 ********************************************************************/

#ifndef TYPE_DEFS_H_
#define TYPE_DEFS_H_

/*********************************************************************
 * Type definitions
 ********************************************************************/

#define MAX_RC_SOURCES 9				// Maximum input channels including non-RX channels
#define MAX_RC_CHANNELS 8				// Maximum input channels from RX
#define PSUEDO_OUTPUTS 16				// Total number of mixer channels
#define MAX_OUTPUTS 8					// Maximum output channels
#define MAX_ZGAIN 200					// Maximum amount of Z-based height dampening

#define	FLIGHT_MODES 2
#define NUMBEROFAXIS 3

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
	int8_t		P1_RevFlags;			// P1 sensor reverse flags (5)
	int8_t		P2_RevFlags;			// P2 sensor reverse flags (5)
	
	// Mixer menu (21 bytes, 32 items)
	int8_t		P1n_position;			// Position of P1.n offset for this output
	int8_t		P1_offset;				// P1 Offset for this output
	int8_t		P1n_offset;				// P1.n Offset for this output
	int8_t		P2_offset;				// P2 Offset for this output
	int8_t		P1_throttle_volume;		// Percentage of throttle to use in P1
	int8_t		P2_throttle_volume;		// Percentage of throttle to use in P2
	int8_t		Throttle_curve;			// Throttle transition curve (Linear, Sine)

	int8_t		P1_sensors;				// Sensor switches (6), motor marker (1)
	int8_t		P2_sensors;				// Sensor switches (6)

	int8_t		P1_source_a;			// Source A for calculation
	int8_t		P1_source_a_volume;		// Percentage of source to use
	int8_t		P2_source_a;			// Source A for calculation
	int8_t		P2_source_a_volume;		// Percentage of source to use
	int8_t		P1_source_b;			// Source B for calculation
	int8_t		P1_source_b_volume;		// Percentage of source to use
	int8_t		P2_source_b;			// Source B for calculation
	int8_t		P2_source_b_volume;		// Percentage of source to use
	int8_t		P1_source_c;			// Source C for calculation
	int8_t		P1_source_c_volume;		// Percentage of source to use
	int8_t		P2_source_c;			// Source C for calculation
	int8_t		P2_source_c_volume;		// Percentage of source to use

} channel_t;

// PID type
typedef struct
{
	int8_t	P_mult;
	int8_t	I_mult;
} PID_mult_t;

// Flight_control type (14)
typedef struct
{
	PID_mult_t	Roll;					// Gyro PI
	int8_t		Roll_limit;				// I-term limits (0 to 125%)
	int8_t		A_Roll_P_mult;			// Acc gain settings
	int8_t		AccRollZeroTrim;		// User-set ACC trim (+/-127)
	PID_mult_t	Pitch;
	int8_t		Pitch_limit;
	int8_t		A_Pitch_P_mult;
	int8_t		AccPitchZeroTrim;
	PID_mult_t	Yaw;
	int8_t		Yaw_limit;
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
	// Servo travel limts
	servo_limits_t	Limits[MAX_OUTPUTS];// Actual, respanned travel limits to save recalculation each loop

	// RC items (10)
	int8_t		RxMode;					// PWM, CPPM or serial types
	int8_t		TxSeq;					// Channel order of transmitter (JR/Futaba etc)
	int8_t		FlightChan;				// Channel number to select flight mode
	int8_t		DynGainSrc;				// Dynamic gain source channel
	int8_t		DynGain;				// Dynamic gain attenuation (0% to 100%)
	int8_t		AileronPol;				// Aileron RC input polarity
	int8_t		ElevatorPol;			// Elevator RC input polarity
	int8_t		RudderPol;				// Rudder RC input polarity
	int8_t		Stick_Lock_rate;		// Axis lock mode stick rate
	int8_t		TransitionSpeed;		// Transition speed/channel 0 = tied to channel, 1 to 5 seconds.

	// Flight mode settings
	flight_control_t FlightMode[FLIGHT_MODES];	// Flight control settings

	// Servo travel limts
	int32_t		Raw_I_Limits[FLIGHT_MODES][NUMBEROFAXIS];		// Actual, unspanned I-term output limits to save recalculation each loop
	int32_t		Raw_I_Constrain[FLIGHT_MODES][NUMBEROFAXIS];	// Actual, unspanned I-term input limits to save recalculation each loop

	// Triggers
	int16_t		PowerTriggerActual;		// LVA alarm * 10;
	
	//Dynamic gain divisor
	int16_t		DynGainDiv;				// Precalculated dynamic gain variable

	// General items (9)
	int8_t		Orientation;			// Horizontal / vertical / upside-down / (others)
	int8_t		Contrast;				// Contrast setting
	int8_t		ArmMode;				// Arming mode on/off
	int8_t		Disarm_timer;			// Auto-disarm setting
	int8_t		Status_timer;			// Status screen timer
	int8_t		Servo_rate;				// Servo rate for camstab (Low = ~50Hz, High = ~300Hz)
	int8_t		Acc_LPF;				// LPF for accelerometers
	int8_t		CF_factor;				// Gyro/Acc Complementary Filter mix
	int8_t		PowerTrigger;			// LVA voltage (0 to 127 = 0 to 12.7V)

	// Channel configuration
	channel_t	Channel[MAX_OUTPUTS];	// Channel mixing data	

	// Servo menu
	int8_t		Servo_reverse[MAX_OUTPUTS];	// Reversal of output channel
	int8_t		min_travel[MAX_OUTPUTS];	// Minimum output value (-125 to 125)
	int8_t		max_travel[MAX_OUTPUTS];	// Maximum output value (-125 to 125)

	// RC inputs
	uint16_t 	RxChannelZeroOffset[MAX_RC_CHANNELS];	// RC channel offsets for actual radio channels

	// Throttle minimum offset
	uint16_t 	ThrottleMinOffset;

	// Acc zeros
	uint16_t	AccZero[NUMBEROFAXIS];	// Acc calibration results

	// Flight mode
	int8_t		FlightSel;				// User set flight mode

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
