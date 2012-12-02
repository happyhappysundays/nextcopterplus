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
#define MAX_RC_SOURCES 15				// Maximum input channels including non-RX channels
#define MAX_RC_CHANNELS 8				// Maximum input channels from RX
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

// Input mixer definition
typedef struct
{
	int16_t		value;					// Current value
	int8_t		source_a;				// Source A RC input for calculation
	int8_t		source_a_volume;		// Percentage of source to pass on
	int8_t		source_b;				// Optional source B RC input for calculation
	int8_t		source_b_volume;		// Percentage of source to pass on
	int8_t		roll_gyro;				// Use roll gyro
	int8_t		roll_gyro_polarity;		// Roll gyro normal/reverse
	int8_t		pitch_gyro;				// Use pitch gyro
	int8_t		pitch_gyro_polarity;	// Pitch gyro normal/reverse
	int8_t		yaw_gyro;				// Use yaw gyro
	int8_t		yaw_gyro_polarity;		// Yaw gyro normal/reverse
	int8_t		roll_acc;				// Use roll acc
	int8_t		roll_acc_polarity;		// Roll acc normal/reverse
	int8_t		pitch_acc;				// Use pitch acc
	int8_t		pitch_acc_polarity;		// Pitch acc normal/reverse
	int8_t		output_a;				// Channel A for calculation
	int8_t		output_a_volume;		// Percentage of output to use
	int8_t		output_b;				// Channel B for calculation
	int8_t		output_b_volume;		// Percentage of output to use
	int8_t		output_c;				// Channel C for calculation
	int8_t		output_c_volume;		// Percentage of output to use
	int8_t		output_d;				// Channel D for calculation
	int8_t		output_d_volume;		// Percentage of output to use
	int8_t		min_travel;				// Minimum output value (-125 to 125)
	int8_t		max_travel;				// Maximum output value (-125 to 125)
	int8_t		Failsafe;				// Failsafe position (-125 to 125)
	int8_t		Offset;					// Trim/neutral position (-125 to 125)
} channel_t;

// PID type
typedef struct
{
	int8_t	P_mult;
	int8_t	I_mult;
	int8_t	D_mult;
} PID_mult_t;

// Settings structure
typedef struct
{
	uint8_t	setup;						// Byte to identify if already setup

	// Menu adjustable items
	// RC settings
	uint8_t		ChannelOrder[MAX_RC_CHANNELS];	// Assign channel numbers to hard-coded channel order
										// OpenAeroEvo uses Thr, Ail, Ele, Rud, Gear, Flap, Aux1, Aux2
										// THROTTLE will always return the correct data for the assigned throttle channel
										// AILERON will always return the correct data for the assigned aileron channel
										// ELEVATOR will always return the correct data for the assigned elevator channel
										// RUDDER will always return the correct data for the assigned rudder channel
										// AUX1 to AUX4 are fixed
	// Servo travel limts
	servo_limits_t	Limits[MAX_OUTPUTS];	// Actual, respanned travel limits to save recalculation each loop

	// RC items
	int8_t		TxSeq;					// Channel order of transmitter (JR/Futaba etc)
	int8_t		RxMode;					// PWM or CPPM mode
	int8_t		StabChan;				// Channel number to select stability mode
	int8_t		AutoChan;				// Channel number for Autolevel switch input
	int8_t		FlapChan;				// Channel number for second aileron input

	// Expo items
	uint8_t		AileronExpo;			// Amount of expo on Aileron channel
	uint8_t		ElevatorExpo;			// Amount of expo on Elevator channel
	uint8_t		RudderExpo;				// Amount of expo on Rudder channel
	uint8_t		Differential;			// Amount of differential on Aileron channels
	
	// Autolevel settings
	int8_t		AutoMode;
	int8_t		Autolimit;				// Autolevel switch point (-125% to 125%)
	int8_t		A_Roll_P_mult;			// Acc gain settings
	int8_t		A_Pitch_P_mult;
	int8_t		A_Limits;				// Max inclination (in degrees) via sticks during autolevel 
	int8_t		AccRollZeroTrim;		// User-set ACC trim (+/-127)
	int8_t		AccPitchZeroTrim;
	int8_t		LaunchMode;				// Launch mode on/off
	int8_t		LaunchThrPos;			// Launch mode throttle position

	// Stability PID settings
	int8_t		StabMode;				// Stability switch mode
	int8_t		Stablimit;				// Stability switch point (-125% to 125%)
	int8_t		DynGainSrc;				// Dynamic gain source channel
	int8_t		DynGain;				// Dynamic gain attenuation (0% to 100%)
	PID_mult_t	Roll;					// Gyro PID settings
	PID_mult_t	Pitch;
	PID_mult_t	Yaw;
	int8_t		I_Limits[3];			// I-term limits for all axis (0 to 125%)

	// Servo travel limts
	int32_t		Raw_I_Limits[3];		// Actual, unspanned I-term out limits to save recalculation each loop
	int32_t		Raw_I_Constrain[3];		// Actual, unspanned I-term in limits to save recalculation each loop

	// Triggers
	int16_t		Stabtrigger;			// Actual, unspanned stability trigger to save recalculation each loop
	int16_t		Autotrigger;			// Actual, unspanned autolevel trigger to save recalculation each loop
	int16_t		Launchtrigger;			// Actual, unspanned launch trigger
	uint8_t		HandsFreetrigger;		// Actual, unspanned hands-free trigger

	//Dynamic gain divisor
	int16_t		DynGainDiv;				// Precalculated dynamic gain variable

	// Battery settings
	int16_t		BatteryType;			// LiPo, NiMh
	int16_t		BatteryCells;			// Number of cells (2~5)	
	int16_t		PowerTrigger;			// Trip voltage
	int16_t		MaxVoltage;				// Maximum cell voltage in charged state
	int16_t		MinVoltage;				// Minimum cell voltage in discharge state

	// General items
	int8_t		MixMode;				// Aeroplane/Flying Wing/Manual
	int8_t		Orientation;			// Horizontal / vertical
	int8_t		Contrast;				// Contrast setting
	int8_t		Status_timer;			// Status screen timer
	int8_t		LMA_enable;				// LMA setting
	int8_t		CamStab;				// Camstab. Removes dependence on RC input.
	int8_t		Servo_rate;				// Servo rate for camstab (Low = 50Hz, High = 200Hz?)
	int8_t		Acc_LPF;				// LPF for accelerometers
	int8_t		CF_factor;				// Gyro/Acc Complementary Filter mix
	int8_t		AutoCenter;				// Yaw heading hold auto centering
	int8_t		FlightMode;				// Normal/Fly-by-wire
			
	// Non-menu items 
	// Input channel configuration
	channel_t	Channel[MAX_OUTPUTS];	// RC channel mixing data	

	// RC inputs
	uint16_t 	RxChannelZeroOffset[MAX_RC_CHANNELS];	// RC channel offsets for actual radio channels

	// Acc
	uint16_t	AccRollZero;			// Acc calibration results
	uint16_t	AccPitchZero;
	uint16_t	AccZedZero;

	// 
	uint16_t	Dummy_2;

} CONFIG_STRUCT;

typedef struct
{
	int16_t lower;						// Lower limit for menu item
	int16_t upper;						// Upper limit for menu item
	uint8_t increment;					// Increment for menu item
	uint8_t style;						// 0 = numeral, 1 = text
	int16_t default_value;				// Default value for this item
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
