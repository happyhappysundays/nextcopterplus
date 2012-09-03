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
	uint16_t	minimum;
	uint16_t	maximum;
	uint16_t	failsafe;
} servo_limits_t;

// Channel mixer definition
// Size = 20 bytes
typedef struct
{
	uint16_t	value;					// Current value
	int8_t		source;					// Source RC input for calculation
	int8_t		source_polarity;		// Normal/reverse RC input
	int8_t		source_volume;			// Percentage of source to pass on
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
	int8_t		min_travel;				// Minimum output value (2250 to 5250)
	int8_t		max_travel;				// Maximum output value
	int8_t		Failsafe;				// Failsafe position
} channel_t;

// PID type
// Size = 3 bytes
typedef struct
{
	int8_t	P_mult;
	int8_t	I_mult;
	int8_t	D_mult;
} PID_mult_t;

typedef struct
{
	int8_t		source_a;				// Source RC input for calculation	
	int8_t		source_a_volume;		// Percentage of source to pass on	
	int8_t		source_b;				// Source RC input for calculation	
	int8_t		source_b_volume;		// Percentage of source to pass on
} mixer_t; 

// Settings structure
// Size =  bytes
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
	int8_t		ThreePos;				// Channel number for ThreePos switch
	int8_t		FlapChan;				// Channel number for second aileron input
	int8_t		Preset1;				// RC presets for camstab
	int8_t		Preset2;
	int8_t		Preset3;
	int8_t		Preset4;

	// Expo items
	uint8_t		AileronExpo;			// Amount of expo on Aileron channel
	uint8_t		ElevatorExpo;			// Amount of expo on Elevator channel
	uint8_t		RudderExpo;				// Amount of expo on Rudder channel
	uint8_t		Differential;			// Amount of differential on Aileron channels
	
	// Autolevel settings
	int8_t		AutoMode;
	//PID_mult_t	A_Roll;					// Acc PID settings
	//PID_mult_t	A_Pitch;
	int8_t		A_Roll_P_mult;			// Acc gain settings
	int8_t		A_Pitch_P_mult;
	int8_t		AccRollZeroTrim;		// User-set ACC trim (+/-127)
	int8_t		AccPitchZeroTrim;

	// Stability PID settings
	int8_t		StabMode;				// Stability switch mode
	PID_mult_t	Roll;					// Gyro PID settings
	PID_mult_t	Pitch;
	PID_mult_t	Yaw;
	int8_t		I_Limits[3];			// I-term limits for all axis (0 to 125%)

	// Servo travel limts
	int16_t		Raw_I_Limits[3];		// Actual, unspanned I-term limits to save recalculation each loop

	// Battery settings
	int16_t		BatteryType;			// LiPo, NiMh
	int16_t		BatteryCells;			// Number of cells (2~5)	
	int16_t		PowerTrigger;			// Trip voltage
	int16_t		MaxVoltage;				// Maximum cell voltage in charged state
	int16_t		MinVoltage;				// Minimum cell voltage in discharge state

	// RC mixer settings
	mixer_t		mixer_data[NUM_MIXERS];
	uint16_t	Mix_value[NUM_MIXERS];	// Mixer values

	// General items
	int8_t		MixMode;				// Aeroplane/Flying Wing/Manual
	int8_t		Orientation;			// Horizontal / vertical
	int8_t		Contrast;				// Contrast setting
	int8_t		Status_timer;			// Status screen timer
	int8_t		RCMix;					// RC mixer enable
	int8_t		LMA_enable;				// LMA setting
	int8_t		CamStab;				// Camstab. Removes dependence on RC input.
	int8_t		Servo_rate;				// Servo rate for camstab (Low = 50Hz, High = 200Hz?)
	int8_t		Acc_LPF;				// LPF for accelerometers
	int8_t		CF_factor;				// Gyro/Acc Complementary Filter mix
	int8_t		AutoCenter;				// Yaw heading hold auto centering
			
	// Non-menu items 
	// Channel configuration
	channel_t	Channel[MAX_OUTPUTS];// Channel mixing data	

	// Misc
	int8_t		Modes;					// Misc flight mode flag


	// RC inputs
	uint16_t 	RxChannelZeroOffset[MAX_RC_CHANNELS];	// RC channel offsets for actual radio channels

	// Acc zeros
	uint16_t	AccRollZero;			// Acc calibration results
	uint16_t	AccPitchZero;
	uint16_t	AccZedZero;

	// 
	uint16_t	Dummy;

} CONFIG_STRUCT;

typedef struct
{
	int16_t lower;						// Lower limit for menu item
	int16_t upper;						// Upper limit for menu item
	uint8_t increment;					// Increment for menu item
	uint8_t style;						// 0 = numeral, 1 = text
	int16_t default_value;				// Default value for this item
} menu_range_t; 

// MWC IMU structures

typedef struct {
  float X;
  float Y;
  float Z;
} t_fp_vector_def;

typedef union {
  float   A[3];
  t_fp_vector_def V;
} t_fp_vector;

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
