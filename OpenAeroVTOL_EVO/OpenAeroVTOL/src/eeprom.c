//***********************************************************
//* eeprom.c
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include "compiledefs.h"
#include <avr/io.h>
#include <string.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <stdbool.h>
#include "io_cfg.h"
#include <util/delay.h>
#include "mixer.h"
#include "menu_ext.h"
#include "MPU6050.h"

//************************************************************
// Prototypes
//************************************************************

bool Initial_EEPROM_Config_Load(void);
void Save_Config_to_EEPROM(void);
void Set_EEPROM_Default_Config(void);
void eeprom_write_byte_changed(uint8_t *addr, uint8_t value);
void eeprom_write_block_changes(uint8_t *src, uint8_t *dest, uint16_t size);
void Update_V1_0_to_V1_1(void);
void Update_V1_1_to_V1_2(void);
uint8_t convert_filter_V1_0_V1_1(uint8_t);

void Load_eeprom_preset(uint8_t preset);

//************************************************************
// Defines
//************************************************************

#define EEPROM_DATA_START_POS 0	// Make sure Rolf's signature is over-written for safety

// eePROM signature - change for each eePROM structure change to force factory reset or upgrade
#define V1_0_SIGNATURE 0x35		// EEPROM signature for V1.0 (V1.0 release)
#define V1_1_SIGNATURE 0x39		// EEPROM signature for V1.1 (V1.1 release)
#define V1_2_SIGNATURE 0x3D		// EEPROM signature for V1.2 (V1.2 release)

#define MAGIC_NUMBER V1_2_SIGNATURE // Set current signature

// eePROM data update locations
#define RCITEMS_V1_0 41		// RAM location of start of RC items data in V1.0, 1.1 and 1.2
#define GENITEMS_V1_0 136	// RAM location of start of General items data in V1.0, 1.1 and 1.2
#define CHANNEL_V1_0 146	// RAM location of start of Channel data in V1.0, 1.1 and 1.2
// V1.0
#define SERVOREV_V1_0 378	// RAM location of Servo_reverse[] start for V1.0
// V1.1
#define ALCORRECT_V1_1 144 //  RAM location of AL Correct variable in V1.1
#define SERVOREV_V1_1 450	// RAM location of Servo_reverse[] start for V1.1 and V1.2
#define RUDDERPOL_V1_1 520	// RAM location of RudderPol for V1.1
// V1.2
#define V1_2_NEWDATA 522	// RAM location of new data area for V1.2 (size = 43 bytes)
#define ADVANCED_V1_2B10 561// RAM location of Advanced block V1.2B10
#define ADVANCED_V1_2B14 562// RAM location of Advanced block V1.2B14+


//************************************************************
// Code
//************************************************************

const uint8_t	JR[MAX_RC_CHANNELS] PROGMEM 	= {0,1,2,3,4,5,6,7}; 	// JR/Spektrum channel sequence (TAERG123)
const uint8_t	FUTABA[MAX_RC_CHANNELS] PROGMEM = {1,2,0,3,4,5,6,7}; 	// Futaba channel sequence (AETRGF12)
const uint8_t	MPX[MAX_RC_CHANNELS] PROGMEM	= {1,2,3,5,0,4,6,7}; 	// Multiplex channel sequence (AER1TG23)
	
void Save_Config_to_EEPROM(void)
{
	// Write to eeProm
	cli();
	eeprom_write_block_changes((uint8_t*)&Config, (uint8_t*)EEPROM_DATA_START_POS, sizeof(CONFIG_STRUCT));	
	sei();
}

// src is the address in RAM
// dest is the address in eeprom
void eeprom_write_block_changes(uint8_t *src, uint8_t *dest, uint16_t size)
{ 
	uint16_t len;
	uint8_t value;

	for (len = 0; len < size; len++)
	{
		// Get value at src
		value = *src;
		
		// Write the value at src to dest
		eeprom_write_byte_changed(dest, value);
		src++;
		dest++;
	}
}

// addr is the address in eeprom
// value is the value to be written
void eeprom_write_byte_changed(uint8_t *addr, uint8_t value)
{
	if (eeprom_read_byte(addr) != value)
	{
		eeprom_write_byte(addr, value);
	}
}

bool Initial_EEPROM_Config_Load(void)
{
	bool updated = false;
	
	// Read eeProm data into RAM
	eeprom_read_block((void*)&Config, (const void*)EEPROM_DATA_START_POS, sizeof(CONFIG_STRUCT));
	
	// See if we know what to do with the current eeprom data
	// Config.setup holds the magic number from the current EEPROM
	switch(Config.setup)
	{
		case V1_0_SIGNATURE:				// V1.0 detected
			Update_V1_0_to_V1_1();
			updated = true;
			// Fall through...

		case V1_1_SIGNATURE:				// V1.1 detected
			Update_V1_1_to_V1_2();
			updated = true;
			// Fall through...
			
		case V1_2_SIGNATURE:				// V1.2 detected
			// Fall through...
			break;
			
		default:							// Unknown solution - restore to factory defaults
			// Load factory defaults
			Set_EEPROM_Default_Config();
			break;
	}
	
	// Save back to eeprom	
	Save_Config_to_EEPROM();
	
	// Return info regarding eeprom structure changes 
	return updated;
}

//************************************************************
// Config data restructure code
//************************************************************
// Upgrade V1.0 structure to V1.1 structure
void Update_V1_0_to_V1_1(void)
{
	#define		OLDSIZE 29				// Old channel_t was 29 bytes
	#define		NEWSIZE 38				// New channel_t is 38 bytes

	uint8_t		i, j;
	uint8_t		*src;
	uint8_t		*dst;
	uint8_t		mixer_buffer[NEWSIZE * 8]; // 304 bytes
	
	int8_t		P1_sensors;				// Sensor switches (6), motor marker (1)
	int8_t		P2_sensors;				// Sensor switches (6)
	int8_t		P1_scale;				// P1 sensor scale flags (6)
	int8_t		P2_scale;				// P2 sensor scale flags (6)

	int8_t		buffer[12];
	int8_t		temp = 0;
	
	// RC items
	memcpy((void*)&buffer[0],(void*)((&Config.setup) + (RCITEMS_V1_0)),1);		// RxMode
	memcpy((void*)&buffer[1],(void*)((&Config.setup) + (GENITEMS_V1_0 + 5)),1);	// Servo_rate
	memcpy((void*)&buffer[2],(void*)((&Config.setup) + (RCITEMS_V1_0 + 1)),1);	// PWM_Sync
	memcpy((void*)&buffer[3],(void*)((&Config.setup) + (RCITEMS_V1_0 + 2)),1);	// TxSeq
	memcpy((void*)&buffer[4],(void*)((&Config.setup) + (RCITEMS_V1_0 + 3)),1);	// FlightChan
	memcpy((void*)&buffer[5],(void*)((&Config.setup) + (RCITEMS_V1_0 + 7)),1);	// TransitionSpeed
	memcpy((void*)&buffer[6],(void*)((&Config.setup) + (RCITEMS_V1_0 + 8)),1);	// Transition_P1n
	memcpy((void*)&buffer[7],(void*)((&Config.setup) + (RCITEMS_V1_0 + 4)),1);	// AileronPol
	memcpy((void*)&buffer[8],(void*)((&Config.setup) + (RCITEMS_V1_0 + 5)),1);	// ElevatorPol
	memcpy((void*)&buffer[9],(void*)((&Config.setup) + (RCITEMS_V1_0 + 6)),1);	// RudderPol
	
	// Copy back to new RC items structure
	memcpy((void*)((&Config.setup) + (RCITEMS_V1_0)), &buffer, 9);				// RxMode to ElevatorPol (9 items)
		
	// New General items - MPU6050LPF to AL correct (4 items)
	memcpy((void*)&buffer[0],(void*)((&Config.setup) + (GENITEMS_V1_0 + 9)),1);	// MPU6050LPF
	memcpy((void*)&buffer[1],(void*)((&Config.setup) + (GENITEMS_V1_0 + 6)),1);	// AccLPF
	memcpy((void*)&buffer[2],(void*)((&Config.setup) + (GENITEMS_V1_0 + 7)),1);	// GyroLPF
	memcpy((void*)&buffer[3],(void*)((&Config.setup) + (GENITEMS_V1_0 + 8)),1);	// AL correct
	
	// Copy back to new General items structure
	memcpy((void*)((&Config.setup) + (GENITEMS_V1_0 + 5)), &buffer, 4);
	
	// "None" no longer an option for this channel, so set to AUX3
	memcpy((void*)&temp,(void*)((&Config.setup) + (RCITEMS_V1_0 + 4)),1);		// FlightChan

	if (temp == NOCHAN)
	{
		temp = AUX3;
		memcpy((void*)((&Config.setup) + (RCITEMS_V1_0 + 4)), &temp, 1);
	}

	// Set mixer preset to default
	memset((void*)((&Config.setup) + (GENITEMS_V1_0 + 9)), QUADX, 1);			// Preset
	
	// Move data that exists after the channel mixer to new location
	memmove((void*)((&Config.setup) + (SERVOREV_V1_1)), (void*)((&Config.setup) + (SERVOREV_V1_0)), 72); // (New channel_t size - old size = 72)
	
	// Copy the old channel[] structure into buffer, spaced out to match the new structure
	for (i = 0; i < MAX_OUTPUTS; i++)
	{
		src = (void*)((&Config.setup) + (CHANNEL_V1_0));
		dst = (void*)mixer_buffer;
		src += (i * OLDSIZE);			// Step to next old data in (corrupted) config structure
		dst += (i * NEWSIZE);			// Step to next location for new data in the buffer
		memcpy(dst, src, OLDSIZE);		// Move only the old (smaller) data
	}

	// Rearrange one output at a time
	for (i = 0; i < MAX_OUTPUTS; i++)
	{
		// Move all bytes from the OLD P1_offset [4] up by one to make space for the Motor_marker byte
		src = &mixer_buffer[4 + (i * NEWSIZE)];	// The old P1_offset byte
		dst = &mixer_buffer[5 + (i * NEWSIZE)];
		memmove(dst, src, (OLDSIZE - 4));// Move all but P1_value, P2_value

		// Save the old switches
		P1_sensors = mixer_buffer[18 + (i * NEWSIZE)];
		P2_sensors = mixer_buffer[19 + (i * NEWSIZE)];
		P1_scale = mixer_buffer[20 + (i * NEWSIZE)];
		P2_scale = mixer_buffer[21 + (i * NEWSIZE)];
		
		// Take old motor marker switch and convert
		if (P1_sensors & (1 << MotorMarker))
		{
			// Set the new value in the right place
			mixer_buffer[4 + (i * NEWSIZE)] = MOTOR;
		}
		else
		{
			mixer_buffer[4 + (i * NEWSIZE)] = ASERVO;
		}

		// Move the universal source bytes (8) up eight bytes
		src = &mixer_buffer[22 + (i * NEWSIZE)]; // 21 + 1
		dst = &mixer_buffer[30 + (i * NEWSIZE)];
		memmove(dst, src, 8);

		// Convert old "None" settings to new ones
		// Skip every second byte
		for (j = 0; j < 8; j += 2)
		{
			if (mixer_buffer[30 + (i * NEWSIZE) + j] == 13) // 13 was the old "None"
			{
				mixer_buffer[30 + (i * NEWSIZE) + j] = NOMIX;
			}
		}

		// Expand the old switches into new bytes
		// P1 roll gyro
		if (P1_sensors & (1 << RollGyro))
		{
			if (P1_scale & (1 << RollScale))
			{
				mixer_buffer[18 + (i * NEWSIZE)] = SCALE;
			}
			else
			{
				mixer_buffer[18 + (i * NEWSIZE)] = ON;
			}
		}
		else
		{
			mixer_buffer[18 + (i * NEWSIZE)] = OFF;
		}

		// P2 roll gyro
		if (P2_sensors & (1 << RollGyro))
		{
			if (P2_scale & (1 << RollScale))
			{
				mixer_buffer[19 + (i * NEWSIZE)] = SCALE;
			}
			else
			{
				mixer_buffer[19 + (i * NEWSIZE)] = ON;
			}
		}
		else
		{
			mixer_buffer[19 + (i * NEWSIZE)] = OFF;
		}

		// P1 pitch gyro
		if (P1_sensors & (1 << PitchGyro))
		{
			if (P1_scale & (1 << PitchScale))
			{
				mixer_buffer[20 + (i * NEWSIZE)] = SCALE;
			}
			else
			{
				mixer_buffer[20 + (i * NEWSIZE)] = ON;
			}
		}
		else
		{
			mixer_buffer[20 + (i * NEWSIZE)] = OFF;
		}

		// P2 pitch gyro
		if (P2_sensors & (1 << PitchGyro))
		{
			if (P2_scale & (1 << PitchScale))
			{
				mixer_buffer[21 + (i * NEWSIZE)] = SCALE;
			}
			else
			{
				mixer_buffer[21 + (i * NEWSIZE)] = ON;
			}
		}
		else
		{
			mixer_buffer[21 + (i * NEWSIZE)] = OFF;
		}

		// P1 yaw_gyro
		if (P1_sensors & (1 << YawGyro))
		{
			if (P1_scale & (1 << YawScale))
			{
				mixer_buffer[22 + (i * NEWSIZE)] = SCALE;
			}
			else
			{
				mixer_buffer[22 + (i * NEWSIZE)] = ON;
			}
		}
		else
		{
			mixer_buffer[22 + (i * NEWSIZE)] = OFF;
		}

		// P2 yaw gyro
		if (P2_sensors & (1 << YawGyro))
		{
			if (P2_scale & (1 << YawScale))
			{
				mixer_buffer[23 + (i * NEWSIZE)] = SCALE;
			}
			else
			{
				mixer_buffer[23 + (i * NEWSIZE)] = ON;
			}
		}
		else
		{
			mixer_buffer[23 + (i * NEWSIZE)] = OFF;
		}

		// P1 roll acc
		if (P1_sensors & (1 << RollAcc))
		{
			if (P1_scale & (1 << AccRollScale))
			{
				mixer_buffer[24 + (i * NEWSIZE)] = SCALE;
			}
			else
			{
				mixer_buffer[24 + (i * NEWSIZE)] = ON;
			}
		}
		else
		{
			mixer_buffer[24 + (i * NEWSIZE)] = OFF;
		}

		// P2 roll acc
		if (P2_sensors & (1 << RollAcc))
		{
			if (P2_scale & (1 << AccRollScale))
			{
				mixer_buffer[25 + (i * NEWSIZE)] = SCALE;
			}
			else
			{
				mixer_buffer[25 + (i * NEWSIZE)] = ON;
			}
		}
		else
		{
			mixer_buffer[25 + (i * NEWSIZE)] = OFF;
		}

		// P1 pitch acc
		if (P1_sensors & (1 << PitchAcc))
		{
			if (P1_scale & (1 << AccPitchScale))
			{
				mixer_buffer[26 + (i * NEWSIZE)] = SCALE;
			}
			else
			{
				mixer_buffer[26 + (i * NEWSIZE)] = ON;
			}
		}
		else
		{
			mixer_buffer[26 + (i * NEWSIZE)] = OFF;
		}

		// P2 pitch acc
		if (P2_sensors & (1 << PitchAcc))
		{
			if (P2_scale & (1 << AccPitchScale))
			{
				mixer_buffer[27 + (i * NEWSIZE)] = SCALE;
			}
			else
			{
				mixer_buffer[27 + (i * NEWSIZE)] = ON;
			}
		}
		else
		{
			mixer_buffer[27 + (i * NEWSIZE)] = OFF;
		}

		// P1 Z delta acc
		if (P1_sensors & (1 << ZDeltaAcc))
		{
			if (P1_scale & (1 << AccZScale))
			{
				mixer_buffer[28 + (i * NEWSIZE)] = SCALE;
			}
			else
			{
				mixer_buffer[28 + (i * NEWSIZE)] = ON;
			}
		}
		else
		{
			mixer_buffer[28 + (i * NEWSIZE)] = OFF;
		}

		// P2 Z delta acc
		if (P2_sensors & (1 << ZDeltaAcc))
		{
			if (P2_scale & (1 << AccZScale))
			{
				mixer_buffer[29 + (i * NEWSIZE)] = SCALE;
			}
			else
			{
				mixer_buffer[29 + (i * NEWSIZE)] = ON;
			}
		}
		else
		{
			mixer_buffer[29 + (i * NEWSIZE)] = OFF;
		}
	}
	
	// Copy buffer back into new structure
	src = (void*)mixer_buffer;
	dst = (void*)((&Config.setup) + (CHANNEL_V1_0));
	memcpy(dst, src, sizeof(mixer_buffer));
	
	// Convert old filter values to more appropriate ones
	memcpy((void*)&temp,(void*)((&Config.setup) + (GENITEMS_V1_0 + 6)),1);		// Config.Acc_LPF
	temp = convert_filter_V1_0_V1_1(temp);
	memcpy((void*)((&Config.setup) + (GENITEMS_V1_0 + 6)), &temp, 1);

	memcpy((void*)&temp,(void*)((&Config.setup) + (GENITEMS_V1_0 + 7)),1);		// Config.Gyro_LPF
	temp = convert_filter_V1_0_V1_1(temp);
	memcpy((void*)((&Config.setup) + (GENITEMS_V1_0 + 7)), &temp, 1);	
	
	// Finally, copy the RudderPol value up into its new location
	memcpy((void*)((&Config.setup) + (RUDDERPOL_V1_1)),(void*)&buffer[9],1);	// RudderPol
}

// Upgrade V1.1 structure to V1.2
void Update_V1_1_to_V1_2(void)
{
	int8_t	Orientation_P2 = 0;
	int8_t	temp = 0;

	// Save old Config.CF_factor value
	memcpy((void*)&temp,(void*)((&Config.setup) + (ALCORRECT_V1_1)),1);			// Config.CF_factor

	// Convert old Config.CF_factor to new
	// (old) 1 = 10% 2 = 11%, 3 = 12.5%, 4 = 14%, 5 = 17%, 60= 20%, 7 = 25%, 8 = 33%, 9 = 50%, 10 = 100%
	// (newest) 11 = 10%, 10 = 20%, 9 = 30%, 8 = 40%, 7 = 50%, 6 = 60%, 5 = 70%, 4 = 80%, 3 = 90%, 2 = 100%
	switch(temp)
	{
		case 10:
			temp = 2;
			break;
		case 9:
			temp = 7;
			break;
		case 8:
		case 7:
			temp = 9;
			break;
		case 6:
		case 5:
			temp = 10;
			break;
		case 4:
		case 3:
		case 2:
		case 1:
			temp = 11;
			break;	
		default:
			temp = 6;
			break;		
	}
	
	// Write updated Config.CF_factor value
	memcpy((void*)((&Config.setup) + (ALCORRECT_V1_1)),(void*)&temp,1);
		
	// Copy AileronPol from RCitems to its new location
	memcpy((void*)((&Config.setup) + (RUDDERPOL_V1_1 + 1)),(void*)((&Config.setup) + (RCITEMS_V1_0 + 7)),1);
	
	// Set the new Vibe value to OFF
	memset((void*)((&Config.setup) + (RCITEMS_V1_0 + 7)), OFF, 1);

	// Update the orientation byte to be the P2 orientation
	memcpy((void*)&Orientation_P2,(void*)((&Config.setup) + (GENITEMS_V1_0)),1);
	
	// Convert to new 24-orientation system
	switch(Orientation_P2)
	{
		case HORIZONTAL:
			Orientation_P2 = UP_BACK;
			break;
		case VERTICAL:
			Orientation_P2 = RIGHT_DOWN;
			break;
		case UPSIDEDOWN:
			Orientation_P2 = DOWN_BACK;
			break;
		case AFT:
			Orientation_P2 = UP_FRONT;
			break;
		case SIDEWAYS:
			Orientation_P2 = UP_RIGHT;
			break;
		case PITCHUP:
			Orientation_P2 = BACK_DOWN;
			break;
		default:
			Orientation_P2 = UP_BACK;
			break;	
	}

	// Clear new data area at end of data											// Log pointer onwards
	memset((void*)((&Config.setup) + (V1_2_NEWDATA)), 0, 43);
	
	// Move everything from Config.Contrast up by one byte to make room for Config.P1_Reference	
	// AileronPol has already been moved up past RudderPol so we need to add one byte
	memmove((void*)((&Config.setup) + (GENITEMS_V1_0 + 2)), (void*)((&Config.setup) + (GENITEMS_V1_0 + 1)), ((RUDDERPOL_V1_1 + 1) - (GENITEMS_V1_0 + 1))); // (520 + 1 - 136 + 1 = 386 bytes)

	// Save updated orientation
	memcpy((void*)((&Config.setup) + (GENITEMS_V1_0 )),(void*)&Orientation_P2,1);	// Updated P2 orientation value
	memset((void*)((&Config.setup) + (GENITEMS_V1_0 + 1)), NO_ORIENT, 1);			// New P1_Reference
	
	// Set magic number to V1.2 signature
	Config.setup = V1_2_SIGNATURE;
}

// Convert V1.0 filter settings
uint8_t convert_filter_V1_0_V1_1(uint8_t old_filter)
{
	// V1.0 Software LPF conversion table 5Hz, 10Hz, 21Hz, 32Hz, 44Hz, 74Hz, None
	// V1.1 Software LPF conversion table 5Hz, 10Hz, 21Hz, 44Hz, 94Hz, 184Hz, 260Hz, None
	uint8_t new_filter;
	
	switch (old_filter)
	{
		case 0:
			new_filter = HZ5;
			break;
		case 1:
			new_filter = HZ10;
			break;
		case 2:
			new_filter = HZ21;
			break;
		case 3:
			new_filter = HZ44;
			break;
		case 4:
			new_filter = HZ94;
			break;
		case 5:
			new_filter = HZ94;
			break;
		case 6:
			new_filter = NOFILTER;
			break;
		default:
			new_filter = NOFILTER;
			break;
	}

	return new_filter;
}

// Force a factory reset
void Set_EEPROM_Default_Config(void)
{
	uint8_t i;
	
	// Clear entire Config space first
	memset(&Config.setup,0,(sizeof(Config)));

	// Set magic number to current signature
	Config.setup = MAGIC_NUMBER;

	// General
	Config.Orientation_P2 = UP_BACK;
	Config.RxMode = SBUS;
	Config.FlightChan = GEAR;
	Config.ArmMode = ARMABLE;
	Config.Servo_rate = FAST;
	Config.PWM_Sync = GEAR;
	Config.Acc_LPF = HZ21;
	Config.Gyro_LPF = NOFILTER;
	Config.MPU6050_LPF = HZ44;
	Config.CF_factor = 5;
	Config.Disarm_timer = 30;			// Default to 30 seconds
	Config.Transition_P1n = 50;			// Set P1.n point to 50%
	
	// Advanced
	Config.Orientation_P1 = UP_BACK;
	Config.P1_Reference = NO_ORIENT;
	
	// Preset AccZeroNormZ
	Config.AccZeroNormZ_P1		= 128;
	Config.AccZeroNormZ_P2		= 128;

	#ifdef KK2Mini
	Config.Contrast = 30;				// Contrast (KK2 Mini)
	#else
	Config.Contrast = 36;				// Contrast (Everything else)
	#endif
	
	// Servo defaults
	for (i = 0; i < MAX_RC_CHANNELS; i++)
	{
		Config.ChannelOrder[i] = pgm_read_byte(&JR[i]);
		Config.RxChannelZeroOffset[i] = 3750;
	}
	
	// Monopolar throttle is a special case. Set to -100% or -1000
	Config.RxChannelZeroOffset[THROTTLE] = 2750;

	// Preset mixers to safe values
	for (i = 0; i < MAX_OUTPUTS; i++)
	{
		Config.Channel[i].P1n_position	= 50;
		Config.Channel[i].P1_source_a 	= NOMIX;
		Config.Channel[i].P1_source_b 	= NOMIX;
		Config.Channel[i].P2_source_a 	= NOMIX;
		Config.Channel[i].P2_source_b 	= NOMIX;
		Config.min_travel[i] = -100;
		Config.max_travel[i] = 100;
	}

	// Load manual defaults
	Load_eeprom_preset(QUADX);
	Config.Preset = OPTIONS; // Menu will display "Options"
}

void Load_eeprom_preset(uint8_t preset)
{
	uint8_t i;

	// Erase current profile settings
	memset(&Config.FlightMode[P1],0,sizeof(flight_control_t));
	memset(&Config.FlightMode[P2],0,sizeof(flight_control_t));

	// Erase current mixer settings
	memset(&Config.Channel[OUT1],0,sizeof(channel_t) * MAX_OUTPUTS);

	// Preset mixers to safe values
	for (i = 0; i < MAX_OUTPUTS; i++)
	{
		Config.Channel[i].P1n_position	= 50;
		Config.Channel[i].P1_source_a 	= NOMIX;
		Config.Channel[i].P1_source_b 	= NOMIX;
		Config.Channel[i].P2_source_a 	= NOMIX;
		Config.Channel[i].P2_source_b 	= NOMIX;
		Config.min_travel[i] = -100;
		Config.max_travel[i] = 100;
	}

	switch (preset)
	{
		case BLANK:
			Config.ArmMode = ARMABLE;
	
			break;
		
		case QUADP:
			// Preset mixing for primary channels
			Config.ArmMode = ARMABLE;
			
			// Profile 1 (Baseline)
			Config.FlightMode[P1].Roll_P_mult = 50;
			Config.FlightMode[P1].Roll_I_mult = 10;
			Config.FlightMode[P1].Roll_limit = 10;
			Config.FlightMode[P1].Roll_Rate = 2;
			Config.FlightMode[P1].A_Roll_P_mult = 10;
			
			Config.FlightMode[P1].Pitch_P_mult = 50;
			Config.FlightMode[P1].Pitch_I_mult = 10;
			Config.FlightMode[P1].Pitch_limit = 10;
			Config.FlightMode[P1].Pitch_Rate = 2;
			Config.FlightMode[P1].A_Pitch_P_mult = 10;
			
			Config.FlightMode[P1].Yaw_P_mult = 60;
			Config.FlightMode[P1].Yaw_I_mult = 40;
			Config.FlightMode[P1].Yaw_limit = 25;
			Config.FlightMode[P1].Yaw_Rate = 2;
			Config.FlightMode[P1].A_Zed_P_mult = 50;
			
			// Profile 2 (For comparison)
			Config.FlightMode[P2].Roll_P_mult = 40;
			Config.FlightMode[P2].Roll_I_mult = 19;
			Config.FlightMode[P2].Roll_limit = 14;
			Config.FlightMode[P2].Roll_Rate = 3;
			Config.FlightMode[P2].A_Roll_P_mult = 1;
			
			Config.FlightMode[P2].Pitch_P_mult = 40;
			Config.FlightMode[P2].Pitch_I_mult = 19;
			Config.FlightMode[P2].Pitch_limit = 14;
			Config.FlightMode[P2].Pitch_Rate = 3;
			Config.FlightMode[P2].A_Pitch_P_mult = 1;
			
			Config.FlightMode[P2].Yaw_P_mult = 60;
			Config.FlightMode[P2].Yaw_I_mult = 40;
			Config.FlightMode[P2].Yaw_limit = 25;
			Config.FlightMode[P2].Yaw_Rate = 3;
			Config.FlightMode[P2].A_Zed_P_mult = 30;
	
			for (i = 0; i <= OUT4; i++)
			{
				Config.Channel[i].P1_throttle_volume = 100;
				Config.Channel[i].P2_throttle_volume = 100;
				Config.Channel[i].Motor_marker = MOTOR;
				Config.Channel[i].P1_Z_delta_acc = ON;
				Config.Channel[i].P2_Z_delta_acc = ON;
			}

			// OUT1
			Config.Channel[OUT1].P1_aileron_volume = 0;
			Config.Channel[OUT1].P2_aileron_volume = 0;
			Config.Channel[OUT1].P1_elevator_volume = -20;
			Config.Channel[OUT1].P2_elevator_volume = -30;
			Config.Channel[OUT1].P1_rudder_volume = -30;
			Config.Channel[OUT1].P2_rudder_volume = -40;
			Config.Channel[OUT1].P1_Pitch_gyro = ON;
			Config.Channel[OUT1].P1_Pitch_acc = ON;
			Config.Channel[OUT1].P2_Pitch_gyro = ON;
			Config.Channel[OUT1].P2_Pitch_acc = ON;
			Config.Channel[OUT1].P1_Yaw_gyro = ON;
			Config.Channel[OUT1].P2_Yaw_gyro = ON;
	
			// OUT2
			Config.Channel[OUT2].P1_aileron_volume = -20;
			Config.Channel[OUT2].P2_aileron_volume = -30;
			Config.Channel[OUT2].P1_elevator_volume = 0;
			Config.Channel[OUT2].P2_elevator_volume = 0;
			Config.Channel[OUT2].P1_rudder_volume = 30;
			Config.Channel[OUT2].P2_rudder_volume = 40;
			Config.Channel[OUT2].P1_Roll_gyro = ON;
			Config.Channel[OUT2].P1_Roll_acc = ON;
			Config.Channel[OUT2].P2_Roll_gyro = ON;
			Config.Channel[OUT2].P2_Roll_acc = ON;
			Config.Channel[OUT2].P1_Yaw_gyro = ON;
			Config.Channel[OUT2].P2_Yaw_gyro = ON;
	
			// OUT3
			Config.Channel[OUT3].P1_aileron_volume = 0;
			Config.Channel[OUT3].P2_aileron_volume = 0;
			Config.Channel[OUT3].P1_elevator_volume = 20;
			Config.Channel[OUT3].P2_elevator_volume = 30;
			Config.Channel[OUT3].P1_rudder_volume = -30;
			Config.Channel[OUT3].P2_rudder_volume = -40;
			Config.Channel[OUT3].P1_Pitch_gyro = ON;
			Config.Channel[OUT3].P1_Pitch_acc = ON;
			Config.Channel[OUT3].P2_Pitch_gyro = ON;
			Config.Channel[OUT3].P2_Pitch_acc = ON;
			Config.Channel[OUT3].P1_Yaw_gyro = ON;
			Config.Channel[OUT3].P2_Yaw_gyro = ON;
	
			// OUT4
			Config.Channel[OUT4].P1_aileron_volume = 20;
			Config.Channel[OUT4].P2_aileron_volume = 30;
			Config.Channel[OUT4].P1_elevator_volume = 0;
			Config.Channel[OUT4].P2_elevator_volume = 0;
			Config.Channel[OUT4].P1_rudder_volume = 30;
			Config.Channel[OUT4].P2_rudder_volume = 40;
			Config.Channel[OUT4].P1_Roll_gyro = ON;
			Config.Channel[OUT4].P1_Roll_acc = ON;
			Config.Channel[OUT4].P2_Roll_gyro = ON;
			Config.Channel[OUT4].P2_Roll_acc = ON;
			Config.Channel[OUT4].P1_Yaw_gyro = ON;
			Config.Channel[OUT4].P2_Yaw_gyro = ON;		

			// OUT5
			Config.Channel[OUT5].P1_elevator_volume = 75;
			Config.Channel[OUT5].P2_elevator_volume = 100;

			// OUT6
			Config.Channel[OUT6].P1_aileron_volume = 75;
			Config.Channel[OUT6].P2_aileron_volume = 100;

			// OUT7
			Config.Channel[OUT7].P1_rudder_volume = 75;
			Config.Channel[OUT7].P2_rudder_volume = 100;

			// OUT8
			Config.Channel[OUT8].P1_offset = -100;
			Config.Channel[OUT8].P2_offset = 100;			
			break;
				
		case QUADX:
			// Preset mixing for primary channels
			Config.ArmMode = ARMABLE;
			
			// Profile 1 (Baseline)
			Config.FlightMode[P1].Roll_P_mult = 40;
			Config.FlightMode[P1].Roll_I_mult = 10;
			Config.FlightMode[P1].Roll_limit = 10;
			Config.FlightMode[P1].Roll_Rate = 2;
			Config.FlightMode[P1].A_Roll_P_mult = 10;
	
			Config.FlightMode[P1].Pitch_P_mult = 40;
			Config.FlightMode[P1].Pitch_I_mult = 10;
			Config.FlightMode[P1].Pitch_limit = 10;
			Config.FlightMode[P1].Pitch_Rate = 2;
			Config.FlightMode[P1].A_Pitch_P_mult = 10;
	
			Config.FlightMode[P1].Yaw_P_mult = 60;
			Config.FlightMode[P1].Yaw_I_mult = 40;
			Config.FlightMode[P1].Yaw_limit = 25;
			Config.FlightMode[P1].Yaw_Rate = 2;
			Config.FlightMode[P1].A_Zed_P_mult = 50;
	
			// Profile 2 (For comparison)
			Config.FlightMode[P2].Roll_P_mult = 40;
			Config.FlightMode[P2].Roll_I_mult = 19;
			Config.FlightMode[P2].Roll_limit = 14;
			Config.FlightMode[P2].Roll_Rate = 3;
			Config.FlightMode[P2].A_Roll_P_mult = 1;
	
			Config.FlightMode[P2].Pitch_P_mult = 40;
			Config.FlightMode[P2].Pitch_I_mult = 19;
			Config.FlightMode[P2].Pitch_limit = 14;
			Config.FlightMode[P2].Pitch_Rate = 3;
			Config.FlightMode[P2].A_Pitch_P_mult = 1;
	
			Config.FlightMode[P2].Yaw_P_mult = 60;
			Config.FlightMode[P2].Yaw_I_mult = 40;
			Config.FlightMode[P2].Yaw_limit = 25;
			Config.FlightMode[P2].Yaw_Rate = 2;
			Config.FlightMode[P2].A_Zed_P_mult = 30;
	
			for (i = 0; i <= OUT4; i++)
			{
				Config.Channel[i].P1_throttle_volume = 100;
				Config.Channel[i].P2_throttle_volume = 100;
				Config.Channel[i].Motor_marker = MOTOR;
				Config.Channel[i].P1_Roll_gyro = ON;
				Config.Channel[i].P1_Roll_acc = ON;
				Config.Channel[i].P2_Roll_gyro = ON;
				Config.Channel[i].P2_Roll_acc = ON;
				Config.Channel[i].P1_Pitch_gyro = ON;
				Config.Channel[i].P1_Pitch_acc = ON;
				Config.Channel[i].P2_Pitch_gyro = ON;
				Config.Channel[i].P2_Pitch_acc = ON;
				Config.Channel[i].P1_Yaw_gyro = ON;
				Config.Channel[i].P2_Yaw_gyro = ON;
				Config.Channel[i].P1_Z_delta_acc = ON;
				Config.Channel[i].P2_Z_delta_acc = ON;
			}

			// OUT1
			Config.Channel[OUT1].P1_aileron_volume = 15;
			Config.Channel[OUT1].P2_aileron_volume = 20;
			Config.Channel[OUT1].P1_elevator_volume = -15;
			Config.Channel[OUT1].P2_elevator_volume = -20;
			Config.Channel[OUT1].P1_rudder_volume = -40;
			Config.Channel[OUT1].P2_rudder_volume = -50;
	
			// OUT2
			Config.Channel[OUT2].P1_aileron_volume = -15;
			Config.Channel[OUT2].P2_aileron_volume = -20;
			Config.Channel[OUT2].P1_elevator_volume = -15;
			Config.Channel[OUT2].P2_elevator_volume = -20;
			Config.Channel[OUT2].P1_rudder_volume = 40;
			Config.Channel[OUT2].P2_rudder_volume = 50;
	
			// OUT3
			Config.Channel[OUT3].P1_aileron_volume = -15;
			Config.Channel[OUT3].P2_aileron_volume = -20;
			Config.Channel[OUT3].P1_elevator_volume = 15;
			Config.Channel[OUT3].P2_elevator_volume = 20;
			Config.Channel[OUT3].P1_rudder_volume = -40;
			Config.Channel[OUT3].P2_rudder_volume = -50;
	
			// OUT4
			Config.Channel[OUT4].P1_aileron_volume = 15;
			Config.Channel[OUT4].P2_aileron_volume = 20;
			Config.Channel[OUT4].P1_elevator_volume = 15;
			Config.Channel[OUT4].P2_elevator_volume = 20;
			Config.Channel[OUT4].P1_rudder_volume = 40;
			Config.Channel[OUT4].P2_rudder_volume = 50;

			// OUT5
			Config.Channel[OUT5].P1_elevator_volume = 75;
			Config.Channel[OUT5].P2_elevator_volume = 100;
			
			// OUT6
			Config.Channel[OUT6].P1_aileron_volume = 75;
			Config.Channel[OUT6].P2_aileron_volume = 100;
						
			// OUT7
			Config.Channel[OUT7].P1_rudder_volume = 75;
			Config.Channel[OUT7].P2_rudder_volume = 100;
									
			// OUT8
			Config.Channel[OUT8].P1_offset = -100;
			Config.Channel[OUT8].P2_offset = 100;
			break;
		
		case TRICOPTER:
			// Preset simple mixing for primary channels
			Config.ArmMode = ARMABLE;
		
			// Profile 1 (Baseline)
			Config.FlightMode[P1].Roll_P_mult = 40;
			Config.FlightMode[P1].Roll_I_mult = 10;
			Config.FlightMode[P1].Roll_limit = 10;
			Config.FlightMode[P1].Roll_Rate = 2;
			Config.FlightMode[P1].A_Roll_P_mult = 10;
			
			Config.FlightMode[P1].Pitch_P_mult = 40;
			Config.FlightMode[P1].Pitch_I_mult = 10;
			Config.FlightMode[P1].Pitch_limit = 10;
			Config.FlightMode[P1].Pitch_Rate = 2;
			Config.FlightMode[P1].A_Pitch_P_mult = 10;
			
			Config.FlightMode[P1].Yaw_P_mult = 60;
			Config.FlightMode[P1].Yaw_I_mult = 40;
			Config.FlightMode[P1].Yaw_limit = 25;
			Config.FlightMode[P1].Yaw_Rate = 2;
			Config.FlightMode[P1].A_Zed_P_mult = 50;
			
			// Profile 2 (For comparison)
			Config.FlightMode[P2].Roll_P_mult = 40;
			Config.FlightMode[P2].Roll_I_mult = 19;
			Config.FlightMode[P2].Roll_limit = 14;
			Config.FlightMode[P2].Roll_Rate = 3;
			Config.FlightMode[P2].A_Roll_P_mult = 1;
			
			Config.FlightMode[P2].Pitch_P_mult = 40;
			Config.FlightMode[P2].Pitch_I_mult = 19;
			Config.FlightMode[P2].Pitch_limit = 14;
			Config.FlightMode[P2].Pitch_Rate = 3;
			Config.FlightMode[P2].A_Pitch_P_mult = 1;
			
			Config.FlightMode[P2].Yaw_P_mult = 60;
			Config.FlightMode[P2].Yaw_I_mult = 40;
			Config.FlightMode[P2].Yaw_limit = 25;
			Config.FlightMode[P2].Yaw_Rate = 2;
			Config.FlightMode[P2].A_Zed_P_mult = 30;
		
			for (i = 0; i <= OUT3; i++)
			{
				Config.Channel[i].P1_throttle_volume = 100;
				Config.Channel[i].P2_throttle_volume = 100;
				Config.Channel[i].Motor_marker = MOTOR;
			}

			// OUT1
			Config.Channel[OUT1].P1_aileron_volume = 30;
			Config.Channel[OUT1].P2_aileron_volume = 40;
			Config.Channel[OUT1].P1_elevator_volume = -15;
			Config.Channel[OUT1].P2_elevator_volume = -20;
			Config.Channel[OUT1].P1_Roll_gyro = ON;
			Config.Channel[OUT1].P1_Roll_acc = ON;
			Config.Channel[OUT1].P2_Roll_gyro = ON;
			Config.Channel[OUT1].P2_Roll_acc = ON;
			Config.Channel[OUT1].P1_Pitch_gyro = SCALE;
			Config.Channel[OUT1].P1_Pitch_acc = SCALE;
			Config.Channel[OUT1].P2_Pitch_gyro = SCALE;
			Config.Channel[OUT1].P2_Pitch_acc = SCALE;
			Config.Channel[OUT1].P1_Z_delta_acc = ON;
			Config.Channel[OUT1].P2_Z_delta_acc = ON;
		
			// OUT2
			Config.Channel[OUT2].P1_aileron_volume = -30;
			Config.Channel[OUT2].P2_aileron_volume = -40;
			Config.Channel[OUT2].P1_elevator_volume = -15;
			Config.Channel[OUT2].P2_elevator_volume = -20;
			Config.Channel[OUT2].P1_Roll_gyro = ON;
			Config.Channel[OUT2].P1_Roll_acc = ON;
			Config.Channel[OUT2].P2_Roll_gyro = ON;
			Config.Channel[OUT2].P2_Roll_acc = ON;
			Config.Channel[OUT2].P1_Pitch_gyro = SCALE;
			Config.Channel[OUT2].P1_Pitch_acc = SCALE;
			Config.Channel[OUT2].P2_Pitch_gyro = SCALE;
			Config.Channel[OUT2].P2_Pitch_acc = SCALE;
			Config.Channel[OUT2].P1_Z_delta_acc = ON;
			Config.Channel[OUT2].P2_Z_delta_acc = ON;
		
			// OUT3
			Config.Channel[OUT3].P1_elevator_volume = 30;
			Config.Channel[OUT3].P2_elevator_volume = 40;
			Config.Channel[OUT3].P1_Pitch_gyro = SCALE;
			Config.Channel[OUT3].P1_Pitch_acc = SCALE;
			Config.Channel[OUT3].P2_Pitch_gyro = SCALE;
			Config.Channel[OUT3].P2_Pitch_acc = SCALE;
			Config.Channel[OUT3].P1_Z_delta_acc = ON;
			Config.Channel[OUT3].P2_Z_delta_acc = ON;
			
			// OUT4
			Config.Channel[OUT4].Motor_marker = ASERVO;
			Config.Channel[OUT4].P1_rudder_volume = 75;
			Config.Channel[OUT4].P2_rudder_volume = 100;
			Config.Channel[OUT4].P1_Yaw_gyro = ON;
			Config.Channel[OUT4].P2_Yaw_gyro = ON;
			
			// OUT5
			Config.Channel[OUT5].P1_elevator_volume = 75;
			Config.Channel[OUT5].P2_elevator_volume = 100;

			// OUT6
			Config.Channel[OUT6].P1_aileron_volume = 75;
			Config.Channel[OUT6].P2_aileron_volume = 100;

			// OUT7
			Config.Channel[OUT7].P1_rudder_volume = 75;
			Config.Channel[OUT7].P2_rudder_volume = 100;

			// OUT8
			Config.Channel[OUT8].P1_offset = -100;
			Config.Channel[OUT8].P2_offset = 100;
			break;
		
		default:
			break;
	}
}