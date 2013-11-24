//***********************************************************
//* menu_mixer.c
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include <avr/pgmspace.h> 
#include <avr/io.h>
#include <stdbool.h>
#include <string.h>
#include <util/delay.h>
#include "..\inc\io_cfg.h"
#include "..\inc\init.h"
#include "..\inc\mugui.h"
#include "..\inc\glcd_menu.h"
#include "..\inc\menu_ext.h"
#include "..\inc\glcd_driver.h"
#include "..\inc\main.h"
#include "..\inc\eeprom.h"
#include "..\inc\mixer.h"

//************************************************************
// Prototypes
//************************************************************

// Menu items
void menu_mixer(uint8_t i);

//************************************************************
// Defines
//************************************************************

#define MIXERITEMS 33	// Number of mixer menu items
#define MIXERSTART 192 	// Start of Menu text items
#define MIXOFFSET  85	// Value offsets

//************************************************************
// RC menu items
//************************************************************
	 
const uint8_t MixerMenuText[MIXERITEMS] PROGMEM = 
{
	46,0,0,0,0,143,143,143,143,143,143,228,0,228,0,228,0,228,0,143,143,143,143,143,143,228,0,228,0,228,0,228,0
};

const menu_range_t mixer_menu_ranges[MIXERITEMS] PROGMEM = 
{
		{SERVO,MOTOR,1,1,SERVO},		// Motor marker
		{5,95,5,0,50},					// P1.n Position (%)
		{-125,125,5,0,0},				// P1 Offset (%)
		{-125,125,5,0,0},				// P1.n Offset (%)
		{-125,125,5,0,0},				// P2 Offset (%)

		// Mixer ranges - P1
		{OFF, REV,1,1,OFF},				// roll_gyro
		{OFF, REV,1,1,OFF},				// pitch_gyro
		{OFF, REV,1,1,OFF},				// yaw_gyro
		{OFF, REV,1,1,OFF},				// roll_acc
		{OFF, REV,1,1,OFF},				// pitch_acc
		{OFF, REV,1,1,OFF},				// Z_delta_acc
		{SRC1,NOMIX,1,1,NOMIX},			// Source A
		{-125,125,5,0,0},				// Source A volume (%)
		{SRC1,NOMIX,1,1,NOMIX},			// Source B
		{-125,125,5,0,0},				// Source B volume
		{SRC1,NOMIX,1,1,NOMIX},			// Source C
		{-125,125,5,0,0},				// Source C volume
		{SRC1,NOMIX,1,1,NOMIX},			// Source D
		{-125,125,5,0,0},				// Source D volume

		// Mixer ranges - P2
		{OFF, REV,1,1,OFF},				// roll_gyro
		{OFF, REV,1,1,OFF},				// pitch_gyro
		{OFF, REV,1,1,OFF},				// yaw_gyro
		{OFF, REV,1,1,OFF},				// roll_acc
		{OFF, REV,1,1,OFF},				// pitch_acc
		{OFF, REV,1,1,OFF},				// Z_delta_acc
		{SRC1,NOMIX,1,1,NOMIX},			// Source A
		{-125,125,5,0,0},				// Source A volume (%)
		{SRC1,NOMIX,1,1,NOMIX},			// Source B
		{-125,125,5,0,0},				// Source B volume
		{SRC1,NOMIX,1,1,NOMIX},			// Source C
		{-125,125,5,0,0},				// Source C volume
		{SRC1,NOMIX,1,1,NOMIX},			// Source D
		{-125,125,5,0,0},				// Source D volume
};

//************************************************************
// Main menu-specific setup
//************************************************************

void menu_mixer(uint8_t i)
{
	static uint8_t mix_top = MIXERSTART;
	int8_t *value_ptr;
	int8_t temp_sensors;
	int8_t temp_reverse;

	int8_t values[MIXERITEMS];
	menu_range_t range;
	uint8_t text_link = 0;

	// If submenu item has changed, reset submenu positions
	if (menu_flag)
	{
		mix_top = MIXERSTART;
		menu_flag = 0;
	}

	while(button != BACK)
	{
		value_ptr = &values[0];

		// Load values from eeprom
		temp_sensors = Config.Channel[i].P1_sensors; // debug P1
		temp_reverse = Config.Channel[i].P1_RevFlags;

		// Expand sensor bit values into values array
		if ((temp_sensors & (1 << MotorMarker)) != 0)
		{
			values[0] = ON;
		}
		else
		{
			values[0] = OFF;
		}

		// P1 roll gyro
		if ((temp_sensors & (1 << RollGyro)) != 0)
		{
			if ((temp_reverse & (1 << RollRev)) != 0)
			{	
				values[5] = REV;
			}
			else
			{
				values[5] = ON;
			}
		}
		else
		{
			values[5] = OFF;
		}

		// P1 pitch gyro
		if ((temp_sensors & (1 << PitchGyro)) != 0)
		{
			if ((temp_reverse & (1 << PitchRev)) != 0)
			{	
				values[6] = REV;
			}
			else
			{
				values[6] = ON;
			}
		}
		else
		{
			values[6] = OFF;
		}


		// P1 yaw_gyro
		if ((temp_sensors & (1 << YawGyro)) != 0)
		{
			if ((temp_reverse & (1 << YawRev)) != 0)
			{	
				values[7] = REV;
			}
			else
			{
				values[7] = ON;
			}
		}
		else
		{
			values[7] = OFF;
		}


		// P1 roll acc
		if ((temp_sensors & (1 << RollAcc)) != 0)
		{
			if ((temp_reverse & (1 << AccRollRev)) != 0)
			{	
				values[8] = REV;
			}
			else
			{
				values[8] = ON;
			}
		}
		else
		{
			values[8] = OFF;
		}


		// P1 pitch acc
		if ((temp_sensors & (1 << PitchAcc)) != 0)
		{
			if ((temp_reverse & (1 << AccPitchRev)) != 0)
			{	
				values[9] = REV;
			}
			else
			{
				values[9] = ON;
			}
		}
		else
		{
			values[9] = OFF;
		}

		// P1 Z delta acc
		if ((temp_sensors & (1 << ZDeltaAcc)) != 0)
		{
			if ((temp_reverse & (1 << AccZRev)) != 0)
			{	
				values[10] = REV;
			}
			else
			{
				values[10] = ON;
			}
		}
		else
		{
			values[10] = OFF;
		}

		// P2 roll gyro
		temp_sensors = Config.Channel[i].P2_sensors; // debug P2
		temp_reverse = Config.Channel[i].P2_RevFlags;

		if ((temp_sensors & (1 << RollGyro)) != 0)
		{
			if ((temp_reverse & (1 << RollRev)) != 0)
			{	
				values[19] = REV;
			}
			else
			{
				values[19] = ON;
			}
		}
		else
		{
			values[19] = OFF;
		}

		// P2 pitch gyro
		if ((temp_sensors & (1 << PitchGyro)) != 0)
		{
			if ((temp_reverse & (1 << PitchRev)) != 0)
			{	
				values[20] = REV;
			}
			else
			{
				values[20] = ON;
			}
		}
		else
		{
			values[20] = OFF;
		}

		// P2 yaw gyro
		if ((temp_sensors & (1 << YawGyro)) != 0)
		{
			if ((temp_reverse & (1 << YawRev)) != 0)
			{	
				values[21] = REV;
			}
			else
			{
				values[21] = ON;
			}
		}
		else
		{
			values[21] = OFF;
		}

		// P2 roll acc
		if ((temp_sensors & (1 << RollAcc)) != 0)
		{
			if ((temp_reverse & (1 << AccRollRev)) != 0)
			{	
				values[22] = REV;
			}
			else
			{
				values[22] = ON;
			}
		}
		else
		{
			values[22] = OFF;
		}

		// P2 pitch acc
		if ((temp_sensors & (1 << PitchAcc)) != 0)
		{
			if ((temp_reverse & (1 << AccPitchRev)) != 0)
			{	
				values[23] = REV;
			}
			else
			{
				values[23] = ON;
			}
		}
		else
		{
			values[23] = OFF;
		}

		// P1 Z delta acc
		if ((temp_sensors & (1 << ZDeltaAcc)) != 0)
		{
			if ((temp_reverse & (1 << AccZRev)) != 0)
			{	
				values[24] = REV;
			}
			else
			{
				values[24] = ON;
			}
		}
		else
		{
			values[24] = OFF;
		}

		// Assemble remaining byte data for P1 offset to P2 Offset into array
		memcpy(&values[1],&Config.Channel[i].P1n_position, 4);
		// Assemble remaining byte data for P1_source_a to P1_source_d_volume into array
		memcpy(&values[11],&Config.Channel[i].P1_source_a, 8);
		// Assemble remaining byte data for P2_source_a to P2_source_d_volume into array
		memcpy(&values[25],&Config.Channel[i].P2_source_a, 8);

		// Print menu
		print_menu_items(mix_top, MIXERSTART, value_ptr, 1, (prog_uchar*)mixer_menu_ranges, 0, MIXOFFSET, (prog_uchar*)MixerMenuText, cursor);

		// Handle menu changes
		update_menu(MIXERITEMS, MIXERSTART, 0, button, &cursor, &mix_top, &menu_temp);
		range = get_menu_range ((prog_uchar*)mixer_menu_ranges, menu_temp - MIXERSTART);

		if (button == ENTER)
		{
			text_link = pgm_read_byte(&MixerMenuText[menu_temp - MIXERSTART]);
			do_menu_item(menu_temp, value_ptr + (menu_temp - MIXERSTART), 1, range, 0, text_link, false, 0);
		}

		// Save modified byte data for P1 offset to P2 Offset back to Config
		memcpy(&Config.Channel[i].P1n_position,&values[1], 4);
		// Save modified byte data for P1_source_a to P1_source_d_volume back to Config
		memcpy(&Config.Channel[i].P1_source_a,&values[11], 8);
		// Save modified byte data for P2_source_a to P2_source_d_volume back to Config
		memcpy(&Config.Channel[i].P2_source_a,&values[25], 8);

		temp_sensors = 0; // debug
		temp_reverse = 0;

		// Recompress byte data for servos back into bit values for the sensors byte
		if (values[0] == ON)
		{
			temp_sensors |= (1 << MotorMarker);
		}
			else
		{
			temp_sensors &= ~(1 << MotorMarker);	
		}

		// P1 roll gyro
		switch (values[5])
		{
			case OFF:
				break;
			case REV:
				temp_reverse |= (1 << RollRev);
			case ON:
				temp_sensors |= (1 << RollGyro);
				break;
		}
		// P1 pitch gyro
		switch (values[6])
		{
			case OFF:
				break;
			case REV:
				temp_reverse |= (1 << PitchRev);
			case ON:
				temp_sensors |= (1 << PitchGyro);
				break;
		}
		// P1 yaw gyro
		switch (values[7])
		{
			case OFF:
				break;
			case REV:
				temp_reverse |= (1 << YawRev);
			case ON:
				temp_sensors |= (1 << YawGyro);
				break;
		}
		// P1 roll acc
		switch (values[8])
		{
			case OFF:
				break;
			case REV:
				temp_reverse |= (1 << AccRollRev);
			case ON:
				temp_sensors |= (1 << RollAcc);
				break;
		}
		// P1 pitch acc
		switch (values[9])
		{
			case OFF:
				break;
			case REV:
				temp_reverse |= (1 << AccPitchRev);
			case ON:
				temp_sensors |= (1 << PitchAcc);
				break;
		}
		// P1 Z delta acc
		switch (values[10])
		{
			case OFF:
				break;
			case REV:
				temp_reverse |= (1 << AccZRev);
			case ON:
				temp_sensors |= (1 << ZDeltaAcc);
				break;
		}

		Config.Channel[i].P1_sensors = temp_sensors;
		Config.Channel[i].P1_RevFlags = temp_reverse;

		temp_sensors = 0;
		temp_reverse = 0;

		// P2 roll gyro
		switch (values[19])
		{
			case OFF:
				break;
			case REV:
				temp_reverse |= (1 << RollRev);
			case ON:
				temp_sensors |= (1 << RollGyro);
				break;
		}
		// P2 pitch gyro
		switch (values[20])
		{
			case OFF:
				break;
			case REV:
				temp_reverse |= (1 << PitchRev);
			case ON:
				temp_sensors |= (1 << PitchGyro);
				break;
		}
		// P2 yaw gyro
		switch (values[21])
		{
			case OFF:
				break;
			case REV:
				temp_reverse |= (1 << YawRev);
			case ON:
				temp_sensors |= (1 << YawGyro);
				break;
		}
		// P2 roll acc
		switch (values[22])
		{
			case OFF:
				break;
			case REV:
				temp_reverse |= (1 << AccRollRev);
			case ON:
				temp_sensors |= (1 << RollAcc);
				break;
		}
		// P2 pitch acc
		switch (values[23])
		{
			case OFF:
				break;
			case REV:
				temp_reverse |= (1 << AccPitchRev);
			case ON:
				temp_sensors |= (1 << PitchAcc);
				break;
		}
		// P2 Z delta acc
		switch (values[24])
		{
			case OFF:
				break;
			case REV:
				temp_reverse |= (1 << AccZRev);
			case ON:
				temp_sensors |= (1 << ZDeltaAcc);
				break;
		}

		Config.Channel[i].P2_sensors = temp_sensors;
		Config.Channel[i].P2_RevFlags = temp_reverse;

		// Save and exit
		if (button == ENTER)
		{
			UpdateLimits();			 // Update travel limits based on percentages
			Save_Config_to_EEPROM(); // Save value and return
		}

	} // while(button != BACK)

	_delay_ms(200);
}




