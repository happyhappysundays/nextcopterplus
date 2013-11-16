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

#define MIXERITEMS 31	// Number of mixer menu items
#define MIXERSTART 192 	// Start of Menu text items
#define MIXOFFSET  85	// Value offsets

//************************************************************
// RC menu items
//************************************************************
	 
const uint8_t MixerMenuText[MIXERITEMS] PROGMEM = 
{
	143,0,0,0,0,143,143,143,143,143,228,0,228,0,228,0,228,0,143,143,143,143,143,228,0,228,0,228,0,228,0
};

const menu_range_t mixer_menu_ranges[MIXERITEMS] PROGMEM = 
{
		{OFF, ON,1,1,OFF},				// Motor marker
		{-125,125,5,0,0},				// P1 Offset (%)
		{-125,125,5,0,0},				// P1.n Offset (%)
		{5,95,5,0,50},					// P1.n Position (%)
		{-125,125,5,0,0},				// P2 Offset (%)

		// Mixer ranges - P1
		{OFF, ON,1,1,OFF},				// roll_gyro
		{OFF, ON,1,1,OFF},				// pitch_gyro
		{OFF, ON,1,1,OFF},				// yaw_gyro
		{OFF, ON,1,1,OFF},				// roll_acc
		{OFF, ON,1,1,OFF},				// pitch_acc
		{SRC1,NOMIX,1,1,NOMIX},			// Source A
		{-125,125,5,0,0},				// Source A volume (%)
		{SRC1,NOMIX,1,1,NOMIX},			// Source B
		{-125,125,5,0,0},				// Source B volume
		{SRC1,NOMIX,1,1,NOMIX},			// Source C
		{-125,125,5,0,0},				// Source C volume
		{SRC1,NOMIX,1,1,NOMIX},			// Source D
		{-125,125,5,0,0},				// Source D volume

		// Mixer ranges - P2
		{OFF, ON,1,1,OFF},				// roll_gyro
		{OFF, ON,1,1,OFF},				// pitch_gyro
		{OFF, ON,1,1,OFF},				// yaw_gyro
		{OFF, ON,1,1,OFF},				// roll_acc
		{OFF, ON,1,1,OFF},				// pitch_acc
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

		// Expand sensor bit values into values array
		if ((temp_sensors & (1 << MotorMarker)) != 0)
		{
			values[0] = ON;
		}
		else
		{
			values[0] = OFF;
		}
		if ((temp_sensors & (1 << RollGyro)) != 0)
		{
			values[5] = ON;
		}
		else
		{
			values[5] = OFF;
		}
		if ((temp_sensors & (1 << PitchGyro)) != 0)
		{
			values[6] = ON;
		}
		else
		{
			values[6] = OFF;
		}
		if ((temp_sensors & (1 << YawGyro)) != 0)
		{
			values[7] = ON;
		}
		else
		{
			values[7] = OFF;
		}
		if ((temp_sensors & (1 << RollAcc)) != 0)
		{
			values[8] = ON;
		}
		else
		{
			values[8] = OFF;
		}
		if ((temp_sensors & (1 << PitchAcc)) != 0)
		{
			values[9] = ON;
		}
		else
		{
			values[9] = OFF;
		}

		temp_sensors = Config.Channel[i].P2_sensors; // debug P2

		if ((temp_sensors & (1 << RollGyro)) != 0)
		{
			values[18] = ON;
		}
		else
		{
			values[18] = OFF;
		}
		if ((temp_sensors & (1 << PitchGyro)) != 0)
		{
			values[19] = ON;
		}
		else
		{
			values[19] = OFF;
		}
		if ((temp_sensors & (1 << YawGyro)) != 0)
		{
			values[20] = ON;
		}
		else
		{
			values[20] = OFF;
		}
		if ((temp_sensors & (1 << RollAcc)) != 0)
		{
			values[21] = ON;
		}
		else
		{
			values[21] = OFF;
		}
		if ((temp_sensors & (1 << PitchAcc)) != 0)
		{
			values[22] = ON;
		}
		else
		{
			values[22] = OFF;
		}

		// Assemble remaining byte data for P1 offset to P2 Offset into array
		memcpy(&values[1],&Config.Channel[i].P1_offset, 4);
		// Assemble remaining byte data for P1_source_a to P1_source_d_volume into array
		memcpy(&values[10],&Config.Channel[i].P1_source_a, 8);
		// Assemble remaining byte data for P2_source_a to P2_source_d_volume into array
		memcpy(&values[23],&Config.Channel[i].P2_source_a, 8);

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
		memcpy(&Config.Channel[i].P1_offset,&values[1], 4);
		// Save modified byte data for P1_source_a to P1_source_d_volume back to Config
		memcpy(&Config.Channel[i].P1_source_a,&values[10], 8);
		// Save modified byte data for P2_source_a to P2_source_d_volume back to Config
		memcpy(&Config.Channel[i].P2_source_a,&values[23], 8);

		temp_sensors = 0; // debug

		// Recompress byte data for servos back into bit values for the sensors byte
		if (values[0] == ON)
		{
			temp_sensors |= (1 << MotorMarker);
		}
			else
		{
			temp_sensors &= ~(1 << MotorMarker);	
		}
		if (values[5] == ON)
		{
			temp_sensors |= (1 << RollGyro);
		}
		else
		{
			temp_sensors &= ~(1 << RollGyro);	
		}
		if (values[6] == ON)
		{
			temp_sensors |= (1 << PitchGyro);
		}
		else
		{
			temp_sensors &= ~(1 << PitchGyro);	
		}
		if (values[7] == ON)
		{
			temp_sensors |= (1 << YawGyro);
		}
		else
		{
			temp_sensors &= ~(1 << YawGyro);	
		}
		if (values[8] == ON)
		{
			temp_sensors |= (1 << RollAcc);
		}
		else
		{
			temp_sensors &= ~(1 << RollAcc);	
		}
		if (values[9] == ON)
		{
			temp_sensors |= (1 << PitchAcc);
		}
		else
		{
			temp_sensors &= ~(1 << PitchAcc);	
		}

		Config.Channel[i].P1_sensors = temp_sensors; // debug
		temp_sensors = 0; // debug

		// Recompress byte data for servos back into bit values for the sensors byte
		if (values[18] == ON)
		{
			temp_sensors |= (1 << RollGyro);
		}
		else
		{
			temp_sensors &= ~(1 << RollGyro);	
		}
		if (values[19] == ON)
		{
			temp_sensors |= (1 << PitchGyro);
		}
		else
		{
			temp_sensors &= ~(1 << PitchGyro);	
		}
		if (values[20] == ON)
		{
			temp_sensors |= (1 << YawGyro);
		}
		else
		{
			temp_sensors &= ~(1 << YawGyro);	
		}
		if (values[21] == ON)
		{
			temp_sensors |= (1 << RollAcc);
		}
		else
		{
			temp_sensors &= ~(1 << RollAcc);	
		}
		if (values[22] == ON)
		{
			temp_sensors |= (1 << PitchAcc);
		}
		else
		{
			temp_sensors &= ~(1 << PitchAcc);	
		}

		Config.Channel[i].P2_sensors = temp_sensors; // debug

		// Save and exit
		if (button == ENTER)
		{
			UpdateLimits();			 // Update travel limits based on percentages
			Save_Config_to_EEPROM(); // Save value and return
		}

	} // while(button != BACK)

	_delay_ms(200);
}




