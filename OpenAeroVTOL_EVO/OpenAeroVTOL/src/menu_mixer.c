//***********************************************************
//* menu_mixer.c
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include "compiledefs.h"
#include <avr/pgmspace.h> 
#include <avr/io.h>
#include <stdbool.h>
#include <string.h>
#include <util/delay.h>
#include <stdlib.h>
#include "io_cfg.h"
#include "init.h"
#include "mugui.h"
#include "glcd_menu.h"
#include "menu_ext.h"
#include "glcd_driver.h"
#include "main.h"
#include "eeprom.h"
#include "mixer.h"

//************************************************************
// Prototypes
//************************************************************

// Menu items
void menu_mixer(uint8_t i);

//************************************************************
// Defines
//************************************************************

#define MIXERITEMS 30	// Number of mixer menu items
#define MIXERSTARTE 190	// Start of Menu text items (Earth)
#define MIXERSTARTM 346	// Start of Menu text items (Model)
#define MIXOFFSET  85	// Value offsets

//************************************************************
// RC menu items
//************************************************************
	 
const uint16_t MixerMenuTextE[MIXERITEMS] PROGMEM =
{
	226,0,0,56,								// Motor control and offsets (4)
	0,0,0,0,0,0,							// Flight controls (6)
	68,68,68,68,68,68,68,68,68,68,68,68,	// Mixer ranges (12)
	421,0,421,0,421,0,421,0					// Other sources (8)
};

const uint16_t MixerMenuTextM[MIXERITEMS] PROGMEM =
{
	226,0,0,56,								// Motor control and offsets (4)
	0,0,0,0,0,0,							// Flight controls (6)
	68,68,68,68,68,68,68,68,68,68,68,68,	// Mixer ranges (12)
	442,0,421,0,442,0,421,0					// Other sources (8)
};

const uint16_t MixerMenuOffsets[MIXERITEMS] PROGMEM =
{
	MIXOFFSET,92,92,92,						// Motor control and offsets (4)
	92,92,92,92,92,92,						// Flight controls (6)
	92,92,92,92,92,92,92,92,92,92,92,92,	// Mixer ranges (12)
	77,77,77,77,77,77,77,77					// Other sources (8)
};

const menu_range_t mixer_menu_ranges[MIXERITEMS] PROGMEM = 
{
		// Motor control and offsets (4)
		{ASERVO,MOTOR,1,1,MOTOR},		// Motor marker (0)
		{0,125,1,0,100},				// P1 throttle volume 
		{0,125,1,0,100},				// P2 throttle volume
		{LINEAR,SQRTSINE,1,1,LINEAR},	// Throttle curves

		// Flight controls (6)
		{-125,125,1,0,0},				// P1 Aileron volume (8)
		{-125,125,1,0,0},				// P2 Aileron volume
		{-125,125,1,0,0},				// P1 Elevator volume
		{-125,125,1,0,0},				// P2 Elevator volume
		{-125,125,1,0,0},				// P1 Rudder volume
		{-125,125,1,0,0},				// P2 Rudder volume

		// Mixer ranges (12)
		{OFF, SCALE,1,1,OFF},			// P1 roll_gyro (14)
		{OFF, SCALE,1,1,OFF},			// P2 roll_gyro
		{OFF, SCALE,1,1,OFF},			// P1 pitch_gyro
		{OFF, SCALE,1,1,OFF},			// P2 pitch_gyro
		{OFF, SCALE,1,1,OFF},			// P1 yaw_gyro
		{OFF, SCALE,1,1,OFF},			// P2 yaw_gyro
		{OFF, SCALE,1,1,OFF},			// P1 roll_acc
		{OFF, SCALE,1,1,OFF},			// P2 roll_acc
		{OFF, SCALE,1,1,OFF},			// P1 pitch_acc
		{OFF, SCALE,1,1,OFF},			// P2 pitch_acc
		{OFF, SCALE,1,1,OFF},			// P1 Z_delta_acc
		{OFF, SCALE,1,1,OFF},			// P2 Z_delta_acc

		// Sources (8)
		{SRC2,NOMIX,1,1,NOMIX},			// P1 Source A (26)
		{-125,125,1,0,0},				// P1 Source A volume
		{SRC2,NOMIX,1,1,NOMIX},			// P2 Source A
		{-125,125,1,0,0},				// P2 Source A volume
		{SRC2,NOMIX,1,1,NOMIX},			// P1 Source B
		{-125,125,1,0,0},				// P1 Source B volume
		{SRC2,NOMIX,1,1,NOMIX},			// P2 Source B
		{-125,125,1,0,0},				// P2 Source B volume
};

//************************************************************
// Main menu-specific setup
//************************************************************

void menu_mixer(uint8_t i)
{
	int8_t *value_ptr;
	menu_range_t range;
	uint16_t text_link = 0;
	uint16_t reference;

	// Set the correct text list for the selected reference
	if (Config.P1_Reference != MODEL)
	{
		reference = MIXERSTARTE;
	}
	else
	{
		reference = MIXERSTARTM;
	}
	
	// If sub-menu item has changed, reset sub-menu positions
	if (menu_flag)
	{
		// Set the correct text list for the selected reference
		if (Config.P1_Reference != MODEL)
		{
			sub_top = MIXERSTARTE;
		}
		else
		{
			sub_top = MIXERSTARTM;
		}
		menu_flag = 0;
	}

	while(button != BACK)
	{
		value_ptr = &Config.Channel[i].Motor_marker;

		// Print menu
		// Set the correct text list for the selected reference
		if (Config.P1_Reference != MODEL)
		{
			print_menu_items(sub_top, reference, value_ptr, (const unsigned char*)mixer_menu_ranges, 0, (const uint16_t*) MixerMenuOffsets, (const uint16_t*)MixerMenuTextE, cursor);
		}
		else
		{
			print_menu_items(sub_top, reference, value_ptr, (const unsigned char*)mixer_menu_ranges, 0, (const uint16_t*) MixerMenuOffsets, (const uint16_t*)MixerMenuTextM, cursor);
		}

		// Handle menu changes
		update_menu(MIXERITEMS, reference, 0, button, &cursor, &sub_top, &menu_temp);
		range = get_menu_range ((const unsigned char*)mixer_menu_ranges, menu_temp - reference);

		if (button == ENTER)
		{
			// Set the correct text list for the selected reference
			if (Config.P1_Reference != MODEL)
			{
				text_link = pgm_read_word(&MixerMenuTextE[menu_temp - reference]);
			}
			else
			{
				text_link = pgm_read_word(&MixerMenuTextM[menu_temp - reference]);
			}
			
			do_menu_item(menu_temp, value_ptr + (menu_temp - reference), 1, range, 0, text_link, false, 0);
		}

		// Update limits when exiting
		if (button == ENTER)
		{
			UpdateLimits();			 // Update travel limits based on percentages
			Save_Config_to_EEPROM(); // Save value and return
			Wait_BUTTON4();			 // Wait for user's finger off the button
		}
	}
}
