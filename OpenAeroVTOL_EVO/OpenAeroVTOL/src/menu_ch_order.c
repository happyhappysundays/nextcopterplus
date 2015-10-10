//***********************************************************
//* menu_channel.c
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include <avr/pgmspace.h> 
#include <avr/io.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <util/delay.h>
#include "io_cfg.h"
#include "init.h"
#include "mugui.h"
#include "glcd_menu.h"
#include "menu_ext.h"
#include "glcd_driver.h"
#include "main.h"
#include "eeprom.h"
#include "mixer.h"
#include "imu.h"

//************************************************************
// Prototypes
//************************************************************

// Menu items
void menu_channel(void);

//************************************************************
// Defines
//************************************************************

#define CHSTART 384 // Start of Menu text items
#define CHOFFSET 85	// LCD offsets
#define CHTEXT 105 	// Start of value text items
#define CHITEMS 8 	// Number of menu items

//************************************************************
// RC menu items
//************************************************************

const uint16_t ChMenuText[MAX_RC_CHANNELS] PROGMEM = {CHTEXT, CHTEXT, CHTEXT, CHTEXT, CHTEXT, CHTEXT, CHTEXT, CHTEXT};
	
const uint16_t ChMenuOffsets[MAX_RC_CHANNELS] PROGMEM = {CHOFFSET, CHOFFSET, CHOFFSET, CHOFFSET, CHOFFSET, CHOFFSET, CHOFFSET, CHOFFSET};

const menu_range_t Ch_menu_ranges[MAX_RC_CHANNELS] PROGMEM = 
{
	// Channels (8)
	{THROTTLE,AUX3,1,1,THROTTLE},	// Ch.1
	{THROTTLE,AUX3,1,1,AILERON},	// Ch.2
	{THROTTLE,AUX3,1,1,ELEVATOR},	// Ch.3
	{THROTTLE,AUX3,1,1,RUDDER},		// Ch.4
	{THROTTLE,AUX3,1,1,GEAR},		// Ch.5
	{THROTTLE,AUX3,1,1,AUX1}, 		// Ch.6
	{THROTTLE,AUX3,1,1,AUX2},		// Ch.7
	{THROTTLE,AUX3,1,1,AUX3}, 		// Ch.8
};

//************************************************************
// Main menu-specific setup
//************************************************************

void menu_channel(void)
{
	int8_t *value_ptr;
	int8_t i = 0; 
	menu_range_t range;
	uint8_t text_link;
	uint16_t reference = CHSTART;

	// If sub-menu item has changed, reset sub-menu positions
	if (menu_flag)
	{
		sub_top = CHSTART;
		menu_flag = 0;
	}

	while(button != BACK)
	{
		value_ptr = &Config.CustomChannelOrder[0];

		// Print menu
		print_menu_items(sub_top, reference, value_ptr, (const unsigned char*)Ch_menu_ranges, 0, (const uint16_t*)ChMenuOffsets, (const uint16_t*)ChMenuText, cursor);

		// Handle menu changes
		update_menu(CHITEMS, reference, 0, button, &cursor, &sub_top, &menu_temp);
		range = get_menu_range ((const unsigned char*)Ch_menu_ranges, (menu_temp - reference));

		if (button == ENTER)
		{
			text_link = pgm_read_word(&ChMenuText[menu_temp - reference]);
			do_menu_item(menu_temp, value_ptr + (menu_temp - reference), 1, range, 0, text_link, false, 0);
		}

		// Update limits when exiting
		if (button == ENTER)
		{
			// Update current channel order with the custom one on exit
			if (Config.TxSeq == CUSTOM)
			{
				for (i = 0; i < MAX_RC_CHANNELS; i++)
				{
					Config.ChannelOrder[i] = Config.CustomChannelOrder[i];
				}
			}
			
			Save_Config_to_EEPROM(); // Save value and return
			Wait_BUTTON4();			 // Wait for user's finger off the button
		}
	}
}

