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

#define MIXERITEMS 23 	// Number of menu items
#define INPUTITEMS 16
#define OUTPUTITEMS 7

#define MIXERSTART 205 	// Start of Menu text items
#define MIXOFFSET  80	// Value offsets

//************************************************************
// RC menu items
//************************************************************
	 
const uint8_t MixerMenuText[MIXERITEMS] PROGMEM = 
{
	62,105,0,105,0,143,143,141,143,141,143,141,143,141,143,141,
	62,62,0,62,0,62,0
};

const menu_range_t mixer_menu_ranges[] PROGMEM = 
{
	// Input mixer ranges (16)
	{CH1,CH8,1,1,CH1},				// Ch. number
	{THROTTLE,NOCHAN,1,1,CH1}, 		// Source A
	{-125,125,5,0,100},				// Source A volume (%)
	{THROTTLE,NOCHAN,1,1,NOCHAN}, 	// Source B
	{-125,125,5,0,0},				// Source B volume (%)
	{OFF, ON,1,1,OFF},				// Source mix enable
	{OFF, ON,1,1,OFF},				// roll_gyro
	{NORMAL, REVERSED,1,1,NORMAL},	// polarity
	{OFF, ON,1,1,OFF},				// pitch_gyro
	{NORMAL, REVERSED,1,1,NORMAL},	// polarity
	{OFF, ON,1,1,OFF},				// yaw_gyro
	{NORMAL, REVERSED,1,1,NORMAL},	// polarity
	{OFF, REVERSED,1,1,OFF},		// roll_acc
	{NORMAL, REVERSED,1,1,NORMAL},	// polarity
	{OFF, REVERSED,1,1,OFF},		// pitch_acc
	{NORMAL, REVERSED,1,1,NORMAL},	// polarity

	// Output mixer ranges (7)
	{CH1,CH8,1,1,CH1},				// Ch. number 62
	{CH1,UNUSED,1,1,UNUSED},		// Output B
	{-125,125,5,0,0},				// Output B volume
	{CH1,UNUSED,1,1,UNUSED},		// Output C
	{-125,125,5,0,0},				// Output C volume
	{CH1,UNUSED,1,1,UNUSED},		// Output D
	{-125,125,5,0,0},				// Output D volume
};

//************************************************************
// Main menu-specific setup
//************************************************************

void menu_mixer(uint8_t section)
{
	static	uint8_t mix_top = MIXERSTART;
	static	uint8_t old_section;

	int8_t values[MIXERITEMS];
	menu_range_t range;
	uint8_t text_link = 0;

	uint8_t offset;			// Index into channel structure
	uint8_t	items;			// Items in group

	// Get mixer menu offsets
	// 1 = input mixer data, 2 = output mixer data
	switch(section)
	{
		case 1:
			offset = 0;
			items = INPUTITEMS;
			break;
		case 2:
			offset = INPUTITEMS;
			items = OUTPUTITEMS;
			break;
		default:
			offset = 0;
			items = INPUTITEMS;
			break;
	}

	// If submenu item has changed, reset submenu positions
	if (section != old_section)
	{
		mix_top = MIXERSTART;
		old_section = section;
	}

	while(button != BACK)
	{
		// Load values from eeprom and insert channel number at the top of each - messy
		values[0] = Config.MenuChannel;
		memcpy(&values[1],&Config.Channel[Config.MenuChannel].source_a,sizeof(int8_t) * (INPUTITEMS - 1));
		values[INPUTITEMS] = Config.MenuChannel;
		memcpy(&values[INPUTITEMS + 1],&Config.Channel[Config.MenuChannel].output_b,sizeof(int8_t) * (OUTPUTITEMS - 1));

		// Print menu
		print_menu_items(mix_top + offset, MIXERSTART, &values[0], MIXERITEMS,(prog_uchar*)mixer_menu_ranges, MIXOFFSET, (prog_uchar*)MixerMenuText, cursor);

		// Handle menu changes
		update_menu(items, MIXERSTART, offset, button, &cursor, &mix_top, &menu_temp);
		range = get_menu_range ((prog_uchar*)mixer_menu_ranges, menu_temp - MIXERSTART);

		if (button == ENTER)
		{
			text_link = pgm_read_byte(&MixerMenuText[menu_temp - MIXERSTART]);
			values[menu_temp - MIXERSTART] = do_menu_item(menu_temp, values[menu_temp - MIXERSTART], range, 0, text_link, false, 0);
		}

		// Save modified data back to Config
		// Copy channel number back to global
		switch(section)
		{
			case 1:
				memcpy(&Config.Channel[Config.MenuChannel].source_a,&values[1],sizeof(int8_t) * (INPUTITEMS - 1));
				Config.MenuChannel = values[0];
				break;
			case 2:
				memcpy(&Config.Channel[Config.MenuChannel].output_b,&values[INPUTITEMS + 1],sizeof(int8_t) * (OUTPUTITEMS - 1));
				Config.MenuChannel = values[INPUTITEMS];
				break;
			default:
				break;
		}

		// Save and exit
		if (button == ENTER)
		{
			UpdateLimits();			 // Update travel limits based on percentages
			Save_Config_to_EEPROM(); // Save value and return
		}

	}
	_delay_ms(200);
}




