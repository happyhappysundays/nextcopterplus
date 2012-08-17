//***********************************************************
//* menu_rcmixer.c
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include <avr/pgmspace.h> 
#include <avr/io.h>
#include <stdlib.h>
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

//************************************************************
// Prototypes
//************************************************************

// Menu items
void menu_rcmixer(uint8_t i);

//************************************************************
// Defines
//************************************************************

#define RCMIXITEMS 4 	// Number of menu items
#define RCMIXSTART 78 	// Start of Menu text items
#define RCMIXTEXT 149 	// Start of value text items
#define	RCMIXOFFSET 75	// Value text offset

//************************************************************
// RC mixer menu items
//************************************************************

const uint8_t RCmixMenuText[RCMIXITEMS] PROGMEM = {149, 0, 149, 0};
const menu_range_t RCmix_menu_ranges[] PROGMEM = 
{
	{THROTTLE,NOCHAN,1,1,AILERON}, 	// Min, Max, Increment, Style, Default
	{-125,125,5,0,0}, 
	{THROTTLE,NOCHAN,1,1,ELEVATOR},
	{-125,125,5,0,0},
};

//************************************************************
// Main menu-specific setup
//************************************************************

void menu_rcmixer(uint8_t i)
{
	uint8_t cursor = LINE0;
	uint8_t top = RCMIXSTART;
	uint8_t temp = 0;
	int8_t values[RCMIXITEMS];
	menu_range_t range;
	uint8_t text_link = 0;
	
	while(button != BACK)
	{
		// Load values from eeprom
		memcpy(&values[0], &Config.mixer_data[i].source_a, sizeof(mixer_t));

		// Print menu
		print_menu_items(top, RCMIXSTART, &values[0], (prog_uchar*)RCmix_menu_ranges, RCMIXOFFSET, (prog_uchar*)RCmixMenuText, cursor);

		// Handle menu changes
		update_menu(RCMIXITEMS, RCMIXSTART, button, &cursor, &top, &temp);
		range = get_menu_range ((prog_uchar*)RCmix_menu_ranges, temp - RCMIXSTART);

		if (button == ENTER)
		{
			text_link = pgm_read_byte(&RCmixMenuText[temp - RCMIXSTART]);
			values[temp - RCMIXSTART] = do_menu_item(temp, values[temp - RCMIXSTART], range, 0, text_link);
		}

		// Update value in config structure
		memcpy(&Config.mixer_data[i].source_a, &values[0], sizeof(mixer_t));

		if (button == ENTER)
		{
			Save_Config_to_EEPROM(); // Save value and return
		}
	}
	menu_beep(1);
	_delay_ms(200);
}
