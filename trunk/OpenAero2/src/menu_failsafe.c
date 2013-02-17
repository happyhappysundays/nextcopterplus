//***********************************************************
//* menu_failsafe.c
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
void menu_failsafe(void);

//************************************************************
// Defines
//************************************************************

#define FSITEMS 5 		// Number of menu items
#define FSSTART 158 	// Start of Menu text items
#define FSTEXT 103	 	// Start of value text items
#define FSOFFSET 75		// Value offsets

//************************************************************
// Battery menu items
//************************************************************

const uint8_t FSMenuText[FSITEMS] PROGMEM = {FSTEXT, 0, 0, 0, 0};
const menu_range_t fs_menu_ranges[] PROGMEM = 
{
	// Min, Max, Increment, Style, Default
	{0,1,1,1,1}, 	
	{-100,100,1,0,-100},
	{-127,127,1,0,0},
	{-127,127,1,0,0},
	{-127,127,1,0,0},
};
//************************************************************
// Main menu-specific setup
//************************************************************

void menu_failsafe(void)
{
	static uint8_t fs_top = FSSTART;
	int8_t values[FSITEMS];
	menu_range_t range;
	uint8_t text_link = 0;
	
	while(button != BACK)
	{
		// Load values from eeprom
		memcpy(&values[0],&Config.FailsafeType,sizeof(int8_t) * FSITEMS);

		// Print menu
		print_menu_items(fs_top, FSSTART, &values[0], FSITEMS, (prog_uchar*)fs_menu_ranges, FSOFFSET, (prog_uchar*)FSMenuText, cursor);

		// Handle menu changes
		update_menu(FSITEMS, FSSTART, button, &cursor, &fs_top, &menu_temp);
		range = get_menu_range ((prog_uchar*)fs_menu_ranges, menu_temp - FSSTART);

		if (button == ENTER)
		{
			text_link = pgm_read_byte(&FSMenuText[menu_temp - FSSTART]);
			values[menu_temp - FSSTART] = do_menu_item(menu_temp, values[menu_temp - FSSTART], range ,0, text_link);
		}

		// Update value in config structure
		memcpy(&Config.FailsafeType,&values[0],sizeof(int8_t) * FSITEMS);

		if (button == ENTER)
		{
			Save_Config_to_EEPROM(); // Save value and return
		}
	}
}

