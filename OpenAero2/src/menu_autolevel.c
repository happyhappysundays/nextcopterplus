//***********************************************************
//* menu_autolevel.c
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
void menu_al_control(void);

//************************************************************
// Defines
//************************************************************

#define AUTOITEMS 9 	// Number of menu items
#define AUTOSTART 50 	// Start of Menu text items
#define AUTOTEXT 61 	// Start of value text items
#define AUTOOFFSET 75	// Value offsets

//************************************************************
// AUTO menu items
//************************************************************

const uint8_t AutoMenuText[AUTOITEMS] PROGMEM = {AUTOTEXT, 0, 0, 0, 0, 0, 0, 0, 0};
const menu_range_t auto_menu_ranges[] PROGMEM = 
{
	{DISABLED,ALWAYSON,1,1,AUTOCHAN}, 	// Min, Max, Increment, Style, Default
	{0,127,1,0,60},
	{0,127,1,0,0},
	{0,127,1,0,0},
	{0,127,1,0,100},
	{0,127,1,0,0}, 
	{0,127,1,0,0},
	{-127,127,1,0,0}, 
	{-127,127,1,0,0}
};

//************************************************************
// Main menu-specific setup
//************************************************************

void menu_al_control(void)
{
	uint8_t cursor = LINE0;
	uint8_t top = AUTOSTART;
	uint8_t temp = 0;
	int8_t values[AUTOITEMS];
	menu_range_t range;
	uint8_t text_link = 0;
	
	while(button != BACK)
	{
		// Load values from eeprom
		memcpy(&values[0],&Config.AutoMode,sizeof(int8_t) * AUTOITEMS);

		// Print menu
		print_menu_items(top, AUTOSTART, &values[0], (prog_uchar*)auto_menu_ranges, AUTOOFFSET, (prog_uchar*)AutoMenuText, cursor);

		// Handle menu changes
		update_menu(AUTOITEMS, AUTOSTART, button, &cursor, &top, &temp);
		range = get_menu_range ((prog_uchar*)auto_menu_ranges, temp - AUTOSTART);

		if (button == ENTER)
		{
			text_link = pgm_read_byte(&AutoMenuText[temp - AUTOSTART]);
			values[temp - AUTOSTART] = do_menu_item(temp, values[temp - AUTOSTART], range, 0, text_link);
		}

		// Update value in config structure
		memcpy(&Config.AutoMode,&values[0],sizeof(int8_t) * AUTOITEMS);

		if (button == ENTER)
		{
			Save_Config_to_EEPROM(); // Save value and return
		}
	}
	menu_beep(1);
	_delay_ms(200);
}
