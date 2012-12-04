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
#include "..\inc\mixer.h"

//************************************************************
// Prototypes
//************************************************************

// Menu items
void menu_al_control(void);

//************************************************************
// Defines
//************************************************************

#define AUTOITEMS 9 	// Number of menu items
#define AUTOSTART 37 	// Start of Menu text items
#define AUTOTEXT 199 	// Start of value text items
#define AUTOOFFSET 75	// Value offsets

//************************************************************
// AUTO menu items
//************************************************************

const uint8_t AutoMenuText[AUTOITEMS] PROGMEM = {AUTOTEXT, 0, 0, 0, 0, 0, 0, 101, 0};
const menu_range_t auto_menu_ranges[] PROGMEM = 
{
	{DISABLED,HANDSFREE,1,1,AUTOCHAN}, 	// Min, Max, Increment, Style, Default
	{-125,125,10,0,10},
	{0,127,1,0,60},
	{0,127,1,0,60},
	{10, 60, 5, 0, 45},	// Maximum angle
	{-127,127,1,0,0}, 
	{-127,127,1,0,0},
	{OFF,ON,1,1,OFF},	// Launch mode
	{-125,125,10,0,0}	// Launch trigger
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
		print_menu_items(top, AUTOSTART, &values[0], AUTOITEMS, (prog_uchar*)auto_menu_ranges, AUTOOFFSET, (prog_uchar*)AutoMenuText, cursor);

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
			UpdateLimits();			 // Update I-term limits and triggers based on percentages
			Save_Config_to_EEPROM(); // Save value and return
		}
	}
}
