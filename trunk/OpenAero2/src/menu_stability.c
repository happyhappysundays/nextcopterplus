//***********************************************************
//* menu_stability.c
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include <avr/pgmspace.h> 
#include <avr/io.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
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
void menu_stab_control(void);

//************************************************************
// Defines
//************************************************************

#define STABITEMS 16 	// Number of menu items
#define STABSTART 124 	// Start of Menu text items
#define STABTEXT  199 	// Start of value text items
#define STABOFFSET 75	// Value offsets

//************************************************************
// STAB menu items
//************************************************************

const uint8_t StabMenuText[STABITEMS] PROGMEM = {STABTEXT, 0, 105, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
const menu_range_t stab_menu_ranges[] PROGMEM = 
{
	{DISABLED,ALWAYSON,1,1,STABCHAN}, 	// Min, Max, Increment, Style, Default
	{-125,125,10,0,-30},
	{THROTTLE,NOCHAN,1,1,NOCHAN}, 		// Dynamic P gain channel
	{0,100,5,0,0},						// Dynamic gain amount
	{0,127,1,0,60},
	{0,127,1,0,0},
	{0,127,1,0,0},
	{0,127,1,0,60},
	{0,127,1,0,0}, 
	{0,127,1,0,0},
	{0,127,1,0,60},
	{0,127,1,0,0},
	{0,127,1,0,0},
	{0,125,5,0,0},
	{0,125,5,0,0},
	{0,125,5,0,0}
};

//************************************************************
// Main menu-specific setup
//************************************************************

void menu_stab_control(void)
{
	uint8_t cursor = LINE0;
	uint8_t top = STABSTART;
	uint8_t temp = 0;
	int8_t values[STABITEMS];
	menu_range_t range;
	uint8_t text_link = 0;
	
	while(button != BACK)
	{
		// Load values from eeprom
		memcpy(&values[0],&Config.StabMode,sizeof(int8_t) * STABITEMS);

		// Print menu
		print_menu_items(top, STABSTART, &values[0], STABITEMS, (prog_uchar*)stab_menu_ranges, STABOFFSET, (prog_uchar*)StabMenuText, cursor);

		// Handle menu changes
		update_menu(STABITEMS, STABSTART, button, &cursor, &top, &temp);
		range = get_menu_range ((prog_uchar*)stab_menu_ranges, temp - STABSTART);

		if (button == ENTER)
		{
			text_link = pgm_read_byte(&StabMenuText[temp - STABSTART]);
			values[temp - STABSTART] = do_menu_item(temp, values[temp - STABSTART], range, 0, text_link);
		}

		// Update value in config structure
		memcpy(&Config.StabMode,&values[0],sizeof(int8_t) * STABITEMS);

		if (button == ENTER)
		{
			UpdateLimits();			 // Update I-term limits and triggers based on percentages
			Save_Config_to_EEPROM(); // Save value and return
		}
	}
}
