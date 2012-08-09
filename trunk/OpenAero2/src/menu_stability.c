//***********************************************************
//* menu_stability.c
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include <avr/pgmspace.h> 
#include <avr/io.h>
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

//************************************************************
// Prototypes
//************************************************************

// Menu items
void menu_stab_control(void);

//************************************************************
// Defines
//************************************************************

#define STABITEMS 10 	// Number of menu items
#define STABSTART 82 	// Start of Menu text items
#define STABTEXT 61 	// Start of value text items

//************************************************************
// STAB menu items
//************************************************************

const uint8_t StabMenuOffsets[STABITEMS] PROGMEM = {75, 75, 75, 75, 75, 75, 75, 75, 75, 75};
const uint8_t StabMenuText[STABITEMS] PROGMEM = {STABTEXT, 0, 0, 0, 0, 0, 0, 0, 0, 0};
const menu_range_t stab_menu_ranges[] PROGMEM = 
{
	{DISABLED,ALWAYSON,1,1,STABCHAN}, 	// Min, Max, Increment, Style, Default
	{0,250,1,0,60},
	{0,250,1,0,0},
	{0,250,1,0,0},
	{0,250,1,0,60},
	{0,250,1,0,0}, 
	{0,250,1,0,0},
	{0,250,1,0,60},
	{0,250,1,0,0},
	{0,250,1,0,0}
};

//************************************************************
// Main menu-specific setup
//************************************************************

void menu_stab_control(void)
{
	uint8_t cursor = LINE0;
	uint8_t button = 0;
	uint8_t top = STABSTART;
	uint8_t temp = 0;
	int16_t values[STABITEMS];
	menu_range_t range;
	uint8_t text_link = 0;
	
	while(button != BACK)
	{
		// Clear buffer before each update
		clear_buffer(buffer);	

		// Load values from eeprom
		values[0] = Config.StabMode;		// DISABLED = 0, AUTOCHAN, STABCHAN, THREEPOS, ALWAYSON
		values[1] = Config.Roll.P_mult;		
		values[2] = Config.Roll.I_mult;
		values[3] = Config.Roll.D_mult;
		values[4] = Config.Pitch.P_mult;
		values[5] = Config.Pitch.I_mult;
		values[6] = Config.Pitch.D_mult;
		values[7] = Config.Yaw.P_mult;
		values[8] = Config.Yaw.I_mult;
		values[9] = Config.Yaw.D_mult;

		// Print menu
		print_menu_items(top, STABSTART, &values[0], (prog_uchar*)stab_menu_ranges, (prog_uchar*)StabMenuOffsets, (prog_uchar*)StabMenuText, cursor);

		// Poll buttons when idle
		button = poll_buttons();
		while (button == NONE)					
		{
			button = poll_buttons();
		}

		// Handle menu changes
		update_menu(STABITEMS, STABSTART, button, &cursor, &top, &temp);
		range = get_menu_range ((prog_uchar*)stab_menu_ranges, temp - STABSTART);

		if (button == ENTER)
		{
			text_link = pgm_read_byte(&StabMenuText[temp - STABSTART]);
			values[temp - STABSTART] = do_menu_item(temp, values[temp - STABSTART], range, 0, text_link);
		}

		// Update value in config structure
		Config.StabMode = values[0];	
		Config.Roll.P_mult = values[1];	
		Config.Roll.I_mult = values[2]; 
		Config.Roll.D_mult = values[3]; 
		Config.Pitch.P_mult = values[4];
		Config.Pitch.I_mult = values[5];
		Config.Pitch.D_mult = values[6];
		Config.Yaw.P_mult = values[7];
		Config.Yaw.I_mult = values[8];
		Config.Yaw.D_mult = values[9];

		if (button == ENTER)
		{
			Save_Config_to_EEPROM(); // Save value and return
		}
	}
	menu_beep(1);
	_delay_ms(200);
}
