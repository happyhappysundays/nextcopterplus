//***********************************************************
//* menu_offsets.c
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
void menu_offsets(void);

//************************************************************
// Defines
//************************************************************

#define OFFSETSSTART 230	// Start of Menu text items for curves
#define OFFSETSOFFSET 128	// LCD offsets

//************************************************************
// RC menu items
//************************************************************

const uint16_t OffsetsMenuText[MAX_OUTPUTS] PROGMEM = {0, 0, 0, 0, 0, 0, 0, 0};
const uint16_t OffsetsMenuOffsets[MAX_OUTPUTS] PROGMEM = {OFFSETSOFFSET, OFFSETSOFFSET, OFFSETSOFFSET, OFFSETSOFFSET, OFFSETSOFFSET, OFFSETSOFFSET, OFFSETSOFFSET, OFFSETSOFFSET};
const menu_range_t Offsets_menu_ranges[MAX_OUTPUTS][NUMBEROFPOINTS+1] PROGMEM = 
{
	{
		// OUT1 offset curve (8)
		{-125,125,1,0,0},				// Point 1
		{-125,125,1,0,0},				// Point 2
		{-125,125,1,0,0},				// Point 3
		{-125,125,1,0,0},				// Point 4
		{-125,125,1,0,0},				// Point 5
		{-125,125,1,0,0},				// Point 6
		{-125,125,1,0,0},				// Point 7
		{SRC1, SRC1, 1, 1, SRC1},		// Associated channel (unused)
	},
	{
		// OUT2 offset curve (8)
		{-125,125,1,0,0},				// Point 1
		{-125,125,1,0,0},				// Point 2
		{-125,125,1,0,0},				// Point 3
		{-125,125,1,0,0},				// Point 4
		{-125,125,1,0,0},				// Point 5
		{-125,125,1,0,0},				// Point 6
		{-125,125,1,0,0},				// Point 7
		{SRC1, SRC1, 1, 1, SRC1},		// Associated channel (unused)
	},
	{
		// OUT3 offset curve (8)
		{-125,125,1,0,0},				// Point 1
		{-125,125,1,0,0},				// Point 2
		{-125,125,1,0,0},				// Point 3
		{-125,125,1,0,0},				// Point 4
		{-125,125,1,0,0},				// Point 5
		{-125,125,1,0,0},				// Point 6
		{-125,125,1,0,0},				// Point 7
		{SRC1, SRC1, 1, 1, SRC1},		// Associated channel (unused)
	},
	{
		// OUT4 offset curve (8)
		{-125,125,1,0,0},				// Point 1
		{-125,125,1,0,0},				// Point 2
		{-125,125,1,0,0},				// Point 3
		{-125,125,1,0,0},				// Point 4
		{-125,125,1,0,0},				// Point 5
		{-125,125,1,0,0},				// Point 6
		{-125,125,1,0,0},				// Point 7
		{SRC1, SRC1, 1, 1, SRC1},		// Associated channel (unused)
	},	
	{
		// OUT5 offset curve (8)
		{-125,125,1,0,0},				// Point 1
		{-125,125,1,0,0},				// Point 2
		{-125,125,1,0,0},				// Point 3
		{-125,125,1,0,0},				// Point 4
		{-125,125,1,0,0},				// Point 5
		{-125,125,1,0,0},				// Point 6
		{-125,125,1,0,0},				// Point 7
		{SRC1, SRC1, 1, 1, SRC1},		// Associated channel (unused)
	},
	{
		// OUT6 offset curve (8)
		{-125,125,1,0,0},				// Point 1
		{-125,125,1,0,0},				// Point 2
		{-125,125,1,0,0},				// Point 3
		{-125,125,1,0,0},				// Point 4
		{-125,125,1,0,0},				// Point 5
		{-125,125,1,0,0},				// Point 6
		{-125,125,1,0,0},				// Point 7
		{SRC1, SRC1, 1, 1, SRC1},		// Associated channel (unused)
	},
	{
		// OUT7 offset curve (8)
		{-125,125,1,0,0},				// Point 1
		{-125,125,1,0,0},				// Point 2
		{-125,125,1,0,0},				// Point 3
		{-125,125,1,0,0},				// Point 4
		{-125,125,1,0,0},				// Point 5
		{-125,125,1,0,0},				// Point 6
		{-125,125,1,0,0},				// Point 7
		{SRC1, SRC1, 1, 1, SRC1},		// Associated channel (unused)
	},
	{
		// OUT8 offset curve (8)
		{-125,125,1,0,0},				// Point 1
		{-125,125,1,0,0},				// Point 2
		{-125,125,1,0,0},				// Point 3
		{-125,125,1,0,0},				// Point 4
		{-125,125,1,0,0},				// Point 5
		{-125,125,1,0,0},				// Point 6
		{-125,125,1,0,0},				// Point 7
		{SRC1, SRC1, 1, 1, SRC1},		// Associated channel (unused)
	},		
};

//************************************************************
// Main menu-specific setup
//************************************************************

void menu_offsets(void)
{
	int8_t *value_ptr;
	uint16_t reference = OFFSETSSTART;

	// If sub-menu item has changed, reset sub-menu positions
	if (menu_flag)
	{
		sub_top = OFFSETSSTART;			
		menu_flag = 0;
	}

	while(button != BACK)
	{
		value_ptr = &Config.Offsets[0].Point1;

		// Print top level menu
		print_menu_items(sub_top, reference, value_ptr, (const unsigned char*)Offsets_menu_ranges, 0, (const uint16_t*)OffsetsMenuOffsets, (const uint16_t*)OffsetsMenuText, cursor);

		// Handle menu navigation
		update_menu(MAX_OUTPUTS, reference, 0, button, &cursor, &sub_top, &menu_temp);

		// Edit selected curve
		if (button == ENTER)
		{
			edit_curve_item(menu_temp - reference, OFFSET); // Curves after NUMBEROFCURVES are offsets
		}

		// Stop unwanted exit to main menu
		if (button == ABORT)
		{
			Wait_BUTTON1();			 // Wait for user's finger off the button
			button = NONE;
		}

		// Save and exit
		if (button == ENTER)
		{
			Save_Config_to_EEPROM(); // Save value and return
			Wait_BUTTON4();			 // Wait for user's finger off the button
			Wait_BUTTON1();	
		}
	}
}

