//***********************************************************
//* menu_curves.c
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
void menu_curves(void);

//************************************************************
// Defines
//************************************************************

#define CURVESSTARTE 403	// Start of Menu text items for curves
#define CURVESOFFSET 128	// LCD offsets

//************************************************************
// RC menu items
//************************************************************

const uint16_t CurvesMenuText[NUMBEROFCURVES] PROGMEM = {0, 0, 0, 0, 0, 0};
const uint16_t CurvesMenuOffsets[NUMBEROFCURVES] PROGMEM = {CURVESOFFSET, CURVESOFFSET, CURVESOFFSET, CURVESOFFSET, CURVESOFFSET, CURVESOFFSET};
const menu_range_t Curves_menu_ranges[NUMBEROFCURVES][NUMBEROFPOINTS+1] PROGMEM = 
{
	{
		// P1 Throttle Curve (8)
		{0,100,1,0,0},					// Point 1
		{0,100,1,0,17},					// Point 2
		{0,100,1,0,33},					// Point 3
		{0,100,1,0,50},					// Point 4
		{0,100,1,0,66},					// Point 5		
		{0,100,1,0,83},					// Point 6
		{0,100,1,0,100},				// Point 7
		{SRC1, SRC1, 1, 1, SRC1},		// Associated channel
	},
	{
		// P2 Throttle Curve (8)
		{0,100,1,0,0},					// Point 1
		{0,100,1,0,17},					// Point 2
		{0,100,1,0,33},					// Point 3
		{0,100,1,0,50},					// Point 4
		{0,100,1,0,66},					// Point 5
		{0,100,1,0,83},					// Point 6
		{0,100,1,0,100},				// Point 7
		{SRC1, SRC1, 1, 1, SRC1},		// Associated channel
	},
	{
		// P1 Collective Curve (8)
		{-100,100,1,0,-100},			// Point 1
		{-100,100,1,0,-66},				// Point 2
		{-100,100,1,0,-33},				// Point 3
		{-100,100,1,0,0},				// Point 4
		{-100,100,1,0,33},				// Point 5
		{-100,100,1,0,66},				// Point 6
		{-100,100,1,0,100},				// Point 7
		{SRC1, SRC1, 1, 1, SRC1},		// Associated channel		
	},
	{
		// P2 Collective Curve (8)
		{-100,100,1,0,-100},			// Point 1
		{-100,100,1,0,-66},				// Point 2
		{-100,100,1,0,-33},				// Point 3
		{-100,100,1,0,0},				// Point 4
		{-100,100,1,0,33},				// Point 5
		{-100,100,1,0,66},				// Point 6
		{-100,100,1,0,100},				// Point 7
		{SRC1, SRC1, 1, 1, SRC1},		// Associated channel
	},
	{
		// Generic Curve C (8)
		{-100,100,1,0,-100},			// Point 1
		{-100,100,1,0,-66},				// Point 2
		{-100,100,1,0,-33},				// Point 3
		{-100,100,1,0,0},				// Point 4
		{-100,100,1,0,33},				// Point 5
		{-100,100,1,0,66},				// Point 6
		{-100,100,1,0,100},				// Point 7
		{SRC5, NOMIX, 1, 1, NOMIX},		// Associated channel
	},
	{
		// Generic Curve D (8)
		{-100,100,1,0,-100},			// Point 1
		{-100,100,1,0,-66},				// Point 2
		{-100,100,1,0,-33},				// Point 3
		{-100,100,1,0,0},				// Point 4
		{-100,100,1,0,33},				// Point 5
		{-100,100,1,0,66},				// Point 6
		{-100,100,1,0,100},				// Point 7
		{SRC5, NOMIX, 1, 1, NOMIX},		// Associated channel
	},
};

//************************************************************
// Main menu-specific setup
//************************************************************

void menu_curves(void)
{
	int8_t *value_ptr;
	uint16_t reference = CURVESSTARTE;

	// If sub-menu item has changed, reset sub-menu positions
	if (menu_flag)
	{
		sub_top = CURVESSTARTE;			
		menu_flag = 0;
	}

	while(button != BACK)
	{
		value_ptr = &Config.Curve[0].Point1;

		// Print top level menu
		print_menu_items(sub_top, reference, value_ptr, (const unsigned char*)Curves_menu_ranges, 0, (const uint16_t*)CurvesMenuOffsets, (const uint16_t*)CurvesMenuText, cursor);

		// Handle menu navigation
		update_menu(NUMBEROFCURVES, reference, 0, button, &cursor, &sub_top, &menu_temp);

		// Edit selected curve
		if (button == ENTER)
		{
			edit_curve_item(menu_temp - reference, CURVE);
		}

		// Stop unwanted exit to main menu
		if (button == ABORT)
		{
			Wait_BUTTON1();			 // Wait for user's finger off the button
			button = NONE;
		}

		// Update limits when exiting
		if (button == ENTER)
		{
			Save_Config_to_EEPROM(); // Save value and return
			Wait_BUTTON4();			 // Wait for user's finger off the button
			Wait_BUTTON1();	
		}
	}
}

