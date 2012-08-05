//***********************************************************
//* menu_general.c
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include <avr/pgmspace.h> 
#include <avr/io.h>
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
void menu_general(void);

//************************************************************
// Defines
//************************************************************

#define GENERALITEMS 5 	// Number of menu items
#define GENERALSTART 96 	// Start of Menu text items

//************************************************************
// RC menu items
//************************************************************
	 
const uint8_t GeneralMenuOffsets[GENERALITEMS] PROGMEM = {40, 73, 50, 80, 60};
const uint8_t GeneralMenuText[GENERALITEMS] PROGMEM = {33, 103, 101, 101, 101};
const menu_range_t general_menu_ranges[] = 
{
	{AEROPLANE,MANUAL,1,1}, 	// Min, Max, Increment
	{HORIZONTAL,VERTICAL,1,1},
	{OFF,ON,1,1},
	{OFF,ON,1,1},
	{28,45,1,0}, // Numeric
};

//************************************************************
// Main menu-specific setup
//************************************************************

void menu_general(void)
{
	uint8_t cursor = LINE0;
	uint8_t button = 0;
	uint8_t top = GENERALSTART;
	int16_t values[GENERALITEMS];
	menu_range_t range;
	uint8_t temp = 0;
	uint8_t text_link = 0;

	while(button != BACK)
	{
		// Clear buffer before each update
		clear_buffer(buffer);	

		// Load values from eeprom
		values[0] = Config.MixMode;
		values[1] = Config.Orientation;
		values[2] = Config.RCMix;
		values[3] = Config.CamStab;
		values[4] = Config.Contrast;

		// Print menu
		//print_menu_items(top, GENERALSTART, &values[0], &general_menu_ranges[0], &GeneralMenuOffsets[0], &GeneralMenuText[0], cursor);
		print_menu_items(top, GENERALSTART, &values[0], &general_menu_ranges[0], (prog_uchar*)GeneralMenuOffsets, (prog_uchar*)GeneralMenuText, cursor);

		// Poll buttons when idle
		button = poll_buttons();
		while (button == NONE)					
		{
			button = poll_buttons();
		}

		// Handle menu changes
		update_menu(GENERALITEMS, GENERALSTART, button, &cursor, &top, &temp);
		range = general_menu_ranges[temp - GENERALSTART];

		if (button == ENTER)
		{
			text_link = pgm_read_byte(&GeneralMenuText[temp - GENERALSTART]);
			values[temp - GENERALSTART] = do_menu_item(temp, values[temp - GENERALSTART], range, 0, text_link);
		}

		// Update value in config structure
		Config.MixMode = values[0];
		Config.Orientation = values[1];
		Config.RCMix = values[2];
		Config.CamStab = values[3];
		Config.Contrast = values[4];

		// Update contrast
		//st7565_set_brightness(Config.Contrast);

		if (button == ENTER)
		{
			switch(Config.MixMode)  // Load selected mix
			{
				case AEROPLANE:
					get_preset_mix(AEROPLANE_MIX);
					break;	
				case FLYINGWING:
					get_preset_mix(FLYING_WING_MIX);
					break;
				case MANUAL:
					get_preset_mix(MANUAL_MIX);
					break;
				default:
					break;
			}
			Save_Config_to_EEPROM(); // Save value and return
		}
	}
	menu_beep();
	_delay_ms(200);
}




