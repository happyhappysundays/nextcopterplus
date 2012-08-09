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

#define GENERALITEMS 4 		// Number of menu items
#define GENERALSTART 164 	// Start of Menu text items

//************************************************************
// RC menu items
//************************************************************

/*	 
const uint8_t GeneralMenuOffsets[GENERALITEMS] PROGMEM = {50, 80, 80, 80, 80, 80};
const uint8_t GeneralMenuText[GENERALITEMS] PROGMEM = {33, 103, 101, 101, 101, 101};
const menu_range_t general_menu_ranges[] PROGMEM = 
{
	{AEROPLANE,MANUAL,1,1,AEROPLANE}, 	// Min, Max, Increment, Style, Default
	{HORIZONTAL,VERTICAL,1,1,HORIZONTAL},
	{28,45,1,0,38}, 	// Contrast
	{OFF,ON,1,1,OFF},	// Auto-update status menu
	{OFF,ON,1,1,OFF},
	{OFF,ON,1,1,OFF},
};
*/
const uint8_t GeneralMenuOffsets[GENERALITEMS] PROGMEM = {50, 80, 80, 80};
const uint8_t GeneralMenuText[GENERALITEMS] PROGMEM = {33, 103, 101, 101};
const menu_range_t general_menu_ranges[] PROGMEM = 
{
	{AEROPLANE,MANUAL,1,1,AEROPLANE}, 	// Min, Max, Increment, Style, Default
	{HORIZONTAL,VERTICAL,1,1,HORIZONTAL},
	{28,45,1,0,38}, 	// Contrast
	{OFF,ON,1,1,OFF},	// Auto-update status menu
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
		values[2] = Config.Contrast;
		values[3] = Config.AutoUpdateEnable;
	//	values[4] = Config.RCMix;
	//	values[5] = Config.CamStab;


		// Print menu
		print_menu_items(top, GENERALSTART, &values[0], (prog_uchar*)general_menu_ranges, (prog_uchar*)GeneralMenuOffsets, (prog_uchar*)GeneralMenuText, cursor);

		// Poll buttons when idle
		button = poll_buttons();
		while (button == NONE)					
		{
			button = poll_buttons();
		}

		// Handle menu changes
		update_menu(GENERALITEMS, GENERALSTART, button, &cursor, &top, &temp);
		range = get_menu_range ((prog_uchar*)general_menu_ranges, temp - GENERALSTART);

		if (button == ENTER)
		{
			text_link = pgm_read_byte(&GeneralMenuText[temp - GENERALSTART]);
			values[temp - GENERALSTART] = do_menu_item(temp, values[temp - GENERALSTART], range, 0, text_link);
		}

		// Update value in config structure
		Config.MixMode = values[0];
		Config.Orientation = values[1];
		Config.Contrast = values[2];
		Config.AutoUpdateEnable = values[3];
	//	Config.RCMix = values[4];
	//	Config.CamStab = values[5];

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
	menu_beep(1);
	_delay_ms(200);
}




