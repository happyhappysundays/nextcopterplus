//***********************************************************
//* menu_servos.c
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
void menu_servo_setup(uint8_t section);

//************************************************************
// Defines
//************************************************************

#define SERVOSTART 230 	// Start of OUT1-OUT8 Menu text items
#define SERVOOFFSET 80	// LCD horizontal offsets
#define SERVOITEMS 8 	// Number of menu items

//************************************************************
// Servo menu items
//************************************************************
	 
const uint8_t ServoMenuText[3][SERVOITEMS] PROGMEM = 
{
	{141,141,141,141,141,141,141,141},
	{0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0},
};

// As all of these are the same, a new range cloning menu option is used
// to save a lot of PROGMEM sapce
const menu_range_t servo_menu_ranges[3][1] PROGMEM = 
{
	{
		{OFF, ON,1,1,OFF},				// Reverse
	},
	{
		{-125,0,1,0,-100}, 				// Min travel
	},
	{
		{0,125,1,0,100}, 				// Max travel
	},
};

//************************************************************
// Main menu-specific setup
//************************************************************

void menu_servo_setup(uint8_t section)
{
	int8_t *value_ptr = &Config.Servo_reverse[0];

	menu_range_t range;
	uint8_t text_link;
	uint8_t i = 0;
	bool	servo_enable = false;
	bool	zero_setting = false;

	// If submenu item has changed, reset submenu positions
	if (menu_flag)
	{
		sub_top = SERVOSTART;
		menu_flag = 0;
	}

	// Get menu offsets
	// 1 = Reverse, 2 = Min, 3 = Max
	while(button != BACK)
	{
		// Load values from eeprom
		for (i = 0; i < SERVOITEMS; i++)
		{
			switch(section)
			{
				case 1:
					break;
				case 2:
					value_ptr = &Config.min_travel[0];
					servo_enable = true;
					zero_setting = true;
					break;
				case 3:
					value_ptr = &Config.max_travel[0];
					servo_enable = true;
					zero_setting = true;
					break;
				default:
					break;
			}
		}

		// Print menu
		print_menu_items(sub_top, SERVOSTART, value_ptr, (const unsigned char*)servo_menu_ranges[section - 1], 1, SERVOOFFSET, (const unsigned char*)ServoMenuText[section - 1], cursor);

		// Handle menu changes
		update_menu(SERVOITEMS, SERVOSTART, 0, button, &cursor, &sub_top, &menu_temp);
		range = get_menu_range ((const unsigned char*)servo_menu_ranges[section - 1], 0);

		if (button == ENTER)
		{
			text_link = pgm_read_byte(&ServoMenuText[section - 1][menu_temp - SERVOSTART]);

			// Zero limits if adjusting
			if (zero_setting)
			{
				value_ptr[menu_temp - SERVOSTART] = 0;
			}

			// Do not allow servo enable for throttle if in CPPM mode
			if ((Config.Channel[menu_temp - SERVOSTART].P1_source_a == THROTTLE) && (Config.RxMode == CPPM_MODE))
			{
				servo_enable = false;
			}

			do_menu_item(menu_temp, value_ptr + (menu_temp - SERVOSTART), 1, range, 0, text_link, servo_enable, (menu_temp - SERVOSTART));
		}

		// Disable servos
		servo_enable = false;

		if (button == ENTER)
		{
			UpdateLimits();				// Update actual servo trims
			Save_Config_to_EEPROM();	// Save value and return
			Wait_BUTTON4();				// Wait for user's finger off the button
		}
	}
}

