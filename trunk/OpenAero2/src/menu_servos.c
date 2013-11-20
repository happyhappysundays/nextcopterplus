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
#include "..\inc\io_cfg.h"
#include "..\inc\init.h"
#include "..\inc\mugui.h"
#include "..\inc\glcd_menu.h"
#include "..\inc\menu_ext.h"
#include "..\inc\glcd_driver.h"
#include "..\inc\main.h"
#include "..\inc\eeprom.h"
#include "..\inc\mixer.h"
#include "..\inc\imu.h"

//************************************************************
// Prototypes
//************************************************************

// Menu items
void menu_servo_setup(uint8_t section);

//************************************************************
// Defines
//************************************************************

#define SERVOSTART 228 	// Start of OUT1-OUT8 Menu text items
#define SERVOOFFSET 80	// LCD horizontal offsets
#define SERVOITEMS 8 	// Number of menu items

//************************************************************
// Servo menu items
//************************************************************
	 
const uint8_t ServoMenuText[5][SERVOITEMS] PROGMEM = 
{
	{141,141,141,141,141,141,141,141},
	{0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0},
};

// As all of these are the same, a new range cloning menu option is used
// to save a lot of PROGMEM sapce
const menu_range_t servo_menu_ranges[5][1] PROGMEM = 
{
	{
		{OFF, ON,1,1,OFF},				// Reverse
	},
	{
		{-125,125,1,0,0}, 				// Offset
	},
	{
		{-125,0,5,0,-100}, 				// Min travel
	},
	{
		{0,125,5,0,100}, 				// Max travel
	},
	{
		{-125,125,5,0,0}, 				// Failsafe
	}
};

//************************************************************
// Main menu-specific setup
//************************************************************

void menu_servo_setup(uint8_t section)
{
	static uint8_t servo_top = SERVOSTART;
	static	uint8_t old_section;
	int8_t *value_ptr;

	menu_range_t range;
	uint8_t text_link;
	uint8_t i = 0;
	bool	servo_enable = false;
	bool	zero_setting = false;

	// If submenu item has changed, reset submenu positions
	if (section != old_section)
	{
		servo_top = SERVOSTART;
		old_section = section;
	}

	// Get menu offsets
	// 1 = Reverse, 2 = Offset, 3 = Min, 4 = Max, 5 = Failsafe
	while(button != BACK)
	{
		// Load values from eeprom
		for (i = 0; i < SERVOITEMS; i++)
		{
			switch(section)
			{
				case 1:
					value_ptr = &Config.Servo_reverse[0];
					break;
				case 2:
					value_ptr = &Config.Offset[0];
					servo_enable = true;
					break;
				case 3:
					value_ptr = &Config.min_travel[0];
					servo_enable = true;
					zero_setting = true;
					break;
				case 4:
					value_ptr = &Config.max_travel[0];
					servo_enable = true;
					zero_setting = true;
					break;
				case 5:
					value_ptr = &Config.Failsafe[0];
					servo_enable = true;
					zero_setting = true;
					break;
				default:
					value_ptr = &Config.Servo_reverse[0];
					break;
			}
		}

		// Print menu
		print_menu_items(servo_top, SERVOSTART, value_ptr, 1, (prog_uchar*)servo_menu_ranges[section - 1], 1, SERVOOFFSET, (prog_uchar*)ServoMenuText[section - 1], cursor);

		// Handle menu changes
		update_menu(SERVOITEMS, SERVOSTART, 0, button, &cursor, &servo_top, &menu_temp);
		range = get_menu_range ((prog_uchar*)servo_menu_ranges[section - 1], 0);

		if (button == ENTER)
		{
			text_link = pgm_read_byte(&ServoMenuText[section - 1][menu_temp - SERVOSTART]);
			// Zero limits if adjusting
			if (zero_setting)
			{
				value_ptr[menu_temp - SERVOSTART] = 0;
			}

			// Do not allow servo enable for throttle if in CPPM mode
			if ((Config.Channel[menu_temp - SERVOSTART].source_a == THROTTLE) && (Config.RxMode == CPPM_MODE))
			{
				servo_enable = false;
			}

			do_menu_item(menu_temp, value_ptr + (menu_temp - SERVOSTART), 1, range, 0, text_link, servo_enable, (menu_temp - SERVOSTART));
		}

		// Disable servos
		servo_enable = false;

		if (button == ENTER)
		{
			Save_Config_to_EEPROM(); // Save value and return
		}
	}

	UpdateLimits();					// Update actual servo trims
	_delay_ms(200);
}

