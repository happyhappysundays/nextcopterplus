//***********************************************************
//* menu_mixer.c
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include <avr/pgmspace.h> 
#include <avr/io.h>
#include <stdbool.h>
#include <string.h>
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

//************************************************************
// Prototypes
//************************************************************

// Menu items
void menu_mixer(uint8_t i);

//************************************************************
// Defines
//************************************************************

#define MIXERITEMS 14

#define MIXERSTART 214 	// Start of Menu text items
#define MIXOFFSET  80	// Value offsets

//************************************************************
// RC menu items
//************************************************************
	 
const uint8_t MixerMenuText[MIXERITEMS] PROGMEM = 
{
	41,105,0,105,0,143,143,143,143,143,
	176,0,176,0
};

const menu_range_t mixer_menu_ranges[MIXERITEMS] PROGMEM = 
{
	{ASERVO,MOTOR,1,1,ASERVO},		// Motor marker (0)
	{THROTTLE,NOCHAN,1,1,CH1}, 		// Source A
	{-125,125,1,0,0},				// Source A volume (%)
	{THROTTLE,NOCHAN,1,1,NOCHAN}, 	// Source B
	{-125,125,1,0,0},				// Source B volume (%)
	{OFF, REV,1,1,OFF},				// roll_gyro
	{OFF, REV,1,1,OFF},				// pitch_gyro
	{OFF, REV,1,1,OFF},				// yaw_gyro
	{OFF, REV,1,1,OFF},				// roll_acc
	{OFF, REV,1,1,OFF},				// pitch_acc
	{SRC1,NOMIX,1,1,NOMIX},			// Output B
	{-125,125,1,0,0},				// Output B volume
	{SRC1,NOMIX,1,1,NOMIX},			// Output C
	{-125,125,1,0,0},				// Output C volume
};

//************************************************************
// Main menu-specific setup
//************************************************************

void menu_mixer(uint8_t i)
{
	int8_t *value_ptr;
	menu_range_t range;
	uint8_t text_link = 0;

	// If submenu item has changed, reset submenu positions
	if (menu_flag)
	{
		sub_top = MIXERSTART;
		menu_flag = 0;
	}

	while(button != BACK)
	{
		value_ptr = &Config.Channel[i].Motor_marker;

		// Print menu
		print_menu_items(sub_top, MIXERSTART, value_ptr, 1, (const unsigned char*)mixer_menu_ranges, 0, MIXOFFSET, (const unsigned char*)MixerMenuText, cursor);

		// Handle menu changes
		update_menu(MIXERITEMS, MIXERSTART, 0, button, &cursor, &sub_top, &menu_temp);
		range = get_menu_range ((const unsigned char*)mixer_menu_ranges, menu_temp - MIXERSTART);

		if (button == ENTER)
		{
			text_link = pgm_read_byte(&MixerMenuText[menu_temp - MIXERSTART]);
			do_menu_item(menu_temp, value_ptr + (menu_temp - MIXERSTART), 1, range, 0, text_link, false, 0);
		}

		// Save and exit
		if (button == ENTER)
		{
			UpdateLimits();			 // Update travel limits based on percentages
			Save_Config_to_EEPROM(); // Save value and return
			Wait_BUTTON4();			 // Wait for user's finger off the button
		}
	} // while(button != BACK)
}




