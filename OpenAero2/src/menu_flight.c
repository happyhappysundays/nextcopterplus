//***********************************************************
//* menu_flight.c
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
void menu_flight(uint8_t i);

//************************************************************
// Defines
//************************************************************

#define FLIGHTSTART 187 // Start of Menu text items
#define FLIGHTOFFSET 79	// LCD offsets
#define FLIGHTTEXT 38 	// Start of value text items
#define FLIGHTITEMS 22 	// Number of menu items

//************************************************************
// RC menu items
//************************************************************
	 
const uint8_t FlightMenuText[FLIGHTITEMS] PROGMEM = {0, FLIGHTTEXT, FLIGHTTEXT, 48, 0, 0, 0, 0, 0, 0, 48, 0, 0, 0, 0, 0, 0, 48, 0, 0, 0, 0};

// Have to size each element to STABITEMS even though they are  smaller... fix this later
const menu_range_t flight_menu_ranges[FLIGHTITEMS] PROGMEM = 
{
	// Flight (22)
	{-125,125,5,0,0},				// Trigger
	{DISABLED,ALWAYSON,1,1,DISABLED},// Min, Max, Increment, Style, Default
	{DISABLED,HANDSFREE,1,1,DISABLED},
	{RATE,LOCK, 1, 1, RATE},		// Gyro type
	{0,127,1,0,80},					// Roll PID
	{0,127,1,0,0},
	{0,127,1,0,0},
	{0,125,5,0,0},					// I-limits
	{0,127,1,0,60},					// Acc gain
	{-127,127,1,0,0}, 				// Acc trim
	{RATE,LOCK, 1, 1, RATE},		// Gyro type
	{0,127,1,0,80},					// Pitch PID
	{0,127,1,0,0}, 
	{0,127,1,0,0},
	{0,125,5,0,0},					// I-limits
	{0,127,1,0,60},
	{-127,127,1,0,0},
	{RATE,LOCK, 1, 1, RATE},		// Gyro type
	{0,127,1,0,80},					// Yaw PID
	{0,127,1,0,0},	
	{0,127,1,0,0},
	{0,125,5,0,0},					// I-limits
};

//************************************************************
// Main menu-specific setup
//************************************************************

void menu_flight(uint8_t mode)
{
	static uint8_t flight_top = FLIGHTSTART;
	static	uint8_t old_mode;
	int8_t *value_ptr;

	menu_range_t range;
	uint8_t text_link;
	int8_t temp_gyro_roll = 0;
	int8_t temp_gyro_pitch = 0;
	int8_t temp_gyro_yaw = 0;

	// If submenu item has changed, reset submenu positions
	if (mode != old_mode)
	{
		flight_top = FLIGHTSTART;
		old_mode = mode;
	}

	while(button != BACK)
	{
		value_ptr = &Config.FlightMode[mode-1].Profilelimit;

		// Save pre-edited value for gyro types
		temp_gyro_roll = Config.FlightMode[mode - 1].Roll_type;
		temp_gyro_pitch = Config.FlightMode[mode - 1].Pitch_type;
		temp_gyro_yaw = Config.FlightMode[mode - 1].Yaw_type;

		// Print menu
		print_menu_items(flight_top, FLIGHTSTART, value_ptr, 1, (prog_uchar*)flight_menu_ranges, 0, FLIGHTOFFSET, (prog_uchar*)FlightMenuText, cursor);

		// Handle menu changes
		update_menu(FLIGHTITEMS, FLIGHTSTART, 0, button, &cursor, &flight_top, &menu_temp);
		range = get_menu_range ((prog_uchar*)flight_menu_ranges, (menu_temp - FLIGHTSTART));

		if (button == ENTER)
		{
			text_link = pgm_read_byte(&FlightMenuText[menu_temp - FLIGHTSTART]);
			do_menu_item(menu_temp, value_ptr + (menu_temp - FLIGHTSTART), 1, range, 0, text_link, false, 0);
		}

		// Preset I-limits when gyro mode changes
		if (button == ENTER)
		{
			// If roll gyro type has changed, reset to an appropriate start point
			if (temp_gyro_roll != Config.FlightMode[mode-1].Roll_type)
			{
				// Use Gyro type value to preset limits
				if(Config.FlightMode[mode-1].Roll_type == LOCK)
				{
					Config.FlightMode[mode - 1].Roll_limit = 125;
				}
				else
				{
					Config.FlightMode[mode - 1].Roll_limit = 0;
				}
			}

			if (temp_gyro_pitch != Config.FlightMode[mode-1].Pitch_type)
			{
				if(Config.FlightMode[mode-1].Pitch_type == LOCK)
				{
					Config.FlightMode[mode - 1].Pitch_limit = 125;
				}
				else
				{
					Config.FlightMode[mode - 1].Pitch_limit = 0;
				}
			}

			if (temp_gyro_yaw != Config.FlightMode[mode-1].Yaw_type)
			{
				if(Config.FlightMode[mode-1].Yaw_type == LOCK)
				{
					Config.FlightMode[mode - 1].Yaw_limit = 125;
				}
				else
				{
					Config.FlightMode[mode - 1].Yaw_limit = 0;
				}
			}

			UpdateLimits();			 // Update I-term limits and triggers based on percentages

			Save_Config_to_EEPROM(); // Save value and return
		}
	}
}

