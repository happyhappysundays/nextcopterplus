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
void menu_flight(uint8_t i);

//************************************************************
// Defines
//************************************************************

#define FLIGHTSTART 175 // Start of Menu text items
#define FLIGHTOFFSET 79	// LCD offsets
#define FLIGHTTEXT 38 	// Start of value text items
#define FLIGHTITEMS 14 	// Number of menu items

//************************************************************
// RC menu items
//************************************************************
	 
const uint8_t FlightMenuText[FLIGHTITEMS] PROGMEM = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

const menu_range_t flight_menu_ranges[FLIGHTITEMS] PROGMEM = 
{
	// Flight (14)
	{-125,125,5,0,0},				// Trigger
	{0,127,1,0,80},					// Roll PID
	{0,127,1,0,50},
	{0,125,5,0,0},					// I-limits
	{0,127,1,0,60},					// Acc gain
	{-127,127,1,0,0}, 				// Acc trim
	{0,127,1,0,80},					// Pitch PID
	{0,127,1,0,50}, 
	{0,125,5,0,0},					// I-limits
	{0,127,1,0,60},
	{-127,127,1,0,0},
	{0,127,1,0,80},					// Yaw PID
	{0,127,1,0,50},	
	{0,125,5,0,60},					// I-limits
};

//************************************************************
// Main menu-specific setup
//************************************************************

void menu_flight(uint8_t mode)
{
	static uint8_t flight_top = FLIGHTSTART;
	int8_t *value_ptr;

	menu_range_t range;
	uint8_t text_link;

	// If submenu item has changed, reset submenu positions
	if (menu_flag)
	{
		flight_top = FLIGHTSTART;
		menu_flag = 0;
	}

	while(button != BACK)
	{
		value_ptr = &Config.FlightMode[mode].Profilelimit;

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

		// Update limits when exiting
		if (button == ENTER)
		{
			UpdateLimits();			 // Update I-term limits and triggers based on percentages
			Save_Config_to_EEPROM(); // Save value and return
		}
	}
}

