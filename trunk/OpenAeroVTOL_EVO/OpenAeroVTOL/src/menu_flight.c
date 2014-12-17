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

#define FLIGHTSTART 170 // Start of Menu text items
#define FLIGHTOFFSET 79	// LCD offsets
#define FLIGHTTEXT 38 	// Start of value text items
#define FLIGHTITEMS 18 	// Number of menu items

//************************************************************
// RC menu items
//************************************************************

const uint8_t FlightMenuText[FLIGHTITEMS] PROGMEM = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

const menu_range_t flight_menu_ranges[FLIGHTITEMS] PROGMEM = 
{
	// Flight (18/20)
	{0,127,1,0,60},					// Roll gyro P
	{0,127,1,0,40},					// Roll gyro I
	{0,125,1,0,10},					// Roll gyro I-limits
	{0,4,1,0,2},					// Roll gyro rate
	{0,127,1,0,5},					// Roll Acc gain
	{-127,127,1,0,0}, 				// Roll Acc trim

	{0,127,1,0,60},					// Pitch gyro P
	{0,127,1,0,40}, 				// Pitch gyro I
	{0,125,1,0,10},					// Pitch gyro I-limits
	{0,4,1,0,2},					// Pitch gyro rate
	{0,127,1,0,5},					// Pitch Acc gain			
	{-127,127,1,0,0},				// Pitch Acc trim

	{0,127,1,0,80},					// Yaw gyro P
	{0,127,1,0,0},					// Yaw gyro I
	{0,125,1,0,0},					// Yaw gyro I-limits
	{0,4,1,0,1},					// Yaw gyro rate
	{-127,127,1,0,0},				// Yaw trim

	{0,127,1,0,0},					// Z Acc P gain
};

//************************************************************
// Main menu-specific setup
//************************************************************

void menu_flight(uint8_t mode)
{
	int8_t *value_ptr;
	menu_range_t range;
	uint8_t text_link;

	// If sub-menu item has changed, reset sub-menu positions
	if (menu_flag)
	{
		sub_top = FLIGHTSTART;
		menu_flag = 0;
	}

	while(button != BACK)
	{
		value_ptr = &Config.FlightMode[mode].Roll_P_mult;

		// Print menu
		print_menu_items(sub_top, FLIGHTSTART, value_ptr, (const unsigned char*)flight_menu_ranges, 0, FLIGHTOFFSET, (const unsigned char*)FlightMenuText, cursor);

		// Handle menu changes
		update_menu(FLIGHTITEMS, FLIGHTSTART, 0, button, &cursor, &sub_top, &menu_temp);
		range = get_menu_range ((const unsigned char*)flight_menu_ranges, (menu_temp - FLIGHTSTART));

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
			Wait_BUTTON4();			 // Wait for user's finger off the button
		}
	}
}

