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

#define FLIGHTSTARTE 172 // Start of Menu text items for EARTH
#define FLIGHTSTARTM 328 // Start of Menu text items for MODEL
#define FLIGHTOFFSET 85	// LCD offsets
#define FLIGHTTEXT 38 	// Start of value text items
#define FLIGHTITEMS 20 	// Number of menu items

//************************************************************
// RC menu items
//************************************************************

const uint16_t FlightMenuText[FLIGHTITEMS] PROGMEM = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	
const uint16_t FlightMenuOffsets[FLIGHTITEMS] PROGMEM = {FLIGHTOFFSET, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85};

const menu_range_t flight_menu_ranges[FLIGHTITEMS] PROGMEM = 
{
	// Flight (20)
	{0,127,1,0,40},					// Roll gyro P
	{0,127,1,0,10},					// Roll gyro I
	{0,125,1,0,10},					// Roll gyro I-limit
	{0,7,1,0,2},					// Roll gyro rate
	{0,127,1,0,10},					// Roll Acc gain
	{-127,127,1,0,0}, 				// Roll Acc trim

	{0,127,1,0,40},					// Pitch gyro P
	{0,127,1,0,10}, 				// Pitch gyro I
	{0,125,1,0,10},					// Pitch gyro I-limit
	{0,7,1,0,2},					// Pitch gyro rate
	{0,127,1,0,10},					// Pitch Acc gain			
	{-127,127,1,0,0},				// Pitch Acc trim

	{0,127,1,0,60},					// Yaw gyro P
	{0,127,1,0,40},					// Yaw gyro I
	{0,125,1,0,25},					// Yaw gyro I-limit
	{0,7,1,0,2},					// Yaw gyro rate
	{-127,127,1,0,0},				// Yaw trim

	{0,127,1,0,40},					// Z Acc P gain
	{0,127,1,0,20},					// Z Acc I gain
	{0,125,1,0,10},					// Z Acc I-limit
};

//************************************************************
// Main menu-specific setup
//************************************************************

void menu_flight(uint8_t mode)
{
	int8_t *value_ptr;
	menu_range_t range;
	uint8_t text_link;
	uint16_t reference;

	// Set the correct text list for the selected reference
	if ((Config.P1_Reference == MODEL) && (mode == P1))
	{
		reference = FLIGHTSTARTM;
	}
	else
	{
		reference = FLIGHTSTARTE;
	}

	// If sub-menu item has changed, reset sub-menu positions
	if (menu_flag)
	{
		if ((Config.P1_Reference == MODEL) && (mode == P1))
		{
			sub_top = FLIGHTSTARTM;		
		}
		else
		{
			sub_top = FLIGHTSTARTE;			
		}

		menu_flag = 0;
	}

	while(button != BACK)
	{
		value_ptr = &Config.FlightMode[mode].Roll_P_mult;

		// Print menu
		print_menu_items(sub_top, reference, value_ptr, (const unsigned char*)flight_menu_ranges, 0, (const uint16_t*)FlightMenuOffsets, (const uint16_t*)FlightMenuText, cursor);

		// Handle menu changes
		update_menu(FLIGHTITEMS, reference, 0, button, &cursor, &sub_top, &menu_temp);
		range = get_menu_range ((const unsigned char*)flight_menu_ranges, (menu_temp - reference));

		if (button == ENTER)
		{
			text_link = pgm_read_word(&FlightMenuText[menu_temp - reference]);
			do_menu_item(menu_temp, value_ptr + (menu_temp - reference), 1, range, 0, text_link, false, 0);
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

