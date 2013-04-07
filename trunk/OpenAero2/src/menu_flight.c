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

#define FLIGHTSTART 180 // Start of Menu text items
#define FLIGHTOFFSET 79	// LCD offsets
#define FLIGHTTEXT 38 	// Start of value text items
#define FLIGHTITEMS 22 	// Number of menu items

//************************************************************
// RC menu items
//************************************************************
	 
const uint8_t FlightMenuText[FLIGHTITEMS] PROGMEM = {FLIGHTTEXT, FLIGHTTEXT, 0, 48, 0, 0, 0, 0, 0, 0, 48, 0, 0, 0, 0, 0, 0, 48, 0, 0, 0, 0};

// Have to size each element to STABITEMS even though they are  smaller... fix this later
const menu_range_t flight_menu_ranges[FLIGHTITEMS] PROGMEM = 
{
	// Flight (22)
	{DISABLED,ALWAYSON,1,1,DISABLED},// Min, Max, Increment, Style, Default
	{DISABLED,HANDSFREE,1,1,DISABLED},
	{-125,125,5,0,0},				// Trigger
	{RATE,SP_LOCK, 1, 1, RATE},		// Gyro type
	{0,127,1,0,80},					// Roll PID
	{0,127,1,0,0},
	{0,127,1,0,0},
	{0,125,5,0,0},					// I-limits
	{0,127,1,0,60},					// Acc gain
	{-127,127,1,0,0}, 				// Acc trim
	{RATE,SP_LOCK, 1, 1, RATE},		// Gyro type
	{0,127,1,0,80},					// Pitch PID
	{0,127,1,0,0}, 
	{0,127,1,0,0},
	{0,125,5,0,0},					// I-limits
	{0,127,1,0,60},
	{-127,127,1,0,0},
	{RATE,SP_LOCK, 1, 1, RATE},		// Gyro type
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

	int8_t values[FLIGHTITEMS];
	menu_range_t range;
	uint8_t text_link;
	uint8_t temp_type = 0;

	while(button != BACK)
	{
		// Get values from eeprom
		memcpy(&values[0],&Config.FlightMode[mode - 1].StabMode,sizeof(int8_t) * FLIGHTITEMS);


		// Save pre-edited value for mixer mode
		temp_type = Config.MixMode;

		// Print menu
		print_menu_items(flight_top, FLIGHTSTART, &values[0], FLIGHTITEMS,  (prog_uchar*)flight_menu_ranges, 0, FLIGHTOFFSET, (prog_uchar*)FlightMenuText, cursor);

		// Handle menu changes
		update_menu(FLIGHTITEMS, FLIGHTSTART, 0, button, &cursor, &flight_top, &menu_temp);
		range = get_menu_range ((prog_uchar*)flight_menu_ranges, (menu_temp - FLIGHTSTART));

		if (button == ENTER)
		{
			text_link = pgm_read_byte(&FlightMenuText[menu_temp - FLIGHTSTART]);
			values[menu_temp - FLIGHTSTART] = do_menu_item(menu_temp, values[menu_temp - FLIGHTSTART], range, 0, text_link, false, 0);
		}

		// Update value in config structure
		memcpy(&Config.FlightMode[mode - 1].StabMode,&values[0],sizeof(int8_t) * FLIGHTITEMS);

		if (button == ENTER)
		{
			UpdateLimits();			 // Update I-term limits and triggers based on percentages

			Save_Config_to_EEPROM(); // Save value and return
		}
	}
}

