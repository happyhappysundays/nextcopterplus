//***********************************************************
//* menu_battery.c
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include <avr/pgmspace.h> 
#include <avr/io.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <util/delay.h>
#include "..\inc\io_cfg.h"
#include "..\inc\init.h"
#include "..\inc\mugui.h"
#include "..\inc\glcd_menu.h"
#include "..\inc\menu_ext.h"
#include "..\inc\glcd_driver.h"
#include "..\inc\main.h"
#include "..\inc\eeprom.h"

//************************************************************
// Prototypes
//************************************************************

// Menu items
void menu_battery(void);

//************************************************************
// Defines
//************************************************************

#define BATTITEMS 5 	// Number of menu items
#define BATTSTART 53 	// Start of Menu text items
#define BATTTEXT 58 	// Start of value text items
#define BATTOFFSET 92	// Value offsets

//************************************************************
// Battery menu items
//************************************************************

const uint8_t BattMenuText[BATTITEMS] PROGMEM = {BATTTEXT, 0, 0, 0, 0};
const menu_range_t batt_menu_ranges[] PROGMEM = 
{
	{0,1,1,1,LIPO}, 	// Min, Max, Increment, Style, Default
	{0,12,1,0,0},		// Cells
	{0,127,1,0,27},		// Trigger
	{30,108,1,0,105},	// Max
	{20,100,1,0,83}		// Min
};

//************************************************************
// Main menu-specific setup
//************************************************************

void menu_battery(void)
{
	uint8_t batt_top = BATTSTART;
	int8_t *value_ptr;
	menu_range_t range;
	uint8_t text_link = 0;
	int8_t temp_cells = 0;
	int8_t temp_minvoltage = 0;

	
	while(button != BACK)
	{
		// Save pre-edited values
		temp_cells = Config.BatteryCells;
		temp_minvoltage = Config.MinVoltage;

		value_ptr = (int8_t*)&Config.BatteryType;

		// Print menu
		print_menu_items(batt_top, BATTSTART, &Config.BatteryType, 4, (prog_uchar*)batt_menu_ranges, 0, BATTOFFSET, (prog_uchar*)BattMenuText, cursor);

		// Handle menu changes
		update_menu(BATTITEMS, BATTSTART, 0, button, &cursor, &batt_top, &menu_temp);
		range = get_menu_range ((prog_uchar*)batt_menu_ranges, menu_temp - BATTSTART);

		if (button == ENTER)
		{
			text_link = pgm_read_byte(&BattMenuText[menu_temp - BATTSTART]);
			do_menu_item(menu_temp, value_ptr + (menu_temp - BATTSTART), 4, range, 0, text_link, false, 0);
		}

		// See if cell number or min_volts has changed
		if ((temp_cells != Config.BatteryCells) || (temp_minvoltage != Config.MinVoltage))
		{
			// Recalculate if more cells
			Config.PowerTrigger = Config.BatteryCells * Config.MinVoltage;
		}

		if (button == ENTER)
		{
			Save_Config_to_EEPROM(); // Save value and return
		}
	}
}

