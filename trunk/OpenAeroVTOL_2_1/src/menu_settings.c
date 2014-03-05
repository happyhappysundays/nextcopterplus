//***********************************************************
//* menu_settings.c
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include "compiledefs.h"
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
#include "uart.h"
#include "i2c.h"
#include "MPU6050.h"

//************************************************************
// Prototypes
//************************************************************

// Menu items
void menu_rc_setup(uint8_t i);

//************************************************************
// Defines
//************************************************************

#define RCSTART 149 	// Start of Menu text items
#define RCOFFSET 79		// LCD offsets

#define RCTEXT 62 		// Start of value text items
#define GENERALTEXT	124
#define RCITEMS 10 		// Number of menu items
#define GENERALITEMS 8

//************************************************************
// RC menu items
//************************************************************
	 
const uint8_t RCMenuText[2][RCITEMS] PROGMEM = 
{
	{RCTEXT, 105, 116, 105, 141, 141, 141, 0, 0, 0},				// RC setup
	{GENERALTEXT, 0, 44, 0, 0, 119, 0, 0},							// General
};

const menu_range_t rc_menu_ranges[2][RCITEMS] PROGMEM = 
{
	{
		// RC setup (10)				// Min, Max, Increment, Style, Default
		{CPPM_MODE,SPEKTRUM,1,1,PWM},	// Receiver type
		{THROTTLE,GEAR,1,1,GEAR},		// PWM sync channel
		{JRSEQ,FUTABASEQ,1,1,JRSEQ}, 	// Channel order
		{THROTTLE,NOCHAN,1,1,GEAR},		// Profile select channel
		{NORMAL,REVERSED,1,1,NORMAL},	// Aileron reverse
		{NORMAL,REVERSED,1,1,NORMAL},	// Elevator reverse
		{NORMAL,REVERSED,1,1,NORMAL},	// Rudder reverse
		{0,4,1,0,3},					// Axis lock stick rate (0 is fastest, 4 is slowest).
		{0,10,1,0,0},					// TransitionSpeed 0 to 10
		{1,99,1,0,50},					// Transition P1n point
	},
	{
		// General (8)
		{HORIZONTAL,PITCHUP,1,1,HORIZONTAL}, // Orientation
		{28,50,1,0,38}, 				// Contrast
		{ARMED,ARMABLE,1,1,ARMABLE},	// Arming mode Armable/Armed
		{0,127,1,0,30},					// Auto-disarm enable
		{0,127,1,0,108},				// Low battery alarm voltage
		{LOW,HIGH,1,1,LOW},				// Servo rate
		{1,127,1,0,8},					// Acc. LPF
		{1,100,1,0,30},					// CF factor
	}
};
//************************************************************
// Main menu-specific setup
//************************************************************

void menu_rc_setup(uint8_t section)
{
	static uint8_t rc_top = RCSTART;

	int8_t *value_ptr = &Config.RxMode;

	menu_range_t range;
	uint8_t text_link;
	uint8_t i;
	uint8_t offset = 0;			// Index into channel structure
	uint8_t	items= RCITEMS;		// Items in group

	// If submenu item has changed, reset submenu positions
	if (menu_flag)
	{
		rc_top = RCSTART;
		menu_flag = 0;
	}

	while(button != BACK)
	{
		// Get menu offsets and load values from eeprom
		// 1 = RC, 2 = General
		switch(section)
		{
			case 1:				// RC setup menu
				break;
			case 2:				// General menu
				offset = RCITEMS;
				items = GENERALITEMS;
				value_ptr = &Config.Orientation;
				break;
			default:
				break;
		}

		// Print menu
		print_menu_items(rc_top + offset, RCSTART + offset, value_ptr, 1, (const unsigned char*)rc_menu_ranges[section - 1], 0, RCOFFSET, (const unsigned char*)RCMenuText[section - 1], cursor);

		// Handle menu changes
		update_menu(items, RCSTART, offset, button, &cursor, &rc_top, &menu_temp);
		range = get_menu_range ((const unsigned char*)rc_menu_ranges[section - 1], (menu_temp - RCSTART - offset)); 

		if (button == ENTER)
		{
			text_link = pgm_read_byte(&RCMenuText[section - 1][menu_temp - RCSTART - offset]);
			do_menu_item(menu_temp, value_ptr + (menu_temp - RCSTART - offset), 1, range, 0, text_link, false, 0);
		}

		if (button == ENTER)
		{
			init_int();				// In case RC type has changed, reinitialise interrupts
			init_uart();			// and UART

			UpdateIMUvalues();		// Update IMU variables
			UpdateLimits();			// Update I-term limits and triggers based on percentages

			// Update channel sequence
			for (i = 0; i < MAX_RC_CHANNELS; i++)
			{
				if (Config.TxSeq == FUTABASEQ)
				{
					Config.ChannelOrder[i] = pgm_read_byte(&FUTABA[i]);
				}
				else
				{
					Config.ChannelOrder[i] = pgm_read_byte(&JR[i]);
				}
			}

			Save_Config_to_EEPROM(); // Save value and return
		}
	}
}

