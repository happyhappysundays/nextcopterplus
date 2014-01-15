//***********************************************************
//* menu_settings.c
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
#include "uart.h"

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

#define RCITEMS 9 		// Number of menu items
#define GENERALITEMS 8 

//************************************************************
// RC menu items
//************************************************************
	 
const uint8_t RCMenuText[3][RCITEMS] PROGMEM = 
{
	{RCTEXT, 116, 105, 141, 141, 141, 0, 0, 0},						// RC setup
	{GENERALTEXT, 0, 44, 0, 0, 119, 0, 0},							// General
};

const menu_range_t rc_menu_ranges[4][RCITEMS] PROGMEM = 
{
	{
		// RC setup (9)					// Min, Max, Increment, Style, Default
		{CPPM_MODE,SPEKTRUM,1,1,PWM1},	// Receiver type
		{JRSEQ,SATSEQ,1,1,JRSEQ}, 		// Channel order
		{THROTTLE,NOCHAN,1,1,GEAR},		// Profile select channel
		{NORMAL,REVERSED,1,1,NORMAL},	// Aileron reverse
		{NORMAL,REVERSED,1,1,NORMAL},	// Elevator reverse
		{NORMAL,REVERSED,1,1,NORMAL},	// Rudder reverse
		{0,4,1,0,2},					// Axis lock stick rate (0 is fastest, 4 is slowest).
		{0,10,1,0,0},					// TransitionSpeed 0 to 10
		{1,99,1,0,50},					// Transition P1n point
	},
	{
		// General (8)
		{HORIZONTAL,SIDEWAYS,1,1,HORIZONTAL}, // Orientation
		{28,50,1,0,38}, 				// Contrast
		{ARMED,ARMABLE,1,1,ARMABLE},	// Arming mode Armable/Armed
		{0,127,1,0,30},					// Auto-disarm enable
		{0,127,1,0,108},				// Low battery alarm voltage
		{LOW,HIGH,1,1,LOW},				// Servo rate
		{1,64,1,0,8},					// Acc. LPF
		{10,100,1,0,30},				// CF factor
	}
};
//************************************************************
// Main menu-specific setup
//************************************************************

void menu_rc_setup(uint8_t section)
{
	static uint8_t rc_top = RCSTART;

	int8_t *value_ptr;

	menu_range_t range;
	uint8_t text_link;
	uint8_t i = 0;
	uint8_t mult = 1;		// Multiplier
	uint8_t offset;			// Index into channel structure
	uint8_t	items;			// Items in group

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
				offset = 0;
				items = RCITEMS;
				value_ptr = &Config.RxMode;
				mult = 1;
				break;
			case 2:				// General menu
				offset = RCITEMS;
				items = GENERALITEMS;
				value_ptr = &Config.Orientation;
				mult = 1;
				break;
			default:
				offset = 0;
				items = RCITEMS;
				value_ptr = &Config.RxMode;
				mult = 1;
				break;
		}
		// Save pre-edited values
		int8_t temp_arm = Config.ArmMode;

		// Print menu
		print_menu_items(rc_top + offset, RCSTART + offset, value_ptr, mult, (prog_uchar*)rc_menu_ranges[section - 1], 0, RCOFFSET, (prog_uchar*)RCMenuText[section - 1], cursor);

		// Handle menu changes
		update_menu(items, RCSTART, offset, button, &cursor, &rc_top, &menu_temp);
		range = get_menu_range ((prog_uchar*)rc_menu_ranges[section - 1], (menu_temp - RCSTART - offset)); 

		if (button == ENTER)
		{
			text_link = pgm_read_byte(&RCMenuText[section - 1][menu_temp - RCSTART - offset]);
			do_menu_item(menu_temp, value_ptr + (menu_temp - RCSTART - offset), mult, range, 0, text_link, false, 0);
		}

		if (button == ENTER)
		{
			// See if arming mode has changed to ON
			if ((temp_arm == OFF) && (Config.ArmMode == ON))
			{
				General_error |= (1 << DISARMED);		// Set flags to disarmed
			}

			init_int();				// In case RC type has changed, reinitialise interrupts
			init_uart();			// and UART

			UpdateIMUvalues();		// Update IMU variables
			UpdateLimits();			// Update I-term limits and triggers based on percentages

			// Update channel sequence
			for (i = 0; i < MAX_RC_CHANNELS; i++)
			{
				if (Config.TxSeq == JRSEQ) 
				{
					Config.ChannelOrder[i] = pgm_read_byte(&JR[i]);
				}
				else if (Config.TxSeq == FUTABASEQ)
				{
					Config.ChannelOrder[i] = pgm_read_byte(&FUTABA[i]);
				}
				else if (Config.TxSeq == SATSEQ)
				{
					Config.ChannelOrder[i] = pgm_read_byte(&SATELLITE[i]);
				}
			}

			Save_Config_to_EEPROM(); // Save value and return
		}
	}
}

