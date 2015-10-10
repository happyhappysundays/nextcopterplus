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
#include "isr.h"
#include "MPU6050.h"

//************************************************************
// Prototypes
//************************************************************

// Menu items
void menu_rc_setup(uint8_t section);

//************************************************************
// Defines
//************************************************************

#define RCSTART 149 	// Start of Menu text items
#define RCTEXT 396 		// Start of "Receiver type" value text list
#define RCITEMS 11 		// Number of menu items displayed	
#define RCITEMSOFFSET 11// Actual number of menu items
#define RCOFFSET 65		// LCD offsets

#define GENERALTEXT	295 // Start of "Orientations" value text list
#define GENERALITEMS 11	// Number of menu items displayed
#define GENOFFSET 70	// LCD offsets

#define PRESETITEM 170	// Location of Preset menu item in list

//************************************************************
// RC menu items
//************************************************************
	 
const uint16_t RCMenuText[2][GENERALITEMS] PROGMEM = 
{
	{RCTEXT, 118, 105, 130, 105, 0, 0, 0, 0, 0,68},					// RC setup
	{GENERALTEXT, 320, 0, 53, 0, 0, 37, 37, 37, 0, 273},			// General 
};

const uint16_t RCMenuOffsets[2][GENERALITEMS] PROGMEM =
{
	{RCOFFSET, 65, 65, 60, 75, 95, 95, 95, 95, 95, 95},				// RC setup
	{GENOFFSET, 67, 67, 67, 80, 80, 80, 80, 80, 80, 80},			// General
};

const menu_range_t rc_menu_ranges[2][GENERALITEMS] PROGMEM = 
{
	{
		// RC setup (11)				// Min, Max, Increment, Style, Default
		{CPPM_MODE,MODEB,1,1,SBUS},		// Receiver type (PWM to MODEB/UDI)	
		{LOW,FAST,1,1,FAST},			// Servo rate
		{THROTTLE,GEAR,1,1,GEAR},		// PWM sync channel
		{JRSEQ,CUSTOM,1,1,JRSEQ},		// Channel order
		{THROTTLE,AUX3,1,1,GEAR},		// Profile select channel
		{0,40,1,0,0},					// Outbound TransitionSpeed 0 to 40
		{0,40,1,0,0},					// Inbound TransitionSpeed 0 to 40
		{0,99,1,0,0},					// Transition P1 point
		{1,99,1,0,50},					// Transition P1n point
		{1,100,1,0,100},				// Transition P2 point
		{OFF,ON,1,1,OFF},				// Vibration display
	},
	{
		// General (11)
		{UP_BACK,RIGHT_FRONT,1,1,UP_BACK},	// Orientation (P2)
		{NO_ORIENT,MODEL,1,1,NO_ORIENT},	// Orientation usage (Tail sitter)
		// Limit contrast range for KK2 Mini
#ifdef KK2Mini
		{26,34,1,0,30}, 				// Contrast (KK2 Mini)
#else
		{28,50,1,0,36}, 				// Contrast (Everything else)
#endif			
		{ARMED,ARMABLE,1,1,ARMABLE},	// Arming mode Armable/Armed
		{0,127,1,0,30},					// Auto-disarm enable
		{0,8,1,1,0},					// Low battery cell voltage
		{HZ5,HZ260,1,1,HZ44},			// MPU6050 LPF. Default is 44Hz
		{HZ5,NOFILTER,1,1,HZ21},		// Acc. LPF 21Hz default	(5, 10, 21, 44, 94, 184, 260, None)
		{HZ5,NOFILTER,1,1,NOFILTER},	// Gyro LPF. No LPF default (5, 10, 21, 44, 94, 184, 260, None)
		{2,11,1,0,6},					// AL correction
		{QUADX,BLANK,1,4,QUADX},		// Mixer preset (note: style 4)
	}
};

// These are the implied P1 orientations based on the user's P2 orientation.
const int8_t P1_Orientation_LUT[NUMBEROFORIENTS] PROGMEM = 
	{
		BACK_DOWN,BACK_LEFT,BACK_UP,BACK_RIGHT,
		DOWN_FRONT,DOWN_LEFT,DOWN_BACK,DOWN_RIGHT,
		FRONT_DOWN,FRONT_RIGHT,FRONT_UP,FRONT_LEFT,
		UP_FRONT,UP_RIGHT,UP_BACK,UP_LEFT,
		LEFT_FRONT,LEFT_UP,LEFT_BACK,LEFT_DOWN,
		RIGHT_FRONT,RIGHT_DOWN,RIGHT_BACK,RIGHT_UP
	};

//************************************************************
// Main menu-specific setup
//************************************************************

void menu_rc_setup(uint8_t section)
{
	int8_t *value_ptr = &Config.RxMode;

	menu_range_t range;
	uint16_t	text_link;
	uint8_t		i;
	uint16_t	offset = 0;			// Index into channel structure
	uint16_t	items= RCITEMS;		// Items in group
	
	// If submenu item has changed, reset submenu positions
	if (menu_flag)
	{
		sub_top = RCSTART;
		menu_flag = 0;
	}

	while(button != BACK)
	{
		// Get menu offsets and load values from eeprom
		// 1 = RC, 2 = General, 3 = Advanced
		switch(section)
		{
			case 1:				// RC setup menu
				break;
			case 2:				// General menu
				offset = RCITEMSOFFSET;
				items = GENERALITEMS;
				value_ptr = &Config.Orientation_P2;
				break;
			default:
				break;
		}

		// Always show preset text as "Options", regardless of actual setting
		Config.Preset = OPTIONS;

		// Print menu - note that print_menu_items() updates button variable.
		print_menu_items(sub_top + offset, RCSTART + offset, value_ptr, (const unsigned char*)rc_menu_ranges[section - 1], 0, (const uint16_t*)RCMenuOffsets[section - 1], (const uint16_t*)RCMenuText[section - 1], cursor);

		// Handle menu changes
		update_menu(items, RCSTART, offset, button, &cursor, &sub_top, &menu_temp);
		range = get_menu_range ((const unsigned char*)rc_menu_ranges[section - 1], (menu_temp - RCSTART - offset)); 

		// If actually editing the preset, show the default setting
		if (menu_temp == PRESETITEM)
		{
			Config.Preset = QUADX;			
		}

		if (button == ENTER)
		{
			text_link = pgm_read_word(&RCMenuText[section - 1][menu_temp - RCSTART - offset]);
			do_menu_item(menu_temp, value_ptr + (menu_temp - RCSTART - offset), 1, range, 0, text_link, false, 0);
		}

		// Handle abort neatly
		if (button == ABORT)
		{
			Wait_BUTTON1();			 // Wait for user's finger off the button
			button = NONE;
			
			// Reset the mixer preset if unchanged
			Config.Preset = OPTIONS;
		}

		if (button == ENTER)
		{
			init_int();				// In case RC type has changed, reinitialise interrupts
			init_uart();			// and UART
			UpdateLimits();			// Update I-term limits and triggers based on percentages
			
			// See if mixer preset has changed. Load new preset only if so
			if ((Config.Preset != OPTIONS) && (menu_temp == PRESETITEM))
			{
				Load_eeprom_preset(Config.Preset);
			}

			// Update MPU6050 LPF and reverse sense of menu items
			writeI2Cbyte(MPU60X0_DEFAULT_ADDRESS, MPU60X0_RA_CONFIG, (6 - Config.MPU6050_LPF));

			// Update channel sequence
			for (i = 0; i < MAX_RC_CHANNELS; i++)
			{
				if (Config.TxSeq == FUTABASEQ)
				{
					Config.ChannelOrder[i] = pgm_read_byte(&FUTABA[i]);
				}
				else if (Config.TxSeq == JRSEQ)
				{
					Config.ChannelOrder[i] = pgm_read_byte(&JR[i]);
				}
				else if (Config.TxSeq == MPXSEQ)
				{
					Config.ChannelOrder[i] = pgm_read_byte(&MPX[i]);
				}
				// Load from custom channel order
				else
				{
					Config.ChannelOrder[i] = Config.CustomChannelOrder[i];
				}
			}

			// Check validity of RX type and PWM speed selection
			// If illegal setting, drop down to RC Sync
			if ((Config.RxMode < SBUS) && (Config.Servo_rate == FAST))
			{
				Config.Servo_rate = SYNC;
			}
						
			if (Config.ArmMode == ARMABLE)
			{
				General_error |= (1 << DISARMED);	// Set flags to disarmed
				LED1 = 0;
			}

			// Work out the P1 orientation from the user's P2 orientation setting
			Config.Orientation_P1 = (int8_t)pgm_read_byte(&P1_Orientation_LUT[Config.Orientation_P2]);

			// Return the preset to "Options" when going back to the General menu
			Config.Preset = OPTIONS;

			Save_Config_to_EEPROM(); // Save value and return
			
			Wait_BUTTON4();			 // Wait for user's finger off the button
		}
	}
}

