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
#include "..\inc\uart.h"

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
#define FSTEXT 53
#define GENERALTEXT	22
#define BATTTEXT 58 

#define RCITEMS 14 		// Number of menu items
#define FSITEMS 5 
#define GENERALITEMS 14 
#define BATTITEMS 5 

//************************************************************
// RC menu items
//************************************************************
	 
const uint8_t RCMenuText[4][GENERALITEMS] PROGMEM = 
{
	{RCTEXT, 116, 105, 105, 105, 0, 141, 141, 141, 141, 0, 0, 0, 0},	// RC setup
	{FSTEXT, 0, 0, 0, 0},												// Failsafe
	{GENERALTEXT, 124, 0, 0, 0, 101, 119, 101, 0, 0, 101, 101, 0, 0},	// General
	{BATTTEXT, 0, 0, 0, 0}												// Battery
};

// Have to size each element to GENERALITEMS even though they are  smaller... fix this later
const menu_range_t rc_menu_ranges[4][GENERALITEMS] PROGMEM = 
{
	{
		// RC setup (14)
		{CPPM_MODE,SPEKTRUM,1,1,PWM1},	// Min, Max, Increment, Style, Default
		{JRSEQ,SATSEQ,1,1,JRSEQ}, 		// Channel order
		{THROTTLE,NOCHAN,1,1,GEAR},		// Profile select channel
		{THROTTLE,NOCHAN,1,1,NOCHAN},	// Second aileron
		{THROTTLE,NOCHAN,1,1,AUX1},		// DynGainSrc
		{0,100,5,0,0},					// Dynamic gain
		{NORMAL,REVERSED,1,1,NORMAL},	// Aileron reverse
		{NORMAL,REVERSED,1,1,NORMAL},	// Second aileron reverse
		{NORMAL,REVERSED,1,1,NORMAL},	// Elevator reverse
		{NORMAL,REVERSED,1,1,NORMAL},	// Rudder reverse
		{0,100,5,0,0},					// Differential
		{0,20,1,0,0},					// Flap speed (0 is fastest, 20 slowest)	
		{0,5,1,0,2},					// Axis lock stick rate(0 is fastest, 5 slowest)
		{0,5,1,0,1},					// RC deadband (%)
	},
	{
		// Failsafe (5)
		{0,1,1,1,0}, 	
		{-100,100,1,0,-100},
		{-125,125,1,0,0},
		{-125,125,1,0,0},
		{-125,125,1,0,0},
	},
	{
		// General (14)
		{AEROPLANE,CAMSTAB,1,1,AEROPLANE}, 	// Mixer mode
		{HORIZONTAL,SIDEWAYS,1,1,HORIZONTAL}, // Orientation
		{28,50,1,0,38}, 				// Contrast
		{1,60,1,0,10},					// Status menu timeout
		{0,30,1,0,3},					// LMA enable
		{OFF,ON,1,1,OFF},				// Camstab enable
		{LOW,HIGH,1,1,LOW},				// Camstab servo rate
		{OFF,ON,1,1,OFF},				// Auto-center
		{1,64,1,0,8},					// Acc. LPF
		{10,100,5,0,30},				// CF factor
		{OFF,ON,1,1,ON},				// Advanced IMUType
		{OFF,ON,1,1,ON},				// Launch mode on/off
		{-55,125,10,0,0},				// Launch mode throttle position
		{0,60,1,0,10},					// Launch mode delay time
	},
	{
		// Battery (5)
		{0,1,1,1,LIPO}, 				// Min, Max, Increment, Style, Default
		{0,12,1,0,0},					// Cells
		{0,127,4,2,27},					// Trigger / Alarm voltage
		{30,108,4,2,105},				// Max
		{20,100,4,2,83},				// Min
	}
};
//************************************************************
// Main menu-specific setup
//************************************************************

void menu_rc_setup(uint8_t section)
{
	static uint8_t rc_top = RCSTART;
	static	uint8_t old_section;

	int8_t *value_ptr;

	menu_range_t range;
	uint8_t text_link;
	uint8_t i = 0;
	uint8_t mult = 1;		// Multiplier
	uint8_t offset;			// Index into channel structure
	uint8_t	items;			// Items in group
	uint16_t temp16_1;

	// If submenu item has changed, reset submenu positions
	if (section != old_section)
	{
		rc_top = RCSTART;
		old_section = section;
	}

	while(button != BACK)
	{
		// Get menu offsets and load values from eeprom
		// 1 = RC, 2 = Failsafe, 3 = General, 4 = Battery
		switch(section)
		{
			case 1:				// RC setup menu
				offset = 0;
				items = RCITEMS;
				value_ptr = &Config.RxMode;
				mult = 1;
				break;
			case 2:				// Failsafe menu
				offset = RCITEMS;
				items = FSITEMS;
				value_ptr = &Config.FailsafeType;
				mult = 1;
				break;
			case 3:				// General menu
				offset = RCITEMS + FSITEMS;
				items = GENERALITEMS;
				value_ptr = &Config.MixMode;
				mult = 1;
				break;
			case 4:				// Battery menu
				offset = RCITEMS + FSITEMS + GENERALITEMS;
				items = BATTITEMS;
				value_ptr = &Config.BatteryType;
				mult = 4;
				break;
			default:
				offset = 0;
				items = RCITEMS;
				value_ptr = &Config.RxMode;
				mult = 1;
				break;
		}
		// Save pre-edited values
		int8_t temp_type = Config.MixMode;
		int8_t temp_cells = Config.BatteryCells;
		int8_t temp_minvoltage = Config.MinVoltage;

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

		// Update Ch7. mixer with source from Config.FlapChan if in Aeroplane mode
		if (Config.MixMode == AEROPLANE)
		{
			Config.Channel[CH7].source_a = Config.FlapChan;
		}

		if (button == ENTER)
		{
			// See if cell number or min_volts has changed
			if ((temp_cells != Config.BatteryCells) || (temp_minvoltage != Config.MinVoltage))
			{
				// Recalculate if more cells
				temp16_1 = Config.MinVoltage;
				temp16_1 = temp16_1 * Config.BatteryCells;
				temp16_1 = temp16_1 / 10;
				Config.PowerTrigger = (int8_t)temp16_1;
			}

			// If model type has changed, reload preset
			if ((section == 3) && (temp_type != Config.MixMode)) 
			{
				switch(Config.MixMode)  // Load selected mix
				{
					case AEROPLANE:
						get_preset_mix(AEROPLANE_MIX);
						break;	
					case FWING:
						get_preset_mix(FLYING_WING_MIX);
						break;
					case CAMSTAB:
						get_preset_mix(CAM_STAB);
						break;
					default:
						break;
				}
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

