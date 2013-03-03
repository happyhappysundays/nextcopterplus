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

//************************************************************
// Prototypes
//************************************************************

// Menu items
void menu_rc_setup(uint8_t i);

//************************************************************
// Defines
//************************************************************

#define NUMMENUITEMS 49	// Total number of all menu items
#define RCSTART 149 	// Start of Menu text items
#define RCOFFSET 78		// LCD offsets

#define RCTEXT 18 		// Start of value text items
#define FSTEXT 103
#define AUTOTEXT 38 
#define STABTEXT 38
#define GENERALTEXT	22

#define RCITEMS 8 		// Number of menu items
#define FSITEMS 5 
#define AUTOITEMS 8 
#define STABITEMS 16 
#define GENERALITEMS 12 

//************************************************************
// RC menu items
//************************************************************
	 
const uint8_t RCMenuText[5][STABITEMS] PROGMEM = 
{
	{RCTEXT, 116, 105, 105, 105, 141, 141, 141},				// RC setup
	{FSTEXT, 0, 0, 0, 0},										// Failsafe
	{AUTOTEXT, 0, 0, 0, 0, 0, 101, 0},							// Autolevel
	{STABTEXT, 0, 105, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},	// Stability
	{GENERALTEXT, 124, 101, 0, 0, 101, 119, 0, 0, 48, 0, 101},	// General
};


// Have to size each element to STABITEMS even though they are  smaller... fix this later
const menu_range_t rc_menu_ranges[5][STABITEMS] PROGMEM = 
{
	{
		// RC setup (8)
		{CPPM_MODE,PWM3,1,1,PWM1},		// Min, Max, Increment, Style, Default
		{JRSEQ,FUTABASEQ,1,1,JRSEQ}, 	// Channel order
		{THROTTLE,NOCHAN,1,1,GEAR},		// Stabchan
		{THROTTLE,NOCHAN,1,1,GEAR},		// Autochan
		{THROTTLE,NOCHAN,1,1,NOCHAN},	// Second aileron
		{NORMAL,REVERSED,1,1,NORMAL},	// Aileron reverse
		{NORMAL,REVERSED,1,1,NORMAL},	// Elevator reverse
		{NORMAL,REVERSED,1,1,NORMAL},	// Rudder reverse
	},
	{
		// Failsafe (5)
		{0,1,1,1,1}, 	
		{-100,100,1,0,-100},
		{-125,125,1,0,0},
		{-125,125,1,0,0},
		{-125,125,1,0,0},
	},
	{
		// Autolevel (8)
		{DISABLED,HANDSFREE,1,1,AUTOCHAN},
		{-125,125,10,0,10},
		{0,127,1,0,60},
		{0,127,1,0,60},
		{-127,127,1,0,0}, 
		{-127,127,1,0,0},
		{OFF,ON,1,1,OFF},				// Launch mode
		{-125,125,10,0,0},				// Launch trigger
	},
	{
		// Stability (16)
		{DISABLED,ALWAYSON,1,1,STABCHAN},// Min, Max, Increment, Style, Default
		{-125,125,10,0,-30},
		{THROTTLE,NOCHAN,1,1,NOCHAN}, 	// Dynamic P gain channel
		{0,100,5,0,0},					// Dynamic gain amount
		{0,127,1,0,80},
		{0,127,1,0,0},
		{0,127,1,0,0},
		{0,127,1,0,80},
		{0,127,1,0,0}, 
		{0,127,1,0,0},
		{0,127,1,0,80},
		{0,127,1,0,0},
		{0,127,1,0,0},
		{0,125,5,0,0},
		{0,125,5,0,0},
		{0,125,5,0,0},
	},
	{
		// General (12)
		{AEROPLANE,CAMSTAB,1,1,AEROPLANE}, 	
		{HORIZONTAL,AFT,1,1,HORIZONTAL},
		{28,50,1,0,38}, 				// Contrast
		{1,30,1,0,5},					// Status menu timeout
		{0,30,1,0,1},					// LMA enable
		{OFF,ON,1,1,OFF},				// Camstab enable
		{LOW,HIGH,1,1,LOW},				// Camstab servo rate
		{1,64,1,0,8},					// Acc. LPF
		{10,100,5,0,30},				// CF factor
		{STD,FIXED,1,1,STD},			// HH mode
		{0,5,1,0,4},					// 3D rate (0 is fastest, 5 slowest)
		{OFF,ON,1,1,ON},				// Advanced IMUType
	}
};
//************************************************************
// Main menu-specific setup
//************************************************************

void menu_rc_setup(uint8_t section)
{
	static	uint8_t rc_top = RCSTART;
	static	uint8_t old_section;

	int8_t values[STABITEMS]; // This has to be large enough to hold the largest number of menu items (currently STABITEMS)
	menu_range_t range;
	uint8_t text_link;
	uint8_t i = 0;
	uint8_t temp_type = 0;

	uint8_t offset;			// Index into channel structure
	uint8_t	items;			// Items in group

	// If submenu item has changed, reset submenu positions
	if (section != old_section)
	{
		rc_top = RCSTART;
		old_section = section;
	}
		
	while(button != BACK)
	{
		// Get menu offsets and load values from eeprom
		// 1 = RC, 2 = Failsafe, 3 = Autolevel, 4 = Stability, 5 = General
		switch(section)
		{
			case 1:				// RC setup menu
				offset = 0;
				items = RCITEMS;
				memcpy(&values[0],&Config.RxMode,sizeof(int8_t) * RCITEMS);
				break;
			case 2:				// Failsafe menu
				offset = RCITEMS;
				items = FSITEMS;
				memcpy(&values[0],&Config.FailsafeType,sizeof(int8_t) * FSITEMS);
				break;
			case 3:				// Autolevel menu
				offset = RCITEMS + FSITEMS;
				items = AUTOITEMS;
				memcpy(&values[0],&Config.AutoMode,sizeof(int8_t) * AUTOITEMS);
				break;
			case 4:				// Stability menu
				offset = RCITEMS + FSITEMS + AUTOITEMS;
				items = STABITEMS;
				memcpy(&values[0],&Config.StabMode,sizeof(int8_t) * STABITEMS);
				break;
			case 5:				// General menu
				offset = RCITEMS + FSITEMS + AUTOITEMS + STABITEMS;
				items = GENERALITEMS;
				memcpy(&values[0],&Config.MixMode,sizeof(int8_t) * GENERALITEMS);
				break;
			default:
				offset = 0;
				items = RCITEMS;
				break;
		}

		// Save pre-edited value for mixer mode
		temp_type = Config.MixMode;

		// Print menu
		print_menu_items(rc_top + offset, RCSTART + offset, &values[0], items, (prog_uchar*)rc_menu_ranges[section - 1], RCOFFSET, (prog_uchar*)RCMenuText[section - 1], cursor);

		// Handle menu changes
		update_menu(items, RCSTART, offset, button, &cursor, &rc_top, &menu_temp);
		range = get_menu_range ((prog_uchar*)rc_menu_ranges[section - 1], (menu_temp - RCSTART - offset)); //186 - 149 - 37 = 0

		if (button == ENTER)
		{
			text_link = pgm_read_byte(&RCMenuText[section - 1][menu_temp - RCSTART - offset]);
			values[menu_temp - RCSTART - offset] = do_menu_item(menu_temp, values[menu_temp - RCSTART - offset], range, 0, text_link, false, 0);
		}

		// Update value in config structure
		switch(section)
		{
			case 1:				// RC setup menu
				memcpy(&Config.RxMode,&values[0],sizeof(int8_t) * RCITEMS);
				break;
			case 2:				// Failsafe menu
				memcpy(&Config.FailsafeType,&values[0],sizeof(int8_t) * FSITEMS);
				break;
			case 3:				// Autolevel menu
				memcpy(&Config.AutoMode,&values[0],sizeof(int8_t) * AUTOITEMS);
				break;
			case 4:				// Stability menu
				memcpy(&Config.StabMode,&values[0],sizeof(int8_t) * STABITEMS);
				break;
			case 5:				// General menu
				memcpy(&Config.MixMode,&values[0],sizeof(int8_t) * GENERALITEMS);
				break;
			default:
				break;
		}

		// Update Ch7. mixer with source from Config.FlapChan if in Aeroplane mode
		if (Config.MixMode == AEROPLANE)
		{
			Config.Channel[CH7].source_a = Config.FlapChan;
		}

		if (button == ENTER)
		{
			// If model type has changed, reload preset
			if ((section == 5) && (temp_type != values[0])) 
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

			UpdateIMUvalues();		 // Update IMU variables
			UpdateLimits();			 // Update I-term limits and triggers based on percentages

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
			}

			Save_Config_to_EEPROM(); // Save value and return
		}
	}
}

