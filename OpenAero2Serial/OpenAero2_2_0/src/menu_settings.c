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
#include "isr.h"
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
#define RCOFFSET 82		// LCD offsets

#define RCTEXT 48 		// Start of value text items
#define FSTEXT 56
#define GENERALTEXT	22

#define RCITEMS 11 		// Number of menu items
#define FSITEMS 5 

// Need to skip over the blank to display the battery menu properly in KK2.0 as it has the MPU6050 LPF item
#ifdef KK21
#define GENERALITEMS 8
#define GENERALEXTRA 0 
#else
#define GENERALITEMS 7
#define GENERALEXTRA 1
#endif

//************************************************************
// RC menu items
//************************************************************
	 
const uint8_t RCMenuText[3][RCITEMS] PROGMEM = 
{
	{RCTEXT, RCTEXT, 116, 105, 105, 105, 0, 0, 0, 0, 0},				// RC setup
	{FSTEXT, 0, 0, 0, 0},												// Failsafe

#ifdef KK21
	{GENERALTEXT, 124, 0, 62, 0, 0, 241, 241},							// General + MPU6050
#else
	{GENERALTEXT, 124, 0, 62, 0, 0, 241},								// General
#endif
};

// Have to size each element to GENERALITEMS even though they are  smaller... fix this later
const menu_range_t rc_menu_ranges[3][RCITEMS] PROGMEM = 
{
	{
		// RC setup (11)
		{CPPM_MODE,XTREME,1,1,SBUS},	// Receiver type IN
		{SBUS,XTREME,1,1,SBUS},			// Receiver type OUT
		{JRSEQ,FUTABASEQ,1,1,JRSEQ}, 	// Channel order
		{THROTTLE,NOCHAN,1,1,GEAR},		// Profile select channel
		{THROTTLE,NOCHAN,1,1,NOCHAN},	// Second aileron
		{THROTTLE,NOCHAN,1,1,AUX1},		// DynGainSrc
		{0,100,5,0,100},				// Dynamic gain
		{0,100,1,0,0},					// Differential
		{0,20,1,0,0},					// Flap speed (0 is fastest, 20 slowest)	
		{0,4,1,0,3},					// Axis lock stick rate (0 is slowest, 4 is fastest).
		{0,5,1,0,2},					// RC deadband (%)
	},
	{
		// Failsafe (5)
		{NOFAILSAFE,ADVANCED,1,1,NOFAILSAFE}, 	// Failsafe type 
		{-100,100,1,0,-100},					// Throttle position in advanced failsafe (-125 to 125%)
		{-125,125,1,0,0},						// Elevator trim in advanced failsafe (-125 to 125%)
		{-125,125,1,0,0},						// Aileron trim in advanced failsafe (-125 to 125%)
		{-125,125,1,0,0},						// Rudder trim in advanced failsafe (-125 to 125%)
	},
	{
		// General (7/8)
		{AEROPLANE,MANUAL,1,1,AEROPLANE}, 	// Mixer mode
		{HORIZONTAL,PITCHUP,1,1,HORIZONTAL}, // Orientation
#ifdef KK2Mini
		{26,34,1,0,30}, 				// Contrast (KK2 Mini)
#else
		{28,50,1,0,36}, 				// Contrast (Everything else)
#endif			
		{0,8,1,1,0},					// Low battery cell voltage
		{0,30,1,0,3},					// LMA enable
		{1,10,1,0,7},					// CF factor
		{HZ5,NOFILTER,1,1,HZ21},		// Acc. LPF 21Hz default	(5, 10, 21, 44, 94, 184, 260, None)
#ifdef KK21
		{HZ5,HZ260,1,1,HZ21},			// MPU6050 LPF
#endif
	},
};

//************************************************************
// Main menu-specific setup
//************************************************************

void menu_rc_setup(uint8_t section)
{
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
		sub_top = RCSTART;
		menu_flag = 0;
	}

	while(button != BACK)
	{
		// Get menu offsets and load values from eeprom
		// 1 = RC, 2 = Failsafe, 3 = General
		switch(section)
		{
			case 1:				// RC setup menu
				offset = 0;
				items = RCITEMS;
				value_ptr = &Config.RxModeIn;
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
			default:
				offset = 0;
				items = RCITEMS;
				value_ptr = &Config.RxModeIn;
				mult = 1;
				break;
		}
		// Save pre-edited values
		int8_t temp_type = Config.MixMode;
		int8_t temp_flapchan = Config.FlapChan;
		int8_t temp_RxModeIn = Config.RxModeIn;

		// Print menu
		print_menu_items(sub_top + offset, RCSTART + offset, value_ptr, mult, (const unsigned char*)rc_menu_ranges[section - 1], 0, RCOFFSET, (const unsigned char*)RCMenuText[section - 1], cursor);

		// Handle menu changes
		update_menu(items, RCSTART, offset, button, &cursor, &sub_top, &menu_temp);
		range = get_menu_range ((const unsigned char*)rc_menu_ranges[section - 1], (menu_temp - RCSTART - offset)); 

		if (button == ENTER)
		{
			text_link = pgm_read_byte(&RCMenuText[section - 1][menu_temp - RCSTART - offset]);
			do_menu_item(menu_temp, value_ptr + (menu_temp - RCSTART - offset), mult, range, 0, text_link, false, 0);
		}

		if (button == ENTER)
		{
			// Update Ch5. mixer with source from Config.FlapChan if in Aeroplane mode and source changed
			if ((Config.MixMode == AEROPLANE) && (Config.FlapChan != temp_flapchan))
			{
				Config.Channel[CH5].source_a = Config.FlapChan;
			}

			// Reset serial in channel masks every time the input type is changed
			if (temp_RxModeIn != Config.RxModeIn)
			{
				Xtreme_Chanmask = 0;
				Xtreme_RSS = 0;
				Spektrum_Chanmask_0 = 0;	
				Spektrum_Chanmask_1 = 0;		
				Spektrum_frameloss = 0;
				SBUS_Flags = 0;
				
				// Clear channel data
				for (i = 0; i < MAX_RC_CHANNELS; i++)
				{
					RxChannel[i] = 0;
					
					// Unused Spektrum channels set to NULL
					if (Config.RxModeOut == SPEKTRUM)
					{
						ExtChannel[i] = 0xFFFF;
					}	
					// Unused channels set to mid-way
					else if (Config.RxModeOut == SBUS)
					{
						ExtChannel[i] = 0x400;
					}
					// Xtreme doesn't care
					else
					{
						ExtChannel[i] = 0;
					}
				}
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
					case MANUAL:
						// Clear all channel info
						memset(&Config.Channel[0].value,0,(sizeof(channel_t) * MAX_OUTPUTS));

						// Preset important settings
						for (i = 0; i < MAX_OUTPUTS; i++)
						{
							Config.Channel[i].source_a = i;			// Set to mirror the inputs
							Config.Channel[i].source_a_volume = 100;
							Config.Channel[i].source_b = NOCHAN;
							Config.Channel[i].output_b = UNUSED;
							Config.Channel[i].output_c = UNUSED;
						}

						break;
					default:
						break;
				}
			}

			init_int();				// In case RC type has changed, reinitialise interrupts
			init_uart();			// and UART
			UpdateLimits();			// Update I-term limits and triggers based on percentages

#ifdef KK21
			// Update MPU6050 LPF
			writeI2Cbyte(MPU60X0_DEFAULT_ADDRESS, MPU60X0_RA_CONFIG, (6 - Config.MPU6050_LPF));
#endif
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
			
			Wait_BUTTON4();			 // Wait for user's finger off the button
		}
	}
}

