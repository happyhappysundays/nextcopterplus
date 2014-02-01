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
#define FSTEXT 53
#define GENERALTEXT	22
#define BATTTEXT 58 

#define RCITEMS 15 		// Number of menu items
#define FSITEMS 5 
#ifdef KK21
#define GENERALITEMS 15 
#else
#define GENERALITEMS 14
#endif
#define BATTITEMS 5 

//************************************************************
// RC menu items
//************************************************************
	 
const uint8_t RCMenuText[4][GENERALITEMS] PROGMEM = 
{
	{RCTEXT, 105, 116, 105, 105, 105, 0, 141, 141, 141, 141, 0, 0, 0, 0},	// RC setup
	{FSTEXT, 0, 0, 0, 0},													// Failsafe

#ifdef KK21
	{GENERALTEXT, 124, 0, 0, 0, 101, 119, 101, 0, 0, 101, 101, 0, 0, 243},	// General + MPU6050
#else
	{GENERALTEXT, 124, 0, 0, 0, 101, 119, 101, 0, 0, 101, 101, 0, 0},		// General
#endif

	{BATTTEXT, 0, 0, 0, 0}													// Battery
};

// Have to size each element to GENERALITEMS even though they are  smaller... fix this later
const menu_range_t rc_menu_ranges[4][GENERALITEMS] PROGMEM = 
{
	{
		// RC setup (15)
		{CPPM_MODE,SPEKTRUM,1,1,PWM},	// Receiver type
		{THROTTLE,GEAR,1,1,GEAR},		// PWM sync channel
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
		{0,4,1,0,2},					// Axis lock stick rate (0 is fastest, 4 is slowest).
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
		// General (14/15)
		{AEROPLANE,MANUAL,1,1,AEROPLANE}, 	// Mixer mode
		{HORIZONTAL,SIDEWAYS,1,1,HORIZONTAL}, // Orientation
		{28,50,1,0,38}, 				// Contrast
		{1,60,1,0,10},					// Status menu timeout
		{0,30,1,0,3},					// LMA enable
		{OFF,ON,1,1,OFF},				// Camstab enable
		{LOW,HIGH,1,1,LOW},				// Camstab servo rate
		{OFF,ON,1,1,OFF},				// Auto-center
		{1,127,1,0,8},					// Acc. LPF
		{10,100,5,0,30},				// CF factor
		{OFF,ON,1,1,ON},				// Advanced IMUType
		{OFF,ON,1,1,ON},				// Launch mode on/off
		{-55,125,10,0,0},				// Launch mode throttle position
		{0,60,1,0,10},					// Launch mode delay time
#ifdef KK21
		{0,6,1,1,6},					// MPU6050 LPF
#endif
	},
	{
		// Battery (5)
		{0,1,1,1,LIPO}, 				// Min, Max, Increment, Style, Default
		{0,12,1,0,0},					// Cells
		{0,127,4,2,27},					// Trigger / Alarm voltage (Display multiplied by 4) (108, Range 0 to 508)
		{30,108,4,2,105},				// Max (Display multiplied by 4) 4.2V (420, Range 120 to 432)
		{20,100,4,2,83},				// Min (Display multiplied by 4) 3.3V (330, Range 80 to 400)
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
	uint16_t temp16_1;

	// If submenu item has changed, reset submenu positions
	if (menu_flag)
	{
		rc_top = RCSTART;
		menu_flag = 0;
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
		int8_t temp_flapchan = Config.FlapChan;

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
			// Update Ch7. mixer with source from Config.FlapChan if in Aeroplane mode and source changed
			if ((Config.MixMode == AEROPLANE) && (Config.FlapChan != temp_flapchan))
			{
				Config.Channel[CH7].source_a = Config.FlapChan;
			}

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
					case MANUAL:
						// Clear all channel info
						memset(&Config.Channel[0].value,0,(sizeof(channel_t) * PSUEDO_OUTPUTS));

						// Preset important settings
						for (i = 0; i < PSUEDO_OUTPUTS; i++)
						{
							Config.Channel[i].source_a = NOCHAN;
							Config.Channel[i].source_b = NOCHAN;
							Config.Channel[i].output_b = UNUSED;
							Config.Channel[i].output_c = UNUSED;
							Config.Channel[i].output_d = UNUSED;
						}

						break;
					default:
						break;
				}
			}

			init_int();				// In case RC type has changed, reinitialise interrupts
			init_uart();			// and UART

			UpdateIMUvalues();		// Update IMU variables
			UpdateLimits();			// Update I-term limits and triggers based on percentages

#ifdef KK21
			// Update MPU6050 LPF
			writeI2Cbyte(MPU60X0_DEFAULT_ADDRESS, MPU60X0_RA_CONFIG, Config.MPU6050_LPF);
#endif
			// Update channel sequence
			for (i = 0; i < MAX_RC_CHANNELS; i++)
			{
				if (Config.TxSeq == FUTABASEQ)
				{
					Config.ChannelOrder[i] = (uint8_t)pgm_read_byte(&FUTABA[i]);
				}
				else
				{
					Config.ChannelOrder[i] = (uint8_t)pgm_read_byte(&JR[i]);
				}
			}

			Save_Config_to_EEPROM(); // Save value and return
		}
	}
}

