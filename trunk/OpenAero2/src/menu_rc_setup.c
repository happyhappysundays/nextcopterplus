//***********************************************************
//* menu_rc_setup.c
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

//************************************************************
// Prototypes
//************************************************************

// Menu items
void menu_rc_setup(void);

//************************************************************
// Defines
//************************************************************

#define RCITEMS 5 	// Number of menu items
#define RCSTART 149 // Start of Menu text items
#define RCTEXT 205 	// Start of value text items
#define RCOFFSET 92	// Value offsets

//************************************************************
// RC menu items
//************************************************************
	 
const uint8_t RCMenuText[RCITEMS] PROGMEM = {RCTEXT, 18, 105, 105, 105};
const menu_range_t rc_menu_ranges[] PROGMEM = 
{
	{JRSEQ,FUTABASEQ,1,1,JRSEQ}, 	// Min, Max, Increment, Style, Default
	{CPPM_MODE,PWM3,1,1,PWM1},
	{THROTTLE,NOCHAN,1,1,GEAR},		// Stabchan
	{THROTTLE,NOCHAN,1,1,GEAR},		// Autochan
	{THROTTLE,NOCHAN,1,1,NOCHAN},	// Second aileron
};
//************************************************************
// Main menu-specific setup
//************************************************************

void menu_rc_setup(void)
{
	uint8_t cursor = LINE0;
	uint8_t top = RCSTART;
	uint8_t temp = 0;
	int8_t values[RCITEMS];
	menu_range_t range;
	uint8_t i = 0;
	uint8_t text_link;
	
	while(button != BACK)
	{
		// Load values from eeprom
		memcpy(&values[0],&Config.TxSeq,sizeof(int8_t) * RCITEMS);

		// Print menu
		print_menu_items(top, RCSTART, &values[0], RCITEMS, (prog_uchar*)rc_menu_ranges, RCOFFSET, (prog_uchar*)RCMenuText, cursor);

		// Handle menu changes
		update_menu(RCITEMS, RCSTART, button, &cursor, &top, &temp);
		range = get_menu_range ((prog_uchar*)rc_menu_ranges, temp - RCSTART);

		if (button == ENTER)
		{
			text_link = pgm_read_byte(&RCMenuText[temp - RCSTART]);
			values[temp - RCSTART] = do_menu_item(temp, values[temp - RCSTART], range, 0, text_link);
		}

		// Update value in config structure
		memcpy(&Config.TxSeq,&values[0],sizeof(int8_t) * RCITEMS);

		// Update Ch7. mixer with source from Config.FlapChan if in Aeroplane mode
		if (Config.MixMode == AEROPLANE)
		{
			Config.Channel[CH7].source_a = Config.FlapChan;
		}

		if (button == ENTER)
		{
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

