//***********************************************************
//* menu_rc_setup.c
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include <avr/pgmspace.h> 
#include <avr/io.h>
#include <stdlib.h>
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
#define RCSTART 74 	// Start of Menu text items
#define RCTEXT 80 	// Start of value text items

//************************************************************
// RC menu items
//************************************************************
	 
const uint8_t RCMenuOffsets[RCITEMS] PROGMEM = {92, 92, 92, 92, 92};
const uint8_t RCMenuText[RCITEMS] PROGMEM = {RCTEXT, 29, 149, 149, 149};
const menu_range_t rc_menu_ranges[] = 
{
	{JRSEQ,FUTABASEQ,1,1}, 	// Min, Max, Increment
	{CPPM_MODE,PWM3,1,1},
	{THROTTLE,NOCHAN,1,1},
	{THROTTLE,NOCHAN,1,1},
	{THROTTLE,NOCHAN,1,1}
};

//************************************************************
// Main menu-specific setup
//************************************************************

void menu_rc_setup(void)
{
	uint8_t cursor = LINE0;
	uint8_t button = 0;
	uint8_t top = RCSTART;
	uint8_t temp = 0;
	int16_t values[RCITEMS];
	menu_range_t range;
	uint8_t i = 0;
	uint8_t text_link;
	
	while(button != BACK)
	{
		// Clear buffer before each update
		clear_buffer(buffer);	

		// Load values from eeprom
		values[0] = Config.TxSeq;
		values[1] = Config.RxMode;
		values[2] = Config.StabChan;
		values[3] = Config.AutoChan;
		values[4] = Config.ThreePos;

		// Print menu
		print_menu_items(top, RCSTART, &values[0], &rc_menu_ranges[0], (prog_uchar*)RCMenuOffsets, (prog_uchar*)RCMenuText, cursor);

		// Poll buttons when idle
		button = poll_buttons();
		while (button == NONE)					
		{
			button = poll_buttons();
		}

		// Handle menu changes
		update_menu(RCITEMS, RCSTART, button, &cursor, &top, &temp);

		range = rc_menu_ranges[temp - RCSTART];

		if (button == ENTER)
		{
			text_link = pgm_read_byte(&RCMenuText[temp - RCSTART]);
			values[temp - RCSTART] = do_menu_item(temp, values[temp - RCSTART], range, 0, text_link);
		}

		// Update value in config structure
		Config.TxSeq = values[0];
		Config.RxMode = values[1];
		Config.StabChan = values[2];
		Config.AutoChan = values[3];
		Config.ThreePos = values[4];

		if (button == ENTER)
		{
			// Update channel sequence
			for (i = 0; i < MAX_OUTPUTS; i++)
			{
				if (Config.TxSeq == JRSEQ) 
				{
					Config.ChannelOrder[i] = JR[i];
				}
				else if (Config.TxSeq == FUTABASEQ)
				{
					Config.ChannelOrder[i] = FUTABA[i];
				}
			}
			Save_Config_to_EEPROM(); // Save value and return
		}
	}
	menu_beep();
	_delay_ms(200);
}

