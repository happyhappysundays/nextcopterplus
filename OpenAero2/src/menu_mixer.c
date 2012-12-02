//***********************************************************
//* menu_mixer.c
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include <avr/pgmspace.h> 
#include <avr/io.h>
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
#include "..\inc\mixer.h"

//************************************************************
// Prototypes
//************************************************************

// Menu items
void menu_mixer(uint8_t i);

//************************************************************
// Defines
//************************************************************

#define MIXERITEMS 26 	// Number of menu items
#define MIXERSTART 169 	// Start of Menu text items
#define MIXOFFSET  80	// Value offsets

//************************************************************
// RC menu items
//************************************************************
	 
const uint8_t MixerMenuText[MIXERITEMS] PROGMEM = {105,0,105,0,143,141,143,141,143,141,143,141,143,141,62,0,62,0,62,0,62,0,0,0,0,0};
const menu_range_t mixer_menu_ranges[] PROGMEM = 
{
	{THROTTLE,NOCHAN,1,1,CH1}, 		// Source A
	{-125,125,5,0,100},				// Source A volume (%)
	{THROTTLE,NOCHAN,1,1,NOCHAN}, 	// Source B
	{-125,125,5,0,0},				// Source B volume (%)
	{OFF, ON,1,1,OFF},				// roll_gyro
	{NORMAL, REVERSED,1,1,NORMAL},	// polarity
	{OFF, ON,1,1,OFF},				// pitch_gyro
	{NORMAL, REVERSED,1,1,NORMAL},	// polarity
	{OFF, ON,1,1,OFF},				// yaw_gyro
	{NORMAL, REVERSED,1,1,NORMAL},	// polarity
	{OFF, REVERSED,1,1,OFF},		// roll_acc
	{NORMAL, REVERSED,1,1,NORMAL},	// polarity
	{OFF, REVERSED,1,1,OFF},		// pitch_acc
	{NORMAL, REVERSED,1,1,NORMAL},	// polarity
	{CH1,CH8,1,1,CH1},				// Primary
	{-125,125,5,0,100},				// Primary volume
	{CH1,CH8,1,1,CH1},				// Output B
	{-125,125,5,0,0},				// Output B volume
	{CH1,CH8,1,1,CH1},				// Output C
	{-125,125,5,0,0},				// Output C volume
	{CH1,CH8,1,1,CH1},				// Output D
	{-125,125,5,0,0},				// Output D volume
	{-125,125,5,0,-100}, 			// Min travel
	{-125,125,5,0,100}, 			// Max travel
	{-125,125,5,0,0}, 				// Failsafe
	{-125,125,1,0,0}, 				// Offset
};

//************************************************************
// Main menu-specific setup
//************************************************************

void menu_mixer(uint8_t i)
{
	uint8_t cursor = LINE0;
	uint8_t top = MIXERSTART;
	int8_t values[MIXERITEMS];
	menu_range_t range;
	uint8_t temp = 0;
	uint8_t text_link = 0;

	while(button != BACK)
	{
		// Load values from eeprom
		memcpy(&values[0],&Config.Channel[i].source_a,sizeof(int8_t) * MIXERITEMS);

		// Print menu
		print_menu_items(top, MIXERSTART, &values[0], MIXERITEMS,(prog_uchar*)mixer_menu_ranges, MIXOFFSET, (prog_uchar*)MixerMenuText, cursor);

		// Handle menu changes
		update_menu(MIXERITEMS, MIXERSTART, button, &cursor, &top, &temp);
		range = get_menu_range ((prog_uchar*)mixer_menu_ranges, temp - MIXERSTART);

		if (button == ENTER)
		{
			text_link = pgm_read_byte(&MixerMenuText[temp - MIXERSTART]);
			values[temp - MIXERSTART] = do_menu_item(temp, values[temp - MIXERSTART], range, 0, text_link);
		}

		// Update value in config structure
		memcpy(&Config.Channel[i].source_a,&values[0],sizeof(int8_t) * MIXERITEMS);

		if (button == ENTER)
		{
			UpdateLimits();			 // Update travel limits based on percentages
			Save_Config_to_EEPROM(); // Save value and return
		}
	}
	menu_beep(1);
	_delay_ms(200);
}




