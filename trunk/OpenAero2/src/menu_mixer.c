//***********************************************************
//* menu_mixer.c
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include <avr/pgmspace.h> 
#include <avr/io.h>
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

//************************************************************
// Prototypes
//************************************************************

// Menu items
void menu_mixer(uint8_t i);

//************************************************************
// Defines
//************************************************************

#define MIXERITEMS 16 	// Number of menu items
#define MIXERSTART 125 	// Start of Menu text items

//************************************************************
// RC menu items
//************************************************************
	 
const uint8_t MixerMenuOffsets[MIXERITEMS] PROGMEM = {80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80};
const uint8_t MixerMenuText[MIXERITEMS] PROGMEM = {149,141,0,143,141,143,141,143,141,143,141,143,141,0,0,0};
const menu_range_t mixer_menu_ranges[] PROGMEM = 
{
	{THROTTLE,PRESET4,1,1,CH1}, 	// Source
	{NORMAL,REVERSED,1,1,NORMAL}, 	// source_polarity
	{0,125,10,0,100},				// source_volume (%)
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
	{-125,125,5,0,-100}, 			// Min travel
	{-125,125,5,0,100}, 			// Max travel
	{-125,125,5,0,0}, 				// failsafe
};

//************************************************************
// Main menu-specific setup
//************************************************************

void menu_mixer(uint8_t i)
{
	uint8_t cursor = LINE0;
	uint8_t button = 0;
	uint8_t top = MIXERSTART;
	int16_t values[MIXERITEMS];
	menu_range_t range;
	uint8_t temp = 0;
	uint8_t text_link = 0;

	while(button != BACK)
	{
		// Clear buffer before each update
		clear_buffer(buffer);	

		// Load values from eeprom
		values[0] = (uint8_t)Config.Channel[i].source;
		values[1] = (uint8_t)Config.Channel[i].source_polarity;
		values[2] = (uint8_t)Config.Channel[i].source_volume;
		values[3] = (uint8_t)Config.Channel[i].roll_gyro;
		values[4] = (uint8_t)Config.Channel[i].roll_gyro_polarity;
		values[5] = (uint8_t)Config.Channel[i].pitch_gyro;
		values[6] = (uint8_t)Config.Channel[i].pitch_gyro_polarity;
		values[7] = (uint8_t)Config.Channel[i].yaw_gyro;
		values[8] = (uint8_t)Config.Channel[i].yaw_gyro_polarity;
		values[9] = (uint8_t)Config.Channel[i].roll_acc;
		values[10] = (uint8_t)Config.Channel[i].roll_acc_polarity;
		values[11] = (uint8_t)Config.Channel[i].pitch_acc;
		values[12] = (uint8_t)Config.Channel[i].pitch_acc_polarity;

		// Re-span travel limits and failsafe to +/- 125%
		values[13] = ((Config.Channel[i].min_travel - 3750) / 12); 
		values[14] = ((Config.Channel[i].max_travel - 3750) / 12); 
		values[15] = ((Config.Channel[i].Failsafe - 3750) / 12); 

		// Print menu
		print_menu_items(top, MIXERSTART, &values[0], (prog_uchar*)mixer_menu_ranges, (prog_uchar*)MixerMenuOffsets, (prog_uchar*)MixerMenuText, cursor);
		

		// Poll buttons when idle
		button = poll_buttons();
		while (button == NONE)					
		{
			button = poll_buttons();
		}

		// Handle menu changes
		update_menu(MIXERITEMS, MIXERSTART, button, &cursor, &top, &temp);

		range = get_menu_range ((prog_uchar*)mixer_menu_ranges, temp - MIXERSTART);
		//range = mixer_menu_ranges[temp - MIXERSTART];

		if (button == ENTER)
		{
			text_link = pgm_read_byte(&MixerMenuText[temp - MIXERSTART]);
			values[temp - MIXERSTART] = do_menu_item(temp, values[temp - MIXERSTART], range, 0, text_link);
		}

		// Update value in config structure
		Config.Channel[i].source = values[0];
		Config.Channel[i].source_polarity = values[1];
		Config.Channel[i].source_volume = values[2];
		Config.Channel[i].roll_gyro = values[3];
		Config.Channel[i].roll_gyro_polarity = values[4];
		Config.Channel[i].pitch_gyro = values[5];
		Config.Channel[i].pitch_gyro_polarity = values[6];
		Config.Channel[i].yaw_gyro = values[7];
		Config.Channel[i].yaw_gyro_polarity = values[8];
		Config.Channel[i].roll_acc = values[9];
		Config.Channel[i].roll_acc_polarity = values[10];
		Config.Channel[i].pitch_acc = values[11];
		Config.Channel[i].pitch_acc_polarity = values[12];

		// Re-span travel limits and failsafe to system units (2500 to 5000)
		Config.Channel[i].min_travel = ((values[13] * 12) + 3750);
		Config.Channel[i].max_travel = ((values[14] * 12) + 3750);
		Config.Channel[i].Failsafe = ((values[15] * 12) + 3750);

		if (button == ENTER)
		{
			Save_Config_to_EEPROM(); // Save value and return
		}
	}
	menu_beep(1);
	_delay_ms(200);
}




