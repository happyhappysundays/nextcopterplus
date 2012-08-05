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

#define MIXERITEMS 11 	// Number of menu items
#define MIXERSTART 130 	// Start of Menu text items

//************************************************************
// RC menu items
//************************************************************
	 
const uint8_t MixerMenuOffsets[MIXERITEMS] PROGMEM = {80,80,80,80,80,80,80,80,80,80,80};
const uint8_t MixerMenuText[MIXERITEMS] PROGMEM = {149,141,0,143,143,143,159,141,0,0,0};
const menu_range_t mixer_menu_ranges[] = 
{
	{CH1,CH8,1,1}, 				// Source
	{NORMAL,REVERSED,1,1}, 	// source_polarity
	{0,1,1,0},					// source_volume
	{G_OFF, G_REVERSED,1,1},	// roll_gyro
	{G_OFF, G_REVERSED,1,1},	// pitch_gyro
	{G_OFF, G_REVERSED,1,1},	// yaw_gyro
	{X,NO_ACC,1,1},			// acc source
	{NORMAL,REVERSED,1,1}, 	// acc polarity
	{2250,5250,10,0}, 			// Min travel
	{2250,5250,10,0}, 			// Max travel
	{2250,5250,10,0}, 			// failsafe
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

		if (Config.Channel[i].roll_gyro_polarity == REVERSED) values[3] = G_REVERSED;
		else values[3] = (uint8_t)Config.Channel[i].roll_gyro;

		if (Config.Channel[i].pitch_gyro_polarity == REVERSED) values[4] = G_REVERSED;
		else values[4] = (uint8_t)Config.Channel[i].pitch_gyro;

		if (Config.Channel[i].yaw_gyro_polarity == REVERSED)	values[5] = G_REVERSED;
		else values[5] = (uint8_t)Config.Channel[i].yaw_gyro;

		values[6] = (uint8_t)Config.Channel[i].acc;
		values[7] = Config.Channel[i].acc_polarity;
		values[8] = Config.Channel[i].min_travel;
		values[9] = Config.Channel[i].max_travel;
		values[10] = Config.Channel[i].Failsafe;

		// Print menu
		print_menu_items(top, MIXERSTART, &values[0], &mixer_menu_ranges[0], (prog_uchar*)MixerMenuOffsets, (prog_uchar*)MixerMenuText, cursor);
		

		// Poll buttons when idle
		button = poll_buttons();
		while (button == NONE)					
		{
			button = poll_buttons();
		}

		// Handle menu changes
		update_menu(MIXERITEMS, MIXERSTART, button, &cursor, &top, &temp);
		range = mixer_menu_ranges[temp - MIXERSTART];

		if (button == ENTER)
		{
			text_link = pgm_read_byte(&MixerMenuText[temp - MIXERSTART]);
			values[temp - MIXERSTART] = do_menu_item(temp, values[temp - MIXERSTART], range, 0, text_link);
		}

		// Update value in config structure
		Config.Channel[i].source = values[0];
		Config.Channel[i].source_polarity = values[1];
		Config.Channel[i].source_volume = values[2];
	
		switch (values[3])
		{
			case G_OFF:
				Config.Channel[i].roll_gyro = OFF;
				break;
			case G_ON:
				Config.Channel[i].roll_gyro = ON;
				Config.Channel[i].roll_gyro_polarity = NORMAL;
				break;
			case G_REVERSED:
				Config.Channel[i].roll_gyro = ON;
				Config.Channel[i].roll_gyro_polarity = REVERSED;
				break;
			default:			
				break;	
		}
		switch (values[4])
		{
			case G_OFF:
				Config.Channel[i].pitch_gyro = OFF;
				break;
			case G_ON:
				Config.Channel[i].pitch_gyro = ON;
				Config.Channel[i].pitch_gyro_polarity = NORMAL;
				break;
			case G_REVERSED:
				Config.Channel[i].pitch_gyro = ON;
				Config.Channel[i].pitch_gyro_polarity = REVERSED;
				break;
			default:					
				break;	
		}
		switch (values[5])
		{
			case G_OFF:
				Config.Channel[i].yaw_gyro = OFF;
				break;
			case G_ON:
				Config.Channel[i].yaw_gyro = ON;
				Config.Channel[i].yaw_gyro_polarity = NORMAL;
				break;
			case G_REVERSED:
				Config.Channel[i].yaw_gyro = ON;
				Config.Channel[i].yaw_gyro_polarity = REVERSED;
				break;
			default:				
				break;	
		}

		Config.Channel[i].acc = values[6];
		Config.Channel[i].acc_polarity = values[7];
		Config.Channel[i].min_travel = values[8];
		Config.Channel[i].max_travel = values[9];
		Config.Channel[i].Failsafe = values[10];

		if (button == ENTER)
		{
			Save_Config_to_EEPROM(); // Save value and return
		}
	}
	menu_beep();
	_delay_ms(200);
}




