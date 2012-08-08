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
const uint8_t MixerMenuText[MIXERITEMS] PROGMEM = {149,141,0,143,143,143,143,143,0,0,0};
const menu_range_t mixer_menu_ranges[] PROGMEM = 
{
	{THROTTLE,PRESET4,1,1,CH1}, 	// Source
	{NORMAL,REVERSED,1,1,NORMAL}, 	// source_polarity
	{0,125,10,0,100},				// source_volume (%)
	{G_OFF, G_REVERSED,1,1,G_OFF},	// roll_gyro
	{G_OFF, G_REVERSED,1,1,G_OFF},	// pitch_gyro
	{G_OFF, G_REVERSED,1,1,G_OFF},	// yaw_gyro
	{G_OFF, G_REVERSED,1,1,G_OFF},	// roll_acc
	{G_OFF, G_REVERSED,1,1,G_OFF},	// pitch_acc
	{900,2100,10,0,900}, 			// Min travel
	{900,2100,10,0,2100}, 			// Max travel
	{900,2100,10,0,1500}, 			// failsafe
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
	int16_t temp16 = 0;

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

		if (Config.Channel[i].yaw_gyro_polarity == REVERSED) values[5] = G_REVERSED;
		else values[5] = (uint8_t)Config.Channel[i].yaw_gyro;

		if (Config.Channel[i].roll_acc_polarity == REVERSED) values[6] = G_REVERSED;
		else values[6] = (uint8_t)Config.Channel[i].roll_acc;

		if (Config.Channel[i].pitch_acc_polarity == REVERSED) values[7] = G_REVERSED;
		else values[7] = (uint8_t)Config.Channel[i].pitch_acc;

		// Re-span travel limits and failsafe to 40%
		temp16 = ((Config.Channel[i].min_travel << 2) / 10); 
		values[8] = temp16;
		temp16 = ((Config.Channel[i].max_travel << 2) / 10); 
		values[9] = temp16;
		temp16 = ((Config.Channel[i].Failsafe << 2) / 10); 
		values[10] = temp16;

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
		switch (values[6])
		{
			case G_OFF:
				Config.Channel[i].roll_acc = OFF;
				break;
			case G_ON:
				Config.Channel[i].roll_acc = ON;
				Config.Channel[i].roll_acc_polarity = NORMAL;
				break;
			case G_REVERSED:
				Config.Channel[i].roll_acc = ON;
				Config.Channel[i].roll_acc_polarity = REVERSED;
				break;
			default:				
				break;	
		}
		switch (values[7])
		{
			case G_OFF:
				Config.Channel[i].pitch_acc = OFF;
				break;
			case G_ON:
				Config.Channel[i].pitch_acc = ON;
				Config.Channel[i].pitch_acc_polarity = NORMAL;
				break;
			case G_REVERSED:
				Config.Channel[i].pitch_acc = ON;
				Config.Channel[i].pitch_acc_polarity = REVERSED;
				break;
			default:				
				break;	
		}

		// Re-span travel limits and failsafe to 40%
		temp16 = ((values[8] * 5) >> 1); 
		Config.Channel[i].min_travel = temp16;
		temp16 = ((values[9] * 5) >> 1);
		Config.Channel[i].max_travel = temp16;
		temp16 = ((values[10] * 5) >> 1);
		Config.Channel[i].Failsafe = temp16;

		if (button == ENTER)
		{
			Save_Config_to_EEPROM(); // Save value and return
		}
	}
	menu_beep(1);
	_delay_ms(200);
}




