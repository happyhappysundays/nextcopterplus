//***********************************************************
//* menu_general.c
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
#include "..\inc\imu.h"

//************************************************************
// Prototypes
//************************************************************

// Menu items
void menu_general(void);

//************************************************************
// Defines
//************************************************************

#define GENERALITEMS 12 	// Number of menu items
#define GENERALSTART 213 	// Start of Menu text items
#define GENERALTEXT	 22 	// Start of value text items
#define GENOFFSET	 78		// Value offsets

//************************************************************
// RC menu items
//************************************************************

const uint8_t GeneralMenuText[GENERALITEMS] PROGMEM = {GENERALTEXT, 124, 101, 0, 0, 101, 119, 0, 0, 48, 0, 101};
const menu_range_t general_menu_ranges[] PROGMEM = 
{
	{AEROPLANE,CAMSTAB,1,1,AEROPLANE}, 		// Min, Max, Increment, Style, Default
	{HORIZONTAL,UPSIDEDOWN,1,1,HORIZONTAL},
	{28,50,1,0,38}, 	// Contrast
	{1,30,1,0,5},		// Status menu timeout
	{0,30,1,0,1},		// LMA enable
	{OFF,ON,1,1,OFF},	// Camstab enable
	{LOW,HIGH,1,1,LOW},	// Camstab servo rate
	{1,64,1,0,8},		// Acc. LPF
	{10,100,5,0,30},	// CF factor
	{STD,FIXED,1,1,STD},// HH mode
	{0,5,1,0,4},		// 3D rate (0 is fastest, 5 slowest)
	{OFF,ON,1,1,OFF}	// Advanced IMUType
};

//************************************************************
// Main menu-specific setup
//************************************************************

void menu_general(void)
{
	static	uint8_t gen_top = GENERALSTART;
	int8_t values[GENERALITEMS];
	menu_range_t range;
	uint8_t text_link = 0;

	uint8_t temp_type = 0;

	while(button != BACK)
	{
		// Load values from eeprom
		memcpy(&values[0],&Config.MixMode,sizeof(int8_t) * GENERALITEMS);

		// Save pre-edited values
		temp_type = Config.MixMode;

		// Print menu
		print_menu_items(gen_top, GENERALSTART, &values[0], GENERALITEMS, (prog_uchar*)general_menu_ranges, GENOFFSET, (prog_uchar*)GeneralMenuText, cursor);

		// Handle menu changes
		update_menu(GENERALITEMS, GENERALSTART, 0, button, &cursor, &gen_top, &menu_temp);
		range = get_menu_range ((prog_uchar*)general_menu_ranges, menu_temp - GENERALSTART);

		if (button == ENTER)
		{
			text_link = pgm_read_byte(&GeneralMenuText[menu_temp - GENERALSTART]);
			values[menu_temp - GENERALSTART] = do_menu_item(menu_temp, values[menu_temp - GENERALSTART], range, 0, text_link);
		}

		// Update value in config structure
		memcpy(&Config.MixMode,&values[0],sizeof(int8_t) * GENERALITEMS);

		if (button == ENTER)
		{
			// If model type has changed, reload preset
			if (temp_type != values[0]) 
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
			Save_Config_to_EEPROM(); // Save value and return
		}
	}
}




