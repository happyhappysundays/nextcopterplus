//***********************************************************
//* menu_main.c
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include <avr/pgmspace.h> 
#include <avr/io.h>
#include <stdbool.h>
#include <util/delay.h>
#include "io_cfg.h"
#include "init.h"
#include "mugui.h"
#include "glcd_menu.h"
#include "menu_ext.h"
#include "glcd_driver.h"
#include "main.h"

//************************************************************
// Prototypes
//************************************************************

// Menu items
void menu_main(void);
void do_main_menu_item(uint8_t menuitem);

//************************************************************
// Defines
//************************************************************

#define MAINITEMS 19	// Number of menu items
#define MAINSTART 77	// Start of Menu text items

//************************************************************
// Main menu-specific setup
//************************************************************

uint8_t menu_flag = 1;	// Preset so that the first use refreshes the submenus to the correct position

void menu_main(void)
{
	static uint8_t main_cursor = LINE0;	// These are now static so as to remember the main menu position
	static uint8_t main_top = MAINSTART;
	static uint8_t main_temp = 0;
	static uint8_t old_menu = 0;

	button = NONE;

	// Wait until user's finger is off button 1
	while(BUTTON1 == 0)
	{
		_delay_ms(50);
	}

	while(button != BACK)
	{
		// Clear buffer before each update
		clear_buffer(buffer);	

		// Print menu
		print_menu_frame(0);													// Frame
		
		for (uint8_t i = 0; i < 4; i++)
		{
			LCD_Display_Text(main_top+i,(const unsigned char*)Verdana8,ITEMOFFSET,(uint8_t)pgm_read_byte(&lines[i]));	// Lines
		}

		print_cursor(main_cursor);												// Cursor
		write_buffer(buffer,1);

		// Poll buttons when idle
		poll_buttons(true);

		// Handle menu changes
		update_menu(MAINITEMS, MAINSTART, 0, button, &main_cursor, &main_top, &main_temp);

		// If main menu item has changed, reset submenu positions
		// and flag to submenus that positions need to be reset
		if (main_temp != old_menu)
		{
			cursor = LINE0;
			menu_temp = 0;
			old_menu = main_temp;
			menu_flag = 1;
		}

		// If ENTER pressed, jump to menu 
		if (button == ENTER)
		{
			do_main_menu_item(main_temp);
			button = NONE;

			// Wait until user's finger is off button 1
			while(BUTTON1 == 0)
			{
				_delay_ms(50);
			}
		}
	}

//	menu_beep(1);
}

void do_main_menu_item(uint8_t menuitem)
{
	switch(menuitem) 
	{
		case MAINSTART:
			menu_rc_setup(2); 		// 1.General
			break;
		case MAINSTART+1:
			menu_rc_setup(1); 		// 2.RX setup	
			break;
		case MAINSTART+2:
			Display_rcinput();		// 3.RX inputs
			break;
		case MAINSTART+3:
			Display_sticks(); 		// 4.Stick polarity	
			break;
		case MAINSTART+4:
			Display_sensors();		// 5.Sensor calibration
			break;
		case MAINSTART+5:
			Display_balance();		// 6.Level meter
			break;
		case MAINSTART+6:
			menu_flight(0);			// 7.Flight profile 1
			break;
		case MAINSTART+7:
			menu_flight(1); 		// 8.Flight profile 2
			break;
		case MAINSTART+8:
			menu_mixer(0);			// 9.OUT1 Mixer
			break;
		case MAINSTART+9:
			menu_mixer(1);			// 10.OUT2 Mixer
			break;
		case MAINSTART+10:
			menu_mixer(2);			// 11.OUT3 Mixer
			break;
		case MAINSTART+11:
			menu_mixer(3);			// 12.OUT4 Mixer
			break;
		case MAINSTART+12:
			menu_mixer(4);			// 13.OUT5 Mixer
			break;
		case MAINSTART+13:
			menu_mixer(5);			// 14.OUT6 Mixer
			break;
		case MAINSTART+14:
			menu_mixer(6);			// 15.OUT7 Mixer
			break;
		case MAINSTART+15:
			menu_mixer(7);			// 16.OUT8 Mixer
			break;
		case MAINSTART+16:
			menu_servo_setup(1);	// 17.Servo direction
			break;
		case MAINSTART+17:
			menu_servo_setup(2); 	// 18.Neg. Servo trvl. (%)
			break;
		case MAINSTART+18:
			menu_servo_setup(3); 	// 19.Pos. Servo trvl. (%)
			break;
		default:
			break;
	} // Switch
}

