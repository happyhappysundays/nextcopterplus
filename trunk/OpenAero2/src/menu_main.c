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
#include "..\inc\io_cfg.h"
#include "..\inc\init.h"
#include "..\inc\mugui.h"
#include "..\inc\glcd_menu.h"
#include "..\inc\menu_ext.h"
#include "..\inc\glcd_driver.h"
#include "..\inc\main.h"

//************************************************************
// Prototypes
//************************************************************

// Menu items
void menu_main(void);
void do_main_menu_item(uint8_t menuitem);

//************************************************************
// Defines
//************************************************************

#define MAINITEMS 17	// Number of menu items
#define MAINSTART 77	// Start of Menu text items

//************************************************************
// Main menu-specific setup
//************************************************************

void menu_main(void)
{
	static uint8_t cursor = LINE0;	// These are now static so as to remember the menu position
	static uint8_t top = MAINSTART;
	static uint8_t temp = 0;
	
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
		print_menu_frame(0);											// Frame
		LCD_Display_Text(top,(prog_uchar*)Verdana8,ITEMOFFSET,LINE0);	// First line
		LCD_Display_Text(top+1,(prog_uchar*)Verdana8,ITEMOFFSET,LINE1); // Second line
		LCD_Display_Text(top+2,(prog_uchar*)Verdana8,ITEMOFFSET,LINE2); // Third line
		LCD_Display_Text(top+3,(prog_uchar*)Verdana8,ITEMOFFSET,LINE3);	// Fourth line
		print_cursor(cursor);											// Cursor
		write_buffer(buffer,1);

		// Poll buttons when idle
		poll_buttons();

		// Handle menu changes
		update_menu(MAINITEMS, MAINSTART, button, &cursor, &top, &temp);
		if (button == ENTER)
		{
			do_main_menu_item(temp);
			button = NONE;
		}
	}
	menu_beep(1);
	_delay_ms(200);
}

void do_main_menu_item(uint8_t menuitem)
{
	switch(menuitem) 
	{
		case MAINSTART:
			menu_general();	
			break;
		case MAINSTART+1:
			menu_rc_setup();	
			break;
		case MAINSTART+2:
			menu_stab_control();
			break;
		case MAINSTART+3:
			menu_al_control();
			break;
		case MAINSTART+4:
			menu_expo();
			break;
		case MAINSTART+5:
			menu_battery();
			break;
		case MAINSTART+6:
			Display_rcinput();
			break;
		case MAINSTART+7:
			Display_sensors();
			break;
		case MAINSTART+8:
			Display_balance();
			break;
		case MAINSTART+9:
			menu_mixer(0);
			break;
		case MAINSTART+10:
			menu_mixer(1);
			break;
		case MAINSTART+11:
			menu_mixer(2);
			break;
		case MAINSTART+12:
			menu_mixer(3);
			break;
		case MAINSTART+13:
			menu_mixer(4);
			break;
		case MAINSTART+14:
			menu_mixer(5);
			break;
		case MAINSTART+15:
			menu_mixer(6);
			break;
		case MAINSTART+16:
			menu_mixer(7);
			break;
		default:
			break;
	} // Switch
	menu_beep(1);
	_delay_ms(200);
}

