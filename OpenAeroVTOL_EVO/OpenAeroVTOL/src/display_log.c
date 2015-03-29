//***********************************************************
//* display_log.c
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include <avr/io.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "io_cfg.h"
#include "glcd_driver.h"
#include "mugui.h"
#include <avr/pgmspace.h>
#include "glcd_menu.h"
#include "main.h"
#include "isr.h"
#include <util/delay.h>
#include "acc.h"
#include "gyros.h"
#include "rc.h"
#include "menu_ext.h"
#include "mixer.h"
#include "eeprom.h"

//************************************************************
// Prototypes
//************************************************************

void menu_log(void);
void add_log(uint8_t error);
// enum Errors			{REBOOT = 0, MANUAL, NOSIGNAL, TIMER};

//************************************************************
// Code
//************************************************************

void add_log(uint8_t error)
{
	Config.Log[Config.log_pointer] = error;
	Config.log_pointer++;
	
	if (Config.log_pointer >= (LOGLENGTH-1))
	{
		Config.log_pointer = (LOGLENGTH-1);
	}
	
	// Save log and log pointer
	Save_Config_to_EEPROM();
}

void menu_log(void)
{
	int8_t	log_start = 0;
	
	while(BUTTON1 != 0)
	{
		if (BUTTON4 == 0)
		{
			// Erase log
			memset(&Config.Log[0],0,LOGLENGTH);
			Config.log_pointer = 0;

			// Save log and log pointer
			Save_Config_to_EEPROM();			
		}
		
		if (BUTTON2 == 0)
		{
			log_start--;
			
			if (log_start < 0)
			{
				log_start = 0;
			}
		}

		if (BUTTON3 == 0)
		{
			log_start++;
			
			if (log_start >= (LOGLENGTH - 5))
			{
				log_start = 15;
			}
		}

		// Print each line
		for (uint8_t i = 0; i < 5; i++)
		{
			LCD_Display_Text(283 + Config.Log[log_start + i],(const unsigned char*)Verdana8,0,(i * 10)); // Throttle
		}

		print_menu_frame(LOG);
		clear_buffer(buffer);
		
		_delay_ms(100);
	}
}

