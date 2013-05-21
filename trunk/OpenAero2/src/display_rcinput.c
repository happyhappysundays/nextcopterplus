//***********************************************************
//* display_rcinput.c
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include <avr/io.h>
#include <stdlib.h>
#include "..\inc\io_cfg.h"
#include "..\inc\glcd_driver.h"
#include "..\inc\mugui.h"
#include <avr/pgmspace.h>
#include "..\inc\glcd_menu.h"
#include "..\inc\main.h"
#include "..\inc\isr.h"
#include <util/delay.h>
#include "..\inc\acc.h"
#include "..\inc\gyros.h"
#include "..\inc\rc.h"
#include "..\inc\menu_ext.h"
#include "..\inc\mixer.h"
#include "..\inc\eeprom.h"

//************************************************************
// Prototypes
//************************************************************

void Display_rcinput(void);

//************************************************************
// Code
//************************************************************

void Display_rcinput(void)
{
	while(BUTTON1 != 0)
	{
		if (BUTTON4 == 0)
		{
			//_delay_ms(100);
			CenterSticks();
		}

		if (BUTTON3 == 0)
		{
			//_delay_ms(100);
			SetFailsafe();
		}

		RxGetChannels();

		LCD_Display_Text(241,(prog_uchar*)Verdana8,0,0);
		LCD_Display_Text(32,(prog_uchar*)Verdana8,0,10);
		LCD_Display_Text(242,(prog_uchar*)Verdana8,0,20);
		LCD_Display_Text(35,(prog_uchar*)Verdana8,0,30);

		LCD_Display_Text(109,(prog_uchar*)Verdana8,70,0);
		LCD_Display_Text(110,(prog_uchar*)Verdana8,70,10);
		LCD_Display_Text(111,(prog_uchar*)Verdana8,70,20);
		LCD_Display_Text(112,(prog_uchar*)Verdana8,70,30);

		mugui_lcd_puts(itoa(RCinputs[THROTTLE],pBuffer,10),(prog_uchar*)Verdana8,37,0);
		mugui_lcd_puts(itoa(RCinputs[AILERON],pBuffer,10),(prog_uchar*)Verdana8,37,10);
		mugui_lcd_puts(itoa(RCinputs[ELEVATOR],pBuffer,10),(prog_uchar*)Verdana8,37,20);
		mugui_lcd_puts(itoa(RCinputs[RUDDER],pBuffer,10),(prog_uchar*)Verdana8,37,30);

		mugui_lcd_puts(itoa(RCinputs[GEAR],pBuffer,10),(prog_uchar*)Verdana8,100,0);
		mugui_lcd_puts(itoa(RCinputs[AUX1],pBuffer,10),(prog_uchar*)Verdana8,100,10);
		mugui_lcd_puts(itoa(RCinputs[AUX2],pBuffer,10),(prog_uchar*)Verdana8,100,20);
		mugui_lcd_puts(itoa(RCinputs[AUX3],pBuffer,10),(prog_uchar*)Verdana8,100,30);


		// Print bottom text and markers
		LCD_Display_Text(12, (prog_uchar*)Wingdings, 0, 57); 	// Left
		LCD_Display_Text(44, (prog_uchar*)Verdana8, 40, 55); 	// Failsafe
		LCD_Display_Text(9, (prog_uchar*)Wingdings, 80, 59); 	// Down
		LCD_Display_Text(60, (prog_uchar*)Verdana8, 100, 55); 	// Cal.
		LCD_Display_Text(9, (prog_uchar*)Wingdings, 119, 59); 	// Down

		// Update buffer
		write_buffer(buffer);
		clear_buffer(buffer);
		_delay_ms(100);
	}
	// Exit
}
