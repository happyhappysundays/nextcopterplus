//***********************************************************
//* menu_rcinput.c
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
			_delay_ms(100);
			CenterSticks();
		}

		if (BUTTON3 == 0)
		{
			_delay_ms(100);
			SetFailsafe();
		}

		RxGetChannels();

		LCD_Display_Text(32,(prog_uchar*)Verdana8,0,0);
		LCD_Display_Text(33,(prog_uchar*)Verdana8,0,10);
		LCD_Display_Text(34,(prog_uchar*)Verdana8,0,20);
		LCD_Display_Text(35,(prog_uchar*)Verdana8,0,30);
		LCD_Display_Text(36,(prog_uchar*)Verdana8,0,40);


		mugui_lcd_puts(itoa(RxChannel[AILERON],pBuffer,10),(prog_uchar*)Verdana8,60,0);
		mugui_lcd_puts(itoa(RxChannel[ELEVATOR],pBuffer,10),(prog_uchar*)Verdana8,60,10);
		mugui_lcd_puts(itoa(RxChannel[THROTTLE],pBuffer,10),(prog_uchar*)Verdana8,60,20);
		mugui_lcd_puts(itoa(RxChannel[RUDDER],pBuffer,10),(prog_uchar*)Verdana8,60,30);
		mugui_lcd_puts(itoa(RxChannel[GEAR],pBuffer,10),(prog_uchar*)Verdana8,60,40);

		mugui_lcd_puts(itoa(RCinputs[AILERON],pBuffer,10),(prog_uchar*)Verdana8,90,0);
		mugui_lcd_puts(itoa(RCinputs[ELEVATOR],pBuffer,10),(prog_uchar*)Verdana8,90,10);
		mugui_lcd_puts(itoa(RCinputs[THROTTLE],pBuffer,10),(prog_uchar*)Verdana8,90,20);
		mugui_lcd_puts(itoa(RCinputs[RUDDER],pBuffer,10),(prog_uchar*)Verdana8,90,30);
		mugui_lcd_puts(itoa(RCinputs[GEAR],pBuffer,10),(prog_uchar*)Verdana8,90,40);

		// Print bottom markers
		LCD_Display_Text(12, (prog_uchar*)Wingdings, 0, 57); 	// Left
		LCD_Display_Text(193, (prog_uchar*)Verdana8, 40, 55); 	// Failsafe
		fillrect(buffer, 77,54, 22, 10, 0);						// Chop off (:) to save memory :)
		LCD_Display_Text(9, (prog_uchar*)Wingdings, 80, 59); 	// Down
		LCD_Display_Text(60, (prog_uchar*)Verdana8, 100, 55); 	// Cal.
		LCD_Display_Text(9, (prog_uchar*)Wingdings, 119, 59); 	// Down

		// Update buffer
		write_buffer(buffer,1);
		clear_buffer(buffer);
		_delay_ms(100);
	}
}
