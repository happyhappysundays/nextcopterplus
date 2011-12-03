//***********************************************************
//* lcd.c
//*
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include <avr/io.h>
#include <stdbool.h>
#include <util/delay.h>
#include "..\inc\io_cfg.h"
#include "..\inc\pots.h"
#include "..\inc\adc.h"
#include "..\inc\init.h"
#include "..\inc\eeprom.h"
#include "..\inc\acc.h"

//************************************************************
// Prototypes
//************************************************************

void LCDprint(uint8_t i);
void LCDgoTo(uint8_t position);
void LCDprint_line1(const char *s);
void LCDprint_line2(const char *s);
void LCDclear(void);
void LCDclearLine(uint8_t line);
void LCDprintstr(const char *s);
void set_menu_item(uint8_t menuitem, int16_t value);
int16_t get_menu_item(uint8_t menuitem);
void LCD_Display_Menu (uint8_t menuitem);
menu_range_t get_menu_range (uint8_t menuitem);

//************************************************************
// LCD menu system
//************************************************************
//#define MENUITEMS 19
const char *lcd_menu[MENUITEMS] = 
{
	"NeXtcopter Menu ",
	"Acc Trim (Roll) ", 
	"Acc Trim (Pitch)",
	"PID Roll P-term ",
	"PID Roll I-term ",
	"PID Roll D-term ",
	"PID Pitch P-term",
	"PID Pitch I-term",
	"PID Pitch D-term",
	"PID Yaw P-term  ",
	"PID Yaw I-term  ",
	"PID Yaw D-term  ",
	"PID Level P-term",
	"PID Level I-term",
	"Roll/Pitch rate ",
	"Yaw rate        ",
	"LVA voltage     ",
	"LVA mode        ",
	"Calibrate Accel."
};

const menu_range_t menu_ranges[MENUITEMS] = 
{
	{0,0},
	{-50,50},
	{-50,50},
	{0,255},
	{0,255},
	{0,255},
	{0,255},
	{0,255},
	{0,255},
	{0,255},
	{0,255},
	{0,255},
	{0,255},
	{0,255},
	{0,4},
	{0,4},
	{700,2000},
	{0,1},
	{0,0}
};

void LCD_Display_Menu (uint8_t menuitem)
{
	LCDprint_line1(lcd_menu[menuitem]);
}

menu_range_t get_menu_range (uint8_t menuitem)
{
	return (menu_ranges[menuitem]);
}

int16_t get_menu_item(uint8_t menuitem)
{
	int16_t value = 0;
	switch(menuitem) 
	{
		case 0:	
			break;
		case 1:
			value = (int16_t) (Config.AccRollZeroTrim - 127);
			break;
		case 2:
			value = (int16_t) (Config.AccPitchZeroTrim - 127);
			break;
		case 3:
			value = (int16_t) Config.P_mult_roll;
			break;
		case 4:
			value = (int16_t) Config.I_mult_roll;
			break;
		case 5:
			value = (int16_t) Config.D_mult_roll;
			break;
		case 6:
			value = (int16_t) Config.P_mult_pitch;
			break;
		case 7:
			value = (int16_t) Config.I_mult_pitch;
			break;
		case 8:
			value = (int16_t) Config.D_mult_pitch;
			break;
		case 9:
			value = (int16_t) Config.P_mult_yaw;
			break;
		case 10:
			value = (int16_t) Config.I_mult_yaw;
			break;
		case 11:
			value = (int16_t) Config.D_mult_yaw;
			break;
		case 12:
			value = (int16_t) Config.P_mult_level;
			break;
		case 13:
			value = (int16_t) Config.I_mult_level;
			break;
		case 14:
			value = (int16_t) Config.RollPitchRate;
			break;
		case 15:
			value = (int16_t) Config.Yawrate;
			break;
		case 16:
			value = (int16_t) Config.PowerTrigger;
			break;
		case 17:
			if ((Config.Modes & 0x10) > 0) value = 1;
			else value = 0;
			break;
		case 18:
			break;
		default:
			break;
	} // Switch
	return(value);
}

void set_menu_item(uint8_t menuitem, int16_t value)
{
	switch(menuitem) 
	{
		case 0:	
			break;
		case 1:
			Config.AccRollZeroTrim = (uint8_t) (value + 127);
			break;
		case 2:
			Config.AccPitchZeroTrim = (uint8_t) (value + 127);
			break;
		case 3:
			Config.P_mult_roll = (uint8_t) value;
			break;
		case 4:
			Config.I_mult_roll = (uint8_t) value;
			break;
		case 5:
			Config.D_mult_roll = (uint8_t) value;
			break;
		case 6:
			Config.P_mult_pitch = (uint8_t) value;
			break;
		case 7:
			Config.I_mult_pitch = (uint8_t) value;
			break;
		case 8:
			Config.D_mult_pitch = (uint8_t) value;
			break;
		case 9:
			Config.P_mult_yaw = (uint8_t) value;
			break;
		case 10:
			Config.I_mult_yaw = (uint8_t) value;
			break;
		case 11:
			Config.D_mult_yaw = (uint8_t) value;
			break;
		case 12:
			Config.P_mult_level = (uint8_t) value;
			break;
		case 13:
			Config.I_mult_level = (uint8_t) value;
			break;
		case 14:
			Config.RollPitchRate = (uint8_t) value;
			break;
		case 15:
			Config.Yawrate = (uint8_t) value;
			break;
		case 16:
			Config.PowerTrigger = (uint16_t) value;
			break;
		case 17:
			// 1 = Autolevel, 2 = Normal, 4 = Acro, 8 = UFO, bit 4 (16) = LVA mode 1 = buzzer, 0 = LED
			if (value == 0) {
				Config.Modes = Config.Modes & 0xEF;				// LVA mode = buzzer (bit 4)
			}
			else {
				Config.Modes = Config.Modes | 0x10;
			}
			break;
		case 18:
			CalibrateAcc();
			break;
		default:
			break;
	} // Switch

	// Save to eeProm	
	Save_Config_to_EEPROM(); 
	LED = !LED;
	_delay_ms(500);
	LED = !LED;
}





//************************************************************
// LCD code snippets from Mike Barton.
// Borrowed from Arduino SoftUsart and the Sparkfun datasheet
//************************************************************

// 1000000 / 9600  = 104 microseconds at 9600 baud.
// This won't work for really bad KK boards.
// Best to get this value from the UART autotune value in the future.

#define LCD_BIT_DELAY 102 	// Allow for interrupts

void LCDprint(uint8_t i)
{
	uint8_t mask;
	LCD_TX = 0;
	_delay_us(LCD_BIT_DELAY);

	for (mask = 0x01; mask; mask <<= 1) 
	{
		if (i & mask) LCD_TX = 1; 
		else LCD_TX = 0;
		_delay_us(LCD_BIT_DELAY);
	}
	LCD_TX = 1;
	_delay_us(LCD_BIT_DELAY);
}

// Position = line 1: 0-15, line 2: 16-31, 31+ defaults back to 0
void LCDgoTo(uint8_t position) 
{
	if (position<16)
	{
		LCDprint(0xFE);
		LCDprint(position+0x80);
	}
	else if (position<32)
	{
		LCDprint(0xFE);
		LCDprint(position+0x80+0x30);
	}
	else 
	{
		LCDgoTo(0);
	}
}

void LCDprint_line1(const char *s)
{
	LCDclearLine(1);
	LCDgoTo(0);

	while (*s) LCDprint(*s++);
}

void LCDprint_line2(const char *s)
{
	LCDclearLine(2);
	LCDgoTo(16);

	while (*s) LCDprint(*s++);
}

void LCDclear(void)
{
	LCDprint(0xFE);
	LCDprint(0x01);
}

void LCDclearLine(uint8_t line)
{
	int8_t j;
	LCDprint(0xFE);
	LCDprint((line<<4)|0x80);
	for(j=0;j<16;j++)
	{
		LCDprint(0x20);
	}
}

void LCDprintstr(const char *s)
{
	while (*s) LCDprint(*s++);
}
