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
#include <avr/pgmspace.h> 

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
void LCDprint_Menu_P(const char *s);

//************************************************************
// LCD menu system
//************************************************************

const char MenuItem1[] PROGMEM = "NeXtcopter Menu ";
const char MenuItem2[] PROGMEM = "Acc Trim (Roll) "; 
const char MenuItem3[] PROGMEM = "Acc Trim (Pitch)";
const char MenuItem4[] PROGMEM = "PID Roll P-term ";
const char MenuItem5[] PROGMEM = "PID Roll I-term ";
const char MenuItem6[] PROGMEM = "PID Roll D-term ";
const char MenuItem7[] PROGMEM = "PID Pitch P-term";
const char MenuItem8[] PROGMEM = "PID Pitch I-term";
const char MenuItem9[] PROGMEM = "PID Pitch D-term";
const char MenuItem10[] PROGMEM = "PID Yaw P-term  ";
const char MenuItem11[] PROGMEM = "PID Yaw I-term  ";
const char MenuItem12[] PROGMEM = "PID Yaw D-term  ";
const char MenuItem13[] PROGMEM = "Gyr level P-term";
const char MenuItem14[] PROGMEM = "Gyr level I-term";
const char MenuItem15[] PROGMEM = "Acc level P-term";
const char MenuItem16[] PROGMEM = "Acc level I-term";
const char MenuItem17[] PROGMEM = "Roll/Pitch rate ";
const char MenuItem18[] PROGMEM = "Yaw rate        ";
const char MenuItem19[] PROGMEM = "LVA voltage     ";
const char MenuItem20[] PROGMEM = "LVA mode        ";
const char MenuItem21[] PROGMEM = "Calibrate Accel.";
const char MenuItem22[] PROGMEM = "Center sticks   ";

const char *lcd_menu[MENUITEMS] PROGMEM = 
	{MenuItem1, MenuItem2, MenuItem3, MenuItem4, MenuItem5, MenuItem6, MenuItem7, MenuItem8,
	 MenuItem9, MenuItem10, MenuItem11, MenuItem12, MenuItem13, MenuItem14, MenuItem15, MenuItem16,
	 MenuItem17, MenuItem18, MenuItem19, MenuItem20, MenuItem21, MenuItem22}; 

const menu_range_t menu_ranges[MENUITEMS] PROGMEM = 
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
	{0,255},
	{0,255},
	{0,5},
	{0,5},
	{700,2000},
	{0,1},
	{0,0},
	{0,0}
};

// Get LCD data from Program memory
void LCD_Display_Menu (uint8_t menuitem)
{
	LCDprint_Menu_P((char*)pgm_read_word(&lcd_menu[menuitem]));
}

// Get range from Program memory
menu_range_t get_menu_range (uint8_t menuitem)
{
	menu_range_t	range;
	range.upper =	pgm_read_word(&menu_ranges[menuitem].upper);
	range.lower =	pgm_read_word(&menu_ranges[menuitem].lower);

	return (range);
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
			value = (int16_t) Config.P_mult_glevel;
			break;
		case 13:
			value = (int16_t) Config.I_mult_glevel;
			break;
		case 14:
			value = (int16_t) Config.P_mult_alevel;
			break;
		case 15:
			value = (int16_t) Config.I_mult_alevel;
			break;
		case 16:
			value = (int16_t) Config.RollPitchRate;
			break;
		case 17:
			value = (int16_t) Config.Yawrate;
			break;
		case 18:
			value = (int16_t) Config.PowerTrigger;
			break;
		case 19:
			if ((Config.Modes & 0x10) > 0) value = 1;
			else value = 0;
			break;
		case 20:
			break;
		case 21:
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
			Config.P_mult_glevel = (uint8_t) value;
			break;
		case 13:
			Config.I_mult_glevel = (uint8_t) value;
			break;
		case 14:
			Config.P_mult_alevel = (uint8_t) value;
			break;
		case 15:
			Config.I_mult_alevel = (uint8_t) value;
			break;
		case 16:
			Config.RollPitchRate = (uint8_t) value;
			break;
		case 17:
			Config.Yawrate = (uint8_t) value;
			break;
		case 18:
			Config.PowerTrigger = (uint16_t) value;
			break;
		case 19:
			// 1 = Autolevel, 2 = Normal, 4 = Acro, 8 = UFO, bit 4 (16) = LVA mode 1 = buzzer, 0 = LED
			if (value == 0) {
				Config.Modes = Config.Modes & 0xEF;				// LVA mode = buzzer (bit 4)
			}
			else {
				Config.Modes = Config.Modes | 0x10;
			}
			break;
		case 20:
			CalibrateAcc();
			break;
		case 21:
			CenterSticks();
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

void LCDprint_Menu_P(const char *s)
{
	LCDclearLine(1);
	LCDgoTo(0);

	while (pgm_read_byte(s) != 0x00) 
		LCDprint(pgm_read_byte(s++)); 
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
