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
void variable_lcd_delay(uint8_t count);
void LCD_fixBL(void);
uint8_t LCD_increment(uint8_t MenuItem);
uint8_t LCD_decrement(uint8_t MenuItem);

//************************************************************
// LCD menu system
//************************************************************

const char MenuItem1[]  PROGMEM = "  OpenAero Menu ";
const char MenuItem2[]  PROGMEM = "Acc Trim (Roll) "; // Only for ACC
const char MenuItem3[]  PROGMEM = "Acc Trim (Pitch)"; // Only for ACC
const char MenuItem4[]  PROGMEM = "PID Roll Gain   ";
const char MenuItem5[]  PROGMEM = "PID Roll D-term ";
const char MenuItem6[]  PROGMEM = "PID Pitch Gain  ";
const char MenuItem7[]  PROGMEM = "PID Pitch D-term";
const char MenuItem8[]  PROGMEM = "PID Yaw Gain    ";
const char MenuItem9[]  PROGMEM = "PID Yaw D-term  ";
const char MenuItem10[] PROGMEM = "Gyr level Gain  "; // Only for ACC
const char MenuItem11[] PROGMEM = "Gyr level D-term"; // Only for ACC
const char MenuItem12[] PROGMEM = "Acc level Gain  "; // Only for ACC
const char MenuItem13[] PROGMEM = "Acc level I-term"; // Only for ACC
const char MenuItem14[] PROGMEM = "LVA voltage     "; // Only for KK Plus
const char MenuItem15[] PROGMEM = "LVA mode        "; // Only for KK Plus
const char MenuItem16[] PROGMEM = "Battery voltage "; // Only for KK Plus
const char MenuItem17[] PROGMEM = "Pot mode        ";
const char MenuItem18[] PROGMEM = "Roll gyro       ";
const char MenuItem19[] PROGMEM = "Pitch gyro      ";
const char MenuItem20[] PROGMEM = "Yaw gyro        ";
const char MenuItem21[] PROGMEM = "Roll servo      ";
const char MenuItem22[] PROGMEM = "Pitch servo     ";
const char MenuItem23[] PROGMEM = "Yaw servo       ";
const char MenuItem24[] PROGMEM = "Calibrate Accel."; // Only for ACC
const char MenuItem25[] PROGMEM = "Center sticks   ";
const char MenuItem26[] PROGMEM = "Stability mode  ";
const char MenuItem27[] PROGMEM = "Autolevel mode  "; // Only for ACC
const char MenuItem28[] PROGMEM = "Stability ch.   "; // Only for CPPM

const char *lcd_menu[MENUITEMS] PROGMEM = 
	{MenuItem1, MenuItem2, MenuItem3, MenuItem4, MenuItem5, MenuItem6, MenuItem7, MenuItem8,
	 MenuItem9, MenuItem10, MenuItem11, MenuItem12, MenuItem13, MenuItem14, MenuItem15, MenuItem16,
	 MenuItem17, MenuItem18, MenuItem19, MenuItem20, MenuItem21, MenuItem22, MenuItem23, MenuItem24, 
	 MenuItem25, MenuItem26, MenuItem27, MenuItem28}; 

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
	{700,2000},	// 7.00 to 20.00 Volts
	{0,1},
	{2000,0},	// This makes the battery voltage display unadjustable
#ifndef ACCELEROMETER
	{0,1},
#else
	{1,1},		// Leave pot mode set to "eeprom"
#endif
	{0,1},
	{0,1},
	{0,1},
	{0,1},
	{0,1},
	{0,1},
	{0,0},
	{0,0},
	{0,1},
	{0,1},
	{1,8}		// Stability channel number
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
			value = (int16_t) Config.P_mult_pitch;
			break;
		case 6:
			value = (int16_t) Config.I_mult_pitch;
			break;
		case 7:
			value = (int16_t) Config.P_mult_yaw;
			break;
		case 8:
			value = (int16_t) Config.I_mult_yaw;
			break;
		case 9:
			value = (int16_t) Config.P_mult_glevel;
			break;
		case 10:
			value = (int16_t) Config.D_mult_glevel;
			break;
		case 11:
			value = (int16_t) Config.P_mult_alevel;
			break;
		case 12:
			value = (int16_t) Config.I_mult_alevel;
			break;
		case 13:
			value = (int16_t) Config.PowerTrigger;
			break;
		case 14:
			if ((Config.Modes & 0x10) > 0) value = 1;
			else value = 0;
			break;
		case 15:
			GetVbat();
			value = (int16_t) vBat;
			break;
		case 16:
			if ((Config.Modes & 4) > 0) value = 1;
			else value = 0;
			break;
		case 17:
			value = (int16_t) Config.RollGyro;
			break;
		case 18:
			value = (int16_t) Config.PitchGyro;
			break;
		case 19:
			value = (int16_t) Config.YawGyro;
			break;
		case 20:
			value = (int16_t) Config.RollServo;
			break;
		case 21:
			value = (int16_t) Config.PitchServo;
			break;
		case 22:
			value = (int16_t) Config.YawServo;
			break;
		case 23:
			break;
		case 24:
			break;
		case 25:
			value = (int16_t) Config.StabMode;
			break;
		case 26:
			value = (int16_t) Config.ALMode;
			break;
		case 27:
			value = (int16_t) Config.StabChan;
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
			Config.P_mult_pitch = (uint8_t) value;
			break;
		case 6:
			Config.I_mult_pitch = (uint8_t) value;
			break;
		case 7:
			Config.P_mult_yaw = (uint8_t) value;
			break;
		case 8:
			Config.I_mult_yaw = (uint8_t) value;
			break;
		case 9:
			Config.P_mult_glevel = (uint8_t) value;
			break;
		case 10:
			Config.D_mult_glevel = (uint8_t) value;
			break;
		case 11:
			Config.P_mult_alevel = (uint8_t) value;
			break;
		case 12:
			Config.I_mult_alevel = (uint8_t) value;
			break;
		case 13:
			Config.PowerTrigger = (uint16_t) value;
			break;
		case 14:
			// 1 = Autolevel, 2 = Unused, 4 = Pot mode, 8 = Stability, 16 = LVA mode 1 = buzzer, 0 = LED
			if (value == 0) {
				Config.Modes = Config.Modes & 0xEF;				// LVA mode = LED (bit 4 = 0)
			}
			else {
				Config.Modes = Config.Modes | 0x10;				// LVA mode = BUZZER (bit 4 = 1)
			}
			break;
		case 15:
			break;
		case 16:
			// 1 = Autolevel, 2 = Unused, 4 = Pot mode (0 = Adjustable, 1 = Unused), 8 = Stability, 16 = LVA mode
			if (value == 0) {
				Config.Modes = Config.Modes & 0xFB;				// Pot mode = Adjustable (bit 2)
			}
			else {
				Config.Modes = Config.Modes | 4;
			}
			break;
		case 17:
			Config.RollGyro = (uint8_t) value;
			break;
		case 18:
			Config.PitchGyro = (uint8_t) value;
			break;
		case 19:
			Config.YawGyro = (uint8_t) value;
			break;
		case 20:
			Config.RollServo = (uint8_t) value;
			break;
		case 21:
			Config.PitchServo = (uint8_t) value;
			break;
		case 22:
			Config.YawServo = (uint8_t) value;
			break;
		case 23:
			CalibrateAcc();
			break;
		case 24:
			CenterSticks();
			break;
		case 25:
			// 0 = Stability switchable, 1 = Always on
			Config.StabMode = (uint8_t) value;
			break;
		case 26:
			// 0 = Autolevel switchable, 1 = Always off
			Config.ALMode = (uint8_t) value;
			break;
		case 27:
			// Stability switch channel number
			Config.StabChan = (uint8_t) value;
			break;
		default:
			break;
	} // Switch

	// Save to eeProm	
	Save_Config_to_EEPROM(); 
	LED1 = !LED1;
	_delay_ms(500);
	LED1 = !LED1;
}

uint8_t LCD_increment(uint8_t MenuItem)
{
	if (MenuItem >= (MENUITEMS-1)) MenuItem = 0;// Wrap back to start	
	MenuItem += 1;

	// Skip menu items that are not supported by the hardware
	#ifndef ACCELEROMETER // Skip Acc-related menu items
	switch (MenuItem)
	{
		case 1:	
		case 2:
			MenuItem = 3;
			break;
		case 9:
		case 10:
		case 11:
		case 12:
			MenuItem = 13;
			break;
		case 23:
			MenuItem = 24;
			break;
		case 26:
		#ifdef ICP_CPPM_MODE
			MenuItem = 27;
		#else
			MenuItem = 3;
		#endif
			break;
		case 28:
			MenuItem = 3;
			break;
		default:
			break;
	}
	#endif

	#ifndef KK_PLUS // Battery LVA items only work on KK Plus boards
	switch (MenuItem)
	{
		case 13:
		case 14:
		case 15:	
			MenuItem = 16;
			break;
		default:
			break;
	}
	#endif

	return MenuItem;
}

uint8_t LCD_decrement(uint8_t MenuItem)
{
	if  (MenuItem <= 1) MenuItem = (MENUITEMS);// Wrap back to end
	MenuItem -= 1;

	#ifndef KK_PLUS // Battery LVA items only work on KK Plus boards
	switch (MenuItem)
	{
		case 13:
		case 14:
		case 15:	
			MenuItem = 12;
			break;
		default:
			break;
	}
	#endif

	// Skip menu items that are not supported by the hardware
	#ifndef ACCELEROMETER // Skip Acc-related menu items
	switch (MenuItem)
	{
		case 1:	
		case 2:
		#ifdef ICP_CPPM_MODE
			MenuItem = 27;
		#else
			MenuItem = 25;
		#endif
			break;
		case 9:
		case 10:
		case 11:
		case 12:
			MenuItem = 8;
			break;
		case 23:
			MenuItem = 22;
			break;
		case 26:
			MenuItem = 25;
			break;
		default:
			break;
	}
	#endif

	return MenuItem;
}



//************************************************************
// LCD code snippets from Mike Barton.
// Borrowed from Arduino SoftUsart and the Sparkfun datasheet
//************************************************************

// LCD timing value from calculated from the UART autotune value.
// Config.AutoTuneTX  -> 51 is the default, *2 = 102

void LCDprint(uint8_t i)
{
	uint8_t mask, delay;
	delay = Config.AutoTuneTX << 1;

	TX = 0;
	variable_lcd_delay(delay);

	for (mask = 0x01; mask; mask <<= 1) 
	{
		if (i & mask) TX = 1; 
		else TX = 0;
		variable_lcd_delay(delay);
	}
	TX = 1;
	variable_lcd_delay(delay);
}

// Position = line 1: 0-15, line 2: 16-31
void LCDgoTo(uint8_t position) 
{
	if (position<16)
	{
		LCDprint(0xFE);
		LCDprint(position+0x80);
	}
	else
	{
		LCDprint(0xFE);
		LCDprint(position+0x80+0x30);
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

void LCD_fixBL(void)
{
	LCDprint(0x7C);
	LCDprint(157);
}

void LCDclearLine(uint8_t line)
{
	int8_t j;

	if (line == 1)
	{
		LCDprint(0xFE); // Line 1
		LCDprint(0x80);
	}
	else
	{
		LCDprint(0xFE); // Line 2
		LCDprint(0xC0);
	}
	for(j=0;j<16;j++)
	{
		LCDprint(0x20);	// Clear line
	}
}

void LCDprintstr(const char *s)
{
	while (*s) LCDprint(*s++);
}

void variable_lcd_delay(uint8_t count)
{
	switch(count) {
		case 96:				// 
			_delay_us(96);
			break;
		case 97:				// 
			_delay_us(97);
			break;
		case 98:				// 
			_delay_us(98);
			break;
		case 99:				// 
			_delay_us(99);
			break;
		case 100:				// 
			_delay_us(100);
			break;
		case 101:				// 
			_delay_us(101);
			break;
		case 102:				// 
			_delay_us(102);
			break;
		case 103:				// 
			_delay_us(103);
			break;
		case 104:				// 
			_delay_us(104);
			break;
		case 105:				// 
			_delay_us(105);
			break;
		case 106:				// 
			_delay_us(106);
			break;
		case 107:				// 
			_delay_us(107);
			break;
		case 108:				// 
			_delay_us(108);
			break;
		default:
			_delay_us(102); 	// Same as case 102
			break;
	}
}
