//***********************************************************
//* glcd_menu.c
//*
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include <avr/pgmspace.h> 
#include <avr/io.h>
#include <stdbool.h>
#include <util/delay.h>
#include "..\inc\io_cfg.h"
#include "..\inc\vbat.h"
#include "..\inc\adc.h"
#include "..\inc\init.h"
#include "..\inc\eeprom.h"
#include "..\inc\acc.h"
#include "..\inc\rc.h"
#include "..\inc\mugui.h"

//************************************************************
// Prototypes
//************************************************************

// Text
// Print an indexed text string from Program memory at a particular location
void LCD_Display_Text (uint8_t menuitem, prog_uchar* font,uint16_t x, uint16_t y);

// Menu items
// Menuitem value subroutines
void set_menu_item(uint8_t menuitem, int16_t value);
int16_t get_menu_item(uint8_t menuitem);
menu_range_t get_menu_range (uint8_t menuitem);
// Menu utils
uint8_t LCD_increment(uint8_t MenuItem);
uint8_t LCD_decrement(uint8_t MenuItem);

// Misc
// Print a string from at a particular location
void gLCDprint_Menu_P(const char *s, prog_uchar* font,uint16_t x, uint16_t y);



//************************************************************
// Text to print (non-menu)
//************************************************************

const char PText0[]  PROGMEM = "OpenAero2";					// Init
const char PText1[]  PROGMEM = "Resetting to";
const char PText2[]  PROGMEM = "defaults";
//
const char StatusText0[]  PROGMEM = "Version:";				// Status menu
const char StatusText1[]  PROGMEM = "Mode:";
const char StatusText2[]  PROGMEM = "Input:";
const char StatusText3[]  PROGMEM = "Status";
const char StatusText4[]  PROGMEM = ".";
const char StatusText5[]  PROGMEM = "0";	
//
const char MenuFrame0[] PROGMEM = "!"; 						// Down marker
const char MenuFrame1[] PROGMEM = "\"";						// Up
const char MenuFrame2[] PROGMEM = "'";						// Right
const char MenuFrame3[] PROGMEM = "(";						// Left
const char MenuFrame4[] PROGMEM = "+";						// Cursor
const char MenuFrame5[] PROGMEM = "Menu";					// Menu
const char MenuFrame6[] PROGMEM = "Back";					// Back
const char MenuFrame7[] PROGMEM = "Def.";					// Default
const char MenuFrame8[] PROGMEM = "Save";					// Save
//
const char MainMenuItem0[]  PROGMEM = "General"; 
const char MainMenuItem1[]  PROGMEM = "RC setup"; 
const char MainMenuItem2[]  PROGMEM = "Stability control";
const char MainMenuItem3[]  PROGMEM = "Autolevel control";
const char MainMenuItem4[]  PROGMEM = "Expo/Differential";
const char MainMenuItem5[]  PROGMEM = "Battery";
const char MainMenuItem6[]  PROGMEM = "Camera stab.";
const char MainMenuItem7[]  PROGMEM = "RC mixer";
const char MainMenuItem8[]  PROGMEM = "Sensors";
const char MainMenuItem9[]  PROGMEM = "RC inputs";
const char MainMenuItem11[] PROGMEM = "M1 mixing";
const char MainMenuItem12[] PROGMEM = "M2 mixing";
const char MainMenuItem13[] PROGMEM = "M3 mixing";
const char MainMenuItem14[] PROGMEM = "M4 mixing";
const char MainMenuItem15[] PROGMEM = "M5 mixing";
const char MainMenuItem16[] PROGMEM = "M6 mixing";
const char MainMenuItem17[] PROGMEM = "M7 mixing";
const char MainMenuItem18[] PROGMEM = "M8 mixing";
const char MainMenuItem10[] PROGMEM = "Balance meter";
//
const char RXMode0[]  PROGMEM = "CPPM"; 					// RX mode text
const char RXMode1[]  PROGMEM = "PWM1"; 		
const char RXMode2[]  PROGMEM = "PWM2"; 
const char RXMode3[]  PROGMEM = "PWM3"; 		
const char RXMode4[]  PROGMEM = "Aeroplane"; 
const char RXMode5[]  PROGMEM = "Flying Wing";
const char RXMode6[]  PROGMEM = "Manual";
//
const char PText8[]  PROGMEM = "Capacity:";
const char PText15[] PROGMEM = "Gyro";
const char PText16[] PROGMEM = "X";
const char PText17[] PROGMEM = "Y";
const char PText18[] PROGMEM = "Z";
const char PText19[] PROGMEM = "Acc";
const char PText20[] PROGMEM = "";
// 	
const char PText21[] PROGMEM = "Aileron:"; 	
const char PText22[] PROGMEM = "Elevator:";
const char PText23[] PROGMEM = "Throttle:";
const char PText24[] PROGMEM = "Rudder:";
const char PText25[] PROGMEM = "Gear:";
const char PText26[] PROGMEM = "Raw";
const char PText27[] PROGMEM = "Offset";
//
const char AutoMenuItem0[]  PROGMEM = "Mode:";  			// Autolevel text
const char AutoMenuItem1[]  PROGMEM = "Gyro P:";
const char AutoMenuItem2[]  PROGMEM = "Gyro I:"; 
const char AutoMenuItem3[]  PROGMEM = "Gyro D:"; 
const char AutoMenuItem4[]  PROGMEM = "Acc  P:";
const char AutoMenuItem5[]  PROGMEM = "Acc  I:";
const char AutoMenuItem6[]  PROGMEM = "Acc  D:";
const char AutoMenuItem7[]  PROGMEM = "Roll trim:";
const char AutoMenuItem8[]  PROGMEM = "Pitch trim:";
const char AutoMenuItem9[]  PROGMEM = "Autolevel:";
const char AutoMenuItem10[] PROGMEM = "Stability:";
const char AutoMenuItem11[] PROGMEM = "OFF";
const char AutoMenuItem12[] PROGMEM = "AutoChan";
const char AutoMenuItem13[] PROGMEM = "StabChan";
const char AutoMenuItem14[] PROGMEM = "3-pos";
const char AutoMenuItem15[] PROGMEM = "ON"; 
//
const char BattMenuItem0[]  PROGMEM = "Battery type:"; 		// Battery text
const char BattMenuItem1[]  PROGMEM = "Cells:"; 
const char BattMenuItem2[]  PROGMEM = "Alarm voltage:";
const char BattMenuItem3[]  PROGMEM = "Max cell voltage:";
const char BattMenuItem4[]  PROGMEM = "Min cell voltage:";
const char BattMenuItem5[]  PROGMEM = "LiPo";
const char BattMenuItem6[]  PROGMEM = "NiMh";
//
const char SensorMenuItem6[]  PROGMEM = "Cal.";				 // Sensors text
//
const char RCMenuItem0[]  PROGMEM = "CPPM Ch. order:"; 		 // RC setup text
const char RCMenuItem1[]  PROGMEM = "RX mode:"; 
const char RCMenuItem2[]  PROGMEM = "Stability input:";
const char RCMenuItem3[]  PROGMEM = "Autolevel input:";
const char RCMenuItem4[]  PROGMEM = "2nd Aileron:";
const char RCMenuItem5[]  PROGMEM = "3-pos switch:";
const char RCMenuItem6[]  PROGMEM = "JR/Spk"; 
const char RCMenuItem7[]  PROGMEM = "Futaba"; 
//
const char StabMenuItem0[]  PROGMEM = "Mode:";  			// Stability text
const char StabMenuItem1[]  PROGMEM = "Roll P Gain:"; 
const char StabMenuItem2[]  PROGMEM = "Roll I Gain:"; 
const char StabMenuItem3[]  PROGMEM = "Roll D Gain:";
const char StabMenuItem4[]  PROGMEM = "Pitch P Gain:";
const char StabMenuItem5[]  PROGMEM = "Pitch I Gain:";
const char StabMenuItem6[]  PROGMEM = "Pitch D Gain:"; 
const char StabMenuItem7[]  PROGMEM = "Yaw P Gain:"; 
const char StabMenuItem8[]  PROGMEM = "Yaw I Gain:";
const char StabMenuItem9[]  PROGMEM = "Yaw D Gain:";
//
const char ExpoMenuItem3[]  PROGMEM = "Differential (%):";		// Expo text
//
const char MixerMenuItem0[]  PROGMEM = "Orientation:";		// Mixer text
const char MixerMenuItem1[]  PROGMEM = "RC mix:";
const char MixerMenuItem2[]  PROGMEM = "Horiz.";
const char MixerMenuItem3[]  PROGMEM = "Vert.";
//
const char StatusText6[]  PROGMEM = "Refresh";				// Status menu
//
const char GeneralText0[]  PROGMEM = "Contrast:";
const char GeneralText1[]  PROGMEM = "Auto-refresh:";
//
const char MixerItem0[] PROGMEM = "Source:";				// Mixer menu items
const char MixerItem1[] PROGMEM = "Polarity:";
const char MixerItem2[] PROGMEM = "Volume(%):";
const char MixerItem4[] PROGMEM = "Roll gyro:";
const char MixerItem5[] PROGMEM = "Pitch gyro:";
const char MixerItem6[] PROGMEM = "Yaw gyro:";
const char MixerItem7[] PROGMEM = "Roll acc:";
const char MixerItem3[] PROGMEM = "Pitch acc:";
const char MixerItem8[] PROGMEM = "Min(%):";
const char MixerItem9[] PROGMEM = "Max(%):";
const char MixerItem10[] PROGMEM = "Failsafe(%):";
//
const char MixerItem11[] PROGMEM = "Normal";
const char MixerItem12[] PROGMEM = "Reversed";
//
const char ChannelRef0[] PROGMEM = "Throttle";
const char ChannelRef1[] PROGMEM = "Aileron"; 
const char ChannelRef2[] PROGMEM = "Elevator"; 
const char ChannelRef3[] PROGMEM = "Rudder"; 
const char ChannelRef4[] PROGMEM = "Gear"; 
const char ChannelRef5[] PROGMEM = "Flap"; 
const char ChannelRef6[] PROGMEM = "Aux1"; 
const char ChannelRef7[] PROGMEM = "Aux2"; 
const char ChannelRef8[] PROGMEM = "None";
const char ChannelRef9[] PROGMEM = "Preset 1";
const char ChannelRef10[] PROGMEM = "Preset 2";
const char ChannelRef11[] PROGMEM = "Preset 3";
const char ChannelRef12[] PROGMEM = "Preset 4";
//
const char OrientRef0[] PROGMEM = "Pitch"; 
const char OrientRef1[] PROGMEM = "Roll"; 
const char OrientRef2[] PROGMEM = "Yaw";
//
const char VersionRef0[] PROGMEM = "Beta 2"; // <-- Change version number here !!!!!!!!!!!!!!!!!!!!
//
const char ErrorText0[] PROGMEM = "Sensor"; //96
const char ErrorText1[] PROGMEM = "Low";
const char ErrorText2[] PROGMEM = "High";
const char ErrorText3[] PROGMEM = "No";
const char ErrorText4[] PROGMEM = "Signal";
const char ErrorText5[] PROGMEM = "Error";
const char ErrorText6[] PROGMEM = "Lost";
const char ErrorText7[] PROGMEM = "Model";
//
const char Dummy0[] PROGMEM = "";
//

const char *text_menu[] PROGMEM = 
	{
		PText0, PText1, PText2, 															// 0 to 2
		//
		StatusText0, StatusText1, StatusText2, StatusText3, StatusText4, StatusText5,		// 3 to 8
		//
		MenuFrame0, MenuFrame1, MenuFrame2, MenuFrame3, MenuFrame4, MenuFrame5, 			// 9 to 17
		MenuFrame6, MenuFrame7, MenuFrame8, 
		//
		StatusText4, StatusText4, StatusText4, StatusText4, StatusText4, 					// 18 to 28 
		StatusText4, StatusText4, StatusText4, StatusText4,StatusText4,
		StatusText4,
		//
		RXMode0, RXMode1, RXMode2, RXMode3, RXMode4, RXMode5, RXMode6,						// 29 to 35
		//
		PText8,																				// 36 to 49
		PText15, PText16,
		PText17, PText18, PText19, PText20, PText21, PText22, PText23, PText24, 
		PText25, PText26, PText27,
		//
		AutoMenuItem0, AutoMenuItem1, AutoMenuItem2, AutoMenuItem3, AutoMenuItem4, 			// 50 to 65
		AutoMenuItem5, AutoMenuItem6, AutoMenuItem7, AutoMenuItem8, AutoMenuItem9, 
		AutoMenuItem10, AutoMenuItem11, AutoMenuItem12, AutoMenuItem13, AutoMenuItem14, 
		AutoMenuItem15, 
		//
		BattMenuItem0, BattMenuItem1, BattMenuItem2, BattMenuItem3, BattMenuItem4, 			// 66 to 72
		BattMenuItem5, BattMenuItem6, 
		//
		SensorMenuItem6,																	// 73
		//
		Dummy0, Dummy0, Dummy0, Dummy0, Dummy0, 											// 74 to 81
		Dummy0, Dummy0, Dummy0,
		//
		StabMenuItem0, StabMenuItem1, StabMenuItem2, StabMenuItem3, StabMenuItem4, 			// 82 to 91
		StabMenuItem5, StabMenuItem6, StabMenuItem7, StabMenuItem8, StabMenuItem9, 
		//
		PText21, PText22, PText24, ExpoMenuItem3,											// 92 to 95
		//
		ErrorText0, ErrorText1, ErrorText2, ErrorText3,	ErrorText4,							// 96 to 100
		AutoMenuItem11, AutoMenuItem15,														// 101 to 102 OFF/ON
		MixerMenuItem2, MixerMenuItem3,														// 103 to 104 H/V
		//
		StatusText6, 																		// 105
		//
		MainMenuItem0, MainMenuItem1, MainMenuItem2, MainMenuItem3, MainMenuItem4, 			// 106 to 124 
		MainMenuItem5, MainMenuItem9, MainMenuItem8,MainMenuItem10,MainMenuItem11,
		MainMenuItem12,
		MainMenuItem13,MainMenuItem14,MainMenuItem15,MainMenuItem16,MainMenuItem17,
		MainMenuItem18,
		MainMenuItem6, MainMenuItem7,

		// 
		MixerItem0, MixerItem1, MixerItem2, MixerItem4,MixerItem1, 							// 125 to 140
		MixerItem5, MixerItem1, MixerItem6, MixerItem1, MixerItem7,
		MixerItem1, MixerItem3, MixerItem1,
		MixerItem8, MixerItem9,	MixerItem10,
		//
		MixerItem11,MixerItem12,															// 141 to 142 Norm/Rev
		AutoMenuItem11, AutoMenuItem15,MixerItem12,											// 143 to 145 off/on/rev
		//
		PText16,PText17,PText18,															// 146 to 148 X/Y/Z
		//
		ChannelRef0, ChannelRef1, ChannelRef2, ChannelRef3, ChannelRef4, 					// 149 to 161 Ch. nums
		ChannelRef5, ChannelRef6, ChannelRef7, ChannelRef8,		
		ChannelRef9, ChannelRef10, ChannelRef11, ChannelRef12,							
		//
		Dummy0,																				// Dummy 162
		//
		//OrientRef1, OrientRef0, PText18, AutoMenuItem11,									// 159 Accs Roll, Pitch, Z, OFF
		//
		VersionRef0,																		// 163 version
		//
		StatusText1, MixerMenuItem0, GeneralText0,											// 164 to 169 general
		GeneralText1,MixerMenuItem1, MainMenuItem6,	
		//
		ErrorText5,	ErrorText6, ErrorText7,													// 170 to 172 Error
		//
		RCMenuItem0, RCMenuItem1, RCMenuItem2, RCMenuItem3, RCMenuItem5, 					// 173 to 182 general menu
		RCMenuItem4, ChannelRef9, ChannelRef10, ChannelRef11, ChannelRef12,
		//
		RCMenuItem6, RCMenuItem7, Dummy0,													// 183 to 185 JR/Futaba
	}; 

//************************************************************
// GLCD text subroutines
//************************************************************

// Print Menuitem from Program memory at a particular location
void LCD_Display_Text (uint8_t menuitem, prog_uchar* font,uint16_t x, uint16_t y)
{
	gLCDprint_Menu_P((char*)pgm_read_word(&text_menu[menuitem]), font, x, y);
}

// Print a string from RAM at a particular location in a particular font
void gLCDprint_Menu_P(const char *s, prog_uchar* font,uint16_t x, uint16_t y)
{
	pgm_mugui_lcd_puts((prog_uchar*)s, font, x, y);
}

