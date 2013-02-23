//***********************************************************
//* glcd_menu.c
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
#include "..\inc\glcd_driver.h"
#include "..\inc\main.h"

//************************************************************
// Prototypes
//************************************************************

// Print an indexed text string from Program memory at a particular location
void LCD_Display_Text (uint8_t menuitem, prog_uchar* font,uint16_t x, uint16_t y);

// Print a string from at a particular location
void gLCDprint_Menu_P(const char *s, prog_uchar* font,uint16_t x, uint16_t y);

// Misc
void idle_screen(void);

//************************************************************
// Text to print (non-menu)
//************************************************************

const char VersionRef0[] PROGMEM = "V1.1b9"; 				// <-- Change version number here !!!
//
const char PText1[]  PROGMEM = "Reset";						// Init
//
const char StatusText0[]  PROGMEM = "Ver:";					// Status menu
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
const char MainMenuItem0[]  PROGMEM = "General"; 			// Main menu list text
const char MainMenuItem1[]  PROGMEM = "RX setup"; 
const char MainMenuItem2[]  PROGMEM = "Stability";
const char MainMenuItem3[]  PROGMEM = "Autolevel";
const char MainMenuItem4[]  PROGMEM = "Failsafe settings";
const char MainMenuItem5[]  PROGMEM = "Battery monitor";
const char MainMenuItem6[]  PROGMEM = "Camstab.";
const char MainMenuItem8[]  PROGMEM = "Sensors";
const char MainMenuItem9[]  PROGMEM = "RX inputs";
const char MainMenuItem12[] PROGMEM = "Channel mixing";
const char MainMenuItem13[] PROGMEM = "Output mixing";
const char MainMenuItem10[] PROGMEM = "Level meter";
const char MainMenuItem20[] PROGMEM = "Servo direction";
const char MainMenuItem21[] PROGMEM = "Servo trim (%)";
const char MainMenuItem22[] PROGMEM = "Neg. Servo trvl. (%)";
const char MainMenuItem23[] PROGMEM = "Pos. Servo trvl. (%)";
const char MainMenuItem24[] PROGMEM = "Failsafe positions";
//
const char RXMode0[]  PROGMEM = "CPPM"; 					// RX mode text
const char RXMode4[]  PROGMEM = "Aero"; 
const char RXMode5[]  PROGMEM = "F.Wing";
//
const char PText15[] PROGMEM = "Gyro";
const char PText16[] PROGMEM = "X";
const char PText17[] PROGMEM = "Y";
const char PText18[] PROGMEM = "Z";
const char PText19[] PROGMEM = "Acc";
// 	
const char PText26[] PROGMEM = "Raw";
//
const char AutoMenuItem2[]  PROGMEM = "Launch:";			// Autolevel text
const char AutoMenuItem1[]  PROGMEM = "Roll P:";
const char AutoMenuItem4[]  PROGMEM = "Pitch P:";
const char AutoMenuItem7[]  PROGMEM = "Roll trim:";
const char AutoMenuItem8[]  PROGMEM = "Pitch trim:";
const char AutoMenuItem9[]  PROGMEM = "Autolevel setup:";
const char AutoMenuItem10[] PROGMEM = "Stability setup:";
const char AutoMenuItem11[] PROGMEM = "OFF";
const char AutoMenuItem12[] PROGMEM = "AutoChan";
const char AutoMenuItem13[] PROGMEM = "StabChan";
const char AutoMenuItem14[] PROGMEM = "3-pos";
const char AutoMenuItem15[] PROGMEM = "ON"; 
const char AutoMenuItem16[] PROGMEM = "THR.pos %"; 
const char AutoMenuItem17[] PROGMEM = "Hands Free"; 
const char AutoMenuItem18[] PROGMEM = "Adv. mode:";
//
const char BattMenuItem0[]  PROGMEM = "Batt type:"; 		// Battery text
const char BattMenuItem1[]  PROGMEM = "Cells:"; 
const char BattMenuItem2[]  PROGMEM = "Alarm vol.:";
const char BattMenuItem3[]  PROGMEM = "Max.cell vol.:";
const char BattMenuItem4[]  PROGMEM = "Min.cell vol:";
const char BattMenuItem5[]  PROGMEM = "LiPo";
const char BattMenuItem6[]  PROGMEM = "NiMh";
//
const char SensorMenuItem1[]  PROGMEM = "Cal.";				 // Sensors text
const char SensorMenuItem2[]  PROGMEM = "Inv.";	
//
const char RCMenuItem0[]  PROGMEM = "CPPM order:"; 			 // RC setup text
const char RCMenuItem1[]  PROGMEM = "RX sync:"; 
const char RCMenuItem2[]  PROGMEM = "Stab input:";
const char RCMenuItem3[]  PROGMEM = "Autolev input:";
const char RCMenuItem4[]  PROGMEM = "2nd Ail:";
const char RCMenuItem6[]  PROGMEM = "JR/Spk"; 
const char RCMenuItem7[]  PROGMEM = "Futaba"; 
const char RCMenuItem8[]  PROGMEM = "Ail. input:";
const char RCMenuItem9[]  PROGMEM = "Ele. input:";
const char RCMenuItem10[]  PROGMEM = "Rud. input:";
//
const char StabMenuItem2[]  PROGMEM = "Roll I:";    			// Stability text
const char StabMenuItem3[]  PROGMEM = "Roll D:";
const char StabMenuItem5[]  PROGMEM = "Pitch I:";
const char StabMenuItem6[]  PROGMEM = "Pitch D:"; 
const char StabMenuItem7[]  PROGMEM = "Yaw P:"; 
const char StabMenuItem8[]  PROGMEM = "Yaw I:";
const char StabMenuItem9[]  PROGMEM = "Yaw D:";
const char StabMenuItem10[]  PROGMEM = "Roll I Limit:"; 
const char StabMenuItem11[]  PROGMEM = "Pitch I Limit:";
const char StabMenuItem12[]  PROGMEM = "Yaw I Limit:";
const char StabMenuItem13[]  PROGMEM = "Trig:";
const char StabMenuItem14[]  PROGMEM = "Dyn.Gain Ch:";
const char StabMenuItem15[]  PROGMEM = "Dyn.Gain:";
//
const char MixerMenuItem0[]  PROGMEM = "Orient:";			// General text
const char MixerMenuItem2[]  PROGMEM = "Forward";
const char MixerMenuItem3[]  PROGMEM = "Vertical";
const char MixerMenuItem4[]  PROGMEM = "Inverted";
const char MixerMenuItem5[]  PROGMEM = "Aft";
//
const char GeneralText0[]  PROGMEM = "Contrast:";
const char GeneralText1[]  PROGMEM = "Status time:";
const char GeneralText2[]  PROGMEM = "LMA time:";
const char GeneralText3[]  PROGMEM = "Servo rate:";
const char GeneralText4[] PROGMEM =  "Low";
const char GeneralText5[] PROGMEM =  "High";
const char GeneralText6[] PROGMEM =  "Acc. LPF:";
const char GeneralText7[] PROGMEM =  "CF factor:";
const char GeneralText8[] PROGMEM =  "H.Hold:";
const char GeneralText9[] PROGMEM =  "3D rate:";
//
const char MixerItem0[] PROGMEM = "Src A:";				// Mixer menu items
const char MixerItem13[] PROGMEM = "Src B:";
const char MixerItem1[] PROGMEM = "Direction:";
const char MixerItem2[] PROGMEM = "Vol.(%):";
const char MixerItem4[] PROGMEM = "Roll gyro:";
const char MixerItem5[] PROGMEM = "Pitch gyro:";
const char MixerItem6[] PROGMEM = "Yaw gyro:";
const char MixerItem7[] PROGMEM = "Roll acc:";
const char MixerItem3[] PROGMEM = "Pitch acc:";
const char MixerItem15[] PROGMEM = "From:";
const char MixerItem8[] PROGMEM = "Trvl Min(%):";
const char MixerItem9[] PROGMEM = "Trvl Max(%):";
const char MixerItem10[] PROGMEM = "Failsafe(%):";
const char MixerItem16[] PROGMEM = "Trim(%):";
const char MixerItem17[] PROGMEM = "Source mix:";
const char MixerItem18[] PROGMEM = "Setting for:";
const char MixerItem19[] PROGMEM = "Reverse:";
//
const char MixerItem11[] PROGMEM = "Normal";
const char MixerItem12[] PROGMEM = "Rev.";
//
const char ChannelRef0[] PROGMEM = "Throttle";				// RC channel text
const char ChannelRef1[] PROGMEM = "Aileron"; 
const char ChannelRef2[] PROGMEM = "Elevator"; 
const char ChannelRef3[] PROGMEM = "Rudder"; 
const char ChannelRef4[] PROGMEM = "Gear"; 
const char ChannelRef5[] PROGMEM = "Flap"; 
const char ChannelRef6[] PROGMEM = "Aux1"; 
const char ChannelRef7[] PROGMEM = "Aux2"; 
const char ChannelRef8[] PROGMEM = "None";
//
const char ErrorText0[] PROGMEM = "Sensor"; 				// Error text
const char ErrorText3[] PROGMEM = "No";
const char ErrorText4[] PROGMEM = "Signal";
const char ErrorText5[] PROGMEM = "Error";
const char ErrorText6[] PROGMEM = "Lost";
const char ErrorText7[] PROGMEM = "Model";
//
const char Status0[] PROGMEM = "Press";						// Idle text
const char Status2[] PROGMEM = "for status";
//
const char MOUT1[] PROGMEM = "OUT1";						// Outputs
const char MOUT2[] PROGMEM = "OUT2";	
const char MOUT3[] PROGMEM = "OUT3";	
const char MOUT4[] PROGMEM = "OUT4";	
const char MOUT5[] PROGMEM = "OUT5";	
const char MOUT6[] PROGMEM = "OUT6";	
const char MOUT7[] PROGMEM = "OUT7";	
const char MOUT8[] PROGMEM = "OUT8";	
//
const char HeadingHold1[] PROGMEM = "Auto";	
const char HeadingHold2[] PROGMEM = "3D";
const char FSmode0[]  PROGMEM = "Fixed";
const char FSmode1[]  PROGMEM = "Adv.";
//
const char Dummy0[] PROGMEM = "";
//
const char IMU0[] PROGMEM = "Adv. IMU";
//

const char *text_menu[] PROGMEM = 
	{
		VersionRef0,																		// 0 Version
		PText1, Dummy0, 																	// 1 to 2 Reset
		//
		StatusText0, StatusText1, StatusText2, StatusText3, StatusText4, StatusText5,		// 3 to 8 Status menu
		//
		MenuFrame0, MenuFrame1, MenuFrame2, MenuFrame3, MenuFrame4, MenuFrame5, 			// 9 to 17 Menu frame text
		MenuFrame6, MenuFrame7, MenuFrame8, 
		//
		RXMode0, ChannelRef3, ChannelRef0, ChannelRef4, 												// 18 to 21 RX mode
		//
		RXMode4, RXMode5, MainMenuItem6, Dummy0,											// 22 to 25 Mix presets for Aero, F.Wing, Camstab
		//
		PText15, PText16, PText17, PText18, PText19, Dummy0, 								// 26 to 31 Sensors
		//
		ChannelRef1, ChannelRef2, ChannelRef0, ChannelRef3, ChannelRef4,					// 32 to 36 RC inputs
		//
		SensorMenuItem2,																	// Inv.Cal. 37
		//
		AutoMenuItem11, AutoMenuItem12, AutoMenuItem13, AutoMenuItem14, 					// 38 to 43  Flight modes 
		AutoMenuItem15, AutoMenuItem17, 
		//
		Dummy0, Dummy0, 
		//
		AutoMenuItem9, AutoMenuItem10,														// 46 to 47 Autolevel, stability													// 
		// 
		MixerItem11, HeadingHold1, HeadingHold2, Dummy0, 									// 48 to 52 HH modes (Normal, Auto, 3D)
		Dummy0, 
		//
		BattMenuItem0, BattMenuItem1, BattMenuItem2, BattMenuItem3, BattMenuItem4, 			// 53 to 59 ******
		BattMenuItem5, BattMenuItem6, 														// LiPo, NiMh
		//
		SensorMenuItem1,																	// 60 calibrate
		//
		IMU0, 																				// 61 was 74 IMU
		//
		MOUT1, MOUT2, MOUT3, MOUT4, MOUT5, 													// 62 to 71 M1-8 + NONE
		MOUT6, MOUT7, MOUT8, ChannelRef8, Dummy0, 
		//
		ErrorText0, GeneralText4, GeneralText5, ErrorText3,	ErrorText4,						// 72 to 76 Error messages
		//
		MainMenuItem0, MainMenuItem1, MainMenuItem2, MainMenuItem3, MainMenuItem4, 			// 77 to 93 Main menu
		MainMenuItem5, MainMenuItem9, MainMenuItem8,MainMenuItem10, 
		MainMenuItem12,MainMenuItem13,MainMenuItem20, MainMenuItem21, MainMenuItem22, 
		MainMenuItem23, MainMenuItem24, Dummy0,
		//
		Dummy0, Dummy0, Dummy0,	Dummy0,														// Dummy 94 to 97
		//
		ErrorText5,	ErrorText6, ErrorText7,													// 98 to 100 Error, lost, model
		//
		AutoMenuItem11, AutoMenuItem15,														// 101 to 102 OFF/ON
		//
		FSmode0, FSmode1, 																	// 103 to 104 Fixed, Adv.
		//
		ChannelRef0, ChannelRef1, ChannelRef2, ChannelRef3, ChannelRef4, 					// 105 to 115 Ch. nums
		ChannelRef5, ChannelRef6, ChannelRef7, ChannelRef8,		
		Dummy0, Dummy0, 
		//
		RCMenuItem6, RCMenuItem7,															// 116 to 117 JR/Futaba was 93
		//		
		Dummy0,																				//118 Spare
		//
		GeneralText4, GeneralText5,															// 119, 120 LOW, HIGH
		//
		Status0, Status2, Dummy0,															// 121 to 123 Press any button
		//
		MixerMenuItem2, MixerMenuItem3,	MixerMenuItem4,	MixerMenuItem5,						// 124 to 127 H/V/UD/Aft
		//
		Dummy0, 																		// 128 to 140 Spare
		Dummy0,Dummy0,Dummy0,Dummy0,Dummy0,
		Dummy0,Dummy0,Dummy0,Dummy0,Dummy0,
		Dummy0,Dummy0,
		//
		MixerItem11,MixerItem12,															// 141 to 142 Norm/Rev
		//
		AutoMenuItem11, AutoMenuItem15,MixerItem12,											// 143 to 145 off/on/rev
		//
		PText16,PText17,PText18,															// 146 to 148 X/Y/Z
		//
		RCMenuItem0, RCMenuItem1, RCMenuItem2, RCMenuItem3, 								// 149 to 156 rc menu
		RCMenuItem4, RCMenuItem8, RCMenuItem9, RCMenuItem10, 
		//
		StatusText1, AutoMenuItem16, ChannelRef2, ChannelRef1, ChannelRef3,					// 157 to 161 Failsafe
		//
		StatusText1, StabMenuItem13, AutoMenuItem1,											// 162 to 169 Autolevel menu
		AutoMenuItem4, AutoMenuItem7, AutoMenuItem8, 
		AutoMenuItem2, AutoMenuItem16,											
		//
		StatusText1,StabMenuItem13, StabMenuItem14, StabMenuItem15, AutoMenuItem1, 			// 170 to 185 Stability menu
		StabMenuItem2, StabMenuItem3,  
		AutoMenuItem4, StabMenuItem5, StabMenuItem6, StabMenuItem7, StabMenuItem8, 
		StabMenuItem9, StabMenuItem10,StabMenuItem11,StabMenuItem12,
		//
		StatusText1, MixerMenuItem0, GeneralText0, GeneralText1,							// 186 to 197 general
		GeneralText2,MainMenuItem6, GeneralText3, GeneralText6,GeneralText7, 				// Don't forget to change the CONTRAST define in menu_driver.c
		GeneralText8, GeneralText9, IMU0,
		//
		MixerItem18, MixerItem19, MixerItem16, MixerItem8, MixerItem9, MixerItem10,			// 198 to 227 Servo (29)
		MixerItem18, MixerItem0, MixerItem2, MixerItem13, MixerItem2,						// Input mixers
		MixerItem17, MixerItem4, MixerItem1, MixerItem5, MixerItem1, 
		MixerItem6, MixerItem1, MixerItem7, MixerItem1, MixerItem3, 
		MixerItem1,
		MixerItem18, MixerItem15, MixerItem2, 												// Output mixers
		MixerItem15, MixerItem2, MixerItem15, MixerItem2,
		//
		
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

// Pop up the Idle screen
void idle_screen(void)
{
	clear_buffer(buffer);
	LCD_Display_Text(121,(prog_uchar*)Verdana14,40,12); // "Press"
	LCD_Display_Text(122,(prog_uchar*)Verdana14,24,32); // "for status"
	write_buffer(buffer,1);
};
