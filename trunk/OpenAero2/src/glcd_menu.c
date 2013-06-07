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
//															// Status menu
const char StatusText0[]  PROGMEM = "Version: V1.2b08";		// <-- Change version number here !!!
const char StatusText1[]  PROGMEM = "Mode:";
const char StatusText3[]  PROGMEM = "Profile:";
const char StatusText4[]  PROGMEM = ".";
const char StatusText5[]  PROGMEM = "0";	
const char StatusText6[]  PROGMEM = "Free RAM:";
//
//
const char MenuFrame0[] PROGMEM = "A"; 						// Down marker
const char MenuFrame2[] PROGMEM = "B";						// Right
const char MenuFrame3[] PROGMEM = "C";						// Left
const char MenuFrame4[] PROGMEM = "D";						// Cursor
const char MenuFrame1[] PROGMEM = "E";						// Up
const char MenuFrame5[] PROGMEM = "Menu";					// Menu
const char MenuFrame6[] PROGMEM = "Back";					// Back
const char MenuFrame7[] PROGMEM = "Def.";					// Default
const char MenuFrame8[] PROGMEM = "Save";					// Save
//
//
const char MainMenuItem0[]  PROGMEM = "1. General"; 		// Main menu list text
const char MainMenuItem1[]  PROGMEM = "2. Receiver setup"; 
const char MainMenuItem7[]  PROGMEM = "3. Stick polarity"; 
const char MainMenuItem9[]  PROGMEM = "4. Receiver inputs";
const char MainMenuItem2[]  PROGMEM = "5. Flight profile 1";
const char MainMenuItem3[]  PROGMEM = "6. Flight profile 2";
const char MainMenuItem11[] PROGMEM = "7. Flight profile 3";
const char MainMenuItem8[]  PROGMEM = "8. Sensor calibration";
const char MainMenuItem10[] PROGMEM = "9. Level meter";
const char MainMenuItem12[] PROGMEM = "10. Channel mixing";
const char MainMenuItem13[] PROGMEM = "11. Output mixing";
const char MainMenuItem20[] PROGMEM = "12. Servo direction";
const char MainMenuItem21[] PROGMEM = "13. Servo trim (%)";
const char MainMenuItem22[] PROGMEM = "14. Neg. Servo trvl. (%)";
const char MainMenuItem23[] PROGMEM = "15. Pos. Servo trvl. (%)";
const char MainMenuItem4[]  PROGMEM = "16. Failsafe settings";
const char MainMenuItem24[] PROGMEM = "17. Failsafe positions";
const char MainMenuItem5[]  PROGMEM = "18. Battery monitor";
//
//
const char PText15[] PROGMEM = "Gyro";		 				// Sensors text
const char PText16[] PROGMEM = "X";
const char PText17[] PROGMEM = "Y";
const char PText18[] PROGMEM = "Z";
const char PText19[] PROGMEM = "Acc";
const char SensorMenuItem1[]  PROGMEM = "Cal.";	
const char SensorMenuItem2[]  PROGMEM = "Inv.";	
const char SensorMenuItem3[]  PROGMEM = "Failsafe";	
//
//
const char StabMenuItem13[]  PROGMEM = "Profile trig.:";	// Flight profile text
const char AutoMenuItem10[] PROGMEM = "Stability:";
const char AutoMenuItem9[]  PROGMEM = "Autolevel:";
const char GyroType1[] PROGMEM = "Roll gyro:";	
const char AutoMenuItem1[]  PROGMEM = "Roll P:";
const char StabMenuItem2[]  PROGMEM = "Roll I:"; 
const char StabMenuItem3[]  PROGMEM = "Roll D:";
const char StabMenuItem10[]  PROGMEM = "Roll I Limit:"; 
const char AutoMenuItem20[]  PROGMEM = "Acc Roll P:";
const char AutoMenuItem7[]  PROGMEM = "Roll trim:";
const char GyroType2[] PROGMEM = "Pitch gyro:";
const char AutoMenuItem4[]  PROGMEM = "Pitch P:";
const char StabMenuItem5[]  PROGMEM = "Pitch I:";
const char StabMenuItem6[]  PROGMEM = "Pitch D:"; 
const char StabMenuItem11[]  PROGMEM = "Pitch I Limit:";
const char AutoMenuItem21[]  PROGMEM = "Acc Pitch P:";
const char AutoMenuItem8[]  PROGMEM = "Pitch trim:";
const char GyroType3[] PROGMEM = "Yaw gyro:";
const char StabMenuItem7[]  PROGMEM = "Yaw P:"; 
const char StabMenuItem8[]  PROGMEM = "Yaw I:";
const char StabMenuItem9[]  PROGMEM = "Yaw D:";
const char StabMenuItem12[]  PROGMEM = "Yaw I Limit:";
//
const char AutoMenuItem11[] PROGMEM = "OFF";
const char AutoMenuItem15[] PROGMEM = "ON"; 
const char AutoMenuItem14[] PROGMEM = "3-pos";
const char AutoMenuItem12[] PROGMEM = "AutoChan";
const char AutoMenuItem13[] PROGMEM = "StabChan";
//
const char AutoMenuItem17[] PROGMEM = "Hands Free"; 
const char AutoMenuItem18[] PROGMEM = "Adv. mode:";
//
//
const char BattMenuItem0[]  PROGMEM = "Batt type:"; 		// Battery text
const char BattMenuItem1[]  PROGMEM = "Cells:"; 
const char BattMenuItem2[]  PROGMEM = "Alarm volt.:";
const char BattMenuItem3[]  PROGMEM = "Max.cell mV:";
const char BattMenuItem4[]  PROGMEM = "Min.cell mV:";
const char BattMenuItem5[]  PROGMEM = "LiPo";
const char BattMenuItem6[]  PROGMEM = "NiMh";
//
//
const char RCMenuItem1[]  PROGMEM = "RX sync:";				// RC setup text
const char RCMenuItem0[]  PROGMEM = "Ch. order:"; 	
const char RCMenuItem2[]  PROGMEM = "Profile Chan.:";
const char RCMenuItem4[]  PROGMEM = "2nd Ail. Chan.:";
const char StabMenuItem14[]  PROGMEM = "Dyn.Gain Ch.:";
const char StabMenuItem15[]  PROGMEM = "Dyn.Gain:";
const char RCMenuItem8[]  PROGMEM = "Aileron pol.:";
const char RCMenuItem12[] PROGMEM = "2nd Ail. pol.:";
const char RCMenuItem9[]  PROGMEM = "Elevator pol.:";
const char RCMenuItem10[] PROGMEM = "Rudder pol.:";
const char RCMenuItem11[] PROGMEM = "Differential:";
const char RCMenuItem20[] PROGMEM = "Flap speed:";
const char RCMenuItem21[] PROGMEM = "Deadband:";
//
const char RXMode0[]  PROGMEM = "CPPM"; 					// RX mode text
const char RXMode1[]  PROGMEM = "Xtreme";
const char RXMode2[]  PROGMEM = "S-Bus";
const char RXMode3[]  PROGMEM = "Spektrum";
//
const char RCMenuItem6[]  PROGMEM = "JR,Spktm"; 			// Channel order
const char RCMenuItem7[]  PROGMEM = "Futaba"; 
const char RCMenuItem13[] PROGMEM = "Satellite";
//
//
const char MixerMenuItem0[]  PROGMEM = "Orientation:";		// General text
const char GeneralText0[]  PROGMEM = "Contrast:";
const char GeneralText1[]  PROGMEM = "Status time:";
const char GeneralText2[]  PROGMEM = "LMA time:";
const char MainMenuItem6[]  PROGMEM = "Camstab.";
const char GeneralText3[]  PROGMEM = "Servo rate:";
const char GeneralText10[] PROGMEM =  "Cam. center:";
const char GeneralText6[] PROGMEM =  "Acc. LPF:";
const char GeneralText7[] PROGMEM =  "CF factor:";
const char IMU0[] PROGMEM = "Adv. IMU:";
const char AutoMenuItem2[]  PROGMEM = "Launch delay:";
const char AutoMenuItem16[] PROGMEM = "THR.pos %:"; 
const char AutoMenuItem19[]  PROGMEM = "Delay time:";
const char GeneralText9[] PROGMEM =  "Lock rate:";			// Stick rate for gyro I-term in Axis-lock mode
//
const char RXMode4[]  PROGMEM = "Aero"; 
const char RXMode5[]  PROGMEM = "F.Wing";
//
//
const char MixerItem0[] PROGMEM = "Source A:";				// Mixer menu items
const char MixerItem13[] PROGMEM = "Source B:";
const char MixerItem1[] PROGMEM = "Direction:";
const char MixerItem2[] PROGMEM = "Volume(%):";
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
const char MixerItem20[] PROGMEM = "Switch:";

//
const char ChannelRef0[] PROGMEM = "Throttle";				// RC channel text
const char ChannelRef1[] PROGMEM = "Aileron"; 
const char ChannelRef2[] PROGMEM = "Elevator"; 
const char ChannelRef3[] PROGMEM = "Rudder"; 
const char ChannelRef4[] PROGMEM = "Gear"; 
const char ChannelRef5[] PROGMEM = "AUX1"; 
const char ChannelRef6[] PROGMEM = "AUX2"; 
const char ChannelRef7[] PROGMEM = "AUX3"; 
const char ChannelRef8[] PROGMEM = "None";
const char ChannelRef10[] PROGMEM = "Throt.";				// RC channel text (Abbr.)
const char ChannelRef12[] PROGMEM = "Elev."; 
//
const char ErrorText0[] PROGMEM = "Sensor"; 				// Error text
const char ErrorText3[] PROGMEM = "No";
const char ErrorText4[] PROGMEM = "Signal";
const char ErrorText5[] PROGMEM = "Error";
const char ErrorText6[] PROGMEM = "Lost";
const char ErrorText7[] PROGMEM = "Model";
const char Status3[] 	PROGMEM = "Battery";
//
const char PText0[]  PROGMEM = "OpenAero2";					// Init
const char PText1[]  PROGMEM = "Reset";	
const char PText2[]  PROGMEM = "Hold steady";
//
const char WizardText0[] PROGMEM = "No RX signal!"; 		// Wizard screen
const char WizardText1[] PROGMEM = "Hold as shown";
const char WizardText2[] PROGMEM = "Done!";
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
const char MOUT9[] PROGMEM = "PSU9";
const char MOUT10[] PROGMEM = "PSU10";
const char MOUT11[] PROGMEM = "PSU11";
const char MOUT12[] PROGMEM = "PSU12";
//
const char HeadingHold1[] PROGMEM = "Rate";					// Gyro modes
const char HeadingHold2[] PROGMEM = "Axis lock";
//const char HeadingHold3[] PROGMEM = "AVCS";
const char FSmode0[]  PROGMEM = "Fixed";
const char FSmode1[]  PROGMEM = "Adv.";
//
const char MixerMenuItem2[]  PROGMEM = "Forward";			// Orientation
const char MixerMenuItem3[]  PROGMEM = "Vertical";
const char MixerMenuItem4[]  PROGMEM = "Inverted";
const char MixerMenuItem5[]  PROGMEM = "Aft";
const char MixerMenuItem6[]  PROGMEM = "Sideways";
//
const char MixerItem11[] PROGMEM = "Normal";				// Misc
const char MixerItem12[] PROGMEM = "Rev.";
//
const char GeneralText4[] PROGMEM =  "Low";
const char GeneralText5[] PROGMEM =  "High";
//
const char Dummy0[] PROGMEM = "";
//

const char *text_menu[] PROGMEM = 
	{
		PText0,																				// 0 Logo
		PText1, PText2, 																	// 1 to 2 Reset
		//
		StatusText0, StatusText1, RCMenuItem1, StatusText3, StatusText4, StatusText5,		// 3 to 8 Status menu
		//
		MenuFrame0, MenuFrame1, MenuFrame2, MenuFrame3, MenuFrame4, MenuFrame5, 			// 9 to 17 Menu frame text
		MenuFrame6, MenuFrame7, MenuFrame8, 
		//
		Dummy0, Dummy0, Dummy0, Dummy0,														// 18 to 21
		//
		RXMode4, RXMode5, MainMenuItem6, Dummy0,											// 22 to 25 Mix presets for Aero, F.Wing, Camstab
		//
		PText15, PText16, PText17, PText18, PText19, Dummy0, 								// 26 to 31 Sensors
		//
		ChannelRef1, ChannelRef2, ChannelRef0, ChannelRef3, ChannelRef4,					// 32 to 36 RC inputs
		//
		SensorMenuItem2,																	// Inv.Cal. 37
		//
		AutoMenuItem11,AutoMenuItem15, AutoMenuItem17, 										// 38 to 40  Flight modes 
		Dummy0, Dummy0, Dummy0, 				
		
		//
		SensorMenuItem3, Dummy0, 															// 44 F/S 
		//
		AutoMenuItem9, AutoMenuItem10,														// 46 to 47 Autolevel, stability													// 
		// 
		HeadingHold1, HeadingHold2, Dummy0, Dummy0, 										// 48 to 52 HH modes (Rate, Hold)
		Dummy0, 
		//
		FSmode0, FSmode1, 																	// 53 and 54 wasa Fixed, Adv.

		Dummy0,  Dummy0, Dummy0, 															// 55 to 57
		BattMenuItem5, BattMenuItem6, 														// LiPo, NiMh
		//
		SensorMenuItem1,																	// 60 calibrate
		//
		IMU0, 																				// 61 IMU
		//
		RXMode0, ChannelRef3, ChannelRef0, ChannelRef4, RXMode1, RXMode2, RXMode3,			// 62 to 68 RX mode
		//
		Dummy0, Dummy0, Dummy0,																// 69 to 71 Spare 
		//
		ErrorText0, GeneralText4, GeneralText5, ErrorText3,	ErrorText4,						// 72 to 76 Error messages
		//
		MainMenuItem0, MainMenuItem1, MainMenuItem7, MainMenuItem9, MainMenuItem2, 
		MainMenuItem3, MainMenuItem11,														// 77 to 94 Main menu
		MainMenuItem8, MainMenuItem10, MainMenuItem12,MainMenuItem13, 
		MainMenuItem20,MainMenuItem21,MainMenuItem22, MainMenuItem23, MainMenuItem4, 
		MainMenuItem24,MainMenuItem5,
		//
		Dummy0, Dummy0,	Dummy0,																// Dummy 95 to 97
		//
		ErrorText5,	ErrorText6, ErrorText7,													// 98 to 100 Error, lost, model
		//
		AutoMenuItem11, AutoMenuItem15, MixerItem12,										// 101 to 103 OFF/ON/REV
		//
		Dummy0, 																			// 104
		//
		ChannelRef0, ChannelRef1, ChannelRef2, ChannelRef3, ChannelRef4, 					// 105 to 115 Ch. nums
		ChannelRef5, ChannelRef6, ChannelRef7, ChannelRef8,		
		Dummy0, Dummy0, 
		//
		RCMenuItem6, RCMenuItem7, RCMenuItem13,												// 116 to 118 JR/Futaba/Sat
		//
		GeneralText4, GeneralText5,															// 119, 120 LOW, HIGH
		//
		Status0, Status2, Dummy0,															// 121 to 123 Press any button
		//
		MixerMenuItem2, MixerMenuItem3,	MixerMenuItem4,	MixerMenuItem5, MixerMenuItem6,		// 124 to 128 H/V/UD/Aft/Sideways
		//																
		Dummy0,Dummy0,Dummy0,Dummy0,														// 129 to 132 Spare
		//
		StatusText6,																		// 133 Free 
		//
		Status3,																			// 134 Battery
		//
		WizardText0,WizardText1,WizardText2,												// 135 to 137
		//
		Dummy0,Dummy0,Dummy0,																// 138 to 140
		//
		MixerItem11,MixerItem12,															// 141 to 142 Norm/Rev
		//
		AutoMenuItem11, AutoMenuItem15,MixerItem12,											// 143 to 145 off/on/rev
		//
		PText16,PText17,PText18,															// 146 to 148 X/Y/Z
		//
		RCMenuItem1, RCMenuItem0, RCMenuItem2, 	RCMenuItem4, 								// 149 to 162 rc menu
		StabMenuItem14, StabMenuItem15,
		RCMenuItem8, RCMenuItem12, RCMenuItem9, RCMenuItem10, 
		RCMenuItem11,RCMenuItem20, GeneralText9, RCMenuItem21,
		//
		StatusText1, AutoMenuItem16, ChannelRef2, ChannelRef1, ChannelRef3,					// 163 to 167 Failsafe
		//
		StatusText1, MixerMenuItem0, GeneralText0, GeneralText1,							// 168 to 181 general
		GeneralText2,MainMenuItem6, GeneralText3, 
		GeneralText10,
		GeneralText6,GeneralText7, IMU0,													// Don't forget to change the CONTRAST define in menu_driver.c
		AutoMenuItem2, AutoMenuItem16, AutoMenuItem19,	
		//
		BattMenuItem0, BattMenuItem1, BattMenuItem2, BattMenuItem3, BattMenuItem4, 			// 182 to 186 Battery menu
		//
		StabMenuItem13, AutoMenuItem10, AutoMenuItem9,										// 187 to 208 Flight menu
		GyroType1, AutoMenuItem1, StabMenuItem2, StabMenuItem3, StabMenuItem10, AutoMenuItem20, AutoMenuItem7,	// Roll gyro
		GyroType2, AutoMenuItem4, StabMenuItem5, StabMenuItem6, StabMenuItem11, AutoMenuItem21, AutoMenuItem8, 	// Pitch gyro
		GyroType3, StabMenuItem7, StabMenuItem8, StabMenuItem9, StabMenuItem12,	 								// Yaw gyro
		//
		MixerItem18, MixerItem0, MixerItem2, MixerItem13, MixerItem2,						// 209 Input mixers
		MixerItem17, MixerItem4, MixerItem5, MixerItem6, MixerItem7, MixerItem3, 

		MixerItem18, MixerItem20, MixerItem15, MixerItem2, 									// 220 Output mixers
		MixerItem15, MixerItem2, MixerItem15, MixerItem2,
		//
		MOUT1, MOUT2, MOUT3, MOUT4, MOUT5, 													// 228 to 240 M1-12 + NONE
		MOUT6, MOUT7, MOUT8, MOUT9, MOUT10,
		MOUT11,MOUT12,ChannelRef8, 
		//
		//
		ChannelRef10, ChannelRef12, 														// 241 - 242
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
	write_buffer(buffer);
};
