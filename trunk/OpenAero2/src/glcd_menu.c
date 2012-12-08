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

const char VersionRef0[] PROGMEM = "V1.1b5"; 				// <-- Change version number here !!!
//
const char PText1[]  PROGMEM = "Resetting to";				// Init
const char PText2[]  PROGMEM = "defaults";
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
const char MainMenuItem1[]  PROGMEM = "RC setup"; 
const char MainMenuItem2[]  PROGMEM = "Stability";
const char MainMenuItem3[]  PROGMEM = "Autolevel";
const char MainMenuItem4[]  PROGMEM = "Expo";
const char MainMenuItem5[]  PROGMEM = "Battery";
const char MainMenuItem6[]  PROGMEM = "Camstab.";
const char MainMenuItem8[]  PROGMEM = "Sensors";
const char MainMenuItem9[]  PROGMEM = "RC inputs";
const char MainMenuItem11[] PROGMEM = "M1 mix";
const char MainMenuItem12[] PROGMEM = "M2 mix";
const char MainMenuItem13[] PROGMEM = "M3 mix";
const char MainMenuItem14[] PROGMEM = "M4 mix";
const char MainMenuItem15[] PROGMEM = "M5 mix";
const char MainMenuItem16[] PROGMEM = "M6 mix";
const char MainMenuItem17[] PROGMEM = "M7 mix";
const char MainMenuItem18[] PROGMEM = "M8 mix";
const char MainMenuItem10[] PROGMEM = "Level meter";
//
const char RXMode0[]  PROGMEM = "CPPM"; 					// RX mode text
const char RXMode1[]  PROGMEM = "PWM1"; 		
const char RXMode2[]  PROGMEM = "PWM2"; 
const char RXMode3[]  PROGMEM = "PWM3"; 		
const char RXMode4[]  PROGMEM = "Aero"; 
const char RXMode5[]  PROGMEM = "F.Wing";
//const char RXMode6[]  PROGMEM = "S.120";
//
const char PText15[] PROGMEM = "Gyro";
const char PText16[] PROGMEM = "X";
const char PText17[] PROGMEM = "Y";
const char PText18[] PROGMEM = "Z";
const char PText19[] PROGMEM = "Acc";
// 	
const char PText26[] PROGMEM = "Raw";
const char PText27[] PROGMEM = "Offset";
//
const char AutoMenuItem2[]  PROGMEM = "Launch:";			// Autolevel text
const char AutoMenuItem1[]  PROGMEM = "Roll P:";
const char AutoMenuItem4[]  PROGMEM = "Pitch P:";
const char AutoMenuItem5[]  PROGMEM = "Max:";
const char AutoMenuItem7[]  PROGMEM = "Roll trim:";
const char AutoMenuItem8[]  PROGMEM = "Pitch trim:";
const char AutoMenuItem9[]  PROGMEM = "Autolevel:";
const char AutoMenuItem10[] PROGMEM = "Stability:";
const char AutoMenuItem11[] PROGMEM = "OFF";
const char AutoMenuItem12[] PROGMEM = "AutoChan";
const char AutoMenuItem13[] PROGMEM = "StabChan";
const char AutoMenuItem14[] PROGMEM = "3-pos";
const char AutoMenuItem15[] PROGMEM = "ON"; 
const char AutoMenuItem16[] PROGMEM = "THR. pos"; 
const char AutoMenuItem17[] PROGMEM = "Hands Free"; 
//
const char BattMenuItem0[]  PROGMEM = "Battery type:"; 		// Battery text
const char BattMenuItem1[]  PROGMEM = "Cells:"; 
const char BattMenuItem2[]  PROGMEM = "Alarm (x10mV):";
const char BattMenuItem3[]  PROGMEM = "Max (x10mV):";
const char BattMenuItem4[]  PROGMEM = "Min (x10mV):";
const char BattMenuItem5[]  PROGMEM = "LiPo";
const char BattMenuItem6[]  PROGMEM = "NiMh";
//
const char SensorMenuItem1[]  PROGMEM = "Cal.";				 // Sensors text
const char SensorMenuItem2[]  PROGMEM = "Inv.";	
//
const char RCMenuItem0[]  PROGMEM = "CPPM Ch. order:"; 		 // RC setup text
const char RCMenuItem1[]  PROGMEM = "RX mode:"; 
const char RCMenuItem2[]  PROGMEM = "Stability input:";
const char RCMenuItem3[]  PROGMEM = "Autolevel input:";
const char RCMenuItem4[]  PROGMEM = "2nd Aileron:";
const char RCMenuItem6[]  PROGMEM = "JR/Spk"; 
const char RCMenuItem7[]  PROGMEM = "Futaba"; 
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
const char StabMenuItem13[]  PROGMEM = "Trigger:";
const char StabMenuItem14[]  PROGMEM = "Dyn.Gain Ch:";
const char StabMenuItem15[]  PROGMEM = "Dyn.Gain:";
//
const char ExpoMenuItem3[]  PROGMEM = "Differential:";		// Expo text
//
const char MixerMenuItem0[]  PROGMEM = "Orientation:";		// General text
const char MixerMenuItem2[]  PROGMEM = "Horiz.";
const char MixerMenuItem3[]  PROGMEM = "Vert.";
//
const char GeneralText0[]  PROGMEM = "Contrast:";
const char GeneralText1[]  PROGMEM = "Status time:";
const char GeneralText2[]  PROGMEM = "LMA timeout:";
const char GeneralText3[]  PROGMEM = "Servo rate:";
const char GeneralText4[] PROGMEM =  "Low";
const char GeneralText5[] PROGMEM =  "High";
const char GeneralText6[] PROGMEM =  "Acc. LPF:";
const char GeneralText7[] PROGMEM =  "CF factor:";
const char GeneralText8[] PROGMEM =  "H.Hold:";
const char GeneralText9[] PROGMEM =  "Flight mode:";
//
const char MixerItem0[] PROGMEM = "Source A:";				// Mixer menu items
const char MixerItem13[] PROGMEM = "Source B:";
const char MixerItem1[] PROGMEM = "Polarity:";
const char MixerItem2[] PROGMEM = "Volume(%):";
const char MixerItem4[] PROGMEM = "Roll gyro:";
const char MixerItem5[] PROGMEM = "Pitch gyro:";
const char MixerItem6[] PROGMEM = "Yaw gyro:";
const char MixerItem7[] PROGMEM = "Roll acc:";
const char MixerItem3[] PROGMEM = "Pitch acc:";
const char MixerItem14[] PROGMEM = "Primary:";
const char MixerItem15[] PROGMEM = "Crossmix:";
const char MixerItem8[] PROGMEM = "Min(%):";
const char MixerItem9[] PROGMEM = "Max(%):";
const char MixerItem10[] PROGMEM = "Failsafe(%):";
const char MixerItem16[] PROGMEM = "Offset(%):";
//
const char MixerItem11[] PROGMEM = "Normal";
const char MixerItem12[] PROGMEM = "Reversed";
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
const char Status0[] PROGMEM = "Press any";					// Idle text
const char Status1[] PROGMEM = "button";
const char Status2[] PROGMEM = "for status";
//
const char MOUT1[] PROGMEM = "M1";							// Outputs
const char MOUT2[] PROGMEM = "M2";	
const char MOUT3[] PROGMEM = "M3";	
const char MOUT4[] PROGMEM = "M4";	
const char MOUT5[] PROGMEM = "M5";	
const char MOUT6[] PROGMEM = "M6";	
const char MOUT7[] PROGMEM = "M7";	
const char MOUT8[] PROGMEM = "M8";	
//
const char HeadingHold1[] PROGMEM = "Auto";	
const char HeadingHold2[] PROGMEM = "3D";
const char FlightMode1[]  PROGMEM = "FlyByWire";
//
const char Dummy0[] PROGMEM = "";
//
const char IMU0[] PROGMEM = "AHRS";
//

const char *text_menu[] PROGMEM = 
	{
		VersionRef0,																		// 0 version
		PText1, PText2, 																	// 1 to 2
		//
		StatusText0, StatusText1, StatusText2, StatusText3, StatusText4, StatusText5,		// 3 to 8
		//
		MenuFrame0, MenuFrame1, MenuFrame2, MenuFrame3, MenuFrame4, MenuFrame5, 			// 9 to 17
		MenuFrame6, MenuFrame7, MenuFrame8, 
		//
		RXMode0, RXMode1, RXMode2, RXMode3, 												// 18 to 21 RX mode
		//
		RXMode4, RXMode5, Dummy0, Dummy0,													// 22 to 25 Mix presets for Aero, F.Wing
		//
		PText15, PText16, PText17, PText18, PText19, Dummy0, 								// 26 to 31 Sensors
		//
		ChannelRef1, ChannelRef2, ChannelRef0, ChannelRef3, ChannelRef4,					// 32 to 36 RC inputs
		//
		//
		StatusText1, StabMenuItem13, AutoMenuItem1,											// 37 to 45 Autolevel
		AutoMenuItem4, AutoMenuItem5, AutoMenuItem7, AutoMenuItem8, 
		AutoMenuItem2, AutoMenuItem16, 
		//
		AutoMenuItem9, AutoMenuItem10,														// 46 to 47 Autolevel, stability
		// 
		MixerItem11, HeadingHold1, HeadingHold2, Dummy0, 									// 48 to 52 HH modes (Normal, Auto, 3D)
		Dummy0, 
		//
		BattMenuItem0, BattMenuItem1, BattMenuItem2, BattMenuItem3, BattMenuItem4, 			// 53 to 59 
		BattMenuItem5, BattMenuItem6, 														// LiPo, NiMh
		//
		SensorMenuItem1,																	// 60 calibrate
		//
		IMU0, 																				// 61 was 74 IMU
		//
		MOUT1, MOUT2, MOUT3, MOUT4, MOUT5, 													// 62 to 71 MOUT1-8 + NONE
		MOUT6, MOUT7, MOUT8, ChannelRef8, Dummy0, 
		//
		ErrorText0, GeneralText4, GeneralText5, ErrorText3,	ErrorText4,						// 72 to 76 Error messages
		//
		MainMenuItem0, MainMenuItem1, MainMenuItem2, MainMenuItem3, MainMenuItem4, 			// 77 to 97  
		MainMenuItem5, MainMenuItem9, MainMenuItem8,MainMenuItem10, MainMenuItem11, 
		MainMenuItem12,MainMenuItem13,MainMenuItem14,MainMenuItem15,MainMenuItem16,
		MainMenuItem17, MainMenuItem18,	Dummy0, Dummy0,	Dummy0, Dummy0,	
		//
		ErrorText5,	ErrorText6, ErrorText7,													// 98 to 100 Error, lost, model
		//
		AutoMenuItem11, AutoMenuItem15,														// 101 to 102 OFF/ON
		MixerMenuItem2, MixerMenuItem3,														// 103 to 104 H/V
		//
		ChannelRef0, ChannelRef1, ChannelRef2, ChannelRef3, ChannelRef4, 					// 105 to 115 Ch. nums
		ChannelRef5, ChannelRef6, ChannelRef7, ChannelRef8,		
		Dummy0, Dummy0, 
		//
		MixerItem11, FlightMode1,															// 116 to 118 Flight mode
		Dummy0,
		//
		GeneralText4, GeneralText5,															// 119, 120 LOW, HIGH
		//
		Status0, Status1, Status2,															// 121 to 123 Press any button
		//
		StatusText1,StabMenuItem13, StabMenuItem14, StabMenuItem15, AutoMenuItem1, 			// 124 to 140 Stability menu
		StabMenuItem2, StabMenuItem3,  
		AutoMenuItem4, StabMenuItem5, StabMenuItem6, StabMenuItem7, StabMenuItem8, 
		StabMenuItem9, StabMenuItem10,StabMenuItem11,StabMenuItem12,
		Dummy0,
		//
		MixerItem11,MixerItem12,															// 141 to 142 Norm/Rev
		//
		AutoMenuItem11, AutoMenuItem15,MixerItem12,											// 143 to 145 off/on/rev
		//
		PText16,PText17,PText18,															// 146 to 148 X/Y/Z
		//
		RCMenuItem0, RCMenuItem1, RCMenuItem2, RCMenuItem3, 								// 149 to 156 rc menu
		RCMenuItem4, Dummy0, Dummy0, Dummy0, 
		//
		SensorMenuItem2,																	// 157 Inv.Cal.
		//
		StatusText1, MixerMenuItem0, GeneralText0, GeneralText1,							// 158 to 168 general
		GeneralText2,MainMenuItem6, GeneralText3, GeneralText6,GeneralText7, 	
		GeneralText8, GeneralText9,
		//
		MixerItem0, MixerItem2, MixerItem13, MixerItem2, MixerItem4, MixerItem1, 			// 169 to 194 
		MixerItem5, MixerItem1, MixerItem6,  MixerItem1, MixerItem7, MixerItem1, 
		MixerItem3, MixerItem1, MixerItem14, MixerItem2, MixerItem15,MixerItem2,
		MixerItem15,MixerItem2, MixerItem15, MixerItem2,
		MixerItem8, MixerItem9,	MixerItem10, MixerItem16, 
		//
		ChannelRef1, ChannelRef2, ChannelRef3, ExpoMenuItem3,								// 195 to 198 
		//
		AutoMenuItem11, AutoMenuItem12, AutoMenuItem13, AutoMenuItem14, 					// 199 to 204  Flight modes
		AutoMenuItem15, AutoMenuItem17, 
		//
		RCMenuItem6, RCMenuItem7, Dummy0,													// 205 to 207 JR/Futaba
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
	LCD_Display_Text(121,(prog_uchar*)Verdana14,26,8); 	// "Press any"
	LCD_Display_Text(122,(prog_uchar*)Verdana14,38,25); // "button"
	LCD_Display_Text(123,(prog_uchar*)Verdana14,24,42); // "for status"
	write_buffer(buffer,1);
};
