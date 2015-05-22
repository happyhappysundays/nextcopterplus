//***********************************************************
//* glcd_menu.c
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include "compiledefs.h"
#include <avr/pgmspace.h> 
#include <avr/io.h>
#include <stdbool.h>
#include <util/delay.h>
#include "io_cfg.h"
#include "vbat.h"
#include "adc.h"
#include "init.h"
#include "eeprom.h"
#include "acc.h"
#include "rc.h"
#include "mugui.h"
#include "glcd_driver.h"
#include "main.h"

//************************************************************
// Prototypes
//************************************************************

// Print an indexed text string from Program memory at a particular location
void LCD_Display_Text (uint8_t menuitem, const unsigned char* font,uint16_t x, uint16_t y);

// Print a string from at a particular location
void gLCDprint_Menu_P(const char *s, const unsigned char* font,uint16_t x, uint16_t y);

// Misc
void idle_screen(void);

//************************************************************
// Text to print (non-menu)
//************************************************************
//															// Status menu
const char StatusText0[]  PROGMEM = "Version:   S1.0 A6";	// <-- Change version number here !!!
const char StatusText1[]  PROGMEM = "Mode:";
const char StatusText3[]  PROGMEM = "Profile:";
const char StatusText4[]  PROGMEM = ".";
const char StatusText5[]  PROGMEM = "0";	
const char StatusText7[]  PROGMEM = "Preset:";
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
const char MainMenuItem0[]  PROGMEM = "1. General"; 		// Main menu list text
const char MainMenuItem1[]  PROGMEM = "2. Receiver setup"; 
const char MainMenuItem7[]  PROGMEM = "3. Stick polarity"; 
const char MainMenuItem9[]  PROGMEM = "4. Receiver inputs";
const char MainMenuItem2[]  PROGMEM = "5. Flight profile 1";
const char MainMenuItem3[]  PROGMEM = "6. Flight profile 2";
const char MainMenuItem11[] PROGMEM = "7. Flight profile 3";
const char MainMenuItem8[]  PROGMEM = "8. Sensor calibration";
const char MainMenuItem10[] PROGMEM = "9. Level meter";
const char MainMenuItem110[] PROGMEM = "10. CH1 Mixer";
const char MainMenuItem12[] PROGMEM = "11. CH2 Mixer";
const char MainMenuItem13[] PROGMEM = "12. CH3 Mixer";
const char MainMenuItem14[] PROGMEM = "13. CH4 Mixer";
const char MainMenuItem15[] PROGMEM = "14. CH5 Mixer";
const char MainMenuItem16[] PROGMEM = "15. CH6 Mixer";
const char MainMenuItem17[] PROGMEM = "16. CH7 Mixer";
const char MainMenuItem18[] PROGMEM = "17. CH8 Mixer";
const char MainMenuItem20[] PROGMEM = "18. Ch. direction";
const char MainMenuItem21[] PROGMEM = "19. Channel trim (%)";
const char MainMenuItem22[] PROGMEM = "20. Neg. travel. (%)";
const char MainMenuItem23[] PROGMEM = "21. Pos. travel. (%)";
const char MainMenuItem4[]  PROGMEM = "22. Failsafe settings";
const char MainMenuItem24[] PROGMEM = "23. Failsafe positions";
//
const char PText15[] PROGMEM = "Gyro";		 				// Sensors text
const char PText16[] PROGMEM = "Roll";
const char PText17[] PROGMEM = "Pitch";
const char PText18[] PROGMEM = "Yaw";
const char PText19[] PROGMEM = "Acc";
const char SensorMenuItem1[]  PROGMEM = "Cal.";	
const char SensorMenuItem2[]  PROGMEM = "Inv.";	
const char SensorMenuItem3[]  PROGMEM = "Failsafe";	
//
const char AutoMenuItem10[] PROGMEM = "Stability:";			// Flight profile text
const char AutoMenuItem9[]  PROGMEM = "Autolevel:";
const char GyroType1[] PROGMEM = "Roll gyro:";	
const char AutoMenuItem1[]  PROGMEM = "Roll P:";
const char StabMenuItem2[]  PROGMEM = "Roll I:"; 
const char StabMenuItem3[]  PROGMEM = "Roll D:";
const char StabMenuItem10[]  PROGMEM = "Roll I Limit:"; 
const char AutoMenuItem20[]  PROGMEM = "Roll AutoLvl:";
const char AutoMenuItem7[]  PROGMEM = "Roll trim:";
const char GyroType2[] PROGMEM = "Pitch gyro:";
const char AutoMenuItem4[]  PROGMEM = "Pitch P:";
const char StabMenuItem5[]  PROGMEM = "Pitch I:";
const char StabMenuItem6[]  PROGMEM = "Pitch D:"; 
const char StabMenuItem11[]  PROGMEM = "Pitch I Limit:";
const char AutoMenuItem21[]  PROGMEM = "Pitch AutoLvl:";
const char AutoMenuItem8[]  PROGMEM = "Pitch trim:";
const char GyroType3[] PROGMEM = "Yaw gyro:";
const char StabMenuItem7[]  PROGMEM = "Yaw P:"; 
const char StabMenuItem8[]  PROGMEM = "Yaw I:";
const char StabMenuItem9[]  PROGMEM = "Yaw D:";
const char StabMenuItem12[]  PROGMEM = "Yaw I Limit:";
const char StabMenuItem30[]  PROGMEM = "Yaw trim:";
//
const char AutoMenuItem11[] PROGMEM = "OFF";
const char AutoMenuItem15[] PROGMEM = "ON"; 
const char AutoMenuItem17[] PROGMEM = "Hands Free"; 
//
const char RCMenuItem1[]  PROGMEM =		"RX type IN:";				// RC setup text
const char RCMenuItem51[]  PROGMEM =	"RX type OUT:";
const char RCMenuItem0[]  PROGMEM =		"Ch. order:"; 	
const char RCMenuItem2[]  PROGMEM =		"Profile Chan.:";
const char RCMenuItem4[]  PROGMEM =		"2nd Ail. Chan.:";
const char StabMenuItem14[]  PROGMEM =	"Dyn.Gain Ch.:";
const char StabMenuItem15[]  PROGMEM =	"Dyn.Gain(%):";
const char RCMenuItem11[] PROGMEM =		"Differential:";
const char RCMenuItem20[] PROGMEM =		"Flap speed:";
const char RCMenuItem21[] PROGMEM =		"Deadband(%):";
const char RCMenuItem100[]  PROGMEM =	"Rx In:";
const char RCMenuItem101[]  PROGMEM =	"Tx Out:";
//
const char RXMode0[]  PROGMEM = "CPPM"; 					// RX mode text
const char RXMode2[]  PROGMEM = "S.Bus";
const char RXMode3[]  PROGMEM = "Spektrum";
const char RXMode1[]  PROGMEM = "Xtreme";
//
const char RCMenuItem6[]  PROGMEM = "JR"; 					// Channel order
const char RCMenuItem7[]  PROGMEM = "Futaba"; 
//
const char MixerMenuItem0[] PROGMEM = "Orientation:";		// General text
const char Contrast0[]		PROGMEM = "Contrast:";
const char BattMenuItem2[]  PROGMEM = "Low V Alarm:";
const char GeneralText2[]	PROGMEM = "LMA time:";
const char GeneralText7[]	PROGMEM =  "AL correct:";
const char GeneralText6[]	PROGMEM =  "Acc. LPF:";
const char GeneralText20[]	PROGMEM =  "MPU6050 LPF:";
//
const char GeneralText9[] PROGMEM =  "Lock rate:";			// Stick rate for gyro I-term in Axis-lock mode
const char AutoMenuItem16[] PROGMEM = "THR.pos %:"; 
//
const char MPU6050LPF1[] PROGMEM =  "5Hz";
const char MPU6050LPF2[] PROGMEM =  "10Hz";
const char MPU6050LPF3[] PROGMEM =  "21Hz";
const char MPU6050LPF4[] PROGMEM =  "44Hz";
const char MPU6050LPF5[] PROGMEM =  "94Hz";
const char MPU6050LPF6[] PROGMEM =  "184Hz";
const char MPU6050LPF7[] PROGMEM =  "260Hz";
//
const char RXMode4[]  PROGMEM = "Aero"; 
const char RXMode5[]  PROGMEM = "F.Wing";
const char RXMode6[]  PROGMEM = "Manual";
//
const char MixerItem0[] PROGMEM = "RC source A:";				// Mixer menu items
const char MixerItem13[] PROGMEM = "RC source B:";
const char MixerItem1[] PROGMEM = "Direction:";
const char MixerItem2[] PROGMEM = "Volume(%):";
const char MixerItem4[] PROGMEM = "Roll gyro:";
const char MixerItem5[] PROGMEM = "Pitch gyro:";
const char MixerItem6[] PROGMEM = "Yaw gyro:";
const char MixerItem7[] PROGMEM = "Roll acc:";
const char MixerItem3[] PROGMEM = "Pitch acc:";
const char MixerItem15[] PROGMEM = "Ext. Source:";
const char MixerItem8[] PROGMEM = "Trvl Min(%):";
const char MixerItem9[] PROGMEM = "Trvl Max(%):";
const char MixerItem10[] PROGMEM = "Failsafe(%):";
const char MixerItem16[] PROGMEM = "Trim(%):";
const char MixerItem19[] PROGMEM = "Reverse:";
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
const char PText4[]  PROGMEM = "Cal. failed";
//
const char WizardText0[] PROGMEM = "No RX signal?"; 		// Wizard screen
const char WizardText1[] PROGMEM = "Hold as shown";
const char WizardText2[] PROGMEM = "Done!";
//
const char Status0[] PROGMEM = "Press";						// Idle text
const char Status2[] PROGMEM = "for status";
//
const char MOUT1[] PROGMEM = "CH1";							// Outputs
const char MOUT2[] PROGMEM = "CH2";	
const char MOUT3[] PROGMEM = "CH3";	
const char MOUT4[] PROGMEM = "CH4";	
const char MOUT5[] PROGMEM = "CH5";	
const char MOUT6[] PROGMEM = "CH6";	
const char MOUT7[] PROGMEM = "CH7";	
const char MOUT8[] PROGMEM = "CH8";	
//
// Generic sensors for universal mixer
const char MixerItem70[] PROGMEM = "GyroRoll";
const char MixerItem71[] PROGMEM = "GyroPitch";
const char MixerItem72[] PROGMEM = "GyroYaw";
const char MixerItem73[] PROGMEM = "AccRoll";
const char MixerItem74[] PROGMEM = "AccPitch";
const char MixerItem80[] PROGMEM = "AL Roll";
const char MixerItem81[] PROGMEM = "AL Pitch";
//
const char HeadingHold1[] PROGMEM = "Rate";					// Gyro modes
const char HeadingHold2[] PROGMEM = "Axis lock";
const char FSmode0[]  PROGMEM = "Fixed";
const char FSmode1[]  PROGMEM = "Adv.";
//
const char MixerMenuItem2[]  PROGMEM = "Forward";			// Orientation
const char MixerMenuItem3[]  PROGMEM = "Vertical";
const char MixerMenuItem4[]  PROGMEM = "Inverted";
const char MixerMenuItem5[]  PROGMEM = "Aft";
const char MixerMenuItem6[]  PROGMEM = "Sideways";
const char MixerMenuItem7[]  PROGMEM = "PitchUp";
//
const char MixerItem11[] PROGMEM = "Normal";				// Misc
const char MixerItem12[] PROGMEM = "Rev.";
//
const char GeneralText4[] PROGMEM =  "Low";					// Output modes
const char GeneralText5[] PROGMEM =  "Sync RC";
const char GeneralText50[] PROGMEM =  "High";
//
const char Dummy0[] PROGMEM = "";
//
const char VBAT32[] PROGMEM =  "3.2V";
const char VBAT33[] PROGMEM =  "3.3V";
const char VBAT34[] PROGMEM =  "3.4V";
const char VBAT35[] PROGMEM =  "3.5V";
const char VBAT36[] PROGMEM =  "3.6V";
const char VBAT37[] PROGMEM =  "3.7V";
const char VBAT38[] PROGMEM =  "3.8V";
const char VBAT39[] PROGMEM =  "3.9V";
//

const char* const text_menu[] PROGMEM =
	{
		PText0,																				// 0 Logo
		PText1, PText2, 																	// 1 to 2 Reset
		//
		StatusText0, StatusText7, RCMenuItem1, StatusText3, StatusText4, StatusText5,		// 3 to 8 Status menu
		//
		MenuFrame0, MenuFrame1, MenuFrame2, MenuFrame3, MenuFrame4, MenuFrame5, 			// 9 to 17 Menu frame text
		MenuFrame6, MenuFrame7, MenuFrame8, 
		//
		Dummy0, ChannelRef10, ChannelRef12, SensorMenuItem3,								// 18 to 21
		//
		RXMode4, RXMode5, RXMode6, Dummy0,													// 22 to 25 Mix presets for Aero, F.Wing, Manual
		//
		PText15, PText16, PText17, PText18, PText19, Dummy0, 								// 26 to 31 Sensors
		//
		ChannelRef1, ChannelRef2, ChannelRef0, ChannelRef3, ChannelRef4,					// 32 to 36 RC inputs
		//
		SensorMenuItem2,																	// Inv.Cal. 37
		//
		AutoMenuItem11,AutoMenuItem15, AutoMenuItem17, 										// 38 to 40 Flight modes 
		//
		//MixerItem40, MixerItem41, MixerItem42, 												// 41 to 43 Devices
		Dummy0, Dummy0,  Dummy0,
		//
		HeadingHold1, HeadingHold2,															// 44, 45 HH modes (Rate, Hold)
		//
		AutoMenuItem9, AutoMenuItem10,														// 46 to 47 Autolevel, stability													// 
		// 
		RXMode0, RXMode2, RXMode3, RXMode1, Dummy0,											// 48 to 52 RX modes
		
		Dummy0, Dummy0,  Dummy0,															// 53 to 55
		//
		AutoMenuItem11, FSmode0, FSmode1, 													// 56 to 58 Off, Fixed, Adv.
		//
		Dummy0,																				// Spare
		//
		SensorMenuItem1,																	// 60 calibrate
		//
		PText4,																				// 61 Cal. failed
		//
		AutoMenuItem11, VBAT32, VBAT33, VBAT34,	VBAT35, VBAT36, VBAT37, VBAT38, VBAT39,		// 62 to 70 cell voltages
		//
		Dummy0,																				// 71 Spare 
		//
		ErrorText0, GeneralText4, GeneralText5, ErrorText3,	ErrorText4,						// 72 to 76 Error messages
		//
		MainMenuItem0, MainMenuItem1, MainMenuItem7, MainMenuItem9, MainMenuItem2, 			// 77 to 99 Main menu (23)
		MainMenuItem3, MainMenuItem11,MainMenuItem8, MainMenuItem10, MainMenuItem110, 	
		MainMenuItem12,MainMenuItem13, MainMenuItem14, MainMenuItem15, MainMenuItem16, 
		MainMenuItem17,MainMenuItem18, MainMenuItem20,MainMenuItem21,MainMenuItem22,
		MainMenuItem23, MainMenuItem4, MainMenuItem24,
		//
		Dummy0, 
		//
		AutoMenuItem11, AutoMenuItem15, MixerItem12,										// 101 to 103 OFF/ON/REV
		//
		Dummy0, 																			// 104
		//
		ChannelRef0, ChannelRef1, ChannelRef2, ChannelRef3, ChannelRef4, 					// 105 to 115 Ch. nums
		ChannelRef5, ChannelRef6, ChannelRef7, ChannelRef8,		
		Dummy0, Dummy0, 
		//
		RCMenuItem6, RCMenuItem7, Dummy0,													// 116 to 118 JR/Futaba
		//
		GeneralText4, GeneralText5, GeneralText50,											// 119, 121 LOW, Sync, HIGH
		//
		Status0, Status2,																	// 122 to 123 Press any button
		//
		MixerMenuItem2, MixerMenuItem3,	MixerMenuItem4,	MixerMenuItem5, MixerMenuItem6,		// 124 to 129 H/V/UD/Aft/Sideways/PU
		MixerMenuItem7,
		//
		ErrorText5,	ErrorText6, ErrorText7,													// 130 to 132 Error, lost, model
		//
		Dummy0,																				// 133 Spare 
		//
		Status3,																			// 134 Battery
		//
		WizardText0,WizardText1,WizardText2,												// 135 to 137
		//
		RCMenuItem100,RCMenuItem101,Dummy0,													// 138 to 140
		//
		MixerItem11,MixerItem12,															// 141 to 142 Norm/Rev
		//
		AutoMenuItem11, AutoMenuItem15,MixerItem12,											// 143 to 145 off/on/rev
		//
		PText16,PText17,PText18,															// 146 to 148 X/Y/Z
		//
		RCMenuItem1, RCMenuItem51, RCMenuItem0, RCMenuItem2, RCMenuItem4,					// 149 to 159 RC menu
		StabMenuItem14, StabMenuItem15,
		RCMenuItem11,RCMenuItem20, GeneralText9, RCMenuItem21,
		//
		StatusText1, AutoMenuItem16, ChannelRef2, ChannelRef1, ChannelRef3,					// 160 to 164 Failsafe
		//
		StatusText7, MixerMenuItem0, Contrast0, 											// 165 to 172 general
		BattMenuItem2,																		// Don't forget to change the CONTRAST define in menu_driver.c
		GeneralText2,GeneralText7,GeneralText6,GeneralText20,
		//
		Dummy0,Dummy0,Dummy0,																// 173 to 175
		//																					// SRC1 to 16
		ChannelRef0, ChannelRef1, ChannelRef2, ChannelRef3,									// 176 to 191 THR to RUDDER,
		ChannelRef4, ChannelRef5, ChannelRef6, ChannelRef7,									// GEAR to AUX3
		MixerItem70, MixerItem71, MixerItem72, MixerItem73, MixerItem74,					// Roll gyro to pitch acc
		MixerItem80, MixerItem81, ChannelRef8,												// AL Roll, AL Pitch + NONE
		//
		AutoMenuItem10, AutoMenuItem9,														// 192 to 213 Flight menu
		GyroType1, AutoMenuItem1, StabMenuItem2, StabMenuItem3, StabMenuItem10, AutoMenuItem20, AutoMenuItem7,	// Roll gyro
		GyroType2, AutoMenuItem4, StabMenuItem5, StabMenuItem6, StabMenuItem11, AutoMenuItem21, AutoMenuItem8, 	// Pitch gyro
		GyroType3, StabMenuItem7, StabMenuItem8, StabMenuItem9, StabMenuItem12,	StabMenuItem30, 				// Yaw gyro
		//
		Dummy0, 
		//
		MixerItem0, MixerItem2, MixerItem13, MixerItem2,									// 215 to 227 Mixers
		MixerItem4, MixerItem5, MixerItem6, MixerItem7, MixerItem3, 
		MixerItem15, MixerItem2, MixerItem15, MixerItem2,
		//
		Dummy0,Dummy0,Dummy0,Dummy0,														// 228 to 231 Spare
		//
		MOUT1, MOUT2, MOUT3, MOUT4, MOUT5, 													// 232 to 240 M1-8 + NONE
		MOUT6, MOUT7, MOUT8, ChannelRef8, 
		//
		MPU6050LPF1, MPU6050LPF2, MPU6050LPF3, MPU6050LPF4,									// 241 to 248  MPU6050 LPF
		MPU6050LPF5, MPU6050LPF6, MPU6050LPF7, ChannelRef8,
		//
		Dummy0,Dummy0, 																		// 249 to 250 Spare
	}; 

//************************************************************
// GLCD text subroutines
//************************************************************

// Print Menuitem from Program memory at a particular location
void LCD_Display_Text (uint8_t menuitem, const unsigned char* font,uint16_t x, uint16_t y)
{
	gLCDprint_Menu_P((char*)pgm_read_word(&text_menu[menuitem]), font, x, y);
}

// Print a string from RAM at a particular location in a particular font
void gLCDprint_Menu_P(const char *s, const unsigned char* font,uint16_t x, uint16_t y)
{
	pgm_mugui_lcd_puts((const unsigned char*)s, font, x, y);
}

// Pop up the Idle screen
void idle_screen(void)
{
	clear_buffer(buffer);

	// Change Status screen depending on arm mode
	LCD_Display_Text(122,(const unsigned char*)Verdana14,40,10);	// "Press"
	LCD_Display_Text(123,(const unsigned char*)Verdana14,24,30);	// "for status"
	
	write_buffer(buffer);
}
