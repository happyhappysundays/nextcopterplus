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
//																// Status menu
const char StatusText0[]  PROGMEM = "Version: VTOL Beta 35";	// <-- Change version number here !!!
const char StatusText1[]  PROGMEM = "Mode:";
const char StatusText3[]  PROGMEM = "Profile:";
const char StatusText4[]  PROGMEM = ".";
const char StatusText5[]  PROGMEM = "0";	
const char StatusText7[]  PROGMEM = "Battery:";
const char StatusText8[]  PROGMEM = "Pos:";
const char StatusText9[]  PROGMEM = "Jitter:";
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
const char MainMenuItem9[]  PROGMEM = "3. Receiver inputs";
const char MainMenuItem7[]  PROGMEM = "4. Stick polarity"; 
const char MainMenuItem8[]  PROGMEM = "5. Sensor calibration";
const char MainMenuItem10[] PROGMEM = "6. Level meter";
const char MainMenuItem2[]  PROGMEM = "7. Flight profile 1";
const char MainMenuItem3[]  PROGMEM = "8. Flight profile 2";
const char MainMenuItem11[] PROGMEM = "9. OUT1 Mixer";
const char MainMenuItem12[] PROGMEM = "10. OUT2 Mixer";
const char MainMenuItem13[] PROGMEM = "11. OUT3 Mixer";
const char MainMenuItem14[] PROGMEM = "12. OUT4 Mixer";
const char MainMenuItem15[] PROGMEM = "13. OUT5 Mixer";
const char MainMenuItem16[] PROGMEM = "14. OUT6 Mixer";
const char MainMenuItem17[] PROGMEM = "15. OUT7 Mixer";
const char MainMenuItem18[] PROGMEM = "16. OUT8 Mixer";
const char MainMenuItem20[] PROGMEM = "17. Servo direction";
const char MainMenuItem22[] PROGMEM = "18. Neg. Servo trvl. (%)";
const char MainMenuItem23[] PROGMEM = "19. Pos. Servo trvl. (%)";
//
const char PText15[] PROGMEM = "Gyro";		 				// Sensors text
const char PText16[] PROGMEM = "Roll";
const char PText17[] PROGMEM = "Pitch";
const char PText18[] PROGMEM = "Yaw";
const char PText19[] PROGMEM = "Acc";
const char SensorMenuItem1[]  PROGMEM = "Cal.";	
const char SensorMenuItem2[]  PROGMEM = "Inv.";	
//
const char StabMenuItem13[]  PROGMEM = "Acc Vert P:";
const char AutoMenuItem1[]  PROGMEM = "Roll P:";
const char StabMenuItem2[]  PROGMEM = "Roll I:"; 
const char StabMenuItem3[]  PROGMEM = "Roll D:";
const char StabMenuItem10[]  PROGMEM = "Roll I Limit:"; 
const char AutoMenuItem20[]  PROGMEM = "Roll AutoLvl:";
const char AutoMenuItem7[]  PROGMEM = "Roll trim:";
const char AutoMenuItem4[]  PROGMEM = "Pitch P:";
const char StabMenuItem5[]  PROGMEM = "Pitch I:";
const char StabMenuItem6[]  PROGMEM = "Pitch D:"; 
const char StabMenuItem11[]  PROGMEM = "Pitch I Limit:";
const char AutoMenuItem21[]  PROGMEM = "Pitch AutoLvl:";
const char AutoMenuItem8[]  PROGMEM = "Pitch trim:";
const char StabMenuItem7[]  PROGMEM = "Yaw P:"; 
const char StabMenuItem8[]  PROGMEM = "Yaw I:";
const char StabMenuItem9[]  PROGMEM = "Yaw D:";
const char StabMenuItem12[]  PROGMEM = "Yaw I Limit:";
const char StabMenuItem30[]  PROGMEM = "Yaw trim:";
//
const char AutoMenuItem11[] PROGMEM = "OFF";
const char AutoMenuItem15[] PROGMEM = "ON"; 
//
const char RCMenuItem1[]  PROGMEM = "RX type:";				// RC setup text
const char RCMenuItem20[] PROGMEM = "PWM sync:"; 	
const char RCMenuItem0[]  PROGMEM = "Ch. order:"; 	
const char RCMenuItem2[]  PROGMEM = "Profile Chan.:";
const char RCMenuItem8[]  PROGMEM = "Aileron pol.:";
const char RCMenuItem9[]  PROGMEM = "Elevator pol.:";
const char RCMenuItem10[] PROGMEM = "Rudder pol.:";
//
const char RXMode0[]  PROGMEM = "CPPM"; 					// RX mode text
const char RXMode1[]  PROGMEM = "PWM";
const char RXMode2[]  PROGMEM = "S-Bus";
const char RXMode3[]  PROGMEM = "Spektrum";
//
const char RCMenuItem6[]  PROGMEM = "JR,Spktm"; 			// Channel order
const char RCMenuItem7[]  PROGMEM = "Futaba"; 
//
const char MixerMenuItem0[]  PROGMEM = "Orientation:";		// General text
const char Contrast[]  PROGMEM = "Contrast:";
const char AutoMenuItem2[]  PROGMEM = "Safety:";
const char GeneralText2[]  PROGMEM = "Disarm time:";
const char GeneralText3[]  PROGMEM = "PWM rate:";
const char GeneralText6[] PROGMEM =  "Acc. LPF:";

const char GeneralText7[] PROGMEM =  "CF factor:";
const char BattMenuItem2[]  PROGMEM = "Bat. LVA x10:";
const char GeneralText9[] PROGMEM =  "Lock rate:";			// Stick rate for gyro I-term in Axis-lock mode
//
const char Transition[] PROGMEM = "Transition";
const char Transition_P1n[] PROGMEM = "Trans. P1n:";
const char P1text[] PROGMEM = "P1";
const char P2text[] PROGMEM = "P1.n";
const char P3text[] PROGMEM = "P2";
const char P4text[] PROGMEM = "P1 - P1.n";
const char P5text[] PROGMEM = "P1.n - P2";
//
// Mixer menu items
const char MixerItem1[]  PROGMEM = "Device:";
const char MixerItem20[] PROGMEM = "P1 Offset:";
const char MixerItem36[] PROGMEM = "P1.n % of trans:";
const char MixerItem35[] PROGMEM = "P1.n Offset:";
const char MixerItem34[] PROGMEM = "P2 Offset:";
const char MixerItem23[] PROGMEM = "P1 Thr. volume:";
const char MixerItem33[] PROGMEM = "P2 Thr. volume:";
const char Mixeritem50[] PROGMEM = "Thottle curve";
//
const char MixerItem51[] PROGMEM = "P1 Ail. volume:";
const char MixerItem54[] PROGMEM = "P2 Ail. volume:";
const char MixerItem52[] PROGMEM = "P1 Ele. volume:";
const char MixerItem55[] PROGMEM = "P2 Ele. volume:";
const char MixerItem53[] PROGMEM = "P1 Rud. volume:";
const char MixerItem56[] PROGMEM = "P2 Rud. volume:";
//
const char MixerItem4[]  PROGMEM = "P1 Roll gyro:";
const char MixerItem24[] PROGMEM = "P2 Roll gyro:"; 
const char MixerItem5[]  PROGMEM = "P1 Pitch gyro:";
const char MixerItem25[] PROGMEM = "P2 Pitch gyro:";
const char MixerItem6[]  PROGMEM = "P1 Yaw gyro:";
const char MixerItem26[] PROGMEM = "P2 Yaw gyro:";
const char MixerItem7[]  PROGMEM = "P1 Roll AL:";
const char MixerItem27[] PROGMEM = "P2 Roll AL:";
const char MixerItem3[]  PROGMEM = "P1 Pitch AL";
const char MixerItem28[] PROGMEM = "P2 Pitch AL:";
const char MixerItem42[] PROGMEM = "P1 Z acc:";
const char MixerItem43[] PROGMEM = "P2 Z acc:";
//
const char MixerItem0[]  PROGMEM = "P1 Source A:";			
const char MixerItem2[]  PROGMEM = "P1 Volume:";
const char MixerItem29[] PROGMEM = "P2 Source A:";			
const char MixerItem30[] PROGMEM = "P2 Volume:";
const char MixerItem21[] PROGMEM = "P1 Source B:";
const char MixerItem31[] PROGMEM = "P2 Source B:";
//
// Generic sensors for universal mixer
const char MixerItem70[] PROGMEM = "GyroRoll"; 
const char MixerItem71[] PROGMEM = "GyroPitch";
const char MixerItem72[] PROGMEM = "GyroYaw";
const char MixerItem73[] PROGMEM = "AccRoll";
const char MixerItem74[] PROGMEM = "AccPitch";
//
const char MixerItem40[] PROGMEM = "Servo";
const char MixerItem41[] PROGMEM = "Motor";
const char MixerItem60[] PROGMEM = "Linear";
const char MixerItem61[] PROGMEM = "Sine";
const char MixerItem62[] PROGMEM = "SqrtSine";
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
const char ErrorText3[] PROGMEM = "No";						// Error text
const char ErrorText4[] PROGMEM = "Signal";	
const char ErrorText5[] PROGMEM = "Error";
const char ErrorText6[] PROGMEM = "Lost";
const char ErrorText7[] PROGMEM = "Model";
const char Status3[] 	PROGMEM = "Battery";
const char ErrorText10[] PROGMEM =  "Low";
const char Disarmed[]	PROGMEM = "Disarmed";
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
const char Status4[] PROGMEM = "(Armed)";
const char Status5[] PROGMEM = "(Disarmed)";
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
const char MixerMenuItem2[]  PROGMEM = "Forward";			// Orientation
const char MixerMenuItem3[]  PROGMEM = "Vertical";
const char MixerMenuItem4[]  PROGMEM = "Inverted";
const char MixerMenuItem5[]  PROGMEM = "Aft";
const char MixerMenuItem6[]  PROGMEM = "Sideways";
const char MixerMenuItem7[]  PROGMEM = "PitchUp";
//
const char MixerItem11[] PROGMEM = "Normal";				// Misc
const char MixerItem12[] PROGMEM = "Rev.";
const char MixerItem15[] PROGMEM = "Scaled";
const char GeneralText5[] PROGMEM =  "Fast";
#ifdef AIRSPEED
const char MiscText1[] PROGMEM =  "Pitot (Pa)";
#endif
//
#ifdef KK21
// Debug
const char DebugText1[] PROGMEM =  "Deltagyro:";
const char DebugText2[] PROGMEM =  "AccSmooth:";
const char DebugText3[] PROGMEM =  "CF output:";
const char DebugText4[] PROGMEM =  "Angle:";
const char DebugText5[] PROGMEM =  "Acc Mag:";
#endif

//
const char Safety1[] PROGMEM =  "Armed";
const char Safety2[] PROGMEM =  "Armable";
const char Random1[] PROGMEM =  "High";
//
const char Dummy0[] PROGMEM = "";
//

const char* const text_menu[] PROGMEM = 
	{
		PText0,	PText1, PText2, 															// 0 Logo 1 to 2 Reset															
		//
		StatusText0, StatusText1, RCMenuItem1, StatusText3, StatusText4, StatusText5,		// 3 to 8 Status menu
		//
		MenuFrame0, MenuFrame1, MenuFrame2, MenuFrame3, MenuFrame4, MenuFrame5, 			// 9 to 17 Menu frame text
		MenuFrame6, MenuFrame7, MenuFrame8, 
		//
		Disarmed, ErrorText5,ErrorText6, ErrorText7,										// 18 to 21, Disarmed, Error, lost, model
		//
		Transition,																			// 22 Misc
		StatusText8, 
		StatusText9, 
		SensorMenuItem2,		
		//
		PText15, PText16, PText17, PText18, PText19, Dummy0, 								// 26 to 31 Sensors
		//
		ChannelRef1, ChannelRef2, ChannelRef0, ChannelRef3, ChannelRef4,					// 32 to 36 RC inputs
		//
		Dummy0,																				// 37 to 43  Spare
		Dummy0,Dummy0,Dummy0,
		Dummy0,Dummy0,Dummy0, 	
		//
		Safety1, Safety2, 																	// 44 Safety 
		//
		MixerItem40, MixerItem41,															// 46 to 47 Device types - Servo/Motor													// 
		// 
		P1text, P2text, P3text, 															// 48 to 52 P1, P1.n, P2, P1 to P1.n, P1.n to P2
		P4text, P5text, 

#ifdef AIRSPEED
		MiscText1,																			// 53 Airspeed 
#else
		Dummy0, 																			// 53 Spare 
#endif
		Dummy0,																				// 54 Spare
		//
		Random1,  																			// 55 High
		//
		MixerItem60, MixerItem61, 															// 56 to 59 Linear, Sine, Sqrt Sine
		MixerItem62,
		
		//
		Dummy0, 
		//
		SensorMenuItem1,																	// 60 calibrate
		//
		Dummy0, 																			// 61 
		//
		RXMode0, RXMode1, RXMode2, RXMode3,													// 62 to 65 RX mode
		Dummy0, Dummy0, 
		//
		AutoMenuItem11, AutoMenuItem15, MixerItem15,MixerItem12,							// 68 to 71 off/on/scale/rev 
		//
		Dummy0, ErrorText10, Dummy0, ErrorText3, ErrorText4,								// 72 to 76 Error messages
		//
		MainMenuItem0, MainMenuItem1, MainMenuItem9, MainMenuItem7, MainMenuItem8, 
		MainMenuItem10, MainMenuItem2, MainMenuItem3,  										// 77 to 95 Main menu
		MainMenuItem11,MainMenuItem12,MainMenuItem13,MainMenuItem14,
		MainMenuItem15,MainMenuItem16,MainMenuItem17,MainMenuItem18,		
		MainMenuItem20,MainMenuItem22, MainMenuItem23, 
		//
		//
#ifdef KK21
		DebugText1, DebugText2,	DebugText3, DebugText4, DebugText5,							// 96 to 100	
#else
		Dummy0, Dummy0, Dummy0, Dummy0, Dummy0, 
#endif	
		//
		AutoMenuItem11, AutoMenuItem15, MixerItem12,										// 101 to 103 OFF/ON/REV
		//
		Dummy0, 																			// 104
		//
		ChannelRef0, ChannelRef1, ChannelRef2, ChannelRef3, ChannelRef4, 					// 105 to 115 Ch. nums
		ChannelRef5, ChannelRef6, ChannelRef7, ChannelRef8,		
		ChannelRef10, ChannelRef12, 														// 114, 115 Ch.ref abbreviations
		//
		RCMenuItem6, RCMenuItem7, Dummy0,													// 116 to 118 JR/Futaba
		//
		MixerItem11, GeneralText5,															// 119, 120 Normal, Fast
		//
		Status0, Status2, Dummy0,															// 121 to 123 Press any button
		//
		MixerMenuItem2, MixerMenuItem3,	MixerMenuItem4,										// 124 to 129 H/V/UD/Aft/Sideways/PitchUp
		MixerMenuItem5, MixerMenuItem6,	MixerMenuItem7,
		//																
		Dummy0,Dummy0,Dummy0,																// 130 to 132 Spare
		//
		StatusText7,																		// 133 Battery:
		//
		Status3,																			// 134 Battery
		//
		WizardText0,WizardText1,WizardText2,												// 135 to 137
		//
		Status4,Status5,Dummy0,																// 138 to 140
		//
		MixerItem11,MixerItem12,															// 141 to 142 Norm/Rev
		//
		Dummy0, Dummy0, Dummy0,																// 143 to 145 spare
		//
		PText16,PText17,PText18,															// 146 to 148 X/Y/Z
		//
		RCMenuItem1, RCMenuItem20, RCMenuItem0, RCMenuItem2, 								// 149 to 158 RC menu
		RCMenuItem8, RCMenuItem9, RCMenuItem10, 
		GeneralText9, Transition, Transition_P1n,
		//
		MixerMenuItem0, Contrast, AutoMenuItem2,											// 159 to 167 General
		GeneralText2, BattMenuItem2, GeneralText3, 
		GeneralText6, GeneralText7, 
		//
		Dummy0,	Dummy0,Dummy0, Dummy0,	Dummy0,
		//
						 																	// 172 to 189 Flight menu
		AutoMenuItem1, StabMenuItem2, StabMenuItem10, StabMenuItem3,						// Roll gyro
		AutoMenuItem20, AutoMenuItem7,														// Roll acc
		AutoMenuItem4, StabMenuItem5, StabMenuItem11, StabMenuItem6, 						// Pitch gyro
		AutoMenuItem21, AutoMenuItem8, 														// Pitch acc
		StabMenuItem7, StabMenuItem8, StabMenuItem12, StabMenuItem9,	 					// Yaw gyro
		StabMenuItem30,	StabMenuItem13,														// Z-Acc, Yaw trim
		//
		MixerItem1,																			// 190 Motor marker (34 mixer items in total)
		MixerItem20, 																		// Offset for P1
		MixerItem36,																		// Position for P1.n	
		MixerItem35,																		// Offset for P1.n
		MixerItem34, 																		// Offset for P2
		MixerItem23,  																		// P1 Throttle
		MixerItem33, 																		// P2 Throttle
		Mixeritem50,																		// Throttle curve
		//	
		//																					// 198 to 203 mixers P1 + P2
		MixerItem51, MixerItem54,															// Aileron volume P1 + P2
		MixerItem52, MixerItem55,															// Elevator volume P1 + P2
		MixerItem53, MixerItem56,															// Rudder volume P1 + P2
		//
		MixerItem4, MixerItem24, MixerItem5, MixerItem25, MixerItem6, MixerItem26, 			// 204 Gyros and Acc P1 + P2
		MixerItem7, MixerItem27, MixerItem3, MixerItem28, MixerItem42,MixerItem43,
		//
		MixerItem0, MixerItem2, 															// 216 Source A and Volume P1
		MixerItem29, MixerItem30, 															// Source A and Volume P2
		MixerItem21, MixerItem2, 															// Source B and Volume P1
		MixerItem31, MixerItem30, 															// Source B and Volume P2
		Dummy0,Dummy0,  
		Dummy0,Dummy0, 	
		Dummy0,Dummy0,
		//
		MOUT1, MOUT2, MOUT3, MOUT4, MOUT5, MOUT6, MOUT7, MOUT8, 							// 230 to 251 Sources OUT1- OUT8, 
		ChannelRef0, ChannelRef1, ChannelRef2, ChannelRef3,									// 238 THR to RUDDER, 
		ChannelRef4, ChannelRef5, ChannelRef6, ChannelRef7,									// GEAR to AUX3 
		MixerItem70, MixerItem71, MixerItem72, MixerItem73, MixerItem74,					// Roll gyro to pitch acc
		ChannelRef8, 																		// + NONE 
		//
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
	LCD_Display_Text(121,(const unsigned char*)Verdana14,41,3); 	// "Press"
	LCD_Display_Text(122,(const unsigned char*)Verdana14,24,23); // "for status."

	if ((General_error & (1 << DISARMED)) != 0) // Disarmed
	{
		LCD_Display_Text(139,(const unsigned char*)Verdana14,20,43); // "(Disarmed)"
	}
	else
	{
		LCD_Display_Text(138,(const unsigned char*)Verdana14,28,43); // "(Armed)"
	}
	
	write_buffer(buffer,1);
};
