/*********************************************************************
 * compiledefs.h  (Compile-time definitions)
 *********************************************************************/
#ifndef COMPILE_DEFS_H_
#define COMPILE_DEFS_H_

//#define ADVANCED_MODE			// Only for the brave experimenter

#ifndef ADVANCED_MODE			// Basic mode
/*********************************************************************
 * Easy configuration (Choose one from each part, and ignore all the settings below)
 *********************************************************************/
//(Part 1 - Mixer type)
#define AEROPLANE_1			// Standard Aileron/Elevator/Rudder aeroplane (M2, M4 and M1 or M3)
//#define AEROPLANE_2			// Advanced Flaperons/Elevator/Rudder aeroplane (M2 and M5, M4 and M1 or M3)
//#define FLYING_WING			// Standard flaperon-controlled flying wing with optional rudder (M2 and M4, and M1 or M3)
//#define AEROPLANE_3			// Same as AEROPLANE_1 but with extra flaperon channel on M3

//(Part 2 - receiver type)
#define PWM					// My radio is a normal radio with multiple outputs
//#define CPPM				// My radio is a special radio with one, combined output

//(Part 3 - KK board type)
#define STD_KK				// My board is an unmodified KK board
//#define KK_PLUS				// I have a KK Plus board with support for LVA and buzzer
//#define ACC_KK				// My KK board has a DIY ACC sensor added
//#define MEMS_KK				// I have a KK Plus board with the MEMS board fitted 
//#define N6_MODE				// My board is an Eagle N6/HobbyKing i86 board

//(Part 4 - special options)
//#define THREE_POS				// I am using a 3-position switch to select stability and autolevel modes

/*********************************************************************
 * Configuration macros for Basic mode - do not edit
 *********************************************************************/
// You only need more than three channels if you have AEROPLANE_2 mode or use CPPM
#if (defined(AEROPLANE_2) || defined(AEROPLANE_3) || defined(CPPM))
	#define SIX_CHANNEL
#else
	#define THREE_CHANNEL
#endif

// Mixer configurations
#ifdef AEROPLANE_1	
	#define STANDARD
#elif (defined(AEROPLANE_2) || defined(AEROPLANE_3))
	#define STD_FLAPERON
#elif defined(FLYING_WING)
	#define FWING
#else
	#error No mixer type defined
#endif

// Receiver encoding
#ifdef PWM
	#if defined(AEROPLANE_3)
		#define HYBRID_PWM_MODE
	#else
	//#define AUTOMAGIC_PWM_MODE
	#define LEGACY_PWM_MODE1
	//#define LEGACY_PWM_MODE2
	//#define DOSD_PWM_MODE
	#endif
#elif defined(CPPM)
	#define ICP_CPPM_MODE
#else
	#error No receiver type defined
#endif

// Board types
#if (defined(STD_KK) || defined(N6_MODE) || defined(KK_PLUS))
	// No special switches needed
#elif defined(ACC_KK)
	#define ACCELEROMETER
#elif defined(MEMS_KK)
	#define ACCELEROMETER
	#define MEMS_MODULE
	#define KK_PLUS
#else
	#error No board type defined
#endif

#else 							// Advanced mode
/*********************************************************************
 * Advanced (manual) configuration (Do not use unless you know what you are doing)
 *********************************************************************/
//#define THREE_CHANNEL			// Three PWM outputs (M1/M3, M2 and M4), 500 step resolution
//#define SIX_CHANNEL			// Six PWM outputs (M1/M3, M2, M4, M5, M6 and THR), 250 step resolution

//#define STANDARD				// Standard aeroplane config (3 or 6 Ch.)
//#define FWING					// Flying Wing config (3 or 6 Ch.)
//#define STD_FLAPERON			// Standard aeroplane config with separate aileron control (6 Ch. only)

//#define VERTICAL				// Mount PCB vertically with arrow facing upwards and pitch gyro aft.
								// This setting swaps the gyros around to maintain normal attitude.
								// NB: OK for MEMS gyros but NOT MEMS accelerometers
//#define XMODE					// Mount PCB square-on to the model, not with the arrow facing forwards to save space.

//#define ACCELEROMETER			// Uncomment this when using an accelerometer module (enables autolevel if CPPM, removes pots)
//#define MEMS_MODULE 			// Also uncomment this when using the MEMS module (MEMS gyros are reversed)
//#define N6_MODE				// My board is an Eagle N6/HobbyKing i86 board

//#define ICP_CPPM_MODE 		// Uncomment this for superior ICP CPPM support on M3 (KK) or M1 (i86/N6)
//#define LEGACY_PWM_MODE1		// Uncomment this for V1.11-style PWM input. Older but "stable" PWM code
//#define LEGACY_PWM_MODE2		// Uncomment this for V1.12-style PWM input. Works better for some people
//#define AUTOMAGIC_PWM_MODE	// Uncomment this for PWM input. New auto-configuring PWM code
//#define DOSD_PWM_MODE			// Uncomment this for simultaneous PWM inputs such as used by DOSD etc
//#define HYBRID_PWM_MODE		// Uncomment this for extra flaperon channel on M3

#endif // Advanced mode

/*********************************************************************
 * Illegal configuration detection
 *********************************************************************/
#if (defined(STD_FLAPERON) && !((defined(HYBRID_PWM_MODE)) || (defined(ICP_CPPM_MODE))))
	#error AEROPLANE_2 and AEROPLANE_3 modes must be used with CPPM or HYBRID_PWM_MODE
#elif (defined(STD_FLAPERON) && defined(THREE_CHANNEL))
	#error AEROPLANE_2 requires SIX_CHANNEL mode
#elif (defined(THREE_CHANNEL) && (defined(ICP_CPPM_MODE)))
	#error CPPM mode requires SIX_CHANNEL mode
#elif (defined(MEMS_MODULE) && !defined(ACCELEROMETER))
	#error MEMS mode requires ACCELEROMETER mode
#elif (!defined(SIX_CHANNEL) && !defined(THREE_CHANNEL))
	#error Forgot to select the number of channels
#elif (!defined(STANDARD) && !defined(STD_FLAPERON) && !defined(FWING))
	#error Forgot to select the aeroplane type
#elif (!defined(ICP_CPPM_MODE) && !defined(LEGACY_PWM_MODE1) && !defined(LEGACY_PWM_MODE2) && !defined(AUTOMAGIC_PWM_MODE) && !defined(DOSD_PWM_MODE) && !defined(HYBRID_PWM_MODE))
	#error Forgot to select a receiver mode
#endif // Illegal configurations
#endif //COMPILE_DEFS_H_
