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
#define AEROPLANE_1			// Standard Aileron/Elevator/Rudder aeroplane (M1, M2 and M4)
//#define AEROPLANE_2			// Advanced Flaperons/Elevator/Rudder aeroplane (M1, M2, M4 and M5)
//#define FLYING_WING			// Standard flaperon-controlled flying wing with optional rudder (M1, M2 and M4)

//(Part 2 - receiver type)
#define PWM					// My radio is a normal radio with multiple outputs
//#define CPPM				// My radio is a special radio with one, combined output

//(Part 3 - KK board type)
#define STD_KK				// My board is an unmodified KK board
//#define ACC_KK				// My KK board has a DIY ACC sensor added
//#define MEMS_KK				// I have a KK Plus board with the MEMS board fitted 

/*********************************************************************
 * Configuration macros for Basic mode - do not edit
 *********************************************************************/
// You only need more than three channels if you have AEROPLANE_2 mode or use CPPM
#if (defined(AEROPLANE_2) || defined(CPPM))
	#define SIX_CHANNEL
#else
	#define THREE_CHANNEL
#endif

// Mixer configurations
#ifdef AEROPLANE_1	
	#define STANDARD
#elif defined(AEROPLANE_2)
	#define STD_FLAPERON
#elif defined(FLYING_WING)
	#define FWING
#else
	#error No mixer type defined
#endif

// Receiver encoding
#ifdef PWM
	#define AUTOMAGIC_PWM_MODE
#elif defined(CPPM)
	#define ICP_CPPM_MODE
#else
	#error No receiver type defined
#endif

// Board types
#ifdef STD_KK
	// No special switches needed
#elif defined(ACC_KK)
	#define ACCELEROMETER
#elif defined(MEMS_KK)
	#define ACCELEROMETER
	#define MEMS_MODULE
#else
	#error No board type defined
#endif

#else 							// Advanced mode
/*********************************************************************
 * Advanced (manual) configuration (Do not use unless you know what you are doing)
 *********************************************************************/
//#define THREE_CHANNEL			// Three PWM outputs (M1, M2 and M4), 500 step resolution
//#define SIX_CHANNEL			// Six PWM outputs (M1, M2, M4, M5, M6 and THR), 250 step resolution

//#define STANDARD				// Standard aeroplane config (3 or 6 Ch.)
//#define FWING					// Flying Wing config (3 or 6 Ch.)
//#define STD_FLAPERON			// Standard aeroplane config with separate aileron control (6 Ch. only)

//#define VERTICAL				// Mount PCB vertically with arrow facing upwards and pitch gyro aft.
								// This setting swaps the gyros around to maintain normal attitude.
								// NB: OK for MEMS gyros but NOT MEMS accelerometers

//#define ACCELEROMETER			// Uncomment this when using an accelerometer module (enables autolevel if CPPM, removes pots)
//#define MEMS_MODULE 			// Also uncomment this when using the MEMS module (MEMS gyros are reversed)

//#define CPPM_MODE 			// Uncomment this for CPPM support on CH2 (elevator)
//#define ICP_CPPM_MODE 		// Uncomment this for superior ICP CPPM support on M3
//#define LEGACY_PWM_MODE		// Uncomment this for PWM input. Older but "stable" PWM code
//#define AUTOMAGIC_PWM_MODE	// Uncomment this for PWM input. New auto-configuring PWM code
//#define DOSD_PWM_MODE			// Uncomment this for simultaneous PWM inputs such as used by DOSD etc

#endif // Advanced mode
/*********************************************************************
 * Illegal configuration detection
 *********************************************************************/
#if (defined(STD_FLAPERON) && (defined(AUTOMAGIC_PWM_MODE)))
	#error AEROPLANE_2 mode must be used with CPPM mode selected and a CPPM receiver
#elif (defined(STD_FLAPERON) && defined(THREE_CHANNEL))
	#error AEROPLANE_2 requires SIX_CHANNEL mode
#elif (defined(THREE_CHANNEL) && (defined(CPPM_MODE) || defined(ICP_CPPM_MODE)))
	#error CPPM mode requires SIX_CHANNEL mode
#elif (defined(MEMS_MODULE) && !defined(ACCELEROMETER))
	#error MEMS mode requires ACCELEROMETER mode
#elif (!defined(SIX_CHANNEL) && !defined(THREE_CHANNEL))
	#error Forgot to select the number of channels
#elif (!defined(STANDARD) && !defined(STD_FLAPERON) && !defined(FWING))
	#error Forgot to select the aeroplane type
#elif (!defined(CPPM_MODE) && !defined(ICP_CPPM_MODE) && !defined(LEGACY_PWM_MODE) && !defined(AUTOMAGIC_PWM_MODE) && !defined(DOSD_PWM_MODE))
	#error Forgot to select a receiver mode
#endif // Illegal configurations
#endif //COMPILE_DEFS_H_
