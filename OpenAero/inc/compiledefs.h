/*********************************************************************
 * compiledefs.h  (Compile-time definitions)
 ********************************************************************/

#ifndef COMPILE_DEFS_H_
#define COMPILE_DEFS_H_

/*********************************************************************
 * Channel configuration
 ********************************************************************/
//#define THREE_CHANNEL		// Three PWM outputs, 500 step resolution
#define SIX_CHANNEL			// Six PWM outputs, 250 step resolution


/*********************************************************************
 * Type of model (Choose one)
 ********************************************************************/
//#define STANDARD			// Standard aeroplane config (3 or 6 Ch.)
//#define FWING				// Flying Wing config (3 or 6 Ch.)
#define STD_FLAPERON		// Standard aeroplane config with separate aileron control (6 Ch. only)


/*********************************************************************
 * Board orientation options
 ********************************************************************/
//#define VERTICAL			// Mount PCB vertically with arrow facing upwards and pitch gyro aft.


/*********************************************************************
 * Optional extensions (Choose where fitted)
 ********************************************************************/
#define CPPM_MODE 			// Uncomment this for CPPM support rather than traditional PWM inputs
#define ICP_CPPM_MODE 		// Also uncomment this for CPPM support on M3, otherwise CPPM support on CH2 (elevator)
//#define ACCELEROMETER		// Uncomment this when using an accelerometer module (enables autolevel if CPPM, removes pots)
#define MEMS_MODULE 		// Uncomment this when using the MEMS module (MEMS gyros are reversed)

#endif //COMPILE_DEFS_H_
