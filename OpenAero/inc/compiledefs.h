/*********************************************************************
 * compiledefs.h  (Compile-time definitions)
 ********************************************************************/

#ifndef COMPILE_DEFS_H_
#define COMPILE_DEFS_H_

/*********************************************************************
 * Channel configuration (Choose one)
 ********************************************************************/
//#define THREE_CHANNEL		// Three PWM outputs (M1, M2 and M4), 500 step resolution
#define SIX_CHANNEL			// Six PWM outputs (M1, M2, M4, M5, M6 and THR), 250 step resolution


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
							// This setting swaps the gyros around to maintain normal attitude.
							// NB: OK for MEMS gyros but NOT MEMS accelerometers

/*********************************************************************
 * Optional extensions (Choose where fitted)
 * Note:
 *	If using MEMS, uncomment both ACCELEROMETER and MEMS_MODULE
 *  If using ICP CPPM, uncomment both CPPM_MODE and ICP_CPPM_MODE
 ********************************************************************/
#define CPPM_MODE 			// Uncomment this for CPPM support rather than traditional PWM inputs
#define ICP_CPPM_MODE 		// Also uncomment this for superior ICP CPPM support on M3, otherwise CPPM support on CH2 (elevator)
#define ACCELEROMETER		// Uncomment this when using an accelerometer module (enables autolevel if CPPM, removes pots)
#define MEMS_MODULE 		// Also uncomment this when using the MEMS module (MEMS gyros are reversed)

#endif //COMPILE_DEFS_H_
