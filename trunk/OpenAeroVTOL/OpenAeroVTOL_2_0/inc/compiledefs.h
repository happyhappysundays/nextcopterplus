/*********************************************************************
 * compiledefs.h
 *
 * Conditional compilation for KK2.0 and KK2.1 boards
 *********************************************************************/

// Comment this line out to build for KK2.0
// Note: For the Studio 6 complete solution, this is done automatically. 
// Please leave this line commented out if using the Studio 6 solution
// #define KK21 

#ifdef KK21
// Uncomment this line if no airspeed sensor used (only for KK2.1)
// #define AIRSPEED

// Uncomment this for logging of gyro data on the sensor screen
// #define DISPLAYLOG

// Uncomment this line to have the factory setup default to a quad "+" setup on OUT1 to OUT4
//#define QUADCOPTER

// Uncomment this for compatibility with the KK2 Mini
// This limits the available LCD contrast
//#define KK2Mini

// Uncomment this line to expose the advanced flight features
// This includes, Acro mode, progressive P and Auto-trim
//#define ADVANCED

#endif
