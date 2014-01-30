/*********************************************************************
 * compiledefs.h
 *
 * Conditional compilation for KK2.0 and KK2.1 boards
 *********************************************************************/

// Comment this line out to build for KK2.0
//#define KK21 

#ifdef KK21
// Comment out this line if no airspeed sensor used (only for KK2.1)
#define AIRSPEED
#endif
