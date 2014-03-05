/*********************************************************************
 * compiledefs.h
 *
 * Conditional compilation for KK2.0 and KK2.1 boards
 *********************************************************************/

// Comment this line out to build for KK2.0
// Note: For the Studio 6 complete solution, this is done automatically. 
// Please leave this line commented out if using the Studio 6 solution
//#define KK21 

#ifdef KK21
// Comment out this line if no airspeed sensor used (only for KK2.1)
//#define AIRSPEED
#endif
