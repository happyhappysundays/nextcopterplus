/*********************************************************************
 * compiledefs.h
 *
 * Conditional compilation for KK2.1 and KK2.1 Mini boards
 *********************************************************************/

// Uncomment this for compatibility with the KK2 Mini
// This limits the available LCD contrast
//#define KK2Mini

// Uncomment this to enable the error log
#define ERROR_LOG

// Debug - choose D-term method
// Uncommented = average differences in gyros
// Commented = measure differences in averaged gyros
#define D_METHOD