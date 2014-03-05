/*********************************************************************
 * vbat.h
 ********************************************************************/

#include "compiledefs.h"

//***********************************************************
//* Externals
//***********************************************************

extern uint16_t GetVbat(void);	

#ifdef AIRSPEED
extern uint16_t GetAirspeed(void);
extern void CalibrateAirspeed(void);
#endif
