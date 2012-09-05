/*********************************************************************
 * main.h
 ********************************************************************/
#include <stdbool.h>

//***********************************************************
//* Externals
//***********************************************************

extern bool	AutoLevel;
extern bool	Stability;
extern uint16_t	cycletime;
extern char pBuffer[16];
extern bool	Failsafe;
extern uint8_t	buffer[];
extern bool	RefreshStatus;
extern int8_t General_error;
extern uint32_t ticker_32;	
extern int16_t deltaGyroRollAngle;

extern uint16_t StackCount(void);	
