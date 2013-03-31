/*********************************************************************
 * main.h
 ********************************************************************/
#include <stdbool.h>

//***********************************************************
//* External defines
//***********************************************************

#define	PBUFFER_SIZE 16 // Print buffer
#define	SBUFFER_SIZE 25 // Serial input buffer (25 for S-Bus)

//***********************************************************
//* Externals
//***********************************************************

extern bool	AutoLevel;
extern bool	Stability;
extern char pBuffer[PBUFFER_SIZE];
extern bool	Failsafe;
extern uint8_t	buffer[];
extern bool	RefreshStatus;
extern volatile int8_t General_error;
extern uint32_t ticker_32;	
extern uint16_t StackCount(void);	

// Debug
extern char sBuffer[SBUFFER_SIZE];
