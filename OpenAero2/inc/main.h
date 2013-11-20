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

// Buffers
extern char pBuffer[PBUFFER_SIZE];
extern uint8_t	buffer[];
extern char sBuffer[SBUFFER_SIZE];

extern bool	RefreshStatus;
extern uint32_t ticker_32;	
extern uint16_t StackCount(void);	

// Flags
extern uint8_t	General_error;
extern uint8_t	Flight_flags;
extern uint8_t	Main_flags;
extern uint8_t	Alarm_flags;


