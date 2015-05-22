/*********************************************************************
 * main.h
 ********************************************************************/
#include <stdbool.h>

//***********************************************************
//* External defines
//***********************************************************

#define	PBUFFER_SIZE 25 // Print buffer
#define	SBUFFER_SIZE 38 // Serial input buffer (Xtreme maximum is 37 bytes)

//***********************************************************
//* Externals
//***********************************************************

// Buffers
extern char pBuffer[PBUFFER_SIZE];
extern uint8_t	buffer[1024];
extern uint8_t sBuffer[SBUFFER_SIZE];
extern bool	RefreshStatus;

// Flags
extern volatile uint8_t	General_error;
extern volatile uint8_t	Flight_flags;
extern volatile uint8_t	Alarm_flags;

// Misc
extern volatile uint16_t InterruptCount;
extern volatile uint16_t LoopStartTCNT1;
extern volatile uint8_t	LoopCount;

extern volatile uint32_t FrameDrop_Output_Rate;
extern volatile uint32_t Failsafe_Output_Rate;
extern volatile uint16_t RC_Timeout;