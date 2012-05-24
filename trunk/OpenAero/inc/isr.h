/*********************************************************************
 * isr.h
 ********************************************************************/

#include "..\inc\io_cfg.h"

//***********************************************************
//* Externals
//***********************************************************

extern volatile bool Interrupted;

extern volatile uint16_t RxChannel1;
extern volatile uint16_t RxChannel2;
extern volatile uint16_t RxChannel3;
extern volatile uint16_t RxChannel4;
extern volatile uint16_t RxChannel5;
extern volatile uint16_t RxChannel6;
extern volatile uint16_t RxChannel7;
extern volatile uint16_t RxChannel8;

extern uint16_t gap;					// Size of inter-PWM pulse gap
volatile extern uint8_t max_chan;		// Number of channel that is before the acceptible gap
extern bool	gapfound;					// Flag to indentify that gap has been calculated
extern bool	gapready;					// Used to skip over the first incidence of an input
extern bool RC_Lock;					// RC sync found/lost flag

