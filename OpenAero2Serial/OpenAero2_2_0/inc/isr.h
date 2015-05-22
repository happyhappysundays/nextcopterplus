/*********************************************************************
 * isr.h
 ********************************************************************/

#include "io_cfg.h"

//***********************************************************
//* Externals
//***********************************************************

extern volatile uint16_t RxChannel[MAX_RC_CHANNELS]; 
extern volatile uint16_t ExtChannel[MAX_EXT_CHANNELS];
extern volatile uint16_t TMR0_counter;	
extern volatile uint16_t checksum;
extern volatile uint8_t max_chan;
extern volatile uint8_t ch_num;
extern volatile bool Interrupted;
extern volatile uint32_t FramePeriod;
extern volatile uint8_t int_count;

// Xtreme
extern volatile uint8_t Xtreme_Flags;
extern volatile uint8_t Xtreme_RSS;
extern volatile uint16_t Xtreme_Chanmask;
extern volatile uint8_t Spektrum_frame_in;
extern volatile uint16_t Spektrum_Chanmask_0;
extern volatile uint16_t Spektrum_Chanmask_1;

// S.Bus
volatile uint8_t SBUS_Flags;

// Spektrum
extern volatile uint8_t Spektrum_frameloss;


extern uint16_t TIM16_ReadTCNT1(void);
extern void init_int(void);
