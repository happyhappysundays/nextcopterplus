/*********************************************************************
 * isr.h
 ********************************************************************/

#include "io_cfg.h"

//***********************************************************
//* Externals
//***********************************************************

extern volatile bool Interrupted;
extern volatile bool FirstInterrupted;
extern volatile uint16_t RxChannel[MAX_RC_CHANNELS]; 
extern volatile uint8_t max_chan;
extern volatile uint8_t ch_num;
extern volatile uint16_t checksum;
extern volatile bool JitterFlag;
extern volatile bool JitterGate;
extern volatile uint32_t RC_Period;
extern volatile uint16_t PWM_Safe_Start;

extern uint16_t TIM16_ReadTCNT1(void);