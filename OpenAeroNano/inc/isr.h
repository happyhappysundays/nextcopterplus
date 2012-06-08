/*********************************************************************
 * isr.h
 ********************************************************************/


//***********************************************************
//* Externals
//***********************************************************

extern volatile bool RxChannelsUpdatedFlag;
extern volatile bool Interrupted;

extern volatile uint16_t RxChannel1;
extern volatile uint16_t RxChannel2;
extern volatile uint16_t RxChannel3;
extern volatile uint16_t RxChannel4;
extern uint16_t RxChannel1Start;
extern uint16_t RxChannel2Start;
extern uint16_t RxChannel3Start;
extern uint16_t RxChannel4Start;

#ifdef CPPM_MODE
extern volatile uint16_t RxChannel5;
extern volatile uint16_t RxChannel6;
extern volatile uint16_t RxChannel7;
extern volatile uint16_t RxChannel8;
extern uint16_t RxChannel5Start;
extern uint16_t RxChannel6Start;
extern uint16_t RxChannel7Start;
extern uint16_t RxChannel8Start;

#endif //CPPM_MODE
