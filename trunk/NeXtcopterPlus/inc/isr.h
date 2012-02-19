/*********************************************************************
 * isr.h
 ********************************************************************/


//***********************************************************
//* Externals
//***********************************************************

extern volatile bool RxChannelsUpdatedFlag;

extern volatile uint16_t RxChannel1;
extern volatile uint16_t RxChannel2;
extern volatile uint16_t RxChannel3;
extern volatile uint16_t RxChannel4;
extern volatile uint16_t RxChannel5;
extern uint16_t RxChannel1Start;
extern uint16_t RxChannel2Start;
extern uint16_t RxChannel3Start;
extern uint16_t RxChannel4Start;
extern uint16_t RxChannel5Start;


#ifdef CPPM_MODE
extern volatile uint16_t RxChannel6;
extern volatile uint16_t RxChannel7;
extern volatile uint16_t RxChannel8;
extern uint16_t RxChannel6Start;
extern uint16_t RxChannel7Start;
extern uint16_t RxChannel8Start;

#ifdef PROX_MODULE
extern volatile uint16_t EchoTime;			// Sonar echo duration
extern uint16_t EchoTimeStart;				// Sonar echo start
#endif

#endif //CPPM_MODE
