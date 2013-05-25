/*********************************************************************
 * rc.h
 ********************************************************************/

//***********************************************************
//* Externals
//***********************************************************

extern void RxGetChannels(void);
extern uint16_t GetChannelData(uint8_t channel);
extern void RC_Deadband(void);
extern void CenterSticks(void);
extern void SetFailsafe(void);

// RC input values
extern int16_t RCinputs[MAX_RC_CHANNELS];

