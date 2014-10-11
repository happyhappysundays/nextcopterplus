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

#ifdef KK21
extern void CenterRPYSticks(void);
extern void Erase_Trims(void);
extern void Transfer_Trims(void);
extern void ResetItermNeutral(void);
#endif

// RC input values
extern int16_t RCinputs[MAX_RC_CHANNELS + 1];	// Normalised RC inputs
extern int16_t MonopolarThrottle;			// Monopolar throttle
