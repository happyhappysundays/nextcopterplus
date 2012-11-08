/*********************************************************************
 * rc.h
 ********************************************************************/

//***********************************************************
//* Externals
//***********************************************************

extern void RxGetChannels(void);
extern uint16_t GetChannelData(uint8_t channel);
extern int16_t get_expo_value (int16_t RCvalue, uint8_t Expolevel);
extern void RC_Deadband(void);
extern void CenterSticks(void);
extern void SetFailsafe(void);

// RC input values
extern int16_t RCinputs[MAX_RC_CHANNELS];
extern bool	RxActivity;		// RX activity flag
extern bool	HandsFree;


