/*********************************************************************
 * rc.h
 ********************************************************************/

//***********************************************************
//* Externals
//***********************************************************

extern void RxGetChannels(void);

// RC input values
extern int16_t	RxInRoll;		// Roll/Left.Aileron input
extern int16_t	RxInPitch;		// Pitch input
extern uint16_t	RxInAux;		// Stability switch input
extern int16_t	RxInYaw;		// Yaw input
extern int16_t	RxInAux1;		// Flap/Right.Aileron input

extern bool		RxActivity;		// RX activity flag
