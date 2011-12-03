/*********************************************************************
 * rc.h
 ********************************************************************/

// STICKSCALE constants
#define NORMAL_ROLL_PITCH_STICK_SCALE	2	// Experimental values
#define NORMAL_YAW_STICK_SCALE			2	// Reducing value by 1 doubles stick sensitivity
#define ACRO_ROLL_PITCH_STICK_SCALE		1
#define ACRO_YAW_STICK_SCALE			1
#define UFO_YAW_STICK_SCALE				0


//***********************************************************
//* Externals
//***********************************************************

extern void RxGetChannels(void);

extern int16_t	RxInRoll;		// RC axis values
extern int16_t	RxInPitch;
extern uint16_t	RxInCollective;
extern int16_t	RxInYaw;
