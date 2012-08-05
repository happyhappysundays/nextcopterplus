/*********************************************************************
 * mixer.h
 ********************************************************************/

//***********************************************************
//* Externals
//***********************************************************

extern void SetMixer(void);
extern void ProcessMixer(void);
extern void UpdateServos(void);
extern void SetServoPositions(void);
extern void get_preset_mix(channel_t*);

extern channel_t MANUAL_MIX[];
extern channel_t FLYING_WING_MIX[];
extern channel_t AEROPLANE_MIX[];
