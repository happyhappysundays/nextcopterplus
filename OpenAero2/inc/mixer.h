/*********************************************************************
 * mixer.h
 ********************************************************************/

//***********************************************************
//* Externals
//***********************************************************

extern void ProcessMixer(void);
extern void UpdateServos(void);
extern void UpdateLimits(void);
extern void get_preset_mix(channel_t*);

extern channel_t MANUAL_MIX[];
//extern channel_t FLYING_WING_MIX[];
extern channel_t AEROPLANE_MIX[];
extern channel_t CAM_STAB[];
extern channel_t SWASH120_MIX[];
