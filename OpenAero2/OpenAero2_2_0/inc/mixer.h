/*********************************************************************
 * mixer.h
 ********************************************************************/

//***********************************************************
//* Externals
//***********************************************************

extern void ProcessMixer(void);
extern void UpdateLimits(void);
extern void get_preset_mix(const channel_t*);
extern int16_t scale32(int16_t value16, int16_t multiplier16);
extern int16_t scale_percent(int8_t value);
extern int16_t scale_percent_nooffset(int8_t value);

extern const channel_t FLYING_WING_MIX[];
extern const channel_t AEROPLANE_MIX[];
extern const channel_t CAM_STAB[];
