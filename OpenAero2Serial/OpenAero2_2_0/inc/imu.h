/*********************************************************************
 * imu.h
 ********************************************************************/

//***********************************************************
//* Externals
//***********************************************************

extern int16_t	angle[2];
extern float accSmooth[NUMBEROFAXIS];

extern void imu_update(uint32_t period);
extern void reset_IMU(void);

extern const float LPF_lookup[8] PROGMEM;
extern const float LPF_lookup_HS[8] PROGMEM;