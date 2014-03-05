/*********************************************************************
 * imu.h
 ********************************************************************/

//***********************************************************
//* Externals
//***********************************************************

extern void getEstimatedAttitude(uint16_t period);
extern void UpdateIMUvalues(void);

extern int16_t	angle[2];
extern float accSmooth[NUMBEROFAXIS];

