/*********************************************************************
 * imu.h
 ********************************************************************/

//***********************************************************
//* Externals
//***********************************************************

extern void getEstimatedAttitude(void);
extern void UpdateIMUvalues(void);
extern int16_t	angle[2];
extern bool FirstTimeIMU;

// Debug
extern int16_t	AccMag;

