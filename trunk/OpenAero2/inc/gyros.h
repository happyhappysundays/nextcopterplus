/*********************************************************************
 * gyros.h
 ********************************************************************/

//***********************************************************
//* Externals
//***********************************************************

extern void ReadGyros(void);
extern void CalibrateGyrosSlow(void);
extern void CalibrateGyrosFast(void);

extern int16_t gyroADC[NUMBEROFAXIS];		// Holds 16-bit gyro values
