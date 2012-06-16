/*********************************************************************
 * gyros.h
 ********************************************************************/

//***********************************************************
//* Externals
//***********************************************************

extern void ReadGyros(void);
extern void CalibrateGyros(void);
extern void init_i2c_gyros(void);

extern bool GyroCalibrated;
extern int16_t gyroADC[3];		// Holds 16-bit gyro values
