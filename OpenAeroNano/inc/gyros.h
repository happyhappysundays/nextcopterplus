/*********************************************************************
 * gyros.h
 ********************************************************************/

//***********************************************************
//* Externals
//***********************************************************

extern void ReadGyros(void);
extern void CalibrateGyros(void);

extern bool GyroCalibrated;
extern int16_t gyroADC[3];		// Holds Gyro ADCs
extern int16_t gyroZero[3];		// Used for calibrating Gyros on ground
