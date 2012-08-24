/*********************************************************************
 * gyros.h
 ********************************************************************/

//***********************************************************
//* Externals
//***********************************************************

extern void ReadGyros(void);
extern void CalibrateGyros(void);

extern int16_t gyroADC[3];		// Holds 16-bit gyro values
extern int16_t gyroZero[3];		// Used for calibrating Gyros on ground
