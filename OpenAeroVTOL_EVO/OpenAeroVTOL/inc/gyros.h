/*********************************************************************
 * gyros.h
 ********************************************************************/

//***********************************************************
//* Externals
//***********************************************************

extern void ReadGyros(void);
extern void CalibrateGyrosFast(void);
extern bool CalibrateGyrosSlow(void);
extern void get_raw_gyros(void);

extern int16_t gyroADC[NUMBEROFAXIS];		// Holds 16-bit gyro values
extern int16_t gyroADCalt[NUMBEROFAXIS];	// Holds Gyro data - always in RPY order (Combined - Alternate)
extern int16_t gyroADC_raw[NUMBEROFAXIS];	// Holds raw Gyro ADCs
extern int16_t gyroADC_P1[NUMBEROFAXIS];	// Holds Gyro data - always in RPY order (P1)
extern int16_t gyroADC_P2[NUMBEROFAXIS];	// Holds Gyro data - always in RPY order (P2)