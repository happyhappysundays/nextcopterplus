/*********************************************************************
 * imu.h
 ********************************************************************/

//***********************************************************
//* Externals
//***********************************************************

extern void getEstimatedAttitude(void);
extern void UpdateIMUvalues(void);

extern int16_t	angle[2];
extern int8_t	ACC_LPF_FACTOR;		// User-set Acc low-pass filter
extern float	GYR_CMPF_FACTOR;
extern float	INV_GYR_CMPF_FACTOR;

