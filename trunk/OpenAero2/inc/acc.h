/*********************************************************************
 * acc.h
 ********************************************************************/

//***********************************************************
//* Externals
//***********************************************************

extern void ReadAcc(void);
extern void AvgAcc(void);
extern void CalibrateAcc(void);
extern void AccInit(void);

extern bool AccCalibrated;
extern int16_t accADC[3];		// Holds Acc ADC values
extern int16_t AvgRoll;
extern int16_t AvgPitch;
