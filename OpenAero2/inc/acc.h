/*********************************************************************
 * acc.h
 ********************************************************************/

//***********************************************************
//* Externals
//***********************************************************

extern void ReadAcc(void);
extern void CalibrateAcc(void);
extern void CalibrateInvAcc(void);
extern void get_raw_accs(void);

extern int16_t accADC[3];				// Holds Acc ADC values
extern int16_t accZero[3];				// Debug
extern int16_t tempaccZero;				// Debug
