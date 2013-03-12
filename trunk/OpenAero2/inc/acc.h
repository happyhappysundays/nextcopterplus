/*********************************************************************
 * acc.h
 ********************************************************************/

//***********************************************************
//* Externals
//***********************************************************

extern void ReadAcc(void);
extern void CalibrateAcc(int8_t type);
extern void get_raw_accs(void);

extern int16_t accADC[3];				// Holds Acc ADC values
extern int8_t Acc_Pol[5][3];
