/*********************************************************************
 * pid.h
 ********************************************************************/

//***********************************************************
//* Externals
//***********************************************************

extern void Calculate_PID(void);

extern int16_t PID_Gyros[3];
extern int16_t PID_ACCs[3];
extern int32_t	IntegralaPitch;			// PID I-terms (acc.) for each axis
extern int32_t	IntegralaRoll;
