/*********************************************************************
 * pid.h
 ********************************************************************/

//***********************************************************
//* Externals
//***********************************************************

extern void Calculate_PID(void);
extern void Sensor_PID(uint32_t period);

extern int16_t 	PID_Gyros[NUMBEROFAXIS];
extern int16_t 	PID_ACCs[NUMBEROFAXIS];
extern int32_t	IntegralGyro[NUMBEROFAXIS];
