/*********************************************************************
 * pid.h
 ********************************************************************/

//***********************************************************
//* Externals
//***********************************************************

extern void Calculate_PID(void);
extern void Sensor_PID(uint32_t period);

extern int16_t 	PID_Gyros[FLIGHT_MODES][NUMBEROFAXIS];
extern int16_t 	PID_ACCs[FLIGHT_MODES][NUMBEROFAXIS];
extern int32_t	IntegralGyro[FLIGHT_MODES][NUMBEROFAXIS];
extern float 	gyroSmooth[NUMBEROFAXIS];					// Filtered gyro data