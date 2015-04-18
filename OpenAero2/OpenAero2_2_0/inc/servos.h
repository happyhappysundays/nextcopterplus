/*********************************************************************
 * servos.h
 ********************************************************************/

//***********************************************************
//* Externals
//***********************************************************

extern void output_servo_ppm(uint8_t ServoFlag);
extern volatile uint16_t ServoOut[MAX_OUTPUTS];
extern void bind_master(void);
extern void output_servo_ppm_asm(volatile uint16_t *ServoOut, uint8_t ServoFlag);
