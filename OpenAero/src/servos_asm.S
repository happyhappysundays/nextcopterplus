/*
 * servos_asm.S
 *
 * Created: 16/04/2012 09:45:00
 * Author: David Thompson - based on code example from Cesco
 *
 */

#include <avr/io.h>
#include "..\inc\compiledefs.h"

// Servo output pin assignments
#define M123_OUT	_SFR_IO_ADDR(PORTB)
#define M456_OUT	_SFR_IO_ADDR(PORTD)

#ifndef N6_MODE
#define M1 M123_OUT,2	// PORTB,2
#define M2 M123_OUT,1	// PORTB,1
#define M4 M456_OUT,7	// PORTD,7
#define M5 M456_OUT,6	// PORTD,6
#define M6 M456_OUT,5	// PORTD,5
#define THR M456_OUT,3	// PORTD,3 (former THR input)
#else // N6 mode
#define M2 M123_OUT,1	// PORTB,1
#define M3 M123_OUT,2	// PORTB,2
#define M4 M123_OUT,3	// PORTB,3
#define M5 M123_OUT,4	// PORTB,4
#define M6 M123_OUT,5	// PORTB,5
#define THR M456_OUT,3	// PORTD,3 (former THR input)
#endif

#ifndef __tmp_reg__
#define __tmp_reg__ 0
#endif

	.section .text

#ifdef THREE_CHANNEL

;*************************************************************************	
; void output_servo_ppm_asm(ServoOut1, ServoOut2, ServoOut4);
; regs = r24,25 (ServoOut1), r22,23 (ServoOut2), r20,21 (ServoOut4)
;
; Servo inputs are 16-bit, 500 to 1000. 500 bits cover 1ms or 2us per step
;
; Variable loop cycles (500) = (16 * 500) / 8 = 1.00ms
; Variable loop cycles (1000)= (16 * 1000) / 8 = 2.00ms
;
;*************************************************************************

	.global output_servo_ppm_asm
	.func   output_servo_ppm_asm
output_servo_ppm_asm:
	push	16			// 2 <---------	Left column is skip execution path
	push	17			// 2 	<----	Right column is pulse end execution path

	ldi 	16,0x12		// 1			Set loop count to 412h or 1042
	ldi		17,0x04		// 1
	clr		0			// 1

#ifndef N6_MODE
	sbi 	M1			// 2			Set all outputs
#else
	sbi 	M3			// 2
#endif
	sbi 	M2			// 2
	sbi 	M4			// 2

f0:
	subi	r24,1		// 1 			ServoOut1
	sbc 	r25,0		// 1
	brne	f1			// 2	1
#ifndef N6_MODE
	cbi 	M1			// 		2		Clear output M1 if done
#else
	cbi 	M3			// 		2		Clear output M3 if done
#endif
f1:
	subi	r22,1		// 1			ServoOut2
	sbc 	r23,0		// 1
	brne	f2			// 2	1
	cbi 	M2			// 		2		Clear output M2 if done
f2:
	subi	r20,1		// 1			ServoOut4
	sbc 	r21,0		// 1
	brne	f3			// 2	1
	cbi 	M4			// 		2		Clear output M4 if done
f3:
	subi 	16,1		// 1
	sbc 	17,0		// 1
	brne	f0			// 2	1		Loop until zero

	pop		17			// 		2		Restore regs
	pop		16			//		2
	ret					//		4
	.endfunc	

#elif defined(SIX_CHANNEL)

;*************************************************************************	
; void output_servo_ppm_asm(Servo1, Servo2, Servo4, Servo5, Servo6, Servo_thr);
; regs = r24 (Servo1), r22 (Servo2), r20 (Servo4), r18(Servo5), r16 (Servo6), r14(Servo_thr)
;
; Servo inputs are 8-bit, 0 to 250. 256 bits cover 1ms or 4us per step
;
; Fixed loop cycles			= (3 + (7  * 1142)) / 8 = 999.6us
; Variable loop cycles (0)	= (3 + (32 * 1)) / 8 = 4.3us (worst case)
; Variable loop cycles (250)= (3 + (32 * 250)) / 8 = 1.00ms
;
;*************************************************************************

	.global output_servo_ppm_asm
	.func   output_servo_ppm_asm
output_servo_ppm_asm:
	push	19			// 2 <---------	Left column is skip execution path
	push	17			// 2 	<----	Right column is pulse end execution path
	push	23			// 2			

	// Copy r14 to r23 so that subi can be used
	mov		r23,r14

#ifndef N6_MODE
	sbi 	M1			// 2			Set all outputs
#else
	sbi 	M3			// 2
#endif
	sbi 	M2			// 2
	sbi 	M4			// 2
	sbi 	M5			// 2
//	sbi 	M6			// 2
#ifdef ICP_CPPM_MODE
	sbi		THR			// 2
#else
	nop
	nop
#endif

	// Create minimum pulse of 1000us
minpulse:
	ldi 	19,0x76		// 1			Set loop count to 476h or 1142d
	ldi		17,0x04		// 1
	clr		0			// 1
looppulse:
	nop					// 1
	nop					// 1
	nop					// 1
	subi 	19,1		// 1
	sbc 	17,0		// 1
	brne	looppulse	// 2	1		Loop until zero

	// Start variable part of pulse
varpulse:				
	ldi 	19,0xD7		// 1			Set loop count to 3D7h or 983
	ldi		17,0x03		// 1
	clr		0			// 1

f0:
	nop					// 1
	nop					// 1
	subi	r24,1		// 1 			Servo1
	brne	f1			// 2	1
#ifndef N6_MODE
	cbi 	M1			// 		2		Clear output M1 if done
#else
	cbi 	M3			// 		2		Clear output M3 if done
#endif
f1:
	nop					// 1
	nop					// 1
	subi	r22,1		// 1			Servo2
	brne	f2			// 2	1
	cbi 	M2			// 		2		Clear output M2 if done
f2:
	nop					// 1
	nop					// 1
	subi	r20,1		// 1			Servo4
	brne	f3			// 2	1
	cbi 	M4			// 		2		Clear output M4 if done
f3:
	nop					// 1
	nop					// 1
	subi	r18,1		// 1 			Servo5
	brne	f4			// 2	1
	cbi 	M5			// 		2		Clear output M5 if done
f4:
	nop					// 1
	subi	r16,1		// 1			Servo6
	brne	f5			// 2	1
//	cbi 	M6			// 		2		Clear output M6 if done
	nop
	nop
//
f5:
	nop					// 1
	subi	r23,1		// 1			Throttle
	brne	f6			// 2	1
#ifdef ICP_CPPM_MODE
	cbi 	THR			// 		2		Clear output THR if done
#else
	nop
	nop
#endif

f6:
	subi 	19,1		// 1
	sbc 	17,0		// 1
	brne	f0			// 2	1		Loop until zero

	pop		23			//		2
	pop		17			// 		2		Restore regs
	pop		19			//		2
	ret					//		4
	.endfunc

#endif
