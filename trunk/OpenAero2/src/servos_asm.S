/*
 * servos_asm.S
 *
 * Created: 25/06/2012 10:53:00
 * Author: David Thompson - based on code example from Cesco
 * Updated: 14/03/2013 - Added 16us staggered output concept borrowed from Jim Drew of Xtreme Power Systems
 *
 */

#include <avr/io.h>

// Servo output pin assignments
#define SERVO_OUT	_SFR_IO_ADDR(PORTC)

#define M1 SERVO_OUT,6	// PORTC,6
#define M2 SERVO_OUT,4	// PORTC,4
#define M3 SERVO_OUT,2	// PORTC,2
#define M4 SERVO_OUT,3	// PORTC,3
#define M5 SERVO_OUT,1	// PORTC,1
#define M6 SERVO_OUT,0	// PORTC,0
#define M7 SERVO_OUT,5	// PORTC,5
#define M8 SERVO_OUT,7	// PORTC,7

#ifndef __tmp_reg__
#define __tmp_reg__ 0
#endif

	.section .text

;*************************************************************************	
; void output_servo_ppm_asm1(ServoOut1, ServoOut2, ServoOut3, ServoOut4);
;
; regs = r24,25 (ServoOut1), r22,23 (ServoOut2), r20,21 (ServoOut3), r18,19 (ServoOut4)
;
; Servo inputs are 16-bit, 1000 to 2000. 1000 bits cover 1ms or 1us per step
;
; Variable loop cycles (1000) = (20 * 1000) / 20MHz = 1.00ms
; Variable loop cycles (2000) = (20 * 2000) / 20MHz = 2.00ms
;
;*************************************************************************

	.global output_servo_ppm_asm1
	.func   output_servo_ppm_asm1
output_servo_ppm_asm1:
	push	16			// 2 <---------	Left column is skip execution path
	push	17			// 2 	<----	Right column is pulse end execution path

	ldi 	16,0x34		// 1			Set loop count to 834h or 2100
	ldi		17,0x08		// 1
	clr		0			// 1

	subi	r24,0x30	// 1 			Trim OUT5 pulsewidth to suit staggering 980/20 - 1 = 48 = 0x30
	sbci 	r25,0x00	// 1
	subi	r22,0x20	// 1 			Trim OUT6 pulsewidth to suit staggering 660/20 - 1 = 32 = 0x20
	sbci 	r23,0x00	// 1
	subi	r20,0x10	// 1 			Trim OUT7 pulsewidth to suit staggering 340/20 - 1 = 16 = 0x10
	sbci 	r21,0x00	// 1
	subi	r18,0x00	// 1 			Trim OUT8 pulsewidth to suit staggering  20/20 - 1 =  0 = 0x00
	sbci 	r19,0x00	// 1

	sbi 	M1			// 2			OUT1 timing measured from here (980)	
	call	pwm_delay	// 318		
	nop					// 1			Balance offset in loop
	nop					// 1
	nop					// 1
	nop					// 1	
	sbi 	M2			// 2			OUT2 timing measured from here (660)
	call	pwm_delay	// 318
	nop					// 1			Balance offset in loop
	nop					// 1
	nop					// 1
	nop					// 1
	sbi 	M3			// 2			OUT3 timing measured from here (340)
	call	pwm_delay	// 318
	nop					// 1			Balance offset in loop
	nop					// 1
	nop					// 1
	nop					// 1
	sbi 	M4			// 2			OUT4 timing measured from here (20)

	nop					// 1			3 cycles to round offsets to multiples of 20
	nop					// 1
	nop					// 1			Measure up from here...

loop1:
	subi	r24,1		// 1 			ServoOut1
	sbc 	r25,0		// 1
	brne	f1			// 2	1
	cbi 	M1			// 		2		Clear output M1 if done (Minimum of +5 cycles to here)
f1:
	subi	r22,1		// 1			ServoOut2
	sbc 	r23,0		// 1
	brne	f2			// 2	1
	cbi 	M2			// 		2		Clear output M2 if done (Minimum of +9 cycles to here)
f2:
	subi	r20,1		// 1			ServoOut3
	sbc 	r21,0		// 1
	brne	f3			// 2	1
	cbi 	M3			// 		2		Clear output M3 if done (Minimum of +13 cycles to here)
f3:
	subi 	r18,1		// 1			ServoOut4
	sbc 	r19,0		// 1
	brne	f4			// 2	1
	cbi 	M4			// 		2		Clear output M4 if done (Minimum of +17 cycles to here)
f4:
	subi 	16,1		// 1
	sbc 	17,0		// 1
	brne	loop1		// 2	1		Loop until zero

	pop		17			// 		2		Restore regs
	pop		16			//		2
	ret					//		4
	.endfunc


;*************************************************************************	
; void output_servo_ppm_asm2(ServoOut5, ServoOut6, ServoOut7, ServoOut8);
;
; regs = r24,25 (ServoOut5), r22,23 (ServoOut6), r20,21 (ServoOut7), r18,19 (ServoOut8)
;
; Servo inputs are 16-bit, 1000 to 2000. 1000 bits cover 1ms or 1us per step
;
; Variable loop cycles (1000) = (20 * 1000) / 20MHz = 1.00ms
; Variable loop cycles (2000) = (20 * 2000) / 20MHz = 2.00ms
;
;*************************************************************************

	.global output_servo_ppm_asm2
	.func   output_servo_ppm_asm2
output_servo_ppm_asm2:
	push	16			// 2 <---------	Left column is skip execution path
	push	17			// 2 	<----	Right column is pulse end execution path

	ldi 	16,0x34		// 1			Set loop count to 834h or 2100
	ldi		17,0x08		// 1
	clr		0			// 1

	subi	r24,0x30	// 1 			Trim OUT5 pulsewidth to suit staggering 980/20 - 1 = 48 = 0x30
	sbci 	r25,0x00	// 1
	subi	r22,0x20	// 1 			Trim OUT6 pulsewidth to suit staggering 660/20 - 1 = 32 = 0x20
	sbci 	r23,0x00	// 1
	subi	r20,0x10	// 1 			Trim OUT7 pulsewidth to suit staggering 340/20 - 1 = 16 = 0x10
	sbci 	r21,0x00	// 1
	subi	r18,0x00	// 1 			Trim OUT8 pulsewidth to suit staggering  20/20 - 1 =  0 = 0x00
	sbci 	r19,0x00	// 1

	sbi 	M5			// 2			OUT5 timing measured from here (980)	
	call	pwm_delay	// 318		
	nop					// 1			Balance offset in loop
	nop					// 1
	nop					// 1
	nop					// 1	
	sbi 	M6			// 2			OUT6 timing measured from here (660)
	call	pwm_delay	// 318
	nop					// 1			Balance offset in loop
	nop					// 1
	nop					// 1
	nop					// 1
	sbi 	M7			// 2			OUT7 timing measured from here (340)
	call	pwm_delay	// 318
	nop					// 1			Balance offset in loop
	nop					// 1
	nop					// 1
	nop					// 1
	sbi 	M8			// 2			OUT8 timing measured from here (20)

	nop					// 1			3 cycles to round offsets to multiples of 20
	nop					// 1
	nop					// 1			Measure up from here...

loop2:
	subi	r24,1		// 1 			ServoOut5
	sbc 	r25,0		// 1
	brne	f5			// 2	1
	cbi 	M5			// 		2		Clear output M5 if done (Minimum of +5 cycles to here)
f5:
	subi	r22,1		// 1			ServoOut6
	sbc 	r23,0		// 1
	brne	f6			// 2	1
	cbi 	M6			// 		2		Clear output M6 if done (Minimum of +9 cycles to here)
f6:
	subi	r20,1		// 1			ServoOut7
	sbc 	r21,0		// 1
	brne	f7			// 2	1
	cbi 	M7			// 		2		Clear output M7 if done (Minimum of +13 cycles to here)
f7:
	subi 	r18,1		// 1			ServoOut8
	sbc 	r19,0		// 1
	brne	f8			// 2	1
	cbi 	M8			// 		2		Clear output M8 if done (Minimum of +17 cycles to here)
f8:
	subi 	16,1		// 1
	sbc 	17,0		// 1
	brne	loop2		// 2	1		Loop until zero

	pop		17			// 		2		Restore regs
	pop		16			//		2
	ret					//		4
	.endfunc	

;*************************************************************************	
; void output_servo_ppm_asm3(servo_number, value);
;
; regs = r24,r25 (servo_number), r22,23 (value)
;
;*************************************************************************

	.global output_servo_ppm_asm3
	.func   output_servo_ppm_asm3
output_servo_ppm_asm3:
	push	16			// Save regs
	push	17
	push	18

	ldi 	16,0x34		// 1			Set loop count to 834h or 2100
	ldi		17,0x08	
	clr		0

jump_table:
	cpi		r24,0x00	; 0				
	breq	in1
	cpi		r24,0x01	; 1
	breq	in2
	cpi		r24,0x02	; 2
	breq	in3
	cpi		r24,0x03	; 3
	breq	in4
	cpi		r24,0x04	; 4
	breq	in5
	cpi		r24,0x05	; 5
	breq	in6
	cpi		r24,0x06	; 6
	breq	in7
	cpi		r24,0x07	; 7
	breq	in8

in1:sbi 	M1			// Set output high
	jmp		loopin
in2:sbi 	M2
	jmp		loopin
in3:sbi 	M3	
	jmp		loopin
in4:sbi 	M4
	jmp		loopin
in5:sbi 	M5
	jmp		loopin
in6:sbi 	M6
	jmp		loopin
in7:sbi 	M7
	jmp		loopin
in8:sbi 	M8
	jmp		loopin

// Loop - 20 cycles = 1us
loopin:
	nop					// 1 <---------	Left column is skip execution path
	nop					// 1 	<----	Right column is pulse end execution path
	nop					// 1
	nop					// 1

	nop					// 1
	nop					// 1
	nop					// 1
	nop					// 1

	nop					// 1
	nop					// 1
	nop					// 1
	nop					// 1			16 cycles

	subi	r22,1		// 1 			Time servo pulse
	sbc 	r23,0		// 1
	brne	loopout		// 2	1

// Clear outputs if done
	ldi		r18,0x00	 // 1
	out 	SERVO_OUT,r18// 1			Boom.

// Exit
loopout:
	subi 	16,1		// 1			+4 cycles = total 20 cycles
	sbc 	17,0		// 1
	brne	loopin		// 2	1		Loop until zero

	pop		18
	pop		17			// Restore regs
	pop		16
	ret	
	.endfunc	

;*************************************************************************	
; void glcd_delay(void) 250ns delay for glcd clock
;*************************************************************************

	.global glcd_delay
	.func   glcd_delay
glcd_delay:
	nop					// 1 		5 * 50ns = 250ns = 4MHz
	ret					// 4
	.endfunc


;*************************************************************************	
; void pwm_delay(void) 16us output spacing delay
;*************************************************************************

	.global pwm_delay
	.func   pwm_delay
pwm_delay:
	push	16			// 2			Save regs
	ldi 	16,0x22		// 1			Set loop count to 22h or 34

pwm_loop:				//				9 cycles, 8 on last
	nop					// 1
	nop					// 1
	nop					// 1
	nop					// 1			33 x 9 + 8 = 305
	nop					// 1
	nop					// 1

	subi 	16,1		// 1
	brne	pwm_loop	// 2	1		Loop until zero	

	pop		16			// 		2		Restore regs
	ret					// 		4
	.endfunc
