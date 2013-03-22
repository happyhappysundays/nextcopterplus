/*
 * servos_asm.S
 *
 * Created: 25/06/2012 10:53:00
 * Author: David Thompson - Originally based on code example from Cesco
 * Updated: 14/03/2013 - Added 16us staggered output concept borrowed from Jim Drew of Xtreme Power Systems
 * Updated: 22/03/2013 - Integrated ninja-level PWM code generation concept suggested by Jim Drew of Xtreme Power Systems
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
; void output_servo_ppm_asm(&ServoOut[0]);
;
; regs = r24,25 (&ServoOut[0])
;
; Servo inputs are 16-bit, 1000 to 2000. 1000 bits cover 1ms or 1us per step
; Lower 3 bits are encoded using them as an offset into a jump table of 1us steps
;
;*************************************************************************

	.global output_servo_ppm_asm
	.func   output_servo_ppm_asm
output_servo_ppm_asm:
// Save regs
	push	ZL			// 2			
	push	ZH			// 2
	push	XL			// 2
	push	XH			// 2
	push	16			// 2
	push	17			// 2
	push	18			// 2
	push	19			// 2
	push	20			// 2
	push	21			// 2
	push	22			// 2
	push	23			// 2

// Get start address of ServoOut[0] into Z
	mov		ZL, r24		// 1 Low byte
	mov		ZH, r25		// 1 High byte

// Save initial address for later
	push	24			// 2
	push	25			// 2

// Trim 16-bit values to suit individual needs, 
// Save result and copy lower 3 bits to r16 to r23 (96 cycles)

// M1
	ld		XL, Z+ 		// 2			Load XL with data at address in Z
	ld		XH, Z+		// 2			Load XH with data at address in Z+1
	subi	XL,	0xb3	// 1 			Trim servo pulse
	sbci 	XH,	0x00	// 1
	push	XL 			// 2			Save trimmed value to the stack
	push	XH 			// 2
	andi	XL,	0x07	// 1			And all but first 3 bits
	mov		r23,XL		// 1 (12)		Copy 8-bit result to regs
// M2
	ld		XL, Z+
	ld		XH, Z+
	subi	XL,	0x9a
	sbci 	XH,	0x00
	push	XL 		
	push	XH 		
	andi	XL,	0x07
	mov		r22,XL	
// M3
	ld		XL, Z+
	ld		XH, Z+
	subi	XL,	0x81
	sbci 	XH,	0x00
	push	XL 		
	push	XH 		
	andi	XL,	0x07
	mov		r21,XL	
// M4
	ld		XL, Z+
	ld		XH, Z+
	subi	XL,	0x68 // ????????????? 67?
	sbci 	XH,	0x00
	push	XL 	
	push	XH 	
	andi	XL,	0x07
	mov		r20,XL	
// M5
	ld		XL, Z+
	ld		XH, Z+
	subi	XL,	0x4f
	sbci 	XH,	0x00
	push	XL 	
	push	XH 	
	andi	XL,	0x07
	mov		r19,XL	
// M6
	ld		XL, Z+
	ld		XH, Z+
	subi	XL,	0x36
	sbci 	XH,	0x00
	push	XL 	
	push	XH 	
	andi	XL,	0x07
	mov		r18,XL
// M7
	ld		XL, Z+
	ld		XH, Z+
	subi	XL,	0x1d
	sbci 	XH,	0x00
	push	XL 	
	push	XH 
	andi	XL,	0x07
	mov		r17,XL
// M8
	ld		XL, Z+
	ld		XH, Z+
	subi	XL,	0x04
	sbci 	XH,	0x00
	push	XL
	push	XH 
	andi	XL,	0x07
	mov		r16,XL

// Time out lower 3 bits of pulse value (Zero to 7us)
// This front section is used to simply balance out the lower section
// Therefore, this chunk is always executed in 186 cycles or 9.30us + 15.8us = 25.1
// This does affect the inter-pulse delay, but not the pulsewidth.

// M1
	ldi ZL, lo8(OUT1H) 	// 1
	ldi ZH, hi8(OUT1H) 	// 1
	mov		r25,r23		// 1 Save a copy of r23 in r25
	call	preproc		// 16
	ijmp				// 2 (21)
OUT1H:
	call	us_delay	// 20 - 0 (7us)
	call	us_delay	// 20
	call	us_delay	// 20
	call	us_delay	// 20
	call	us_delay	// 20
	call	us_delay	// 20
	call	us_delay	// 20 (7 x 20 = 140)
	sbi 	M1			// 2  - 7 (0us)
	ldi ZL, lo8(OUT1L)	// 1
	ldi ZH, hi8(OUT1L) 	// 1
	call	postproc	// 19
	ijmp				// 2
OUT1L:
	call	us_delay	// 20 - 0 (7us)
	call	us_delay	// 20
	call	us_delay	// 20
	call	us_delay	// 20
	call	us_delay	// 20
	call	us_delay	// 20
	call	us_delay	// 20 (7 x 20 = 140)
	call	pwm_delay	// 316 (15.8us)
	rjmp .+0			// 1 Tweak onto exact us boundary
	nop

// M2
	ldi ZL, lo8(OUT2H)
	ldi ZH, hi8(OUT2H)
	mov	r25,r22
	call	preproc
	ijmp
OUT2H:
	call	us_delay
	call	us_delay
	call	us_delay
	call	us_delay
	call	us_delay
	call	us_delay
	call	us_delay
	sbi 	M2
	ldi ZL, lo8(OUT2L)
	ldi ZH, hi8(OUT2L)
	call	postproc
	ijmp
OUT2L:
	call	us_delay
	call	us_delay
	call	us_delay
	call	us_delay
	call	us_delay
	call	us_delay
	call	us_delay
	call	pwm_delay
	rjmp .+0			// 1 Tweak onto exact us boundary
	nop

// M3
	ldi ZL, lo8(OUT3H)
	ldi ZH, hi8(OUT3H)
	mov	r25,r21
	call	preproc
	ijmp
OUT3H:
	call	us_delay
	call	us_delay
	call	us_delay
	call	us_delay
	call	us_delay
	call	us_delay
	call	us_delay
	sbi 	M3
	ldi ZL, lo8(OUT3L)
	ldi ZH, hi8(OUT3L)
	call	postproc
	ijmp
OUT3L:
	call	us_delay
	call	us_delay
	call	us_delay
	call	us_delay
	call	us_delay
	call	us_delay
	call	us_delay
	call	pwm_delay
	rjmp .+0			// 1 Tweak onto exact us boundary
	nop

// M4
	ldi ZL, lo8(OUT4H)
	ldi ZH, hi8(OUT4H)
	mov	r25,r20
	call	preproc
	ijmp
OUT4H:
	call	us_delay
	call	us_delay
	call	us_delay
	call	us_delay
	call	us_delay
	call	us_delay
	call	us_delay
	sbi 	M4
	ldi ZL, lo8(OUT4L)
	ldi ZH, hi8(OUT4L)
	call	postproc
	ijmp
OUT4L:
	call	us_delay
	call	us_delay
	call	us_delay
	call	us_delay
	call	us_delay
	call	us_delay
	call	us_delay
	call	pwm_delay
	rjmp .+0			// 1 Tweak onto exact us boundary
	nop

// M5
	ldi ZL, lo8(OUT5H)
	ldi ZH, hi8(OUT5H)
	mov	r25,r19
	call	preproc
	ijmp
OUT5H:
	call	us_delay
	call	us_delay
	call	us_delay
	call	us_delay
	call	us_delay
	call	us_delay
	call	us_delay
	sbi 	M5
	ldi ZL, lo8(OUT5L)
	ldi ZH, hi8(OUT5L)
	call	postproc
	ijmp
OUT5L:
	call	us_delay
	call	us_delay
	call	us_delay
	call	us_delay
	call	us_delay
	call	us_delay
	call	us_delay
	call	pwm_delay
	rjmp .+0			// 1 Tweak onto exact us boundary
	nop

// M6
	ldi ZL, lo8(OUT6H)
	ldi ZH, hi8(OUT6H)
	mov	r25,r18
	call	preproc
	ijmp
OUT6H:
	call	us_delay
	call	us_delay
	call	us_delay
	call	us_delay
	call	us_delay
	call	us_delay
	call	us_delay
	sbi 	M6
	ldi ZL, lo8(OUT6L)
	ldi ZH, hi8(OUT6L)
	call	postproc
	ijmp
OUT6L:
	call	us_delay
	call	us_delay
	call	us_delay
	call	us_delay
	call	us_delay
	call	us_delay
	call	us_delay
	call	pwm_delay
	rjmp .+0			// 1 Tweak onto exact us boundary
	nop

// M7
	ldi ZL, lo8(OUT7H)
	ldi ZH, hi8(OUT7H)
	mov	r25,r17
	call	preproc
	ijmp
OUT7H:
	call	us_delay
	call	us_delay
	call	us_delay
	call	us_delay
	call	us_delay
	call	us_delay
	call	us_delay
	sbi 	M7
	ldi ZL, lo8(OUT7L)
	ldi ZH, hi8(OUT7L)
	call	postproc
	ijmp
OUT7L:
	call	us_delay
	call	us_delay
	call	us_delay
	call	us_delay
	call	us_delay
	call	us_delay
	call	us_delay
	call	pwm_delay
	rjmp .+0			// 2
	nop					// 1 Tweak onto exact us boundary

// M8
	ldi ZL, lo8(OUT8H)
	ldi ZH, hi8(OUT8H)
	mov	r25,r16
	call	preproc
	ijmp
OUT8H:
	call	us_delay
	call	us_delay
	call	us_delay
	call	us_delay
	call	us_delay
	call	us_delay
	call	us_delay
	sbi 	M8
	ldi ZL, lo8(OUT8L)
	ldi ZH, hi8(OUT8L)
	call	postproc
	ijmp
OUT8L:
	call	us_delay
	call	us_delay
	call	us_delay
	call	us_delay
	call	us_delay
	call	us_delay
	call	us_delay
	rjmp .+0			// 2
	rjmp .+0			// 2
	nop					// 1 Tweak onto exact us boundary

// Copy trimmed array data into regs
// This now represents steps of 8us chunks
	pop		XH			// 2
	pop		XL			// 2
	call	shuffle_8	// 10
	mov		r16,XL		// 1 (15)
	pop		XH
	pop		XL
	call	shuffle_8
	mov		r17,XL
	pop		XH
	pop		XL
	call	shuffle_8
	mov		r18,XL
	pop		XH
	pop		XL
	call	shuffle_8
	mov		r19,XL
	pop		XH
	pop		XL
	call	shuffle_8
	mov		r20,XL
	pop		XH
	pop		XL
	call	shuffle_8
	mov		r21,XL
	pop		XH
	pop		XL
	call	shuffle_8
	mov		r22,XL
	pop		XH
	pop		XL
	call	shuffle_8
	mov		r23,XL

// Loop prep
	push	YL			// 2
	push	YH			// 2

	ldi 	YL,0x13		// 1			Set loop count to 113h or 275 (275 * 8us = 2200us)
	ldi		YH,0x01		// 1
	clr		0			// 1 (7)

// Ninja loop - 7 cycles each side (8 x 7 = 56 + 6 to loop + 98 pad = 160 cycles)
// 20 cyles = 1us, 160 cycles = 8us

nloop:
	subi	r23,1		// 1 			ServoOut1
	brne	j1			// 2	1	
	cbi 	M1			// 		2		Clear output M1 if done (+5 = 1448 = 72.4us = 48)
	jmp		jj1			//		3
j1:
	rjmp 	.+0			// 2
	rjmp 	.+0			// 2
jj1:
	subi	r22,1		// 1			ServoOut2
	brne	j2			// 2	1
	cbi 	M2			// 		2		Clear output M2 if done (+11 = 1285 = 64.25us = 40)
	jmp		jj2			//		3
j2:
	rjmp	.+0
	rjmp	.+0
jj2:
	subi	r21,1
	brne	j3
	cbi 	M3
	jmp		jj3
j3:
	rjmp	.+0
	rjmp	.+0
jj3:
	subi 	r20,1
	brne	j4
	cbi 	M4
	jmp		jj4	
j4:
	rjmp 	.+0
	rjmp 	.+0
jj4:
	subi	r19,1
	brne	j5
	cbi 	M5
	jmp		jj5
j5:
	rjmp 	.+0
	rjmp 	.+0
jj5:
	subi	r18,1
	brne	j6
	cbi 	M6
	jmp		jj6
j6:
	rjmp 	.+0
	rjmp	.+0
jj6:
	subi	r17,1
	brne	j7
	cbi 	M7
	jmp		jj7
j7:
	rjmp 	.+0
	rjmp 	.+0
jj7:
	subi 	r16,1
	brne	j8
	cbi 	M8
	jmp		jj8
j8:
	rjmp 	.+0
	rjmp 	.+0
jj8:
	call	pad_100_delay // (100)

	subi 	YL,1		// 1
	sbc 	YH,0		// 1
	breq	skiploop	// 1	2		Loop until zero 1 false 2 true
	jmp		nloop		// 3			Can't reach nloop with a brne alone

skiploop:
	pop		YH			//		2		Restore regs
	pop		YL			//		2
	pop		25			// 		2
	pop		24			// 		2
	pop		23			//		2
	pop		22			//		2
	pop		21			//		2
	pop		20			//		2
	pop		19			//		2
	pop		18			//		2
	pop		17			//		2
	pop		16			//		2
	pop		XH			//		2
	pop		XL			//		2
	pop		ZH			// 		2
	pop		ZL			//		2

	ret					//		4
	.endfunc

;*************************************************************************	
; void output_servo_ppm_asm3(servo_number, value);
; regs = r24,r25 (servo_number), r22,23 (value)
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
						// <---------	Left column is skip execution path
	rjmp .+0			// 2 	<----	Right column is pulse end execution path
	rjmp .+0			// 2
	rjmp .+0			// 2
	rjmp .+0			// 2
	rjmp .+0			// 2
	rjmp .+0			// 2
	rjmp .+0			// 2			16 cycles

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
	nop					// 1 5 * 50ns = 250ns = 4MHz
	ret					// 4 (5 cycles)
	.endfunc


;*************************************************************************	
; void pwm_delay(void) 16us output spacing delay
;*************************************************************************

pwm_delay:
	push	16			// 2			Save regs
	ldi 	16,0x22		// 1			Set loop count to 22h or 34

pwm_loop:				//				9 cycles, 8 on last
	rjmp .+0			// 2
	rjmp .+0			// 2			33 x 9 + 8 = 305
	rjmp .+0			// 2

	subi 	16,1		// 1
	brne	pwm_loop	// 2	1		Loop until zero	

	pop		16			// 		2		Restore regs
	ret					// 		4 (314 cycles)

;*************************************************************************	
; pad_100_delay 98 cycle padding delay
; Takes 4 to call, so we need 94 cycles burnt here
;*************************************************************************

pad_100_delay:
	push	16			// 		2		Save regs
	ldi 	16,0x09		// 		1		Set loop count to 09h or 9

pad_100:				//				9 cycles, 8 on last
	rjmp .+0			// 2
	rjmp .+0			// 2			8 x 9cyc + 8 = 80
	rjmp .+0			// 2

	subi 	16,1		// 1
	brne	pad_100		// 2	1		Loop until zero	

	nop					// 		1		Trim to precisely the right value
	rjmp .+0			// 		2
	rjmp .+0			// 		2

	pop		16			// 		2		Restore regs
	ret					// 		4 (94 cycles)

;*************************************************************************	
; void us_delay(void) 1us delay for ninja skills
; Takes 4 to call, so we need 16 cycles burnt here
;*************************************************************************

us_delay:
	rjmp .+0			// 2
	rjmp .+0			// 2
	rjmp .+0			// 2
	rjmp .+0			// 2
	rjmp .+0			// 2
	rjmp .+0			// 2
	ret					// 4 (16 cycles)

;*************************************************************************	
; Space-saving macro-ish suroutines
;*************************************************************************

shuffle_8: // (10 cycles)
	lsr		XH			// 1 Rotate right into carry
	ror		XL			// 1 Rotate right with carry from MSB
	lsr		XH			// 1
	ror		XL			// 1
	lsr		XH			// 1
	ror		XL			// 1
	ret					// 4

preproc: // (12 cycles)
	mov		YL,r25		// 1 Save a copy of r23 in YL
	lsr		ZH			// 1 Align with byte boundary from word address
	ror		ZL			// 1 
	clc					// 1
	lsl		YL			// 1 Multiply by 2 as the call table is 4 bytes per entry
	add 	ZL, YL 		// 1 Add offset to ZL
	ldi		r24,0x00	// 1 
	adc 	ZH, r24 	// 1 Roll up carry to ZH
	ret					// 4

postproc: // (15 cycles)
	mov		YL, r25		// 1 Recover r23 (the offset)
	ldi		r25,0x07	// 1
	eor		YL,r25		// 1 Reverse number
	andi	YL,0x07		// 1
	lsl		YL			// 1 Multiply by 2 as the call table is 2 bytes per entry
	lsr		ZH			// 1 Align with byte boundary
	ror		ZL			// 1 ...dammit
	clc					// 1
	add 	ZL, YL 		// 1
	ldi		r24,0x00	// 1 
	adc 	ZH, r24 	// 1 Roll up carry to ZH
	ret					// 4
