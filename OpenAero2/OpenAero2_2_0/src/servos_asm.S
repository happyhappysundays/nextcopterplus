/*
 * servos_asm.S
 *
 * Created: 25/06/2012 10:53:00
 * Author: David Thompson - Originally based on code example from Cesco
 * Updated: 14/03/2013 - Added 16us staggered output concept borrowed from Jim Drew of Xtreme Power Systems
 * Updated: 25/03/2013 - Integrated ninja-level PWM code generation concept suggested by Jim Drew of Xtreme Power Systems
 * Updated: 11/05/2013 - Discovered that M1 incapable of <900us and M8 of >2.04ms due to 128us stagger limitations.
 *						 Reduced stagger to 100us and recalibrated all offsets.
 * Updated: 30/11/2013 - Now compatible with KK2.1 board
 * Updated: 22/12/2014 - Added conditional output based on the ServoFlag. Re-tweaked some of the output timing.
 * Updated: 18/01/2015 - Removed stagger and reduced max loop to speed up PWM generation time
 */

#include <avr/io.h>
#include "compiledefs.h"

// Servo output pin assignments
#define SERVO_OUT_KK20	_SFR_IO_ADDR(PORTC)
#define SERVO_OUT_KK21	_SFR_IO_ADDR(PORTA)

#define M1 SERVO_OUT_KK20,6	// PORTC,6
#define M2 SERVO_OUT_KK20,4	// PORTC,4
#define M3 SERVO_OUT_KK20,2	// PORTC,2
#define M4 SERVO_OUT_KK20,3	// PORTC,3

#ifdef KK21
#define M5 SERVO_OUT_KK21,4	// PORTA,4
#define M6 SERVO_OUT_KK21,5	// PORTA,5
#else
#define M5 SERVO_OUT_KK20,1	// PORTC,1
#define M6 SERVO_OUT_KK20,0	// PORTC,0
#endif

#define M7 SERVO_OUT_KK20,5	// PORTC,5
#define M8 SERVO_OUT_KK20,7	// PORTC,7

#ifndef __tmp_reg__
#define __tmp_reg__ 0
#endif

	.section .text

;*************************************************************************	
; void output_servo_ppm_asm(&ServoOut[0], ServoFlag);
;
; regs = r24,25 (&ServoOut[0]), r22 (ServoFlag)
;
; Servo inputs are 16-bit, 1000 to 2000. 1000 bits cover 1ms or 1us per step
; Lower 3 bits are encoded using them as an offset into a jump table of 1us steps
; Pulses can be generated from below 900us to at least 2100us but unpredictable results
; can and will occur beyond this upper limit.
;
; ServoFlag is used to suppress an output if that output's bit is cleared.
;
;*************************************************************************

	.global output_servo_ppm_asm
	.func   output_servo_ppm_asm
output_servo_ppm_asm:
// Copy r22 (containing ServoFlag) into r15
	mov		r15, r22	
// Save regs
	push	ZL			// 2			
	push	ZH			// 2
	push	XL			// 2
	push	XH			// 2
	push	YL			// 2
	push	YH			// 2
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
	subi	XL,	0xc6	// 1 			Trim servo pulse (was 34a)
	sbci 	XH,	0x00	// 1
	push	XL 			// 2			Save trimmed value to the stack
	push	XH 			// 2
	andi	XL,	0x07	// 1			And all but first 3 bits
	mov		r23,XL		// 1 (12)		Copy 8-bit result to regs
// M2
	ld		XL, Z+
	ld		XH, Z+
	subi	XL,	0xbe  // was 2e6
	sbci 	XH,	0x00
	push	XL 		
	push	XH 		
	andi	XL,	0x07
	mov		r22,XL	
// M3
	ld		XL, Z+
	ld		XH, Z+
	subi	XL,	0xb6 // was 282
	sbci 	XH,	0x00
	push	XL 		
	push	XH 		
	andi	XL,	0x07
	mov		r21,XL	
// M4
	ld		XL, Z+
	ld		XH, Z+
	subi	XL,	0xae // was 21e
	sbci 	XH,	0x00
	push	XL 	
	push	XH 	
	andi	XL,	0x07
	mov		r20,XL	
// M5
	ld		XL, Z+
	ld		XH, Z+
	subi	XL,	0xa5 // was 1ba
	sbci 	XH,	0x00
	push	XL 	
	push	XH 	
	andi	XL,	0x07
	mov		r19,XL	
// M6
	ld		XL, Z+
	ld		XH, Z+
	subi	XL,	0x9D // was 155
	sbci 	XH,	0x00
	push	XL 	
	push	XH 	
	andi	XL,	0x07
	mov		r18,XL
// M7
	ld		XL, Z+
	ld		XH, Z+
	subi	XL,	0x95 // was f1
	sbci 	XH,	0x00
	push	XL 	
	push	XH 
	andi	XL,	0x07
	mov		r17,XL
// M8
	ld		XL, Z+
	ld		XH, Z+
	subi	XL,	0x8d // Was 8d
	sbci 	XH,	0x00
	push	XL
	push	XH 
	andi	XL,	0x07
	mov		r16,XL

// Time out lower 3 bits of pulse value (Zero to 7us)
// Each chunk is always executed in 1+10+1+147 = 159 cycles or 7.95us
// This does affect the inter-pulse delay, but not the pulsewidth.

// M1
	mov		r25,r23		// 1
	call	DelayHigh	// 10 (where r25 = 7)
	sbrc	r15,0		// 1 (False),	2 if true
	sbi 	M1			// 1,			0
	call	DelayLow	// 7 + 140 (where r25 = 0)
// M2
	mov		r25,r22
	call	DelayHigh
	sbrc	r15,1		// Don't switch on if if ServoFlag bit is cleared
	sbi 	M2
	call	DelayLow
// M3
	mov		r25,r21
	call	DelayHigh
	sbrc	r15,2		// Don't switch on if if ServoFlag bit is cleared
	sbi 	M3
	call	DelayLow
// M4
	mov		r25,r20
	call	DelayHigh
	sbrc	r15,3		// Don't switch on if if ServoFlag bit is cleared
	sbi 	M4
	call	DelayLow
// M5
	mov		r25,r19
	call	DelayHigh
	sbrc	r15,4		// Don't switch on if if ServoFlag bit is cleared
	sbi 	M5
	call	DelayLow
// M6
	mov		r25,r18
	call	DelayHigh
	sbrc	r15,5		// Don't switch on if if ServoFlag bit is cleared
	sbi 	M6
	call	DelayLow
// M7
	mov		r25,r17
	call	DelayHigh
	sbrc	r15,6		// Don't switch on if if ServoFlag bit is cleared
	sbi 	M7
	call	DelayLow
// M8
	mov		r25,r16
	call	DelayHigh
	sbrc	r15,7		// Don't switch on if if ServoFlag bit is cleared
	sbi 	M8
	call	DelayLow

// At this point, the largest pulse that we can generate is 256 * 8 = 2.048ms
// However we would like to be able to generate up to 2.100ms (150%) - an extra 52us
// So we can fluff it out another 200us so that we can subtract enough to keep r16 under 0xff or 256 :)
// With this setup, M1 can go down to 850 and M8 can go up to 2150

	call    pwm_delay   // 150us
	call    pwm_delay
	call    pwm_delay

// Copy trimmed array data into regs
// This now represents steps of 8us chunks
	pop		XH			// M8
	pop		XL
	call	shuffle_8
	mov		r16,XL
	pop		XH			// M7
	pop		XL
	call	shuffle_8
	mov		r17,XL
	pop		XH			// M6
	pop		XL
	call	shuffle_8
	mov		r18,XL
	pop		XH			// M5
	pop		XL
	call	shuffle_8
	mov		r19,XL
	pop		XH			// M4
	pop		XL
	call	shuffle_8
	mov		r20,XL
	pop		XH			// M3
	pop		XL
	call	shuffle_8
	mov		r21,XL
	pop		XH			// M2
	pop		XL
	call	shuffle_8
	mov		r22,XL
	pop		XH			// M1
	pop		XL
	call	shuffle_8
	mov		r23,XL

// Loop prep
	ldi 	YL,0xFF		// 1			Set loop count to ffh or 255 (255 * 8us = 2040us + 150us = 2190us)
	ldi		YH,0x00		// 1
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
	subi 	r16,1		// ServoOut 8
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
	breq	skiploop	// 1	2		Loop until zero - 1 false 2 true
	jmp		nloop		// 3			Can't reach nloop with a brne alone

skiploop:
	pop		25			// 		2		Restore regs
	pop		24			// 		2
	pop		23			//		2
	pop		22			//		2
	pop		21			//		2
	pop		20			//		2
	pop		19			//		2
	pop		18			//		2
	pop		17			//		2
	pop		16			//		2
	pop		YH			//		2
	pop		YL			//		2
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

	ldi 	16,0xFC		// 1			Set loop count to 8FCh or 2300(us)
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

	subi	r22,1		// 1 			Time servo pulse
	sbc 	r23,0		// 1
	brne	loopout		// 2	1

// Clear outputs if done
	ldi		r18,0x00	 // 1
	out 	SERVO_OUT_KK20,r18// 1			Boom.
#ifdef KK21
	out 	SERVO_OUT_KK21,r18// 1			KK2.1 has a couple of outputs on Port A also.
#endif

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
; void pwm_delay(void) 50us output spacing delay (8 cycle loop - 400ns)
;*************************************************************************

pwm_delay:				// 4 to call
	push	16			// 1
	ldi 	16,0x65		// 1

pwm_loop:
	rjmp .+0			// 2
	rjmp .+0			// 2
	rjmp .+0			// 2
	subi	r16,1		// 1
	brne	pwm_loop	// 2  1

pwm_exit:
	pop		16			//    1
	ret					//    4 to return

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
; Takes 4 to call, 4 to return, so we need 12 cycles burnt here
;*************************************************************************

us_delay:
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

;*************************************************************************	
; Lowest 8us timing IN - 10 cycles minimum and 20 for each 1us in r25
; 
; This part of the code delays the turn-on of the PWM pulse from between
; 10 and 150 cycles at 1us intervals per value in r26
; 
;*************************************************************************

DelayHigh:				// (0)	(1)	(2+) each extra is +20
	ldi		YL,0x07		// 1	1	1
	andi	r25,0x07	
	eor		YL,r25		// 1 	1	1
	breq	LoopHighOut1// 2	1	1
LoopHigh:
	call    us_delay	// 		16	16
	dec 	YL			// 		1	1
	breq	LoopHighOut2// 		2	1/2
	rjmp	LoopHigh	// 			2
LoopHighOut2:
	rjmp .+0			// 		2	2
LoopHighOut1:
	nop					// 1	1	1
    ret					// 4	4	4

;*************************************************************************	
; Lowest 8us timing OUT - 7 cycles minimum and 20 for each 1us in r25
;
; This part of the code simply provides a delay inversely proportionate
; to the IN delay so that the IN + SBI + OUT is always exactly the same
; regardless of the value of r25
;
;*************************************************************************

DelayLow:				// (0)	(1)	(2+) each extra is +20
	mov		YL,r25		// 1	1	
	andi	YL,0x07	
	breq	LoopLowOut1	// 2	1	1
LoopLow:
	call    us_delay	//		16	16
	dec 	YL			//		1	1
	breq	LoopLowOut2	//		2	1/2
	rjmp	LoopLow		//			2
LoopLowOut2:
	rjmp .+0			// 		2	2
LoopLowOut1:
    ret					// 4	4	4
