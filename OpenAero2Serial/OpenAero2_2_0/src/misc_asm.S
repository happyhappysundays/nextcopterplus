/*
 * misc_asm.S
 *
 * Created: 04/04/2013
 * Author: David Thompson
 *
 */

#include <avr/io.h>

// Servo output pin assignments
#define UART_OUT	_SFR_IO_ADDR(PORTD)

#define BIND UART_OUT,0	// PORTD,0
#ifndef __tmp_reg__
#define __tmp_reg__ 0
#endif

	.section .text

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
; void glcd_delay(void) 1us delay for glcd clock
;*************************************************************************

	.global glcd_delay_1us
	.func   glcd_delay_1us
glcd_delay_1us:
	rcall	glcd_delay
	rcall	glcd_delay
	rcall	glcd_delay
	rcall	glcd_delay
	ret
	.endfunc

;*************************************************************************	
; Do master binding timing	bind_master(pulses)
; regs = r24
;*************************************************************************

	.global bind_master
	.func   bind_master
bind_master:
	push	YL			//	1

master_loop:
	cbi 	BIND		//	2
	ldi		YL,0x76		// 	1 		118us
	call	VarDelay	//	4
	sbi		BIND		//	2
	ldi		YL,0x7A		// 	1 		122us
	call	VarDelay	//	4
	dec		r24
	brne	master_loop	//  2 1

	pop		YL			//	  1
	ret					//	  4 
	.endfunc

;*************************************************************************	
; VarDelay: Loops YL units of us. Destroys YL
; 5 cycles to configure and call, 4 to return and 2 to change bit afterwards = 11
; 20 cycles makes 1.0us
;*************************************************************************

VarDelay:
	rjmp .+0			// 2
	rjmp .+0			// 2
	rjmp .+0			// 2
	rjmp .+0			// 2
	rjmp .+0			// 2
	rjmp .+0			// 2
	rjmp .+0			// 2
	rjmp .+0			// 2
	nop					// 1
	dec 	YL			// 1
	brne	VarDelay	// 2 1
	ret					//   4 
