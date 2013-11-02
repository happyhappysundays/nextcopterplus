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
; Do slave binding timing
;*************************************************************************

	.global bind_slave
	.func   bind_slave
bind_slave:
	push	YL			//	1
	push	XL			//	1

	ldi		XL,0x6		// 	1 Slave is 6 pulses
slave_loop:
	cbi 	BIND		//	2
	ldi		YL,0x78		// 	1 		120us
	call	VarDelay	//	4
	sbi		BIND		//	2
	ldi		YL,0x78		// 	1 		120us
	call	VarDelay	//	4
	dec		XL			//	1
	brne	slave_loop	//  2 1

	pop		XL			//	  1
	pop		YL			//	  1
	ret					//	  4 
	.endfunc

;*************************************************************************	
; Do master binding timing
;*************************************************************************

	.global bind_master
	.func   bind_master
bind_master:
	push	YL			//	1
	push	XL			//	1

	ldi		XL,0x5		// 	1 Master is 5 pulses
master_loop:
	cbi 	BIND		//	2
	ldi		YL,0x76		// 	1 		118us
	call	VarDelay	//	4
	sbi		BIND		//	2
	ldi		YL,0x7A		// 	1 		122us
	call	VarDelay	//	4
	dec		XL			//	1
	brne	master_loop	//  2 1

	pop		XL			//	  1
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
