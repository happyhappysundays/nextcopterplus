;*************************************************************************
; Title	:    I2C (Single) Master Implementation
; Author:    Peter Fleury <pfleury@gmx.ch>  http://jump.to/fleury
;            based on Atmel Appl. Note AVR300
; File:      $Id: i2cmaster.S,v 1.12 2008/03/02 08:51:27 peter Exp $
; Software:  AVR-GCC 3.3 or higher
; Target:    any AVR device
;
; DESCRIPTION
; 	Basic routines for communicating with I2C slave devices. This
;	"single" master implementation is limited to one bus master on the
;	I2C bus. 
;  
;       Based on the Atmel Application Note AVR300, corrected and adapted 
;       to GNU assembler and AVR-GCC C call interface
;       Replaced the incorrect quarter period delays found in AVR300 with 
;       half period delays. 
;
; USAGE
;	These routines can be called from C, refer to file i2cmaster.h.
;   See example test_i2cmaster.c inside i2cmaster.h
; 	Adapt the SCL and SDA port and pin definitions and eventually 
;	the delay routine to your target
; 	Use 4.7k pull-up resistor on the SDA and SCL pin.
;
; NOTES
;	The I2C routines can be called either from non-interrupt or
;	interrupt routines, not both.
;
; Changes for OpenAero
;	i2c_delay_T2 updated, SDA & SCL defines set to i86/N6 hardware
;
;*************************************************************************

#if (__GNUC__ * 100 + __GNUC_MINOR__) < 303
#error "This library requires AVR-GCC 3.3 or later. Update to a newer AVR-GCC compiler!"
#endif

#include <avr/io.h>
#include "..\inc\compiledefs.h"

;***** Adapt these SCA and SCL port and pin definition to your target
;
#define SDA				4				// SDA Port C, Pin 4   
#define SCL				5				// SCL Port C, Pin 5

;******

;-- map the IO register back into the IO address space
#define SDA_DDR		_SFR_IO_ADDR(DDRC)
#define SCL_DDR		_SFR_IO_ADDR(DDRC)
#define SDA_OUT		_SFR_IO_ADDR(PORTC)
#define SCL_OUT		_SFR_IO_ADDR(PORTC)
#define SDA_IN		_SFR_IO_ADDR(PINC)
#define SCL_IN		_SFR_IO_ADDR(PINC)

;******

//#define I2C_400KHz						// For 400kHz i2c speed
#define I2C_100KHz						// For 100kHz i2c speed

;******

#ifndef __tmp_reg__
#define __tmp_reg__ 0
#endif

	.section .text

;*************************************************************************
; Delay half period
; For I2C in normal mode (100kHz), use T/2 = 5us
; For I2C in fast mode (400kHz),   use T/2 = 1.25us
;*************************************************************************
    .stabs  "",100,0,0,i2c_delay_T2
    .stabs  "i2cmaster.S",100,0,0,i2c_delay_T2
	.global i2c_delay_T2
	.func i2c_delay_T2
#ifdef I2C_100KHz		// Delay 5us with 8 MHz crystal	
				   		// Total 40 cyles = 5us with 8 MHz crystal 
						// <-------	Left column is skip execution path
i2c_delay_T2:    		// 4    <--	Right column is end execution path
	push	19	 		// 2   		Save r19
	ldi 	19,0x04		// 1   		Set loop count to 0x04h
	clr		0	 		// 1   
T2_delay:
	nop					// 1   		8 + (4 x 7) -1 + 5 = 40 cycles
	nop					// 1   
	nop					// 1   
	nop					// 1   
	subi 	19,1		// 1   
	brne	T2_delay	// 2   1 	Loop until zero
	pop		19			//	   2 	Restore r19
	ret          		//	   3  

#elif defined(I2C_400KHz) // Delay 1.25us with 8 MHz crystal
			     		// Total 10 cyles = 1.25us with 8 MHz crystal 

i2c_delay_T2:    		// 4 cycles
	nop          		// 1   
	nop          		// 1   
	nop          		// 1   
	ret          		// 3   

#else
#error No i2c speed defined
#endif
	.endfunc

;*************************************************************************
; Initialization of the I2C bus interface. Need to be called only once
; 
; extern void i2c_init(void)
;*************************************************************************
	.global i2c_init
	.func i2c_init
i2c_init:
	cbi SDA_DDR,SDA			;release SDA
	cbi SCL_DDR,SCL			;release SCL
	cbi SDA_OUT,SDA			; Switch off pull-ups on SDA
	cbi SCL_OUT,SCL			; Switch off pull-ups on SCL
	ret
	.endfunc

;*************************************************************************	
; Issues a start condition and sends address and transfer direction.
; return 0 = device accessible, 1= failed to access device
;
; extern unsigned char i2c_start(unsigned char addr);
;	addr = r24, return = r25(=0):r24
;*************************************************************************
	.global i2c_start
	.func   i2c_start
i2c_start:
	sbi 	SDA_DDR,SDA		;force SDA low
	rcall 	i2c_delay_T2	;delay T/2
	
	rcall 	i2c_write		;write address
	ret
	.endfunc		

;*************************************************************************
; Issues a repeated start condition and sends address and transfer direction.
; return 0 = device accessible, 1= failed to access device
;
; extern unsigned char i2c_rep_start(unsigned char addr);
;	addr = r24,  return = r25(=0):r24
;*************************************************************************
	.global i2c_rep_start
	.func	i2c_rep_start
i2c_rep_start:
	sbi	SCL_DDR,SCL			;force SCL low
	rcall 	i2c_delay_T2	;delay  T/2
	cbi	SDA_DDR,SDA			;release SDA
	rcall	i2c_delay_T2	;delay T/2
	cbi	SCL_DDR,SCL			;release SCL
	rcall 	i2c_delay_T2	;delay  T/2
	sbi 	SDA_DDR,SDA		;force SDA low
	rcall 	i2c_delay_T2	;delay	T/2
	
	rcall	i2c_write		;write address
	ret
	.endfunc

;*************************************************************************	
; Issues a start condition and sends address and transfer direction.
; If device is busy, use ack polling to wait until device is ready
;
; extern void i2c_start_wait(unsigned char addr);
;	addr = r24
;*************************************************************************
	.global i2c_start_wait
	.func   i2c_start_wait
i2c_start_wait:
	push	r22
	mov		r22,r24
i2c_start_wait1:
	sbi 	SDA_DDR,SDA		;force SDA low
	rcall 	i2c_delay_T2	;delay T/2
	mov		r24,r22
	rcall 	i2c_write		;write address
	tst	r24					;if device not busy -> done
	breq	i2c_start_wait_done
	rcall	i2c_stop		;terminate write operation
	rjmp	i2c_start_wait1	;device busy, poll ack again
i2c_start_wait_done:
	pop		r22
	ret
	.endfunc	

;*************************************************************************
; Terminates the data transfer and releases the I2C bus
;
; extern void i2c_stop(void)
;*************************************************************************
	.global	i2c_stop
	.func	i2c_stop
i2c_stop:
	sbi	SCL_DDR,SCL			;force SCL low
	sbi	SDA_DDR,SDA			;force SDA low
	rcall	i2c_delay_T2	;delay T/2
	cbi	SCL_DDR,SCL			;release SCL
	rcall	i2c_delay_T2	;delay T/2
	cbi	SDA_DDR,SDA			;release SDA
	rcall	i2c_delay_T2	;delay T/2
	ret
	.endfunc

;*************************************************************************
; Send one byte to I2C device
; return 0 = write successful, 1 = write failed
;
; extern unsigned char i2c_write( unsigned char data );
;	data = r24,  return = r25(=0):r24
;*************************************************************************
	.global i2c_write
	.func	i2c_write
i2c_write:
	sec						;set carry flag
	rol 	r24				;shift in carry and out bit one
	rjmp	i2c_write_first
i2c_write_bit:
	lsl	r24					;if transmit register empty
i2c_write_first:
	breq	i2c_get_ack
	sbi	SCL_DDR,SCL			;force SCL low
	brcc	i2c_write_low
	nop
	cbi	SDA_DDR,SDA			;release SDA
	rjmp	i2c_write_high
i2c_write_low:
	sbi	SDA_DDR,SDA			;force SDA low
	rjmp	i2c_write_high
i2c_write_high:
	rcall 	i2c_delay_T2	;delay T/2
	cbi	SCL_DDR,SCL			;release SCL
	rcall	i2c_delay_T2	;delay T/2
	rjmp	i2c_write_bit
	
i2c_get_ack:
	sbi	SCL_DDR,SCL			;force SCL low
	cbi	SDA_DDR,SDA			;release SDA
	rcall	i2c_delay_T2	;delay T/2
	cbi	SCL_DDR,SCL			;release SCL
i2c_ack_wait:
	sbis	SCL_IN,SCL		;wait for SCL to go high (in case wait states are inserted)
	rjmp	i2c_ack_wait
	
	clr	r24					;return 0
	sbic	SDA_IN,SDA		;if SDA high -> return 1
	ldi	r24,1
	rcall	i2c_delay_T2	;delay T/2
	clr	r25
	ret
	.endfunc

;*************************************************************************
; read one byte from the I2C device, send ack or nak to device
; (ack=1, send ack, request more data from device 
;  ack=0, send nak, read is followed by a stop condition)
;
; extern unsigned char i2c_read(unsigned char ack);
;	ack = r24, return = r25(=0):r24
; extern unsigned char i2c_readAck(void);
; extern unsigned char i2c_readNak(void);
; 	return = r25(=0):r24
;*************************************************************************
	.global i2c_readAck
	.global i2c_readNak
	.global i2c_read		
	.func	i2c_read
i2c_readNak:
	clr	r24
	rjmp	i2c_read
i2c_readAck:
	ldi	r24,0x01
i2c_read:
	ldi	r23,0x01			;data = 0x01
i2c_read_bit:
	sbi	SCL_DDR,SCL			;force SCL low
	cbi	SDA_DDR,SDA			;release SDA (from previous ACK)
	rcall	i2c_delay_T2	;delay T/2
	
	cbi	SCL_DDR,SCL			;release SCL
	rcall	i2c_delay_T2	;delay T/2
	
i2c_read_stretch:
    sbis SCL_IN, SCL        ;loop until SCL is high (allow slave to stretch SCL)
    rjmp	i2c_read_stretch
    	
	clc						;clear carry flag
	sbic	SDA_IN,SDA		;if SDA is high
	sec						;set carry flag
	
	rol	r23					;store bit
	brcc	i2c_read_bit	;while receive register not full
	
i2c_put_ack:
	sbi	SCL_DDR,SCL			;force SCL low	
	cpi	r24,1
	breq	i2c_put_ack_low	;if (ack=0)
	cbi	SDA_DDR,SDA			;release SDA
	rjmp	i2c_put_ack_high
i2c_put_ack_low:			;else
	sbi	SDA_DDR,SDA			;force SDA low
i2c_put_ack_high:
	rcall	i2c_delay_T2	;delay T/2
	cbi	SCL_DDR,SCL			;release SCL
i2c_put_ack_wait:
	sbis	SCL_IN,SCL		;wait SCL high
	rjmp	i2c_put_ack_wait
	rcall	i2c_delay_T2	;delay T/2
	rcall	i2c_delay_T2	;delay T/2
	rcall	i2c_delay_T2	;delay T/2
	mov	r24,r23
	clr	r25
	ret
	.endfunc

;*************************************************************************
; Toggles SDA and SCL to test i2c timing
;
; extern void i2c_test(void)
;*************************************************************************
	.global	i2c_test
	.func	i2c_test
i2c_test:
	sbi	SCL_DDR,SCL			;SCL output LOW
	rcall	i2c_delay_T2	;delay T/2
	sbi	SDA_DDR,SDA			;SDA output LOW
	rcall	i2c_delay_T2	;delay T/2
	cbi	SCL_DDR,SCL			;SCL input FLOAT
	rcall	i2c_delay_T2	;delay T/2
	cbi	SDA_DDR,SDA			;SDA input FLOAT
	rcall	i2c_delay_T2	;delay T/2
	ret
	.endfunc
