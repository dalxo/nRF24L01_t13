/*
The BSD 3-Clause License

Copyright (c) 2022, Dalimir Orfanus


Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
contributors may be used to endorse or promote products derived from
this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

//------------------------------------------------------------------------
// nRF24L01_asm.S
//
// Implementation of some API functions in pure assembler. Initially, these methods 
// were as C functions with inline ASM to reduce the code size.  However, C++ linker 
// (with LDO enabled, e.g. in Arduino IDE) could not link function calls to 
// 'shiftOutByte' (rcall) from inlined ASM. Changing the initial C function with 
// inlining to a pure ASM fixed this problem. Assembler functions below have in 
// a comment original C code to explain what is going on.
//
// See nRF24L01.c for a detailed explanation.
// Pin and role configurations are defined in "projdefs.h" file.

// NOTE: Be careful if you make changes to asm functions. They implicitly rely 
//	on register interfaces between C-function and ASM. For instance: r24(r25) is 1st 
//	parameter as well as the return value. This is heavily exploited in the code below. 
//------------------------------------------------------------------------

#include <avr/io.h>
#include <avr/sfr_defs.h>

#include "projdefs.h"	// contains macros for PIN configuration and feature selection (read the comment above)

.section .text

//========================================================================

/*
//------------------------------------------------------------------------
// Local function to toggle CSN pin. 
//
// Toggling happens in all SPI functions - in the beginning, and end. 
// Making it a function saves at least 6 bytes in the footprint.
//------------------------------------------------------------------------
extern void csnSetCrl() {
	NRF24L01_CSN_SET();	
	NRF24L01_CSN_CLR();
}
*/

csnSetCrl:	; void csnSetCrl()
	sbi _SFR_IO_ADDR(NRF24L01_PORT), NRF24L01_CSN ; // NRF24L01_CSN_SET()
	cbi _SFR_IO_ADDR(NRF24L01_PORT), NRF24L01_CSN ; // NRF24L01_CSN_CLR()
	ret


//========================================================================

/*
//------------------------------------------------------------------------
// Send SPI command to the nRF. 
// The function sends only 1-byte command to the module. Typical 1-byte commands are:
// FLUSH_RX, FLUSH_TX, REUSE_TX_PL. 
// This function is not applicable for multi-byte commands, for instance:
// read/Write to config and status, registers, accessing buffers. 
// For multi-byte commands use other functions (e.g. nrf24_readReg). 
//
// cmd: SPI command to be issued
//------------------------------------------------------------------------
void nrf24_cmd(uint8_t cmd) {
	csnSetCrl();
	shiftOutByte(cmd);
	csnSetCrl();
}
*/

.global nrf24_cmd
nrf24_cmd:		; void nrf24_cmd(uint8_t cmd)
	rcall csnSetCrl			; // toggle CSN
	rcall shiftOutByte		
	rcall csnSetCrl			; // toggle CSN
	ret

//========================================================================

#ifndef NRF24L01_DO_NOT_USE_MISO
/*
//------------------------------------------------------------------------
// Read from a single register
// cmd: SPI command to be issued
// return: read data from a register
//------------------------------------------------------------------------
uint8_t nrf24_readReg(uint8_t cmd) {
	csnSetCrl();	
	cmd = shiftOutByte(cmd);		// done on purpose to reduce ASM size
	cmd = shiftOutByte(cmd);	
	csnSetCrl();
	return cmd;
}
*/

.global nrf24_readReg
nrf24_readReg:		; uint8_t nrf24_readReg(uint8_t cmd)
	rcall csnSetCrl			; // toggle CSN
	rcall shiftOutByte		
	rcall shiftOutByte
	rcall csnSetCrl			; // toggle CSN
	ret

#endif


//========================================================================

/*
//------------------------------------------------------------------------
// Write into a single register
// cmd: SPI command to be issued
// value: data to be written into a register
//------------------------------------------------------------------------
void nrf24_writeReg(uint8_t cmd, uint8_t value) {
	csnSetCrl();		
	asm volatile (
		"rcall shiftOutByte			\n\t"
		"mov	r24, r22			\n\t"
		"rcall shiftOutByte			\n\t"
		:: 
	);	
	csnSetCrl();
}
*/

.global nrf24_writeReg 
nrf24_writeReg:		; void nrf24_writeReg(uint8_t cmd, uint8_t value)
	rcall csnSetCrl			; // toggle CSN

	rcall shiftOutByte
	mov	r24, r22		
	rcall shiftOutByte
	
	rcall csnSetCrl			; // toggle CSN
	ret

//========================================================================

/*
//------------------------------------------------------------------------
// Write into multiple registers form buffer
// cmd: SPI command to be issued
// buff: pointer to the buffer
// size: how many bytes to be written
//------------------------------------------------------------------------
void nrf24_writeRegs(uint8_t cmd, const uint8_t *buff, uint8_t size) {
	csnSetCrl();

	asm volatile(
	"rcall	shiftOutByte		\n\t"	// in r24 is cmd, thus immediately call
	"	WriteReg5_loop_%=:		\n\t"	
	"ld		r24,	%a0+		\n\t"	// load to r24 byte from the buffer
	"rcall	shiftOutByte		\n\t"	// call shift out (note, r24 and r25 are modified)
	"dec	%[len]				\n\t"
	"brne	WriteReg5_loop_%=	\n\t"
	:
	:"e"(buff), [len] "r"(size)
	);

	csnSetCrl();
}
*/

.global nrf24_writeRegs
nrf24_writeRegs:	; void nrf24_writeRegs(uint8_t cmd, const uint8_t *buff, uint8_t size)
	rcall csnSetCrl			; // toggle CSN
	movw	r30, r22		; // move pointer to buff to Z regs
	rcall	shiftOutByte	; // in r24 is cmd, thus immediately call and execute CMD

	WriteRegsLoop:			; // while loop to write data from buffer
	ld		r24,	Z+		; // load to r24 byte from the buffer (pointed by Z) and increment pointer
	rcall	shiftOutByte	; // call shift out (note, r24 and r25 are modified)
	dec	r20					; // --size;
	brne	WriteRegsLoop	; // if size > 0 do loop

	rcall csnSetCrl			; // toggle CSN
	ret

//========================================================================

#ifndef NRF24L01_DO_NOT_USE_MISO
/*
//------------------------------------------------------------------------
// Read multiple registers into buffer
// cmd: SPI command to be issued
// buff: pointer to the buffer
// size: how many bytes to be read into the buffer
//------------------------------------------------------------------------
void nrf24_readRegs(uint8_t cmd, uint8_t *buff, uint8_t size) {
	csnSetCrl();
		
	asm volatile(
	"rcall	shiftOutByte		\n\t"	// in r24 is cmd, thus immediately call
	"	WriteReg6_loop_%=:		\n\t"
	"rcall	shiftOutByte		\n\t"	// call shift out (note, r24 and r25 are modified)
	"st		%a0+, r24			\n\t"	// load byte from r24 to the buffer
	"dec	%[len]				\n\t"
	"brne	WriteReg6_loop_%=	\n\t"
	:
	:"e"(buff), [len] "r"(size)
	);

	csnSetCrl();
}
*/

.global nrf24_readRegs
nrf24_readRegs:	; void nrf24_readRegs(uint8_t cmd, uint8_t *buff, uint8_t size)
	rcall csnSetCrl			; // toggle CSN
	movw	r30, r22		; // move pointer to buffer into the Z pair
	rcall	shiftOutByte	; // in r24 is cmd, thus immediately call and execute CMD

	ReadRegsLoop:			; // while loop to read data from buffer
	rcall	shiftOutByte	; // call shift out (note, r24 and r25 are modified)
	st	Z+, r24				; // store byte from r24 (return from shiftOutByte) to the buffer (pointed by Z) and increment pointer	
	dec	r20					; // --size;
	brne	ReadRegsLoop	; // if size > 0 do loop

	rcall csnSetCrl			; // toggle CSN
	ret

#endif

//========================================================================
