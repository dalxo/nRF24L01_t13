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

/****************************************************************************************
nRF24L01.c - Footprint optimized implementation of the driver to control radio module nRF24L01+

This implementation has only a few hundreds bytes to fit inside ATtiny13 EEPROM (1KiB) and has 
only 7 API methods. The final footprint depends on the way how nRF module is connected to the MCU 
and which "pin conservation" features out of four are used:

--------------
1) Shared CE and CSN pins: 
Both interface signals are connected to a single pin:

                |----- CSN nRF
   MCU PINx ----x----- CE nRF  

This is useful if the MCU is used for transmissions of data (sensors) to the central hub. Between transmissions the module 
is in the power saving mode (CE == 0, i.e. radio is off)  to conserve energy. 
Applicable for battery powered applications. 
The feature is enabled by macro: #define NRF24L01_SHARED_CE_CSN

--------------
2) Bidirectional 3-wire SPI (shared MISO and MOSI pins):
For situations where MCU needs bi-directional access to the nRF radio module, e.g. reading registers
or receiving data. Rather than consuming additional pin on MCU (beside MOSI), both MISO and MOSI 
can be connected to a single MCU pin via a resistor. 
        
                |--------- MISO nRF
   MCU PINx ----x--[ R ]-- MOSI nRF
              4k7 <= R <= 10k

Similar HW solution (resistor is on MISO pin) was tested by Nerd Ralph:
http://nerdralph.blogspot.com/2015/05/nrf24l01-control-with-2-mcu-pins-using.html
Solution from Ralph also deals with SCK and CSN signals (assuming CE is always ON) which we do not apply in our case. 

The feature is enabled by macro: #define NRF24L01_3WIRE_SPI

--------------
3) Unidirectional 3-wire SPI (only write into nRF is supported, MISO disconnected):
We do not read anything from the nRF module. Entire communication on SPI bus is one-directional
from the MCU to nRF. Signal MISO is not connected to the MCU, thus this saves 1 pin on MCU.

                 *-- MISO nRF
MCU PINx ----------- MOSI nRF


This is useful for applications that only transmit data, e.g. sensors.
The feature is enabled by macro: #define NRF24L01_DO_NOT_USE_MISO

--------------
4) Standard 4-wire SPI (independent MISO and MOSI pins):
In this case we have separate pins for MOSI and MISO signals, i.e. standard 4-wire SPI. 
This does not conserve pins on MCU. Nevertheless, the footprint of this solution 
is 10 bytes (5 instructions) smaller than shared pins. This is useful in application 
where the code footprint is more critical than number of used pins.

   MCU PINx ----------- MISO nRF
   MCU PINy ----------- MOSI nRF


This feature is the default configuration - unless it is overridden by macros from (2) or (3) feature.


================================
FEATURE COMBINATIONS and macros:
================================
The feature (1) is ORTHOGONAL to the remaining ones (2) -- (4), i.e. it can be used together with 
any MISO/MOSI combinations.
The features (2) -- (4) are MUTUALY EXCLUSIVE, i.e. only one of them can be enabled (used) in the compiled code.

List of macros for selecting features:
#define	NRF24L01_SHARED_CE_CSN		// Enable feature (1), can be concurrently used with features (2), (3), and (4).
#define	NRF24L01_DO_NOT_USE_MISO	// Enable feature (2), mutually exclusive with feature (3) and (4)
#define	NRF24L01_3WIRE_SPI			// Enable feature (3), mutually exclusive with feature (2) and (4)

Feature (4) is enabled be default unless feature (2) or (3) is enabled (defined). 


================================
PIN CONFIGURATION macros:
================================
Define the following MANDATORY macros for port output, port direction and pin reading:
#define NRF24L01_PORT		PORTB
#define NRF24L01_DDR		DDRB
#define NRF24L01_INPORT		PINB

Define following macros (where applicable - see features above) for pin configuration:
#define NRF24L01_CE			PB0		// Optional - do not define use if feature (1) is enabled (SHARED CE with CSN)
#define NRF24L01_CSN		PB2		// Mandatory
#define NRF24L01_SCK		PB1		// Mandatory
#define NRF24L01_MOSI		PB3		// Mandatory
#define NRF24L01_MISO		PB4		// Optional - used only in feature (4)


================================
DRIVER INIT and API lifecycle
================================
It is mandatory to firstly call fcn [void nrf24_init(void)] before any other fcn of the API is called. 
The init will configure I/O pins of the MCU according to selected features. 

Afterwards, any fcn from API can be used without any restriction and in unrestricted order.
The exception is [uint8_t shiftOutByte(uint8_t data)] which is only for internal (private) use. It assumes certain
usage of registers by a caller fcn. Unless you really know what you do - do not call this fcn directly otherwise
it will mess with registers. There should never be such a situation where this fcn is called directly from API.

***************************************************************************************/

#include <avr/io.h>
#include <avr/sfr_defs.h>
#include <util/delay.h>


#include "nrf24l01.h"
#include "projdefs.h"		// contains macros for PIN configuration and feature selection (read the comment above)

#ifndef NRF24L01_SHARED_CE_CSN
	#define NRF24L01_CE_SET()	asm("sbi %0, %1" :: "I" (_SFR_IO_ADDR(NRF24L01_PORT)), "I" (NRF24L01_CE) )
	#define NRF24L01_CE_CLR()	asm("cbi %0, %1" :: "I" (_SFR_IO_ADDR(NRF24L01_PORT)), "I" (NRF24L01_CE) )
#endif

#define NRF24L01_CSN_SET()	asm("sbi %0, %1" :: "I" (_SFR_IO_ADDR(NRF24L01_PORT)), "I" (NRF24L01_CSN) )
#define NRF24L01_CSN_CLR()	asm("cbi %0, %1" :: "I" (_SFR_IO_ADDR(NRF24L01_PORT)), "I" (NRF24L01_CSN) )

#define NRF24L01_SCK_SET()	asm("sbi %0, %1" :: "I" (_SFR_IO_ADDR(NRF24L01_PORT)), "I" (NRF24L01_SCK) )
#define NRF24L01_SCK_CLR()	asm("cbi %0, %1" :: "I" (_SFR_IO_ADDR(NRF24L01_PORT)), "I" (NRF24L01_SCK) )

#define NRF24L01_MOSI_SET()	asm("sbi %0, %1" :: "I" (_SFR_IO_ADDR(NRF24L01_PORT)), "I" (NRF24L01_MOSI) )
#define NRF24L01_MOSI_CLR()	asm("cbi %0, %1" :: "I" (_SFR_IO_ADDR(NRF24L01_PORT)), "I" (NRF24L01_MOSI) )

#ifndef NRF24L01_DO_NOT_USE_MISO
	#define	NRF24L01_MISO_READ()		(NRF24L01_INPORT & _BV(NRF24L01_MISO))
 #endif 

#define NRF24L01_MOSI_IN()	asm("cbi %0, %1" :: "I" (_SFR_IO_ADDR(NRF24L01_DDR)), "I" (NRF24L01_MOSI) )
#define NRF24L01_MOSI_OUT()	asm("sbi %0, %1" :: "I" (_SFR_IO_ADDR(NRF24L01_DDR)), "I" (NRF24L01_MOSI) )
#define	NRF24L01_MOSI_READ()		(NRF24L01_INPORT & _BV(NRF24L01_MOSI))


//------------------------------------------------------------------------
// Writes and reads a byte from the SPI bus.
// Private fcn only for internal use within this driver.
// Never call it outside of this driver (compilation unit) unless you
// really know what you do.
//
// data: byte to be sent out on the bus
// return: byte received from the nRF
// Note 1: Depending on the PIN configuration, this method has 3
//         different implementations.
// Note 2: r25 is internally used without saving
//------------------------------------------------------------------------
extern uint8_t shiftOutByte(uint8_t data) {	
	// We never read from the module, only pure write
	// The footprint of the entire fcn is 20 bytes == 10 instructions
	#ifdef NRF24L01_DO_NOT_USE_MISO
		asm volatile (
		"ldi		r25, 8				\n\t"	// i = 8
		
		"	shiftOutMSB_loop_%=:		\n\t"	// do {

		"cbi		%[port], %[mosi]	\n\t"	// NRF24L01_MOSI_CLR();
		"sbrc		%[dataOut], 7		\n\t"	// skip next instruction if MSB in data is 0
		"sbi		%[port], %[mosi]	\n\t"	// NRF24L01_MOSI_SET();
		"lsl		%[dataOut]			\n\t"	// data <<= 1
		"sbi		%[port], %[sck]		\n\t"	// NRF24L01_SCK_SET();
		"cbi		%[port], %[sck]		\n\t"	// NRF24L01_SCK_CLR();
			
		// } while(--i)
		"dec		r25					\n\t"
		"brne		shiftOutMSB_loop_%=	\n\t"
		
		: // no output data
		:	[port] "I" (_SFR_IO_ADDR(NRF24L01_PORT)),
			[sck]  "I" (NRF24L01_SCK), 
			[mosi] "I" (NRF24L01_MOSI), 
			[dataOut] "r" (data)
		: "r25");

		return data;

	// We do both read and write but MISO and MOSI are shared (connected) via a single pin
	// The footprint of the entire fcn is 34 bytes == 17 instructions
	#elif defined(NRF24L01_3WIRE_SPI)
		uint8_t out;
		asm volatile (
		"ldi		r25, 8				\n\t"	// i = 8
		"ldi		%[dataOut], 0		\n\t"	// out = 0
		
		"	shiftOutMSB2_loop_%=:		\n\t"	// do {
		"cbi		%[dir], %[mosi]		\n\t"	// NRF24L01_MOSI_IN();
		"lsl		%[dataOut]			\n\t"	// out <<= 1;
			
		"sbic		%[in], %[mosi]		\n\t"	// if(NRF24L01_MOSI_READ())
		"ori		%[dataOut], 0x01	\n\t"	//	out |= 0x01;

		"sbi		%[dir], %[mosi]		\n\t"	// NRF24L01_MOSI_OUT();

			
		"cbi		%[port], %[mosi]	\n\t"	// NRF24L01_MOSI_CLR();
		"sbrc		%[dataIn], 7		\n\t"	// skip next instruction if MSB in data is 0
		"sbi		%[port], %[mosi]	\n\t"	// NRF24L01_MOSI_SET();
			
		"lsl		%[dataIn]			\n\t"	// data <<= 1
			
		"sbi		%[port], %[sck]		\n\t"	// NRF24L01_SCK_SET();
		"cbi		%[port], %[sck]		\n\t"	// NRF24L01_SCK_CLR();
			
		// } while(--i)
		"dec		r25					\n\t"
		"brne		shiftOutMSB2_loop_%=	\n\t"
		: [dataOut] "=&r" (out)
		: [port] "I" (_SFR_IO_ADDR(NRF24L01_PORT)),
		  [in] "I"(_SFR_IO_ADDR(NRF24L01_INPORT)),
		  [dir] "I"(_SFR_IO_ADDR(NRF24L01_DDR)),
		  [sck] "I"(NRF24L01_SCK), 
		  [mosi] "I"(NRF24L01_MOSI), 
		  [dataIn] "r"(data)
		: "r25");

		return out;	
	
	// We do both read and write for regular 4-wire SPI with separate MISO and MOSI pins
	// The footprint of the entire fcn is 24 bytes == 12 instructions
	#else
		uint8_t read;
		asm volatile (
		"ldi		r25, 8				\n\t"	// i = 8
		
		"	shiftOutMSB3_loop_%=:		\n\t"	// do {

		"cbi		%[port], %[mosi]	\n\t"	// NRF24L01_MOSI_CLR();		
		"sbrc		%[dataOut], 7		\n\t"	// skip next instruction if MSB in data is 0
		"sbi		%[port], %[mosi]	\n\t"	// NRF24L01_MOSI_SET();
		
		"lsl		%[dataIn]			\n\t"	// data <<= 1
		
		"sbic		%[in], %[miso]		\n\t"	// skip next instruction if MSB in data is 0
		"sbr		%[dataIn], 0x01		\n\t"	// set bit
		
		"sbi		%[port], %[sck]		\n\t"	// NRF24L01_SCK_SET();
		"cbi		%[port], %[sck]		\n\t"	// NRF24L01_SCK_CLR();
						
		// } while(--i)
		"dec		r25					\n\t"
		"brne		shiftOutMSB3_loop_%=	\n\t"
		:	[dataOut] "=r"(read)
		:	[port] "I" (_SFR_IO_ADDR(NRF24L01_PORT)),
			[in] "I"(_SFR_IO_ADDR(NRF24L01_INPORT)),
			[sck] "I" (NRF24L01_SCK), 
			[mosi] "I" (NRF24L01_MOSI), 
			[miso] "I" (NRF24L01_MISO), 
			[dataIn] "r" (data)
		: "r25");
	
		return read;
	#endif
}

//------------------------------------------------------------------------
// Initializes SPI interface
//------------------------------------------------------------------------
void nrf24_init(void) {
	NRF24L01_MOSI_CLR();
	NRF24L01_SCK_CLR();
	NRF24L01_CSN_CLR();

#ifdef NRF24L01_SHARED_CE_CSN
	NRF24L01_DDR |= _BV(NRF24L01_SCK) | _BV(NRF24L01_CSN) | _BV(NRF24L01_MOSI);
#else
	NRF24L01_CE_CLR();
	NRF24L01_DDR |= _BV(NRF24L01_SCK) | _BV(NRF24L01_CE) | _BV(NRF24L01_CSN) | _BV(NRF24L01_MOSI);
#endif
	
#ifndef NRF24L01_DO_NOT_USE_MISO
	NRF24L01_DDR &= ~_BV(NRF24L01_MISO);
#endif

}


//------------------------------------------------------------------------
// Pulse CE signal for more than 10us to trigger the transmission
//------------------------------------------------------------------------
void nrf24_pulseCE(void) {
	#ifdef NRF24L01_SHARED_CE_CSN
		NRF24L01_CSN_SET();
		_delay_us(FIFTEEN_MICROSECONDS);
		NRF24L01_CSN_CLR();
	#else
		NRF24L01_CE_SET();
		_delay_us(FIFTEEN_MICROSECONDS);
		NRF24L01_CE_CLR();
	#endif
}

//------------------------------------------------------------------------
// Pulse CE signal for given number of milliseconds - intended for the RX.
// Useful when using shared CE/CSN signal and also entering RX mode.
// If node will spent more time in RX than TX, tight CE up to Vcc permanently.
// For TX mode, use nrf24_pulseCE(void) instead.
//------------------------------------------------------------------------
void nrf24_pulseCE_ms(uint16_t millis) {
	#ifdef NRF24L01_SHARED_CE_CSN
		NRF24L01_CSN_SET();
	#else
		NRF24L01_CE_SET();
	#endif

	// method _delay_ms does not take as an argument a variable, it must be a compile time constant
	// thus we have to loop
	do {
		_delay_ms(1);	
	} while (--millis);
	
	#ifdef NRF24L01_SHARED_CE_CSN
		NRF24L01_CSN_CLR();
	#else
		NRF24L01_CE_CLR();
	#endif
}
