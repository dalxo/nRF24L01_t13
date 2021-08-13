/*
The BSD 3-Clause License

Copyright (c) 2021, Dalimir Orfanus


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

/*
 * nRF24L01_t13.c
 *
 */ 

#include <avr/io.h>
#include <util/delay.h>

#include "projdefs.h"
#include "nRF24L01.h"


#define CHANNEL 120 // 0-125
#define DELAY	3	// (== 250us * (n+1))
#define RETRY	10	// 12

#define MAX_BUFF_SIZE	 2

const uint8_t PIPE0_ADDRESS[] = "0link";	// pipe 0 address

#define LED_SET()	asm("sbi %0, %1" :: "I" (_SFR_IO_ADDR(PORTB)), "I" (LED_PIN) )
#define LED_CLR()	asm("cbi %0, %1" :: "I" (_SFR_IO_ADDR(PORTB)), "I" (LED_PIN) )


void testSend(void);

#if	!defined(NRF24L01_DO_NOT_USE_MISO)
	void testRecv(void);
#endif

int main(void)
{
	LED_CLR();
	NRF24L01_DDR |= _BV(LED_PIN);	// set output dir on LED pin
	
	nrf24_init(); // initialize radio (UNDEFINED ==> POWER ON RESET ==> POWER DOWN)
	
	nrf24_writeReg(W_REGISTER | RF_CH,		CHANNEL);
	nrf24_writeReg(W_REGISTER | RF_SETUP,	NRF24_PWR_MAX | NRF24_SPEED_250kbps); // 0dbm TX power, 250kbps
	
	nrf24_writeReg(W_REGISTER | EN_RXADDR,	NRF24_PIPE_0);	// enable RX in pipe 0 for ACK packet
	nrf24_writeReg(W_REGISTER | DYNPD,		NRF24_PIPE_0);   // enable dynamic payload in pipe 0	
	nrf24_writeReg(W_REGISTER | FEATURE,	NRF24_FEATURE_EN_DPL); // enable dynamic payload length

	nrf24_writeRegs(W_REGISTER | TX_ADDR, PIPE0_ADDRESS, 5);	// target pipe 0 address
	nrf24_writeRegs(W_REGISTER | RX_ADDR_P0, PIPE0_ADDRESS, 5);	// RX address on pipe 0

	nrf24_cmd(FLUSH_TX); // clean TX FIFOs thoroughly
	nrf24_cmd(FLUSH_RX); // clean RX FIFOs thoroughly
	//LED_SET();
	
	//testSend();	
	testRecv();
}


void testSend() {
	uint8_t msg[MAX_BUFF_SIZE] = {0, 0};
	
	nrf24_writeReg(W_REGISTER | SETUP_RETR, (DELAY & 0xf) << ARD_SHIFT | (RETRY & 0xf) << ARC_SHIFT); // set waiting and retransmit 
		
	while(1) {
		LED_SET();
		++msg[0];
		--msg[1];
		
		// TX mode, enable CRC with 2 bytes, mask all IRQs, power on nRF radio (POWER DOWN ==> STANDBY-1)
		nrf24_writeReg(W_REGISTER | NRF_CONFIG,
			NRF24_CFG_PWR_UP | NRF24_CFG_TX_MODE | NRF24_CFG_CRC_2B | NRF24_CFG_CRC_EN | NRF24_CFG_IRQ_MASK_ALL);
		
		_delay_ms(5); // there should be 5ms delay to bring nRF from the POWER DOWN to the POWER UP state (the worst case)

		nrf24_cmd(FLUSH_TX); // clean TX FIFOs thoroughly - in case there is a frame with exceeded retransmission (ARC)
		nrf24_writeReg(W_REGISTER | NRF_STATUS, NRF24_STATUS_CLEAR_ALL); // clear all status indications
		
		nrf24_writeRegs(W_TX_PAYLOAD, (uint8_t*)msg, sizeof(msg)); // write message to the TX FIFO
		
		nrf24_pulseCE(); // trigger transmission (STANDBY-1 ==> TX MODE)

		_delay_ms(800); // wait for (re-)transmission(s) is over (TX MODE ==> STANDBY-2)
		
		// power off nRF (STANDBY-2 ==> POWER DOWN)
		nrf24_writeReg(W_REGISTER | NRF_CONFIG,
			NRF24_CFG_PWR_DOWN | NRF24_CFG_TX_MODE | NRF24_CFG_CRC_2B | NRF24_CFG_CRC_EN | NRF24_CFG_IRQ_MASK_ALL);
		
		LED_CLR();
		_delay_ms(3000);
	};
}

#if	!defined(NRF24L01_DO_NOT_USE_MISO)
void testRecv() {
	uint8_t msg[NRF24_MAX_SIZE]; // in reality a smaller size is sufficient 

	nrf24_writeReg(W_REGISTER | EN_AA,		NRF24_PIPE_0);	// en autoack
	nrf24_writeReg(W_REGISTER | NRF_STATUS, NRF24_STATUS_CLEAR_ALL);	// clear status
	
	// set RX mode, enable CRC with 2 bytes, mask all IRQs, power on nRF radio (POWER DOWN ==> STANDBY-1)
	nrf24_writeReg(W_REGISTER | NRF_CONFIG,
	NRF24_CFG_PWR_UP | NRF24_CFG_RX_MODE | NRF24_CFG_CRC_2B | NRF24_CFG_CRC_EN | NRF24_CFG_IRQ_MASK_ALL);
	
	_delay_ms(5); // there should be 5ms delay to bring nRF from the POWER DOWN to the POWER UP state (the worst case)
		
	
	while(1) {
		uint8_t status = nrf24_readReg(R_REGISTER | NRF_STATUS); // get status

		// something received?
		if( status & NRF24_STATUS_RX_DR ) {
			
			// read everything what has been received from RX FIFO
			// (make sure you everything from RX FIFO otherwise RX FIFO will overflow eventually)
			// (if you are lazy, you can always do FLUSH_RX after each read but you loose unprocessed frames RX has 3 FIFOs)
			nrf24_readRegs(R_RX_PAYLOAD, msg, nrf24_readReg( R_RX_PL_WID )); 			
			
			nrf24_writeReg(W_REGISTER | NRF_STATUS, NRF24_STATUS_CLEAR_ALL); // clear received flag
			
			// toggle LED 
			if(msg[0] & 0x01)
				LED_SET();
			else
				LED_CLR();
		}
		
		_delay_ms(500);		
	};

}
#endif

