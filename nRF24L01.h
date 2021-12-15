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


//------------------------------------------------------------------------
// nrf24l01.h - API header for the nRF24L01 driver
//------------------------------------------------------------------------

#pragma once

//------------------------------------------------------------------------
// General
//------------------------------------------------------------------------
#define NRF24_MAX_SIZE	32
#define FIFTEEN_MICROSECONDS	15

//------------------------------------------------------------------------
// SPI Commands
//------------------------------------------------------------------------
#define R_REGISTER		0x00
#define W_REGISTER		0x20
#define REGISTER_MASK	0x1F
#define ACTIVATE		0x50
#define R_RX_PL_WID		0x60
#define R_RX_PAYLOAD	0x61
#define W_TX_PAYLOAD	0xA0
#define W_ACK_PAYLOAD	0xA8
#define FLUSH_TX		0xE1
#define FLUSH_RX		0xE2
#define REUSE_TX_PL		0xE3
#define RF24_NOP		0xFF


//------------------------------------------------------------------------
// Register map
//------------------------------------------------------------------------
#define NRF_CONFIG		0x00
#define EN_AA			0x01
#define EN_RXADDR		0x02
#define SETUP_AW		0x03
#define SETUP_RETR		0x04
#define RF_CH			0x05
#define RF_SETUP		0x06
#define NRF_STATUS		0x07
#define OBSERVE_TX		0x08
#define CD				0x09
#define RX_ADDR_P0		0x0A
#define RX_ADDR_P1		0x0B
#define RX_ADDR_P2		0x0C
#define RX_ADDR_P3		0x0D
#define RX_ADDR_P4		0x0E
#define RX_ADDR_P5		0x0F
#define TX_ADDR			0x10
#define RX_PW_P0		0x11
#define RX_PW_P1		0x12
#define RX_PW_P2		0x13
#define RX_PW_P3		0x14
#define RX_PW_P4		0x15
#define RX_PW_P5		0x16
#define FIFO_STATUS		0x17
#define DYNPD			0x1C
#define FEATURE			0x1D

//------------------------------------------------------------------------
// SETUP_RETR reg macros
//------------------------------------------------------------------------
#define ARD_SHIFT   4
#define ARC_SHIFT   0

//------------------------------------------------------------------------
// RF_SETUP reg macros
//------------------------------------------------------------------------
#define NRF24_PWR_MIN	0x00	//   0dBm
#define NRF24_PWR_LOW	0x02	//  -6dBm
#define NRF24_PWR_HIGH	0x04	// -12dBm
#define NRF24_PWR_MAX	0x06	// -18dBm

#define NRF24_SPEED_2Mbps		0x00
#define NRF24_SPEED_1Mbps		0x08
#define NRF24_SPEED_250kbps		0x20

//------------------------------------------------------------------------
// NRF_CONFIG reg macros
//------------------------------------------------------------------------
#define NRF24_CFG_RX_MODE	0x01
#define NRF24_CFG_TX_MODE	0x00

#define NRF24_CFG_PWR_UP	0x02
#define NRF24_CFG_PWR_DOWN	0x00

#define NRF24_CFG_CRC_2B	0x04
#define NRF24_CFG_CRC_1B	0x00

#define NRF24_CFG_CRC_EN	0x80
#define NRF24_CFG_CRC_OFF	0x00

#define NRF24_CFG_IRQ_MASK_MAX_RT	0x10
#define NRF24_CFG_IRQ_MASK_TX_DS	0x20
#define NRF24_CFG_IRQ_MASK_RX_DR	0x40
#define NRF24_CFG_IRQ_MASK_ALL	(NRF24_CFG_IRQ_MASK_MAX_RT | NRF24_CFG_IRQ_MASK_TX_DS | NRF24_CFG_IRQ_MASK_RX_DR)

//------------------------------------------------------------------------
// NRF_STATUS reg macros
//------------------------------------------------------------------------
#define NRF24_STATUS_CLEAR_ALL		0xFF

#define NRF24_STATUS_TX_FULL		0x01
#define NRF24_STATUS_MAX_RT			0x10
#define NRF24_STATUS_TX_DS			0x20
#define NRF24_STATUS_RX_DR			0x40

//------------------------------------------------------------------------
// FIFO_STATUS reg macros
//------------------------------------------------------------------------
#define NRF24_FIFO_RX_EMPTY			0x01
#define NRF24_FIFO_RX_FULL			0x02
#define NRF24_FIFO_TX_EMPTY			0x10
#define NRF24_FIFO_TX_FULL			0x20
#define NRF24_FIFO_TX_REUSE			0x80

//------------------------------------------------------------------------
// FEATURE reg macros
//------------------------------------------------------------------------
#define NRF24_FEATURE_EN_DPL		0x04
#define NRF24_FEATURE_EN_ACK_PAY	0x02
#define NRF24_FEATURE_EN_DYN_ACK	0x01

//------------------------------------------------------------------------
// Pipe mask macros
//------------------------------------------------------------------------
#define NRF24_PIPE_0		0x01
#define NRF24_PIPE_1		0x02
#define NRF24_PIPE_2		0x04
#define NRF24_PIPE_3		0x08
#define NRF24_PIPE_4		0x10
#define NRF24_PIPE_6		0x20

//------------------------------------------------------------------------
// API
//------------------------------------------------------------------------

void nrf24_init(void);
void nrf24_cmd(uint8_t cmd);

#ifndef NRF24L01_DO_NOT_USE_MISO
uint8_t nrf24_readReg(uint8_t reg);
void nrf24_readRegs(uint8_t cmd, uint8_t *buff, uint8_t size);
#endif

void nrf24_writeReg(uint8_t reg, uint8_t value);
void nrf24_writeRegs(uint8_t reg, const uint8_t *buff, uint8_t size);
void nrf24_pulseCE(void);
void nrf24_pulseCE_ms(uint16_t millis);
