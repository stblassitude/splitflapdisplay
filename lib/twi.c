/*
 * Copyright (c) 2012, Stefan Bethke <stb@lassitu.de>
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 * 
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.  Redistributions in
 * binary form must reproduce the above copyright notice, this list of
 * conditions and the following disclaimer in the documentation and/or other
 * materials provided with the distribution.  THIS SOFTWARE IS PROVIDED BY
 * THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 * NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <stdio.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/atomic.h>
#include <util/twi.h>
#include "twi.h"

#define F_SCL	(100*1000L)
#define TWPS	(0)
#define F_TWPS	(1 << (TWPS*2))

volatile twi_status_t _twi_status;
volatile uint8_t _twi_sla;
volatile uint16_t _twi_idx;
volatile uint16_t _twi_dsz;
volatile uint8_t _twi_twsr;
volatile uint8_t _twi_timeout;
volatile uint8_t _twi_function;

volatile uint8_t twi_data[64]; // falsh page size

#if defined(TWI_DEBUG)
#define TW_DEBUG(x) _twi_debug(PSTR(x));

static void
_twi_debug(const char *pstr)
{
	char s[80];
	char *p;
	char c;
	
	sprintf(s, "TW: cr=%02x sr=0x%02x st=%d idx=%d cnt=%d ", TWCR, _twi_twsr, 
			_twi_status, _twi_idx, _twi_dsz);
	for (p=s; *p != '\0'; p++)
		debug_putchar(*p);
	while ((c = pgm_read_byte(pstr++)))
		debug_putchar(c);
	debug_putchar('\r');
	debug_putchar('\n');
}
#else
#define TW_DEBUG(x)
#endif


#define TWCR_DFLT (_BV(TWINT) | _BV(TWEN) | _BV(TWIE))

static inline void
_twi_start(void)
{
	TWCR = _BV(TWSTA) | _BV(TWEA) | _BV(TWINT) | _BV(TWEN) | _BV(TWIE);
	TW_DEBUG("start");
}


static inline void
_twi_stop(void)
{
	TWCR = _BV(TWSTO) | TWCR_DFLT;
}


static inline void
_twi_transfer(void)
{
	TWCR = _BV(TWEA) | TWCR_DFLT;
}

static inline void
_twi_transfer_last(void)
{
	TWCR = TWCR_DFLT;
}

static inline void
_twi_transfer_next(void)
{
	if (_twi_idx < _twi_dsz) {
		_twi_transfer();
	} else {
		_twi_transfer_last();
	}
}

static void
_twi_final(twi_status_t s)
{
	_twi_status = s;
	_twi_timeout = 0;
	_twi_stop();
	TW_DEBUG("final");
}

static inline uint8_t
_twi_wait(void)
{
	while (_twi_status != TWI_STATUS_SUCCESS &&
			_twi_status != TWI_STATUS_ERROR) {
	}
	return _twi_status;
}

ISR(TWI_vect)
{
	_twi_twsr = TWSR;
	switch(_twi_status) {
	case TWI_STATUS_SUCCESS:
	case TWI_STATUS_ERROR:
		switch (_twi_twsr) {
		case TW_SR_SLA_ACK:
		case TW_SR_GCALL_ACK:
			_twi_dsz = sizeof(twi_data);
			_twi_idx = 0;
			_twi_timeout = 255;
			_twi_transfer();
			_twi_status = TWI_STATUS_START;
			break;
		case TW_ST_SLA_ACK:
			_twi_dsz = twi_slave_write(_twi_function);
			_twi_idx = 0;
			TWDR = twi_data[_twi_idx++];
			_twi_transfer_next();
			_twi_status = TWI_STATUS_WAITDATA;
			break;
		default:
			// unexpected bus activity, issue stop
			_twi_timeout = 0;
			_twi_stop();
			TW_DEBUG("error");
		}
		break;
	case TWI_STATUS_START:
		switch(_twi_twsr) {
		case TW_START:
		case TW_REP_START:
			TW_DEBUG("starting");
			TWDR = _twi_sla;
			_twi_transfer();
			_twi_status = TWI_STATUS_WAITSLA;
			break;
		case TW_SR_DATA_ACK:
		case TW_SR_DATA_NACK:
		case TW_SR_GCALL_DATA_ACK:
		case TW_SR_GCALL_DATA_NACK:
			_twi_function = TWDR;
			_twi_dsz = twi_slave_read_prepare(_twi_function);
			_twi_transfer_next();
			_twi_status = TWI_STATUS_WAITDATA;
			break;
		default:
			_twi_final(TWI_STATUS_ERROR);
		}
		break;
	case TWI_STATUS_WAITSLA:
		// slave is responding
		switch (_twi_twsr) {
		case TW_MR_SLA_ACK:
			TW_DEBUG("MR SLA ack");
			_twi_transfer(); // XXX min read size 1
			_twi_status = TWI_STATUS_WAITDATA;
			break;
		case TW_MT_SLA_ACK:
			TW_DEBUG("MT SLA ack");
			if (_twi_idx < _twi_dsz) {
				TWDR = twi_data[_twi_idx++];
				_twi_transfer();
				_twi_status = TWI_STATUS_WAITDATA;
			} else {
				_twi_final(TWI_STATUS_SUCCESS);
			}
			break;
		default:
			TW_DEBUG("no ack");
			_twi_final(TWI_STATUS_ERROR);
		}
		break;
	case TWI_STATUS_WAITDATA: 
		// send/receive (more) data
		switch (_twi_twsr) {
		case TW_MR_DATA_ACK:
			TW_DEBUG("MR ack");
			twi_data[_twi_idx++] = TWDR;
			if (_twi_idx < _twi_dsz) {
				_twi_transfer();
			} else {
				_twi_transfer_last();
				_twi_status = TWI_STATUS_LAST;
			}
			break;
		case TW_MT_DATA_ACK:
			TW_DEBUG("MT ack");
			if (_twi_idx < _twi_dsz) {
				TWDR = twi_data[_twi_idx++];
				_twi_transfer();
			} else {
				_twi_final(TWI_STATUS_SUCCESS);
			}
			break;
		case TW_MT_DATA_NACK:
			TW_DEBUG("MT Nack");
			_twi_dsz = _twi_idx;
			_twi_final(TWI_STATUS_SUCCESS);
			break;
		case TW_SR_DATA_ACK:
		case TW_SR_DATA_NACK:
		case TW_SR_GCALL_DATA_ACK:
		case TW_SR_GCALL_DATA_NACK:
			if (_twi_idx < _twi_dsz)
				twi_data[_twi_idx++] = TWDR;
			_twi_transfer_next();
			break;
		case TW_SR_STOP:
			_twi_status = TWI_STATUS_SUCCESS;
			_twi_timeout = 0;
			TWCR = TWCR_DFLT | _BV(TWEA);
			twi_slave_read(_twi_function, _twi_idx+1);
			break;
		case TW_ST_DATA_ACK:
			if (_twi_idx < _twi_dsz) {
				TWDR = twi_data[_twi_idx++];
			} else {
				TWDR = 0;
			}
			_twi_transfer_next();
			break;
		case TW_ST_DATA_NACK:
		case TW_ST_LAST_DATA:
			_twi_status = TWI_STATUS_SUCCESS;
			_twi_timeout = 0;
			TWCR = TWCR_DFLT | _BV(TWEA);
			break;
		default:
			TW_DEBUG("r/w error");
			_twi_final(TWI_STATUS_ERROR);
		}
		break;
	case TWI_STATUS_LAST: 
		// received last byte
		switch (_twi_twsr) {
		case TW_MR_DATA_NACK:
			TW_DEBUG("completed");
			_twi_final(TWI_STATUS_SUCCESS);
			break;
		default:
			TW_DEBUG("read error");
			_twi_final(TWI_STATUS_ERROR);
		}
		break;
	}
}


void
twi_step_timeout(uint8_t ms)
{
	int t;
	if (_twi_timeout == 0)
		return;
	t = _twi_timeout - ms;
	if (t < 0) {
		TW_DEBUG("timeout");
		_twi_status = TWI_STATUS_ERROR;
		_twi_stop();
	} else {
		_twi_timeout = t;
	}
}


void
twi_setup(void)
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		_twi_status = TWI_STATUS_SUCCESS;
		// F_SCL = F_CPU / (16 + 2*TWBR * F_TWPS)
		TWBR = ((F_CPU / F_SCL) - 16 ) / (2 * F_TWPS);
		TWSR = TWPS;
		TWAR = 0x80;	// configurable
		TWCR = TWCR_DFLT | _BV(TWEA);
#if defined(TWAMR)
		TWAMR = 0; // all address bits must match
#endif
	}
}


void
twi_readwrite(uint8_t sla, uint8_t readwrite, uint16_t size)
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		if (_twi_status != TWI_STATUS_SUCCESS &&
				_twi_status != TWI_STATUS_ERROR)
			break;
		_twi_dsz = size;
		_twi_idx = 0;
		_twi_sla = (sla & (~TW_READ)) | readwrite;
		_twi_timeout = 255;
		_twi_status = TWI_STATUS_START;
		_twi_start();
	}	
}

twi_status_t
twi_wait(void)
{
	return _twi_wait();
}
