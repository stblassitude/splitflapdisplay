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

#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <util/twi.h>

#include "twi.h"
#include "sfd.h"


// 1 kHz with 1:64 prescaler
#define ONEKILOHERTZTIMERSTART (F_CPU / 64 / 1000)

static char identify_string[] = "twiboot r0";

static uint16_t heartbeat = 0;
static volatile uint8_t dummy;
static volatile uint8_t mainloop;

/*
 * Adjust the state of the heartbeat LED.
 */
static void
step_heartbeat_led(void)
{
	heartbeat++;
	if (heartbeat == 1) {
		PORTD |= 1<<7;
	} else if (heartbeat == 500) {
		PORTD &= ~(1<<7);
	} else if (heartbeat == 1000) {
		heartbeat = 0;
	}
}

/*
 * 1 kHz clock interrupt that services all main state machines.
 */
ISR(TIMER2_COMP_vect)
{
	twi_step_timeout(1);
	step_heartbeat_led();
}


/*
 * Sets up Timer/Counter 2 as a 1 kHz clock. WGM=2 (CTC), COM2=0 (OC2
 * disconnected)
 */
static void
setup_clock(void)
{
	TCNT2 = 0;
	OCR2 = ONEKILOHERTZTIMERSTART;
	TCCR2 = 0x0c;	// waveform generation mode 2, prescaler 1:64
	TIMSK = 0x80;	// only interrupt on compare match of Counter2
}

static void
setup_pins(void)
{
	DDRB = _BV(1);
	DDRC = 0;
	DDRD = 1<<7;
	PORTB = 0xff;
	PORTC = 0xff;
	PORTD = 0xff;
}


void
twi_slave_read(uint8_t function, uint8_t size)
{
	switch (function) {
	case SFDF_IDENTIFY:
		break;
	case SFDF_BYTE:
		if (size > 0) {
			dummy = twi_data[0];
			_delay_ms(10);
			PORTB ^= _BV(1);
		}
		break;
	}
}


uint8_t
twi_slave_read_prepare(uint8_t function)
{
	switch (function) {
	case SFDF_IDENTIFY:
		return 8;
	case SFDF_BYTE:
		return 1;
	}
	return 1;
}


uint8_t
twi_slave_write(uint8_t function)
{
	switch (function) {
	case SFDF_IDENTIFY:
		strcpy((char *)twi_data, identify_string);
		return strlen((char *)twi_data) + 1;
	case SFDF_BYTE:
		twi_data[0] = dummy;
		_delay_ms(10);
		PORTB ^= _BV(1);
		return 1;
	default:
		twi_data[0] = function;
		return 1;
	}
	return 0;
}

int
main(void)
{
	cli();
	// relocate interrupt vectors to boot section
	GICR = (1<<IVCE);
	GICR = (1<<IVSEL);

	setup_pins();
	setup_clock();
	twi_setup();

	dummy = 12;
		
	sei();
	
	/*
	 * enter power saving state until woken by interrupt
	 */	
	while(1) {
		set_sleep_mode(1); // ADC noise reduction
		sleep_enable();
		sei();
		sleep_cpu();
		sleep_disable();
		cli();
	}
}

