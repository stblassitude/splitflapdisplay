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
#include <avr/boot.h>
#include <avr/eeprom.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/atomic.h>
#include <util/delay.h>
#include <util/twi.h>

#include "twi.h"
#include "sfd.h"
#include "twiboot.h"


// 1 kHz with 1:64 prescaler
#define ONEKILOHERTZTIMERSTART (F_CPU / 64 / 1000)

#define FLASHPAGESIZE (64)

static char identify_string[] = "twiboot r0";

static volatile uint16_t heartbeat = 0;
static volatile uint8_t eepromaddr;
static volatile uint8_t flashpage;
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


/*
 * Initialize all pins. B1 and D7 have LEDs on them.
 */
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


/*
 * Inform caller how much data can be handled in response to master write request.
 */
uint8_t
twi_prep_write(uint8_t function)
{
	switch (function) {
		case TWIBOOT_EEPROM:
		case TWIBOOT_FLASH:
			return sizeof(twi_data);
	}
	return 1;
}


/*
 * Perform action with received data.
 */
void
twi_handle_write(uint8_t function, uint8_t size)
{
	switch (function) {
	case TWIBOOT_IDENTIFY:
		break;
	case TWIBOOT_STATUS:
		break;
	case TWIBOOT_FLASH:
		if (size >= 1)
			flashpage = twi_data[0];
		if (size == FLASHPAGESIZE + 1) {
			ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
				eeprom_busy_wait();
				boot_page_erase(flashpage * FLASHPAGESIZE);
				boot_spm_busy_wait();
				for (uint8_t i = 0; i < FLASHPAGESIZE; i += 2) {
					uint16_t w;
					w = twi_data[i+1];
					w |= twi_data[i+2] << 8;
					boot_page_fill(flashpage * FLASHPAGESIZE + i, w);
				}
				boot_page_write(flashpage * FLASHPAGESIZE);
				boot_spm_busy_wait();
				boot_rww_enable();
			}
		}
		break;
	case TWIBOOT_EEPROM:
		if (size >= 1)
			eepromaddr = twi_data[0];
		if (size >= 2) {
			size--;
			for (uint8_t i = 0; i < size; i++) {
				PORTB ^= _BV(1);
				eeprom_write_byte((uint8_t *)(eepromaddr+i), twi_data[i+1]);
			}
		}
		break;
	case TWIBOOT_REBOOT:
		mainloop = 0;
		break;
	}
}


/*
 * Supply data in response to read request from master.
 */
uint8_t
twi_handle_read(uint8_t function)
{
	switch (function) {
	case TWIBOOT_IDENTIFY:
		strcpy((char *)twi_data, identify_string);
		return strlen((char *)twi_data) + 1;
	case TWIBOOT_STATUS:
		twi_data[0] = eepromaddr;
		PORTB ^= _BV(1);
		return 1;
	case TWIBOOT_FLASH:
		for (uint8_t i = 0; i < sizeof(twi_data); i++) {
			twi_data[i] = pgm_read_byte((uint8_t *)((uint16_t)flashpage * FLASHPAGESIZE + i));
		}
		return sizeof(twi_data);
	case TWIBOOT_EEPROM:
		for (uint8_t i = 0; i < sizeof(twi_data); i++)
			twi_data[i] = eeprom_read_byte((uint8_t *)(eepromaddr+i));
		return sizeof(twi_data);
	case TWIBOOT_REBOOT:
		return 0;
	default:
		twi_data[0] = function;
		return 1;
	}
	return 0;
}

int
main(void)
{
	if ((MCUCSR & _BV(EXTRF)) == 0) {
		// no external reset, jump to application
		asm("RJMP __vectors");
	}

	// relocate interrupt vectors to boot section
	GICR = (1<<IVCE);
	GICR = (1<<IVSEL);

	_delay_ms(500);

	setup_pins();
	setup_clock();
	twi_setup();

	sei();
	
	/*
	 * enter power saving state until woken by interrupt
	 */	
	mainloop = 1;
	while(mainloop) {
		set_sleep_mode(1); // ADC noise reduction
		sleep_enable();
		sei();
		sleep_cpu();
		sleep_disable();
		cli();
	}
	// perform a watchdog reset
	wdt_enable(WDTO_15MS);
	while(1) {
		// forever
	}
}

