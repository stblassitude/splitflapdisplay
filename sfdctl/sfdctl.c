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
#include <stdint.h>
#include <util/delay.h>
#include <util/twi.h>
#include "usb_serial.h"

#include "twi.h"
#include "twiboot.h"
#include "sfd.h"

#define LED_CONFIG	(DDRD |= (1<<6))
#define LED_ON		(PORTD |= (1<<6))
#define LED_OFF		(PORTD &= ~(1<<6))
#define CPU_PRESCALE(n) (CLKPR = 0x80, CLKPR = (n))

#define F_SCL	(100*1000L)
#define TWPS	(0)
#define F_TWPS	(1 << (TWPS*2))


static uint8_t sla = 0xfe;


static void
send_pstr(const char *s)
{
	char c;
	while (1) {
		c = pgm_read_byte(s++);
		if (!c) break;
		if (c == '\n')
			usb_serial_putchar('\r');
		usb_serial_putchar(c);
	}
}


static void
send_str(const char *s, uint8_t len)
{
	while (len-- > 0) {
		if (*s == '\n')
			usb_serial_putchar('\r');
		usb_serial_putchar(*s++);
	}
}


#if defined(TWI_DEBUG)
void
debug_putchar(const char c)
{
	usb_serial_putchar(c);
}
#endif

// 1 kHz with 1:64 prescaler
#define ONEKILOHERTZTIMERSTART (F_CPU / 64 / 1000)

static uint16_t heartbeat = 0;

/*
 * Adjust the state of the heartbeat LED.
 */
static void
step_heartbeat_led(void)
{
	heartbeat++;
	if (heartbeat == 1) {
		PORTD |= 1<<6;
	} else if (heartbeat == 900) {
		PORTD &= ~(1<<6);
	} else if (heartbeat == 1000) {
		heartbeat = 0;
	}
}

/*
 * 1 kHz clock interrupt that services all main state machines.
 */
ISR(TIMER0_COMPA_vect)
{
	step_heartbeat_led();
	twi_step_timeout(1);
}

/*
 * Sets up Timer/Counter 0 as a 1 kHz clock. WGM=2 (CTC), COM2=0 (OC0A/B
 * disconnected)
 */
static void
setup_clock(void)
{
	TCNT0 = 0;
	OCR0A = ONEKILOHERTZTIMERSTART;
	TCCR0A = 0x02;	// waveform generation mode 2
	TCCR0B = 0x03; // prescaler 1:64
	TIMSK0 = _BV(OCIE0A);	// only interrupt on compare match of Counter0
}


// Receive a string from the USB serial port.  The string is stored
// in the buffer and this function will not exceed the buffer size.
// A carriage return or newline completes the string, and is not
// stored into the buffer.
// The return value is the number of characters received, or 255 if
// the virtual serial connection was closed while waiting.
//
static uint8_t
recv_str(char *buf, uint8_t size)
{
	int16_t r;
	uint8_t count=0;

	while (count < size) {
		r = usb_serial_getchar();
		if (r != -1) {
			if (r == '\r' || r == '\n')
				return count;
			if (r >= ' ' && r <= '~') {
				*buf++ = r;
				usb_serial_putchar(r);
				count++;
			}
			if ((r == 8 || r == 127) && count > 0) {
				buf--;
				count--;
				usb_serial_putchar(8);
				usb_serial_putchar(' ');
				usb_serial_putchar(8);
			}
		} else {
			if (!usb_configured() ||
			  !(usb_serial_get_control() & USB_SERIAL_DTR)) {
				// user no longer connected
				return 255;
			}
			// just a normal timeout, keep waiting
		}
	}
	return count;
}


uint8_t
twi_prep_write(uint8_t function)
{
	return 1;
}


void
twi_handle_write(uint8_t function, uint8_t size)
{
}


uint8_t
twi_handle_read(uint8_t function)
{
	return 0;
}


static inline int
hexdigit(char c)
{
	if (c >= '0' && c <= '9')
		return c - '0';
	else if (c >= 'a' && c <= 'f')
		return c - 'a' + 10;
	else if (c >= 'A' && c <= 'F')
		return c - 'A' + 10;
	else
		return -1;
}


static int
hexbyte(const char *p)
{
	uint8_t r = 0;
	int i;

	i = hexdigit(*p++);
	if (i < 0)
		return -1;
	r = (uint8_t)i << 4;	
	i = hexdigit(*p++);
	if (i < 0)
		return -1;
	r |= (uint8_t)i;
	return r;
}


// parse a user command and execute it, or print an error message
//
static void
parse_and_execute_command(const char *buf, uint8_t num)
{
	char s[80];
	int d;
	if (num == 0)
		return;
		
	if (num >= 1) {
		switch(buf[0]) {
		case '=':
			// set SLA
			if (num < 3) {
				send_pstr(PSTR("error: format is =xx\n"));
				return;
			}
			d = hexbyte(buf+1);
			if (d < 0) {
				send_pstr(PSTR("error: illegal hex addr\n"));
				return;
			}
			sla = d;
			sprintf(s, "SLA=%02x\n", sla);
			send_str(s, strlen(s));
			return;
		case 'i':
			twi_data[0] = SFDF_IDENTIFY;
			d = twi_writeread(sla, 1, 20);
			if (d < 0) {
				send_pstr(PSTR("error identifying\n"));
			} else {
				sprintf(s, "identify: %s\n", twi_data);
				send_str(s, strlen(s));
				for (uint8_t i = 0; i < d; i++) {
					sprintf(s, "%02x ", twi_data[i]);
					send_str(s, strlen(s));
				}
				send_pstr(PSTR("\n"));
			}
			return;
		case 's':
			twi_data[0] = TWIBOOT_STATUS;
			twi_xfer(sla, TW_WRITE, 1);
			if (twi_wait() != TWI_STATUS_SUCCESS) {
				send_pstr(PSTR("error writing sfdio\n"));
			}
			twi_xfer(sla, TW_READ, 1);
			if (twi_wait() != TWI_STATUS_SUCCESS) {
				send_pstr(PSTR("error reading sfdio\n"));
			} else {
				sprintf(s, "status = 0x%02x\n", twi_data[0]);
				send_str(s, strlen(s));
			}
			return;
		case 'e':
			if (num < 3) {
				send_pstr(PSTR("error: format is exx\n"));
				return;
			}
			d = hexbyte(buf+1);
			if (d < 0) {
				send_pstr(PSTR("error: illegal hex addr\n"));
				return;
			}
			twi_data[0] = TWIBOOT_EEPROM;
			twi_data[1] = d;
			if (num > 3) {
				if (buf[3] != '=' || num != 6) {
					send_pstr(PSTR("error: format us exx=xx\n"));
					return;
				}
				d = hexbyte(buf+4);
				if (d < 0) {
					send_pstr(PSTR("error: illegal hex value\n"));
					return;
				}
				twi_data[2] = d;
				d = twi_writeread(sla, 3, 1);
			} else {
				d = twi_writeread(sla, 2, 1);
			}
			if (d < 0) {
				send_pstr(PSTR("error reading eeprom\n"));
			} else {
				sprintf(s, "eeprom: %02x\n", twi_data[0]);
				send_str(s, strlen(s));
			}
			return;
		case 'f':
			if (num < 3) {
				send_pstr(PSTR("error: format is fxx\n"));
				return;
			}
			d = hexbyte(buf+1);
			if (d < 0) {
				send_pstr(PSTR("error: illegal hex addr\n"));
				return;
			}
			twi_data[0] = TWIBOOT_FLASH;
			twi_data[1] = d;
			d = twi_writeread(sla, 2, 64);
			if (d < 0) {
				send_pstr(PSTR("error reading flash\n"));
			} else {
				sprintf(s, "flash: ");
				send_str(s, strlen(s));
				for (uint8_t i = 0; i < 64; i++) {
					sprintf(s, "%02x ", twi_data[i]);
					send_str(s, strlen(s));
					if (i % 16 == 15)
						send_pstr(PSTR("\n       "));
				}
				send_pstr(PSTR("\n"));
			}
			return;
		case 'F':
			if (num < 3) {
				send_pstr(PSTR("error: format is fxx\n"));
				return;
			}
			d = hexbyte(buf+1);
			if (d < 0) {
				send_pstr(PSTR("error: illegal hex addr\n"));
				return;
			}
			twi_data[0] = TWIBOOT_FLASH;
			twi_data[1] = d;
			for (uint8_t i = 2; i < 64+2; i++) {
				twi_data[i] = i;
			}
			twi_xfer(sla, TW_WRITE, 64+2);
			if (twi_wait() != TWI_STATUS_SUCCESS)
				send_pstr(PSTR("error writing flash\n"));
			return;
		case 'u':
			twi_data[0] = SFDF_BYTE;
			twi_xfer(sla, TW_WRITE, 1);
			if (twi_wait() != TWI_STATUS_SUCCESS) {
				send_pstr(PSTR("error writing sfdio\n"));
			}
			send_pstr(PSTR("update sfdio.\n"));
			twi_xfer(sla, TW_READ, 1);
			if (twi_wait() != TWI_STATUS_SUCCESS) {
				send_pstr(PSTR("error reading sfdio\n"));
			} else {
				sprintf(s, "sfdio = %d\n", twi_data[0]);
				send_str(s, strlen(s));
			}
			twi_data[1] = twi_data[0] + 1;
			twi_data[0] = SFDF_BYTE;
			twi_xfer(sla, TW_WRITE, 2);
			if (twi_wait() != TWI_STATUS_SUCCESS) {
				send_pstr(PSTR("error writing sfdio\n"));
			}
			send_pstr(PSTR("\nupdate sfdio done.\n"));
			return;
		case 't':
			send_pstr(PSTR("LM75 set pointer=0\n"));
			twi_data[0] = 0;
			twi_xfer(0x9e, TW_WRITE, 1);
			if (twi_wait() != TWI_STATUS_SUCCESS) {
				send_pstr(PSTR("error writing pointer\n"));
				return;
			}
			send_pstr(PSTR("LM75 reading temp.\n"));
			twi_xfer(0x9e, TW_READ, 2);
			if (twi_wait() != TWI_STATUS_SUCCESS) {
				send_pstr(PSTR("error reading temp\n"));
			} else {
				sprintf(s, "temp = %d\n", twi_data[0]);
				send_str(s, strlen(s));
			}
			send_pstr(PSTR("\nTWI test done.\n"));
			return;
		}
	}

	// otherwise, error message
	send_pstr(PSTR("Unknown command \""));
	send_str(buf, num);
	send_pstr(PSTR("\"\n"));
}


int
main(void)
{
	char buf[32];
	uint8_t n;

	// set for 16 MHz clock, and turn on the LED
	CPU_PRESCALE(0);
	LED_CONFIG;
	LED_ON;
	twi_setup();
	setup_clock();
	sei();

	// initialize the USB, and then wait for the host
	// to set configuration.  If the Teensy is powered
	// without a PC connected to the USB port, this 
	// will wait forever.
	usb_init();
	while (!usb_configured()) /* wait */ ;
	_delay_ms(1000);

	while (1) {
		// wait for the user to run their terminal emulator program
		// which sets DTR to indicate it is ready to receive.
		while (!(usb_serial_get_control() & USB_SERIAL_DTR)) /* wait */ ;

		// discard anything that was received prior.  Sometimes the
		// operating system or other software will send a modem
		// "AT command", which can still be buffered.
		usb_serial_flush_input();

		// print a nice welcome message
		send_pstr(PSTR("\nSplit Flap Display Controller Console\n\n"));

		// and then listen for commands and process them
		while (1) {
			send_pstr(PSTR("> "));
			n = recv_str(buf, sizeof(buf));
			if (n == 255) break;
			send_pstr(PSTR("\n"));
			parse_and_execute_command(buf, n);
		}
	}
}

