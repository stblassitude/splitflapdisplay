/*
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

#define LED_CONFIG	(DDRD |= (1<<6))
#define LED_ON		(PORTD |= (1<<6))
#define LED_OFF		(PORTD &= ~(1<<6))
#define CPU_PRESCALE(n) (CLKPR = 0x80, CLKPR = (n))

#define F_SCL	(100*1000L)
#define TWPS	(0)
#define F_TWPS	(1 << (TWPS*2))

typedef enum {
	TWSTATUS_IDLE = 0,
	TWSTATUS_ERROR,
	TWSTATUS_SUCCESS,
	TWSTATUS_START,
	TWSTATUS_WAITSLA,
	TWSTATUS_WAITDATA,
	TWSTATUS_LAST,
} twstatus_t;

#define TWBUFFERSZ 70

volatile twstatus_t twstatus;
volatile uint8_t twsla;
volatile uint8_t twbuffer[TWBUFFERSZ];
volatile uint8_t twbufferidx;
volatile uint8_t twbuffercnt;
volatile uint8_t twsr;
volatile uint8_t twtimeout;


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

#define TW_DEBUG(x) tw_debug(PSTR(x));


static void
tw_debug(const char *pstr)
{
	char s[80];
	char *p;
	char c;
	
	sprintf(s, "TW: cr=%02x sr=0x%02x st=%d idx=%d cnt=%d ", TWCR, twsr, 
			twstatus, twbufferidx, twbuffercnt);
	for (p=s; *p != '\0'; p++)
		usb_serial_putchar(*p);
	while ((c = pgm_read_byte(pstr++)))
		usb_serial_putchar(c);
	usb_serial_putchar('\r');
	usb_serial_putchar('\n');
}

#define TWCR_DFLT (_BV(TWINT) | _BV(TWEN) | _BV(TWIE))

static inline void
tw_start(void)
{
	TWCR = _BV(TWSTA) | _BV(TWEA) | _BV(TWINT) | _BV(TWEN) | _BV(TWIE);
	TW_DEBUG("start");
}


static inline void
tw_stop(void)
{
	TWCR = _BV(TWSTO) | TWCR_DFLT;
}


static inline void
tw_transfer(void)
{
	TWCR = _BV(TWEA) | TWCR_DFLT;
}

static inline void
tw_transfer_last(void)
{
	TWCR = TWCR_DFLT;
}

static void
tw_final(twstatus_t s)
{
	twstatus = s;
	twtimeout = 0;
	tw_stop();
	TW_DEBUG("final");
}

static inline uint8_t
tw_status(void)
{
	return TW_STATUS;
}

static inline uint8_t
tw_wait(void)
{
	while (twstatus != TWSTATUS_IDLE &&
			twstatus != TWSTATUS_SUCCESS &&
			twstatus != TWSTATUS_ERROR) {
	}
	return twstatus;
}

ISR(TWI_vect)
{
	twsr = tw_status();
	switch(twstatus) {
	case TWSTATUS_IDLE:
	case TWSTATUS_SUCCESS:
	case TWSTATUS_ERROR:
		// unexpected bus activity, issue stop
		twtimeout = 0;
		tw_stop();
		TW_DEBUG("error");
		break;
	case TWSTATUS_START:
		// start sent
		if (twsr == TW_START || twsr == TW_REP_START) {
			TW_DEBUG("starting");
			TWDR = twsla;
			tw_transfer();
			twstatus = TWSTATUS_WAITSLA;
		} else {
			tw_final(TWSTATUS_ERROR);
		}
		break;
	case TWSTATUS_WAITSLA:
		// slave is responding
		switch (twsr) {
		case TW_MR_SLA_ACK:
			TW_DEBUG("MR SLA ack");
			tw_transfer(); // XXX min read size 1
			twstatus = TWSTATUS_WAITDATA;
			break;
		case TW_MT_SLA_ACK:
			TW_DEBUG("MT SLA ack");
			if (twbufferidx < twbuffercnt) {
				TWDR = twbuffer[twbufferidx++];
				tw_transfer();
				twstatus = TWSTATUS_WAITDATA;
			} else {
				tw_final(TWSTATUS_SUCCESS);
			}
			break;
		default:
			TW_DEBUG("no ack");
			tw_final(TWSTATUS_ERROR);
		}
		break;
	case TWSTATUS_WAITDATA: 
		// send/receive (more) data
		switch (twsr) {
		case TW_MR_DATA_ACK:
			TW_DEBUG("MR ack");
			twbuffer[twbufferidx++] = TWDR;
			if (twbufferidx < twbuffercnt) {
				tw_transfer();
			} else {
				tw_transfer_last();
				twstatus = TWSTATUS_LAST;
			}
			break;
		case TW_MT_DATA_ACK:
			TW_DEBUG("MT ack");
			if (twbufferidx < twbuffercnt) {
				TWDR = twbuffer[twbufferidx++];
				tw_transfer();
			} else {
				tw_final(TWSTATUS_SUCCESS);
			}
			break;
		case TW_MT_DATA_NACK:
			TW_DEBUG("MT Nack");
			twbuffercnt = twbufferidx;
			tw_final(TWSTATUS_SUCCESS);
			break;
		default:
			TW_DEBUG("r/w error");
			tw_final(TWSTATUS_ERROR);
		}
		break;
	case TWSTATUS_LAST: 
		// received last byte
		switch (twsr) {
		case TW_MR_DATA_NACK:
			TW_DEBUG("completed");
			tw_final(TWSTATUS_SUCCESS);
			break;
		default:
			TW_DEBUG("read error");
			tw_final(TWSTATUS_ERROR);
		}
		break;
	}
}


static inline void
step_tw_timeout(void)
{
	if (twtimeout == 0)
		return;
	if (--twtimeout == 0) {
		TW_DEBUG("timeout");
		twstatus = TWSTATUS_ERROR;
		tw_stop();
	}
}


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
	step_tw_timeout();
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


static void
setup_twi(void)
{
	// F_SCL = F_CPU / (16 + 2*TWBR * F_TWPS)
	TWBR = ((F_CPU / F_SCL) - 16 ) / (2 * F_TWPS);
	TWSR = TWPS;
	//TWAR = 0x80;
	//TWAMR = 0; // all address bits must match
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


// parse a user command and execute it, or print an error message
//
static void
parse_and_execute_command(const char *buf, uint8_t num)
{
	char s[80];
	if (num == 0)
		return;
		
	if (num == 1) {
		switch(buf[0]) {
		case 'w':
			send_pstr(PSTR("TWI writing pointer=0\n"));
			twsla = 0x9e;
			twbuffer[0] = 0;
			twbuffercnt = 1;
			twbufferidx = 0;
			twtimeout = 255;
			twstatus = TWSTATUS_START;
			cli();
			tw_start();
			sei();
			if (tw_wait() != TWSTATUS_SUCCESS) {
				send_pstr(PSTR("error writing pointer\n"));
				return;
			}
			send_pstr(PSTR("\nTWI reading temp.\n"));
			twsla = 0x9f;
			twbuffercnt = 2;
			twbufferidx = 0;
			twtimeout = 255;
			twstatus = TWSTATUS_START;
			cli();
			tw_start();
			sei();
			tw_wait();
			send_pstr(PSTR("\nTWI test done.\n"));
			sprintf(s, "temp = %d\n", twbuffer[0]);
			send_str(s, strlen(s));
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
	setup_twi();
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

