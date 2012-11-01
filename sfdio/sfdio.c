/*
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <util/twi.h>

#include "twi.h"


// generic buffer for TWI slave write
volatile uint8_t twibuf[8];

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
	DDRB = 0;
	DDRC = 0;
	DDRD = 1<<7;
	PORTB = 0xff;
	PORTC = 0xff;
	PORTD = 0xff;
}


void
twi_slave_read(volatile uint8_t *data)
{
}


uint8_t
twi_slave_read_prepare(uint8_t function)
{
	return 1;
}


uint8_t
twi_slave_write(uint8_t function, volatile uint8_t **data)
{
	*data = twibuf;
	twibuf[0] = 23;
	return 1;
}

int
main(void)
{
	cli();

	setup_clock();
	setup_pins();
	twi_setup();

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
