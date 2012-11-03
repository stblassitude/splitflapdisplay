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

typedef enum {
	TWI_STATUS_SUCCESS = 0,
	TWI_STATUS_ERROR,
	TWI_STATUS_START,
	TWI_STATUS_WAITSLA,
	TWI_STATUS_WAITDATA,
	TWI_STATUS_LAST,
} twi_status_t;


#if defined(TWI_DEBUG)
extern void debug_putchar(const char c);
#endif

#define TWI_EEPROM_SLA ((const uint8_t *)0x20)

extern volatile uint8_t twi_data[64+2]; // flash page size

void twi_setup(void);
void twi_step_timeout(uint8_t ms);
twi_status_t twi_wait(void);
void twi_xfer(uint8_t sla, uint8_t readwrite, uint16_t size);
int twi_writeread(uint8_t sla, uint8_t wrsz, uint8_t rdsz);
void twi_handle_write(uint8_t function, uint8_t size);
uint8_t twi_prep_write(uint8_t function);
uint8_t twi_handle_read(uint8_t function);
