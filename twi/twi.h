/*
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

void twi_setup(void);
void twi_step_timeout(uint8_t ms);
twi_status_t twi_wait(void);
void twi_readwrite(uint8_t sla, uint8_t readwrite, volatile uint8_t *data, uint16_t size);
void twi_slave_read(volatile uint8_t *data);
uint8_t twi_slave_read_prepare(uint8_t function);
uint8_t twi_slave_write(uint8_t function, volatile uint8_t **data);
