#include <avr/io.h>
#include <util/twi.h>
#include <util/delay.h>
#include "uart.c"
#include "uart.h"
#include <stdio.h>

#define F_CPU 16000000UL
#define BAUD_RATE 9600
#define BAUD_PRESCALER (((F_CPU / (BAUD_RATE * 16UL))) - 1)
#define MYUBRR F_CPU/16/BAUD-1
#define BAUD 9600

#define TWI_START ((1<<TWINT)|(1<<TWSTA)|(1<<TWEN))
#define TWI_STOP ((1<<TWINT)|(1<<TWSTO)|(1<<TWEN))
#define TWI_WAIT while (!(TWCR & (1<<TWINT)))
#define TWI_FREQ 100000L  // Set I2C frequency to 100 kHz

#define DS1307_ADDR 0xD0

void uart_init(unsigned int ubrr);

uint8_t twi_start() {
    TWCR = TWI_START;
    TWI_WAIT;
    return TW_STATUS;
}

uint8_t twi_stop() {
    TWCR = TWI_STOP;
    return TW_STATUS;
}

uint8_t twi_write(uint8_t data) {
    TWDR = data;
    TWCR = (1<<TWINT) | (1<<TWEN);
    TWI_WAIT;
    return TW_STATUS;
}

uint8_t twi_read_ack() {
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
    TWI_WAIT;
    return TWDR;
}

uint8_t twi_read_nack() {
    TWCR = (1<<TWINT) | (1<<TWEN);
    TWI_WAIT;
    return TWDR;
}

void twi_init() {
    TWSR = 0;
    TWBR = ((F_CPU / TWI_FREQ) - 16) / 2;
}

void ds1307_set_time(uint8_t hour, uint8_t minute, uint8_t second) {
    twi_start();
    twi_write(DS1307_ADDR);
    twi_write(0x00); // set register pointer to 0x00
    twi_write(second);
    twi_write(minute);
    twi_write(hour);
    twi_stop();
}

void ds1307_read_time(uint8_t* hour, uint8_t* minute, uint8_t* second) {
    twi_start();
    twi_write(DS1307_ADDR);
    twi_write(0x00); // set register pointer to 0x00
    twi_stop();
    twi_start();
    twi_write(DS1307_ADDR | 1); // read mode
    *second = twi_read_ack();
    *minute = twi_read_ack();
    *hour = twi_read_nack();
    twi_stop();
}

int main() {
    uart_init(MYUBRR);
    twi_init();
    ds1307_set_time(12, 30, 0);
    while (1) {
        uint8_t hour, minute, second;
        ds1307_read_time(&hour, &minute, &second);
        char buf[16];
        sprintf(buf, "%02d:%02d:%02d\r\n", hour, minute, second);
        UART_putstring(buf);
        _delay_ms(1000);
    }
    return 0;
}
