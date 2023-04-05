#include <avr/io.h>
#include <util/twi.h>
#include <util/delay.h>
#include <stdio.h>

#include "inc/uart.c"
#define F_CPU 16000000UL
#define BAUD_RATE 9600
#define BAUD_PRESCALER (((F_CPU / (BAUD_RATE * 16UL))) - 1)

char String[25];
#define I2C_ADDR 0x68  // DS1307 RTC module address

void i2c_init(void)
{
	// Set I2C clock frequency to 100kHz (prescaler = 1)
	TWSR = 0x00;  // prescaler = 1
	TWBR = ((F_CPU / 100000UL) - 16) / 2;
}

void i2c_start(void)
{
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT)));
}

void i2c_stop(void)
{
	TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
	_delay_us(10);
}

void i2c_write(uint8_t data)
{
	TWDR = data;
	TWCR = (1 << TWINT) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT)));
}

uint8_t i2c_read_ack(void)
{
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
	while (!(TWCR & (1 << TWINT)));
	return TWDR;
}

uint8_t i2c_read_nack(void)
{
	TWCR = (1 << TWINT) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT)));
	return TWDR;
}

int main(void)
{
	uint8_t sec, min, hr;
	UART_init(BAUD_PRESCALER);
	// Initialize I2C communication
	i2c_init();

	// Send start condition and device address with write mode
	i2c_start();
	i2c_write((I2C_ADDR << 1) | 0);  // write mode
	i2c_write(0x00);  // set register pointer to 0x00 (seconds)
	i2c_stop();

	// Send start condition and device address with read mode
	i2c_start();
	i2c_write((I2C_ADDR << 1) | 1);  // read mode
	sec = i2c_read_ack();  // read seconds with ACK
	min = i2c_read_ack();  // read minutes with ACK
	hr = i2c_read_nack();  // read hours with NACK
	i2c_stop();

	// Print the current time to serial monitor
	sprintf(String,"Current time: %02x:%02x:%02x\n", hr, min, sec);
	UART_putstring(String);

	while(1)
	{
		// Do nothing
		// Send start condition and device address with read mode
		i2c_start();
		i2c_write((I2C_ADDR << 1) | 1);  // read mode
		sec = i2c_read_ack();  // read seconds with ACK
		min = i2c_read_ack();  // read minutes with ACK
		hr = i2c_read_nack();  // read hours with NACK
		i2c_stop();

		// Print the current time to serial monitor
		sprintf(String,"Current time: %02x:%02x:%02x\n", hr, min, sec);
		UART_putstring(String);
		_delay_ms(1000);
	}
}
