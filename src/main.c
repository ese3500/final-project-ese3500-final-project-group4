#include <avr/io.h>
#include <stdio.h>
//#include <avr/eeprom.h>
#include <util/twi.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "inc/uart.c"
#define F_CPU 16000000UL
#define BAUD_RATE 9600
#define BAUD_PRESCALER (((F_CPU / (BAUD_RATE * 16UL))) - 1)

#define EEPROM_ADDR 0x0000
char String[25];
uint16_t food_pointer;

#define MYUBRR F_CPU/16/BAUD_RATE-1

#define TWI_START ((1<<TWINT)|(1<<TWSTA)|(1<<TWEN))
#define TWI_STOP ((1<<TWINT)|(1<<TWSTO)|(1<<TWEN))
#define TWI_WAIT while (!(TWCR & (1<<TWINT)))
#define TWI_FREQ 100000L  // Set I2C frequency to 100 kHz

#define DS1307_ADDR 0xD0

//void UART_init(int ubrr);
void init_USART(void) {
	UBRR0H = (unsigned char)(BAUD_PRESCALER >> 8);
	UBRR0L = (unsigned char)BAUD_PRESCALER;
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);
	UCSR0C = (3 << UCSZ00);
}
void USART_Transmit(unsigned char data) {
	/* Wait for empty transmit buffer */
	while (!(UCSR0A & (1 << UDRE0)))
	;
	UDR0 = data;
}
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

void ds1307_init() {
	uint8_t hour, minute, second;
	twi_start();
	twi_write(DS1307_ADDR);
	twi_write(0x00); // set register pointer to 0x00
	twi_stop();
	twi_start();
	twi_write(DS1307_ADDR | 1); // read mode
	second = twi_read_ack();
	minute = twi_read_ack();
	hour = twi_read_nack();
	twi_stop();
	if (hour == 0 && minute == 0 && second == 0) {
		// RTC has not been set, set the time
		ds1307_set_time(12, 30, 0);
	}
}

void eeprom_write_byte(unsigned int address, unsigned char data)
{
	while(EECR & (1<<EEPE)); // Wait for previous write to finish
	EEAR = address;
	EEDR = data; // Load data into EEPROM data register
	EECR |= (1<<EEMPE);
	EECR |= (1<<EEPE);
}

unsigned char eeprom_read_byte(unsigned int address)
{
	while(EECR & (1<<EEPE)); // Wait for previous write to finish
	EEAR = address;
	EECR |= (1<<EERE);
	return EEDR;
}

int expiration_times[] = {10,20,50};


void reset_inventory(){
	for (int i=0; i<100; i++){
		eeprom_write_byte((uint8_t *)i, 0x0);
	}
	food_pointer = 0;
}
void store_food(uint8_t id){
	//1 byte: hex code ('AA') for food id
	//2 byte: store expiration time in seconds. 2^16 seconds before full reset
	uint8_t hour, minute, second;
	ds1307_read_time(&hour, &minute, &second);
	int current_time = minute * 100 + second;
	int expiration = current_time + expiration_times[id-1];
	/*sprintf(String, "Expires at %d\n", expiration);
	UART_putstring(String);
	sprintf(String, "Writing to %d, %d\n", food_pointer, id);
	UART_putstring(String);*/
	
	eeprom_write_byte((uint8_t *)food_pointer++, id);
	eeprom_write_byte((uint8_t *)food_pointer++, (expiration >> 8) & 0xFF);
	eeprom_write_byte((uint8_t *)food_pointer++, expiration & 0xFF);
	eeprom_write_byte((uint8_t *)0x0, food_pointer);
}
void poll_expirations(){
	uint8_t fp = eeprom_read_byte((uint8_t *)0x0);
	//int current_time = ...;
	uint8_t hour, minute, second;
	ds1307_read_time(&hour, &minute, &second);
	uint16_t current_time = minute * 100 + second;
	for (int i=1; i < fp; i+=3){
		uint8_t prod_id = eeprom_read_byte((uint8_t *)i);
		uint16_t exp_date = ((uint16_t)eeprom_read_byte((uint8_t *)i+1) << 8) | eeprom_read_byte((uint8_t *)i+2);
		//sprintf(String, "Checking expiration %d, current %d, %d\n", exp_date, current_time, exp_date > current_time);
		//UART_putstring(String);
		if (exp_date < current_time && prod_id != 0){
			sprintf (String, "%d expired!\n", prod_id);
			for (int q = 0; q < 25; q++){
				if(String[q]==0) break;
				USART_Transmit(String[q]);
			}
			eeprom_write_byte((uint8_t *)i, 0);
			eeprom_write_byte((uint8_t *)i+1, 0);
			eeprom_write_byte((uint8_t *)i+2, 0);
		}
	}
}
int main(void)
{
	//EEPROM_ADDR=0 stores the stack pointer to the end of our food reference.
	//Each food block is three bytes, encoding
	//UART_init(BAUD_PRESCALER);
	init_USART();
	reset_inventory();
	food_pointer = eeprom_read_byte((uint8_t *)0x0);
	if (food_pointer==0){
		food_pointer++;
	}
	twi_init();
	ds1307_init();
	store_food(1);
	store_food(2);
	store_food(3);
	USART_Transmit('H');
	USART_Transmit('e');
	USART_Transmit('l');
	USART_Transmit('l');
	USART_Transmit('o');
	USART_Transmit('\r');
	USART_Transmit('\n');
	while (1) {
		poll_expirations();
		_delay_ms(20000);
	}
	
	return 0;
}
