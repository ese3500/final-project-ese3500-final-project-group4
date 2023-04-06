#include <avr/io.h>
#include <stdio.h>
//#include <avr/eeprom.h>
#include "inc/uart.c"
#define F_CPU 16000000UL
#define BAUD_RATE 9600
#define BAUD_PRESCALER (((F_CPU / (BAUD_RATE * 16UL))) - 1)

#define EEPROM_ADDR 0x0000
char String[25];
uint16_t food_pointer;

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


void reset_inventory(){
	for (int i=0; i<food_pointer; i++){
		eeprom_write_byte((uint8_t *)i, 0x0);
	}
	food_pointer = 0;
}
void store_food(uint8_t id, uint8_t expiration){
	//1 byte: hex code ('AA') for food id
	//2 byte: store expiration time in seconds. 2^16 seconds before full reset
	eeprom_write_byte((uint8_t *)food_pointer++, id);
	eeprom_write_byte((uint8_t *)food_pointer++, (expiration >> 8) & 0xFF);
	eeprom_write_byte((uint8_t *)food_pointer++, expiration & 0xFF);
	eeprom_write_byte((uint8_t *)0x0, food_pointer);
}
void poll_expirations(){
	int fp = eeprom_read_byte((uint8_t *)0x0);
	int current_time = ...;
	for (int i=1; i < fp; i+=3){
		uint8_t prod_id = eeprom_read_byte((uint8_t *)i);
		uint16_t exp_date = ((uint16_t)eeprom_read_byte((uint8_t *)i+1) << 8) | eeprom_read_byte((uint8_t *)i+2);
		if (exp_date > current_time && prod_id != 0){
			sprintf (String, "%d expired!\n", prod_id);
			UART_putstring(String);
			eeprom_write_byte((uint8_t *)i, 0);
			eeprom_write_byte((uint8_t *)i+1, 0);
			eeprom_write_byte((uint8_t *)i+2, 0);
		}
	}
}
int main(void)
{
	//EEPROM_ADDR=0 stores the stack pointer to the end of our food reference.
	//Each food block is two bytes, encoding
	UART_init(BAUD_PRESCALER);
	food_pointer = eeprom_read_byte((uint8_t *)0x0);
	return 0;
}
