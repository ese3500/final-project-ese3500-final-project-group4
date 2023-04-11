#include <avr/io.h>
#include <stdio.h>
#include <util/twi.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "uart.c"

#define F_CPU 16000000UL
#define BAUD_RATE 9600
#define BAUD_PRESCALER (((F_CPU / (BAUD_RATE * 16UL))) - 1)

#define SPEAKER_PIN PD3 // pin for the speaker
#define PHOTO_PIN PB0 // pin for photoresistor

#define EEPROM_ADDR 0x0000
char String[25];
uint16_t food_pointer;

#define MYUBRR F_CPU/16/BAUD_RATE-1

#define TWI_START ((1<<TWINT)|(1<<TWSTA)|(1<<TWEN))
#define TWI_STOP ((1<<TWINT)|(1<<TWSTO)|(1<<TWEN))
#define TWI_WAIT while (!(TWCR & (1<<TWINT)))
#define TWI_FREQ 100000L  // Set I2C frequency to 100 kHz

#define DS1307_ADDR 0xD0

int state = 0;

// Add the following lines from the second code
#define LDR_PIN 0 // pin for the LDR
#define LDR_THRESHOLD 512 // threshold for triggering the interrupt

//void UART_init(int ubrr);
void init_USART(void) {
    UBRR0H = (unsigned char)(BAUD_PRESCALER >> 8);
    UBRR0L = (unsigned char)BAUD_PRESCALER;
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
    UCSR0C = (3 << UCSZ00);
}
void USART_Transmit(unsigned char data) {
    /* Wait for empty transmit buffer */
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = data;
}

void init_PWM(void) {
    DDRD |= (1 << SPEAKER_PIN); // set speaker pin as output
    TCCR2A |= (1 << COM2B1) | (1 << WGM21) | (1 << WGM20); // set non-inverting mode and fast PWM
    TCCR2B |= (1 << CS22); // set prescaler to 64
}

void buzz(uint16_t frequency, uint16_t duration) {
    uint16_t period = F_CPU / 64 / frequency;
    OCR2A = period / 2;
    TCCR2B = (TCCR2B & 0b11111000) | 0b011; // Set prescaler to 64

    for (uint16_t i = 0; i < (frequency / 1000) * duration; ++i) {
        _delay_ms(1);
    }

    // Stop the speaker
    TCCR2B &= ~(1 << CS22);
    TCCR2B &= ~(1 << CS21);
    TCCR2B &= ~(1 << CS20); // Set prescaler to 0 to stop PWM
}

uint16_t read_ADC(uint8_t channel) {
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F); // Select the ADC channel
    ADCSRA |= (1 << ADSC); // Start the ADC conversion
    while (ADCSRA & (1 << ADSC)); // Wait for the conversion to complete
    return ADC; // Return the ADC value
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
void poll_expirations() {
    uint8_t fp = eeprom_read_byte((uint8_t *)0x0);
    uint8_t hour, minute, second;
    ds1307_read_time(&hour, &minute, &second);
    uint16_t current_time = minute * 100 + second;
    uint8_t expired = 0;

    for (int i = 1; i < fp; i += 3) {
        uint8_t prod_id = eeprom_read_byte((uint8_t *)i);
        uint16_t exp_date = ((uint16_t)eeprom_read_byte((uint8_t *)i + 1) << 8) | eeprom_read_byte((uint8_t *)i + 2);

        if (exp_date < current_time && prod_id != 0 && !expired) {
            expired = 1;
            sprintf(String, "%d expired!\n", prod_id);
            for (int q = 0; q < 25; q++) {
                if (String[q] == 0) break;
                USART_Transmit(String[q]);
            }
            init_PWM(); // Initialize the PWM
            buzz(2000, 3000); // Add the buzz function call here (frequency: 2000 Hz, duration: 3000 ms)

            eeprom_write_byte((uint8_t *)i, 0);
            eeprom_write_byte((uint8_t *)i + 1, 0);
            eeprom_write_byte((uint8_t *)i + 2, 0);
        } else {
            expired = 0;
        }
    }
}

void send_string(char* str) {
    while (*str != '\0') {
        while (!(UCSR0A & (1 << UDRE0)));
        UDR0 = *str++;
    }
}

void init_ADC(void) {
    ADCSRA |= (1 << ADIE); // Enable ADC interrupt
    ADMUX |= (1 << REFS0); // Set ADC reference voltage to AVCC
    ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
    ADCSRA |= (1 << ADEN); // Enable ADC
    ADCSRA |= (1 << ADSC);
}

/*
ISR(ADC_vect) {
    uint16_t ldr_value = ADC; // Read the digital value of the LDR
    //send_LDR_value(ldr_value);
    if (ldr_value < LDR_THRESHOLD) { // Check if the LDR value is below the threshold
        if (state == 0) { // If the fridge was previously open
            send_string("Fridge is closed.\n");
            state = 1;
        }
    } else { // If the LDR value is above the threshold
        if (state == 1) { // If the fridge was previously closed
            send_string("Fridge is open.\n");
            state = 0;
        }
    }
    ADCSRA |= (1 << ADSC);
}
 */

int main(void) {
    //EEPROM_ADDR=0 stores the stack pointer to the end of our food reference.
    //Each food block is three bytes, encoding
    //UART_init(BAUD_PRESCALER);
    init_USART();
    init_ADC();
    USART_Transmit('H');
    USART_Transmit('e');
    USART_Transmit('l');
    USART_Transmit('l');
    USART_Transmit('o');
    USART_Transmit('1');
    USART_Transmit('\r');
    USART_Transmit('\n');
    reset_inventory();
    food_pointer = eeprom_read_byte((uint8_t *) 0x0);
    if (food_pointer == 0) {
        food_pointer++;
    }
    USART_Transmit('o');
    twi_init();
    ds1307_init();
    USART_Transmit('o');
    store_food(1);
    store_food(2);
    store_food(3);
    USART_Transmit('H');
    USART_Transmit('e');
    USART_Transmit('l');
    USART_Transmit('l');
    USART_Transmit('o');
    USART_Transmit('2');
    USART_Transmit('\r');
    USART_Transmit('\n');
    cli();
    DDRB &= ~(1 << PHOTO_PIN); // set photoresistor pin as input
    PORTB |= (1 << PHOTO_PIN); // enable pull-up resistor
    while (1) {
        poll_expirations();

        ADCSRA |= (1 << ADSC); // Start an ADC conversion
        while (1) {
            uint16_t ldr_value = read_ADC(LDR_PIN); // Read the LDR value
            if (ldr_value < LDR_THRESHOLD) { // Check if the LDR value is below the threshold
                if (state == 0) { // If the fridge was previously open
                    send_string("Fridge is closed.\n");
                    state = 1;
                }
            } else { // If the LDR value is above the threshold
                if (state == 1) { // If the fridge was previously closed
                    send_string("Fridge is open.\n");
                    state = 0;
                }
            }

            poll_expirations();
            _delay_ms(100); // Adjust this delay according to your needs
        }

        return 0;
    }
}