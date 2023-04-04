#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>

#define F_CPU 16000000UL
#define BAUD_RATE 9600
#define BAUD_PRESCALER (((F_CPU / (BAUD_RATE * 16UL))) - 1)

#define PHOTO_PIN PB0 // pin for photoresistor

#define SPEAKER_PIN PD3 // pin for the speaker
#define SPEAKER_VOLUME 255 // speaker volume

#define LDR_PIN 0 // pin for the LDR
#define LDR_THRESHOLD 512 // threshold for triggering the interrupt

char String[25];
int state = 0;

void init_PWM(void) {
	DDRD |= (1 << SPEAKER_PIN); // set speaker pin as output
	TCCR2A |= (1 << COM2B1) | (1 << WGM21) | (1 << WGM20); // set non-inverting mode and fast PWM
	TCCR2B |= (1 << CS22); // set prescaler to 64
}

void init_ADC(void) {
	ADCSRA |= (1 << ADIE); // Enable ADC interrupt
	ADMUX |= (1 << REFS0); // Set ADC reference voltage to AVCC
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
	ADCSRA |= (1 << ADEN); // Enable ADC
	ADCSRA |= (1 << ADSC);
}

void init_USART(void) {
	UBRR0H = (unsigned char)(BAUD_PRESCALER >> 8);
	UBRR0L = (unsigned char)BAUD_PRESCALER;
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);
	UCSR0C = (3 << UCSZ00);
}

void send_string(char* str) {
	while (*str != '\0') {
		while (!(UCSR0A & (1 << UDRE0)));
		UDR0 = *str++;
	}
}

void send_LDR_value(uint16_t value) {
	sprintf(String, "LDR value: %d\n", value);
	send_string(String);
}

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
int main(void) {
	cli();
	init_PWM();
	init_ADC();
	init_USART();
	sei(); // Enable global interrupts
	DDRB &= ~(1 << PHOTO_PIN); // set photoresistor pin as input
	PORTB |= (1 << PHOTO_PIN); // enable pull-up resistor
	EICRA |= (1 << ISC01); // set INT0 to trigger on falling edge
	EIMSK |= (1 << INT0); // enable INT0
	while (1) {
		// Wait for interrupts
	}
	return 0;
}
