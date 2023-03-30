//#include <wiringPi.h>
#define F_CPU 16000000UL  // AVR clock frequency in Hz
#define BAUD 9600         // USART baud rate
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#define PHOTO_PIN PB0 // pin for photo resistorrr
#include <stdio.h>
#include "../src/uart.c"

// Define the analog input pin for the LDR
#define LDR_PIN 0

// Define the threshold value for triggering the interrupt
#define LDR_THRESHOLD 512

// Flag variable for indicating when the interrupt has been triggered

#define F_CPU 16000000UL
#define BAUD_RATE 9600
#define BAUD_PRESCALER (((F_CPU / (BAUD_RATE * 16UL))) - 1)

char String[25];
int state = 0;
int main(void) {
    cli();
    /*ADMUX |= (1 << REFS0); // ref voltage to AVCC
    ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); //128 pres
    ADCSRA |= (1 << ADEN); // enable the ADC
    ADCSRA |= (1 << ADIE);*/

    /*ADCSRA |= (1 << ADIE); // Enable ADC interrupt
    ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Set ADC prescaler to 128
    ADMUX |= (1 << REFS0); // Set ADC reference voltage to AVCC
    DIDR0 |= (1 << LDR_PIN); // Disable digital input buffer for the LDR pin
    ADCSRA |= (1 << ADEN); // Enable ADC*/
    ADCSRA |= (1 << ADIE); // Enable ADC interrupt
    ADMUX |= (1 << REFS0); // Set ADC reference voltage to AVCC
    //DIDR0 |= (1 << LDR_PIN); // Disable digital input buffer for the LDR pin
    ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

    ADCSRA |= (1 << ADEN); // Enable ADC
    ADCSRA |= (1 << ADSC);
    UART_init(BAUD_PRESCALER);
    sei();


    // Enable global interrupts

    while (1) {
        /*ADCSRA |= (1 << ADSC); // start  conversion
        while (ADCSRA & (1 << ADSC));
        uint16_t adc = ADC;*/

    }

    return 0;
}

ISR(ADC_vect) {
    // Read the digital value of the LDR
    uint16_t ldr_value = ADC;
    //sprintf (String, "ADC: %d\n", ADC);
    //UART_putstring(String);

    // Check if the LDR value is below the threshold
    if (ldr_value < LDR_THRESHOLD) {
        // Set the flag variable to indicate that the interrupt has been triggered
        if (state==0){
            state = 1;
            sprintf (String, "Fridge is Closed!\n");
            UART_putstring(String);
        }

    }else{
        if (state==1){
            state = 0;
            sprintf (String, "Fridge is Open\n");
            UART_putstring(String);
        }
    }
    ADCSRA |= (1 << ADSC);
}