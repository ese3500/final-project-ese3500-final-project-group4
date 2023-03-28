//#include <wiringPi.h>
#define F_CPU 16000000UL  // AVR clock frequency in Hz
#define BAUD 9600         // USART baud rate
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#define PHOTO_PIN PB0 // pin for photo resistorrr

volatile uint16_t lightThreshold = 800;  // Set threshold value for photoresistor

void Initialize()
{
    // ENABLE GLOBAL INTERRUPTS
    sei();

    // SET PHOTO PIN AS INPUT
    DDRB &= ~(1 << PHOTO_PIN);

    // ENABLE PULL-UP RESISTOR ON PHOTO PIN
    PORTB |= (1 << PHOTO_PIN);

    // ENABLE EXTERNAL INTERRUPT ON PHOTO PIN
    PCICR |= (1 << PCIE0);
    PCMSK0 |= (1 << PCINT0);
}

void USART_Init(void)
{
    // Set baud rate
    UBRR0H = (uint8_t)(F_CPU / 16 / BAUD - 1) >> 8;
    UBRR0L = (uint8_t)(F_CPU / 16 / BAUD - 1);
    // Enable receiver and transmitter
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
    // Set frame format: 8 data, 1 stop bit
    UCSR0C = (0 << USBS0) | (1 << UCSZ00) | (1 << UCSZ01);
}

void USART_Transmit(uint8_t data)
{
    // Wait for empty transmit buffer
    while (!(UCSR0A & (1 << UDRE0)));
    // Put data into buffer, sends the data
    UDR0 = data;
}

ISR(ADC_vect)
{
    uint16_t adcResult = ADC;
    if (UCSR0A & (1 << UDRE0)) {
        USART_Transmit(adcResult >> 8);
        USART_Transmit(adcResult);
    }
    if (adcResult > lightThreshold)
    {
        // Send signal to Python function to start running
        USART_Transmit('1');
    }
    else
    {
        // Send signal to Python function to stop running
        USART_Transmit('0');
    }
}

int main(void)
{
    Initialize();
    // Initialize USART communication
    USART_Init();

    // Set up ADC for photoresistor reading
    ADMUX = (0 << REFS1) | (1 << REFS0);  // Use AVcc as voltage reference
    ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);  // Enable ADC, set ADC interrupt, and set prescaler to 128

    // Set up interrupt for photoresistor reading
    sei();  // Enable global interrupts
    ADCSRA |= (1 << ADSC);  // Start first ADC conversion

    while (1)
    {
        // Do other stuff here if needed
    }
}

