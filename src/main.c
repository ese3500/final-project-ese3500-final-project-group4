#include <avr/io.h>
#include <util/delay.h>

#define F_CPU 16000000UL
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1

void USART_Init(unsigned int ubrr) {
    /* Set baud rate */
    UBRR0H = (unsigned char)(ubrr >> 8);
    UBRR0L = (unsigned char)ubrr;
    /* Enable receiver and transmitter */
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
    /* Set frame format: 8 data, 1 stop bit */
    UCSR0C = (3 << UCSZ00);
}

void USART_Transmit(unsigned char data) {
    /* Wait for empty transmit buffer */
    while (!(UCSR0A & (1 << UDRE0)))
        ;
    /* Put data into buffer, sends the data */
    UDR0 = data;
}

int main(void) {
    USART_Init(MYUBRR);
    while (1) {
        USART_Transmit('H');
        USART_Transmit('e');
        USART_Transmit('l');
        USART_Transmit('l');
        USART_Transmit('o');
        USART_Transmit('\r');  // Carriage return
        USART_Transmit('\n');  // New line
        _delay_ms(2000);       // Wait for 2 seconds
    }
    return 0;
}
