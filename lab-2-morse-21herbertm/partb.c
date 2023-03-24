#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#define F_CPU 16000000UL
#include <util/delay.h>

// PART B CODE FOR QUESTION 5

ISR(TIMER1_CAPT_vect){
        if((PINB & 1)==1){
            PORTB &=~(1<<PORTB5);
            TCCR1B &=~(1<<ICES1);
        } else {
            PORTB |=(1<<PORTB5);
            TCCR1B |=(1<<ICES1);
        }
}
void Initialize()
{
    // Disable global interrupts
    cli();
    // You must use PB0 for input capture.
    // When port PB0 is used for input capture in this fashion,
    // it is referred to as ICP1 in the literature.
    DDRB &= ~(1<<DDB0);
    PORTB |= (1<<PORTB0); // PULL UP RES
    DDRB |= (1<<DDB5); //   LED
    // looking for the rising edge
    TCCR1B &= ~(1<<ICES1);
    TIFR1 |=(1<<ICF1);
    TIMSK1 |=(1<<ICIE1);
} //INITIALIZE

int main(void)
{
    Initialize();
    sei();
    while(1){
    }// WHILE
} // MAIN