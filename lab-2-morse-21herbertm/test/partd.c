#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include "uart.h"
#define F_CPU 16000000UL
#define BAUD_RATE 9600
#define BAUD_PRESCALER (((F_CPU / (BAUD_RATE * 16UL))) - 1)

// PART D
void Initialize()
{
    // Disable global interrupts
    cli();
    DDRB &= ~(1<<DDB0);
    PORTB |= (1<<PORTB0);
    DDRB |= (1<<DDB1);

    TCCR1B &= ~(1<<ICES1);
    TIFR1 |=(1<<ICF1);
    TIMSK1 |=(1<<ICIE1);

    TCCR0B |= (1<<CS00);
    TCCR0B &= ~(1<<CS01);
    TCCR0B |= (1<<CS02);

    TCCR0A &= ~(1<<WGM00);
    TCCR0A |= (1<<WGM01);
    TCCR0B &= ~(1<<WGM02);
    OCR0A = 216;

    TIMSK0 |= (1<<OCIE0A);

} //INITIALIZE

// volitle long period = 0;
// long edge = 0;
volatile unsigned int count = 0;
volatile int read_lock = 0;
char String[25];
char* letters = "";

// volatile is used to indicate that the value of this variable might be changed by another part of the program
// asynchronously and that any access to this variable should not be optimized by the compiler.
volatile int letters_idx = 0;
char *morsecode[] = {".-","-...","-.-.","-..",".","..-.","--.","....","..",".---", "-.-",".-..","--","-.","---",".--.","--.-", ".-.","...","-","..-", "...-",".--","-..-","-.--","--..", "-----", ".----","..---","...--","....-", ".....", "-....", "--...","---..","----."};

char decode(){
    int i=0;
    for(;i<26;i++){
        //If a match is found, the function returns the ASCII value of the corresponding letter
        // (i.e. the index plus 65 for the letter A-Z), by using the expression (i + 65).
        if(strcmp(letters, morsecode[i]) == 0) return (i+65);
    }
    //loops through the next 10 elements of the morsecode array and compares each element with
    // the letters array.
    for(i=0;i<10;i++){
        //If a match is found, the function returns the ASCII value of the corresponding digit
        // (i.e. the index plus 0x30 for the digit 0-9), by using the expression (i + 0x30).
        if(strcmp(letters, morsecode[i+26]) == 0) return (i+0x30);
    }
    return '?';
}

ISR(TIMER0_COMPA_vect){
    count++;
    if(count>5){
        //the ISR clears (i.e. sets to 0) the second and third bits of
        // the PORTB register
        PORTB &= ~(1<<PORTB1);
        PORTB &= ~(1<<PORTB2);
    }
}
ISR(TIMER1_CAPT_vect){
    // it records the duration of the high state by multiplying the count variable by 13.88889 to get the
    // duration in milliseconds.
    if((PINB & 1) == 1){
        TCCR1B &= ~(1<<ICES1);
        volatile unsigned int millis = count * 13.88889;
        //sets the PORTB1 bit to high and adds a '.' to the letters
        // buffer
        if(millis>30 && read_lock==0){
            read_lock = 1;
            if(millis <= 200){
                PORTB |= (1<<PORTB1);
                letters[letters_idx++] = '.';
                letters[letters_idx] = 0;

                //sets the PORTB2 bit to high and adds a '-' to the letters buffer
            }else if(millis > 200 && millis <= 400){
                PORTB |= (1<<PORTB2);
                letters[letters_idx++] = '-';
                letters[letters_idx] = 0;

                // decodes the current content of the letters buffer and,
                // if it represents a recognizable character, transmits it via UART
            }else if(millis > 400){
                char c = decode();
                if(c!='?'){
                    sprintf(String,"%c",c);
                    UART_putstring(String);
                }
                letters_idx = 0;
                letters[letters_idx] = 0;
            }
        }
        // If the input signal on PINB is low, the ISR resets the count variable,
        // releases the read lock, and sets the ICES1 bit in the TCCR1B register to
        // trigger on the next rising edge
    }else{
        TCCR1B |= (1<<ICES1);
        count = 0;
        read_lock = 0;
    }

}
int main(void)
{
    Initialize();

    UART_init(BAUD_PRESCALER);
    sprintf(String,"Hello world! \n");
    UART_putstring(String);
    sei();
    while(1){
    }// WHILE
} // MAIN