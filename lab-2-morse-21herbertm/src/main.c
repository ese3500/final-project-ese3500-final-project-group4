/*
 *
 * Created By Melanie Herbert
 */
#include <avr/io.h>
#include <string.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include <stdio.h>
#include <stdlib.h>

#include "uart.h"

#define F_CPU 16000000UL
#define BAUD_RATE 9600
#define BAUD_PRESCALER (((F_CPU / (BAUD_RATE * 16UL))) - 1)

volatile int detectedEdge = 0;
volatile int isLightOn = 0;
volatile int shouldAddSpace = 0;
volatile unsigned long timePeriod = 0;
volatile unsigned long capturedEdge = 0;
volatile int overflowCount = 0;
int translatedLetter[5] = {2, 2, 2, 2, 2};
int currentLetterIndex = 0;

void initializeSystem()
{
    cli();

    UART_init(BAUD_PRESCALER);

    //sets pin 0 as input
    DDRB &= ~(1<<DDB0);

    DDRB |= (1<<DDB1);
    DDRB |= (1<<DDB2);
    DDRB |= (1<<DDB3);

    PORTB &= ~(1<<PORTB1);
    PORTB &= ~(1<<PORTB2);
    PORTB &= ~(1<<PORTB3);

    // Enable clock (/8; 1t = 1/2M sec)
    TCCR1B &= ~(1<<CS10);
    TCCR1B |= (1<<CS11);
    TCCR1B &= ~(1<<CS12);

    // Detect rising edge for first button press
    TCCR1B |= (1<<ICES1);

    // Clear input capture flag
    TIFR1 |= (1<<ICF1);

    // Enable input capture interrupt
    TIMSK1 |= (1<<ICIE1);

    // enable overflow interrupt
    TIMSK1 |= (1<<TOIE1);

    //Sets the clock to normal
    TCCR1B &= ~(1<<WGM10);
    TCCR1A &= ~(1<<WGM11);
    TCCR1B &= ~(1<<WGM12);
    TCCR1B &= ~(1<<WGM13);

    sei();
}

ISR(TIMER1_CAPT_vect)
{
    if (!detectedEdge) {
        detectedEdge = 1;
        TIFR1 |= (1 << ICF1);
        capturedEdge = ICR1;
        TCCR1B &= ~(1 << ICES1);
        overflowCount = 0;
    } else {
        detectedEdge = 0;
        TIFR1 |= (1 << ICF1);
        TCCR1B |= (1 << ICES1);
        timePeriod = ICR1 - capturedEdge + overflowCount * 65536;
        isLightOn = 1;
        overflowCount = 0;
    }
    TIFR1 |= (1<<ICF1);
}

ISR(TIMER1_OVF_vect)
{
    overflowCount++;
    if (overflowCount > 16) {
        shouldAddSpace = 1;
        overflowCount = 0;
    }
    TIFR1 |= (1<<ICF1);
}

int main(void)
{
    initializeSystem();

    while (1) {
        char outputString[30];
        if (shouldAddSpace || currentLetterIndex > 4)
        {
            if (currentLetterIndex == 1) {
                if (translatedLetter[0] == 1) {
                    sprintf(outputString, "T");
                }else if (translatedLetter[0] == 0) {
                        sprintf(outputString,"N");
                    }
                } else if (currentLetterIndex == 2) {
                    if (translatedLetter[0] == 2 && translatedLetter[1] == 2) {
                        sprintf(outputString,"A");
                    } else if (translatedLetter[0] == 2 && translatedLetter[1] == 0) {
                        sprintf(outputString,"M");
                    } else if (translatedLetter[0] == 2 && translatedLetter[1] == 1) {
                        sprintf(outputString,"I");
                    } else if (translatedLetter[0] == 1 && translatedLetter[1] == 1) {
                        sprintf(outputString,"U");
                    } else if (translatedLetter[0] == 0 && translatedLetter[1] == 1) {
                        sprintf(outputString,"E");
                    } else if (translatedLetter[0] == 0 && translatedLetter[1] == 0) {
                        sprintf(outputString,"S");
                    }
                } else if (currentLetterIndex == 3) {
                    if (translatedLetter[0] == 2 && translatedLetter[1] == 2 && translatedLetter[2] == 2) {
                        sprintf(outputString," ");
                    } else if (translatedLetter[0] == 2 && translatedLetter[1] == 1 && translatedLetter[2] == 1) {
                        sprintf(outputString,"F");
                    } else if (translatedLetter[0] == 1 && translatedLetter[1] == 2 && translatedLetter[2] == 1) {
                        sprintf(outputString,"P");
                    } else if (translatedLetter[0] == 1 && translatedLetter[1] == 1 && translatedLetter[2] == 2) {
                        sprintf(outputString,"W");
                    } else if (translatedLetter[0] == 0 && translatedLetter[1] == 2 && translatedLetter[2] == 1) {
                        sprintf(outputString,"V");
                    } else if (translatedLetter[0] == 0 && translatedLetter[1] == 1 && translatedLetter[2] == 1) {
                        sprintf(outputString,"R");
                    } else if (translatedLetter[0] == 1 && translatedLetter[1] == 0 && translatedLetter[2] == 1) {
                        sprintf(outputString,"L");
                    } else if (translatedLetter[0] == 2 && translatedLetter[1] == 0 && translatedLetter[2] == 1) {
                        sprintf(outputString,"J");
                    } else if (translatedLetter[0] == 1 && translatedLetter[1] == 0 && translatedLetter[2] == 2) {
                        sprintf(outputString,"G");
                    } else if (translatedLetter[0] == 2 && translatedLetter[1] == 1 && translatedLetter[2] == 2) {
                        sprintf(outputString,"Q");
                    } else if (translatedLetter[0] == 1 && translatedLetter[1] == 2 && translatedLetter[2] == 2) {
                        sprintf(outputString,"Z");
                    } else if (translatedLetter[0] == 0 && translatedLetter[1] == 1 && translatedLetter[2] == 0) {
                        sprintf(outputString,"Y");
                    }
                    } else if (currentLetterIndex == 4) {
                    if (translatedLetter[0] == 2 && translatedLetter[1] == 2 && translatedLetter[2] == 2 && translatedLetter[3] == 1) {
                        sprintf(outputString,"Y");
                    } else if (currentLetterIndex == 2) {
                        if (translatedLetter[0] == 2 && translatedLetter[1] == 2) {
                            sprintf(outputString,"U");
                        }
                        else if (translatedLetter[0] == 2 && translatedLetter[1] == 3) {
                            sprintf(outputString,"V");
                        }
                        else if (translatedLetter[0] == 3 && translatedLetter[1] == 2) {
                            sprintf(outputString,"W");
                        }
                        else if (translatedLetter[0] == 1 && translatedLetter[1] == 2) {
                            sprintf(outputString,"X");
                        }
                        else if (translatedLetter[0] == 1 && translatedLetter[1] == 3) {
                            sprintf(outputString,"Y");
                        }
                        else if (translatedLetter[0] == 3 && translatedLetter[1] == 1) {
                            sprintf(outputString,"Z");
                        }
                    }
                    else {
                        sprintf(outputString,"_");
                    }
                    UART_putstring(outputString);
                    currentLetterIndex = 0;
                    shouldAddSpace = 0;
                    memset(translatedLetter, 2, sizeof(translatedLetter));
                }
            }
        }
    }

