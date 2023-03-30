#include <avr/io.h>
#include <util/delay.h>

int main(void)
{
    // Set up PWM on pin 3 (OC2B)
    DDRD |= (1 << DDD3); // set PD3 as output
    TCCR2A |= (1 << COM2B1) | (1 << WGM21) | (1 << WGM20); // set non-inverting mode and fast PWM
    TCCR2B |= (1 << CS22); // set prescaler to 64

    while (1)
    {
        // Turn on the speaker at full volume
        OCR2B = 255;

        // Play the beep for 10 seconds
        for (int i = 0; i < 1000; i++)
        {
            _delay_ms(10);
        }

        // Turn off the speaker
        OCR2B = 0;

        // Wait for 3 seconds before playing the next beep
        _delay_ms(3000);
    }
}