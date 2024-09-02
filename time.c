/*
 * File:   time.c
 * Author: JETT
 *
 * Created on September 2, 2024, 5:32 PM
 */


#include <avr/io.h>
#include "time.h"


void Timer(uint16_t comparevalue) {
    TCCR1B |= (1 << WGM12);

    OCR1A = comparevalue;

    TIMSK1 |= (1 << OCIE1A);

    TCCR1B |= (1 << CS11) | (1 << CS10);
}


ISR(TIMER1_COMPA_vect) {
    // Test
    PORTB ^= (1 << PORTB0);  // Toggle the LED on pin PB0

}


void delay(uint16_t ms) {
    // Delay loop for the desired amount of milliseconds
    while (ms--) {
        // Loop to create approximately 1 ms delay
        for (uint16_t i = 0; i < 1000; i++) {
            asm volatile ("nop");
        }
    }
}
