
#include <avr/io.h>
#include <stdbool.h>
#include <avr/interrupt.h>
#include <stdint.h>



typedef struct {
    volatile uint8_t *port;
    volatile uint8_t *ddr;
    uint8_t pin;

} NPin;



void PinMode(NPin npin, bool value) {
    if (value) {
        *(npin.ddr) |= (1 << npin.pin); // true--output , false--input
    } else {
        *(npin.ddr) &= ~(1 << npin.pin);
    }
}

void DigitalWrite(NPin npin, bool value) {
    if (value) {
        *(npin.port) |= (1 << npin.pin);  // true--high , false --low
    } else {
        *(npin.port) &= ~(1 << npin.pin);  
    }
}

bool DigitalRead(NPin npin) {
    return (*(npin.port - 2) & (1 << npin.pin)) != 0;  // true--high , false --low
}

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








int main(void) {
    /* Replace with your application code */
    
    NPin D0 = {&PORTD,&DDRD,0} ;
    NPin D1 = {&PORTD,&DDRD,1} ;
    NPin D2 = {&PORTD,&DDRD,2} ;
    NPin D3 = {&PORTD,&DDRD,3} ;
    NPin D4 = {&PORTD,&DDRD,4} ;
    NPin D5 = {&PORTD,&DDRD,5} ;
    NPin D6 = {&PORTD,&DDRD,6} ;
    NPin D7 = {&PORTD,&DDRD,7} ;
    NPin D8 = {&PORTB,&DDRB,0}  ;
    NPin D9 = {&PORTB,&DDRB,1}  ;
    NPin D10= {&PORTB,&DDRB,2}  ;
    NPin D11= {&PORTB,&DDRB,3}  ;
    NPin D12= {&PORTB,&DDRB,4}  ;
    NPin D13= {&PORTB,&DDRB,5}  ;

    //test
    PinMode(D0,true) ;
    PinMode(D1,false);
    PinMode(D2,true) ;
    DigitalWrite(D0,true) ;
    bool x = DigitalRead(D1) ;
    DigitalWrite(D2,!x) ;
    
    Timer(249);
    
    sei();

    
    while (1) {
    }
}
