#include <avr/io.h>
#include "adc.h"
#include "common.h"

unsigned analogRead(ADCPin pin) {
    // Make ADC bit as input
    BIT_CLEAR(DDRC, pin);
    BIT_CLEAR(PORTC, pin);
    
    // Enable ADC, frequency/128
	ADCSRA = 0x87;
    
    // Vref is VCC 5V
    BIT_CLEAR(ADMUX, REFS1);
    BIT_SET(ADMUX, REFS0);
    
    // right-align output
    BIT_CLEAR(ADMUX, ADLAR);
    
    // clear previous ADC selection
    ADMUX &= 0xf0;
    
    // select which pin to read from
    ADMUX |= (pin & 0x0f);
    
    // start conversion of analog to digital
    BIT_SET(ADCSRA, ADSC);
    
    // wait for conversion to end
    while(BIT_READ(ADCSRA, ADIF) == 0)
        ;
    
    // read converted result byte by byte (10 bits or 2 bytes in C)
    unsigned result_low = (int)ADCL;
	unsigned result_high = (int)ADCH*256;
    
    return result_low + result_high;
}
