

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef XC_HEADER_TEMPLATE_H
#define	XC_HEADER_TEMPLATE_H

#include <xc.h> // include processor files - each processor file is guarded.  
#include <avr/io.h>
#include <stdbool.h>
#include <avr/interrupt.h>
#include <stdint.h>


typedef struct {
    volatile uint8_t *port;
    volatile uint8_t *ddr;
    uint8_t pin;

} NPin;
void PinMode(NPin npin, bool value) ;

/*
 * PinMode configures pins as output or input
 PinMode(pin number ,true or false) 
 * true configure the pin as output 
 * false configure the pin as input
 */



void DigitalWrite(NPin npin, bool value) ;

/*
 DigitalWrite gives output pins high or low value
 * DigitalWrite function (pin number , trye or false) 
 * true -- HIGH , false -- LOw
 */

bool DigitalRead(NPin npin) ;
/*
 DigitalRead takes high or from input pins
 * DigitalRead function (pin number)
 * true -- HIGH , false -- LOW
 */

#endif
