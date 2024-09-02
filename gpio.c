/*
 * File:   gpio.c
 * Author: JETT
 *
 * Created on September 2, 2024, 5:03 PM
 */


#include <avr/io.h>
#include "gpio.h"


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

