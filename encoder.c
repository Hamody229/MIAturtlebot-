/*
 * File:   decoder.c
 * Author: MKamel
 *
 * Created on September 2, 2024, 8:55 PM
 */


#include <xc.h>
#include "encoder.h"

volatile uint16_t pulse_count = 0;


void encoder_init(void) {
    DDRD &= ~(1 << PD2);  //encoder signal on PD2 (INT0)
    EICRA = (1 << ISC01); // Trigger interrupt on falling edga
    EIMSK = (1 << INT0);  // Enable INT0
}


ISR(INT0_vect) {
    pulse_count++;
}