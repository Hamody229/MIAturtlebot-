/* 
 * File:   adc_test.c
 * Author: yoyos
 *
 * Created on September 1, 2024, 8:36 PM
 */

#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>

#include "adc.h"

/*
 * 
 */
int main(void) {
    
    unsigned val = 0;
    
    while (1) {
        val = analogRead(ADC0);
        
        // we can't print in avr, so use a debugger to see
        // value of `val`, I am printing to prevent
        // the compiler from optimizing val away
        
        printf("No printing :(, but the value is %u", val);
    }

    return (EXIT_SUCCESS);
}

