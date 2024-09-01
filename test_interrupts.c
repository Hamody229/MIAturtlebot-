/* 
 * File:   test.c
 * Author: yoyos
 *
 * Created on September 1, 2024, 11:36 AM
 */

#include <stdio.h>
#include <stdlib.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <xc.h>

#include "interrupts.h"
#include "common.h"

void my_callback() {
    BIT_SET(PORTD, PORTD7);
}

// only un-comment to run the test
// and make sure there is only one main function

//void main(void) {
//    // disable interrupts temporarily
//    disableInterrupts();
//    
//    // set PORTD as input, except the 7th bit
//    DDRD = 1 << DDD7;
//    
//    // pull-up for PORTD 2
//    PORTD |= (1 << PORTD2);
//    
//    // enable interrupts again
//    enableInterrupts();
//    
//    attachInterrupt(INT0_PIN, my_callback, FALLING);
//    
//    // what's supposed to happen?
//    // when INT0 pin receives a falling edge, PORTD7 should be 1
//    // before that it should be 0
//    
//    while (1) {}
//}
