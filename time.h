
 

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef XC_HEADER_TEMPLATE_H
#define	XC_HEADER_TEMPLATE_H

#include <xc.h> // include processor files - each processor file is guarded.
#include <stdbool.h>
#include <avr/interrupt.h>
#include <stdint.h>

void Timer(uint16_t comparevalue) ;

/*
 give a compare value to put in the ocr1a 
 * ----VERY IMPORTANT----
 * comparevalue = (250,0000/Desired Freq) -1 as ocr1a starts from 0 not one 
 * for ex Freq of 1KHZ will need 250 -1 = 249 comparevalue 
 */
ISR(TIMER1_COMPA_vect) ;
/*
 Toggle B0 to test the time function
 
 */


void delay(uint16_t ms) ;

/*
 dellay ms milli seconds 

 */


#endif	/* XC_HEADER_TEMPLATE_H */

