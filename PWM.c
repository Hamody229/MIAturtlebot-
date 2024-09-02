/*
 * File:   PWM.c
 * Author: MKamel
 *
 * Created on September 2, 2024, 8:14 AM
 */

#include <xc.h>

/* 
 PWM needs the 
 - pulse_width as a 16 bit argument from the readAnalog function 
 - timer which is zero or 1 argm to choose between timer1 and timer0
 - pwm_pin which is the output pin 
*/

void PWM_Timer(uint16_t pulse_width, uint8_t timer, uint8_t pwm_pin) {

    if (timer == 0) {
        DDRD |= (1 << pwm_pin);

        TCCR0A |= (1 << WGM00) | (1 << WGM01); // Fast PWM mode
        
        if (pwm_pin == PD6) {
            TCCR0A |= (1 << COM0A1);
        } 
        else if (pwm_pin == PD5) {
            TCCR0A |= (1 << COM0B1);
        }

        TCCR0B |= (1 << CS00);
        TCNT0 = 0;//clear register
        
        
        // set the compare match
        if (pwm_pin == PD6) {
            OCR0A = (pulse_width & 0xFF); //SCALE THE PULSE WIDTH FROM 16 BIT TO 8 BITS
        } else if (pwm_pin == PD5) {
            OCR0B = (pulse_width & 0xFF); 
        }

    } 
    
    else if (timer == 1) {

        TCCR1A |= (1 << WGM11);
        TCCR1B |= (1 << WGM12) | (1 << WGM13);

        if (pwm_pin == PB1) {
            TCCR1A |= (1 << COM1A1);
        } else if (pwm_pin == PB2) {
            TCCR1A |= (1 << COM1B1); 
        }

        TCCR1B |= (1 << CS12);
        TCNT1 = 0;
        
        // Set the PWM duty cycle
        ICR1 = 0xFFFF;
        
        if (pwm_pin == PB1) {
            OCR1A = pulse_width; 
        } else if (pwm_pin == PB2) {
            OCR1B = pulse_width; 
        }
    }
}