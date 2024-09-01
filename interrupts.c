#include "interrupts.h"
#include "common.h"
#include <stdio.h>

// when de-attached, they are null (zero)
volatile void_func int0_callback = 0;

volatile void_func int1_callback = 0;

void enableInterrupts() {
    sei();
}

void disableInterrupts() {
    cli();
}

void attachInterrupt(InterruptPin pin, void_func callback, InterruptMode mode) {
    // set the control pins to specify interrupt mode
    volatile int interrupt_control_pin0 = pin == INT0_PIN ? ISC00 : ISC10;
    volatile int interrupt_control_pin1 = pin == INT0_PIN ? ISC01 : ISC11;
    
    // tell the atmega which mode to use
    switch (mode) {
        case LOW:
            BIT_CLEAR(EICRA, interrupt_control_pin1);
            BIT_CLEAR(EICRA, interrupt_control_pin0);
            break;
        case CHANGE:
            BIT_CLEAR(EICRA, interrupt_control_pin1);
            BIT_SET(EICRA, interrupt_control_pin0);
            break;
        case FALLING:
            BIT_SET(EICRA, interrupt_control_pin1);
            BIT_CLEAR(EICRA, interrupt_control_pin0);
            break;
        case RISING:
            BIT_SET(EICRA, interrupt_control_pin1);
            BIT_SET(EICRA, interrupt_control_pin0);
            break;
    }
    
    // enable interrupts for the pin
    // then update the function pointer
    if (pin == INT0_PIN) {
        BIT_SET(EIMSK, INT0);
        int0_callback = callback;
    } else if (pin == INT1_PIN) {
        BIT_SET(EIMSK, INT1);
        int1_callback = callback;
    }
}

void detachInterrupt(InterruptPin pin) {
    // disable interrupts for the pin
    if (pin == INT0_PIN) {
        BIT_CLEAR(EIMSK, INT0);
        // set it to NULL
        int0_callback = 0;
    } else if (pin == INT1_PIN) {
        BIT_CLEAR(EIMSK, INT1);
        // set it to NULL
        int1_callback = 0;
    }
}

ISR(INT0_vect, ISR_NAKED) {
    // if attached
    if (int0_callback != 0) {
        int0_callback();
    }
}

ISR(INT1_vect, ISR_NAKED) {
    // if attached
    if (int1_callback != 0) {
        int1_callback();
    }
}
