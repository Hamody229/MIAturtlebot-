/* 
 * File:   interrupts.h
 * Author: yoyos
 *
 * Created on September 1, 2024, 1:18 PM
 */

#ifndef INTERRUPTS_H
#define	INTERRUPTS_H

#include <avr/io.h>
#include <avr/interrupt.h>

/** An enum representing all possible interrupt pins
 * 
 * only INT0 and INT1 can receive interrupts on the atmega328P
 */
typedef enum {
    INT0_PIN,
    INT1_PIN
} InterruptPin;

/**This enum specifies when to trigger an interrupt
 * 
 * LOW: when the pin reads digital logic LOW
 * CHANGE: whenever the logic level on the pin is changed
 * RISING: when there's a rising edge on the pin
 * FALLING: when there's a falling edge on the pin
 */
typedef enum {
    LOW,
    CHANGE,
    RISING,
    FALLING
} InterruptMode;

// simplify void func() pointer type name to `void_func`
typedef void (*void_func)(void);

/**
 * Enables interrupts
 */
void enableInterrupts();

/**
 * Disables interrupts
 */
void disableInterrupts();

/**Attaches the `void callback()` function to the interrupt `pin`
 * 
 * @param pin: The pin where the external interrupt occurs
 * @param callback: a void func() that will be executed on interrupt
 * @param mode: when to trigger an interrupt, see documentation of `InterruptMode` to learn more
 * 
 * see documentation of `InterruptPin` enum to learn which pins
 * can receive interrupts on the atmega328P
 * 
 * for the callback, only type the function name without the `()`
 * 
 * Example: if you have a function `void foo()`, and you want it to
 * be executed when a rising edge happens on int0, you can call this function
 * like this
 * 
 * `attachInterrupt(INT0_PIN, foo, RISING);`
 * 
 * Limitations:
 * - timers use interrupts, so any functions related to timers will not work
 * inside the callback function, things like millis, delay, etc
 * 
 * - gloabal variables shared between main program and callback function
 * should be marked as volatile
 * 
 * - only one interrupt can run at a time
 */
void attachInterrupt(InterruptPin pin, void_func callback, InterruptMode mode);

/**Disables interrupts for the given pin only
 * 
 * @param pin: the pin to disable
 * 
 * see documentation of `InterruptPin` enum to learn which pins
 * can receive interrupts on the atmega328P
 */
void detachInterrupt(InterruptPin pin);

#endif	/* INTERRUPTS_H */
