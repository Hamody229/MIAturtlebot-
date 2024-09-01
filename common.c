#include <avr/io.h>
#include <xc.h>
#include "common.h"

void main(void) {
    DDRD = 0xFF;
    PORTD = PORTD | 1;
    
    while (1) {}
}
