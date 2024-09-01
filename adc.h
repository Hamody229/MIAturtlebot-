/* 
 * File:   adc.h
 * Author: yoyos
 *
 * Created on September 1, 2024, 7:13 PM
 */

#ifndef ADC_H
#define	ADC_H

/** The allowed ADC pins in the atmega
 */
typedef enum {
    ADC0,
    ADC1,
    ADC2,
    ADC3,
    ADC4,
    ADC5
} ADCPin;

/**Reads from the given ADC pin
 * 
 * @param pin which analog ping to read
 * @return the analog value read
 * 
 * you can read documentation of ADCPin enum to learn more
 * 
 * IMPORTANT: in proteus simulation, make sure to connect 5V source
 * to AVCC pin (pin number 20), or the adc won't work
 */
unsigned analogRead(ADCPin pin);

/**Write an analog value to the given DAC pin
 * 
 * @param pin
 * @param value
 */
void analogWrite(unsigned pin, unsigned char value);

#endif	/* ADC_H */

