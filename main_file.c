#include <avr/io.h>
#include <avr/interrupt.h>

// GPIO Functions
void GPIO_SetPinAsOutput(volatile uint8_t *port, uint8_t pin) {
    *port |= (1 << pin);
}

void GPIO_SetPinAsInput(volatile uint8_t *port, uint8_t pin) {
    *port &= ~(1 << pin);
}

void GPIO_SetPinHigh(volatile uint8_t *port, uint8_t pin) {
    *port |= (1 << pin);
}

void GPIO_SetPinLow(volatile uint8_t *port, uint8_t pin) {
    *port &= ~(1 << pin);
}

uint8_t GPIO_ReadPin(volatile uint8_t *pinReg, uint8_t pin) {
    return (*pinReg & (1 << pin)) ? 1 : 0;
}

// ADC Functions
void ADC_Init() {
    ADMUX = (1 << REFS0); // AVcc with external capacitor at AREF pin
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1); // Enable ADC and set prescaler to 64
}

uint16_t ADC_Read(uint8_t channel) {
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F); // Select ADC channel
    ADCSRA |= (1 << ADSC); // Start conversion
    while (ADCSRA & (1 << ADSC)); // Wait for conversion to complete
    return ADC;
}

// TIMER1 PWM Functions
void TIMER1_InitPWM() {
    TCCR1A |= (1 << WGM11) | (1 << COM1A1); // Fast PWM, non-inverting mode
    TCCR1B |= (1 << WGM12) | (1 << WGM13) | (1 << CS11); // Prescaler 8, Fast PWM
    ICR1 = 19999; // Set top value for 20ms period (50Hz)
}

void TIMER1_SetPWM(uint16_t duty) {
    OCR1A = duty;
}

// External Interrupts (INT0) Functions
void INT0_Init() {
    EICRA |= (1 << ISC01); // Falling edge of INT0 generates an interrupt request
    EIMSK |= (1 << INT0);  // Enable INT0
    sei(); // Enable global interrupts
}

volatile uint16_t encoder_count = 0; // Count encoder pulses

ISR(INT0_vect) {
    encoder_count++; // Increment pulse count on each encoder pulse
}

// SPI Functions (Optional)
void SPI_MasterInit(void) {
    DDRB = (1 << PB3) | (1 << PB5) | (1 << PB2); // Set MOSI, SCK, and SS as output
    SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0); // Enable SPI, Master, set clock rate fck/16
}

uint8_t SPI_MasterTransmit(uint8_t data) {
    SPDR = data; // Start transmission
    while (!(SPSR & (1 << SPIF))); // Wait for transmission complete
    return SPDR; // Return received data
}

// Main Function
int main() {
    // Initialize peripherals
    ADC_Init();
    TIMER1_InitPWM();
    INT0_Init();

    // Set pin direction for motor control
    GPIO_SetPinAsOutput(&DDRB, PB1); // Set PB1 as output (OC1A for PWM)

    uint16_t potValue;
    uint16_t dutyCycle;

    while (1) {
        potValue = ADC_Read(0); // Read potentiometer value from ADC0
        dutyCycle = (potValue * 20000) / 1023; // Map ADC value to PWM duty cycle (0-20000)
        TIMER1_SetPWM(dutyCycle); // Set PWM duty cycle to control motor speed

        // Motor speed calculation using encoder pulses can be done here
        // Example: motor_speed = (encoder_count / pulses_per_revolution) * 60;
        // Note: You may need to reset encoder_count periodically for accurate RPM measurement
    }
}