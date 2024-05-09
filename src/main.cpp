#include <util/delay.h>
#include <stdlib.h>
#include <math.h>
#include <avr/interrupt.h>
#include "usart.h"
#include "bit.h"
#include "timer.h"

#define pwmPIN PD5 // PWM output pin
#define COMP_PIN PD7 // Comparator pin (AIN1) as negative input
#define PWM_INPUT_PIN PD6 // PWM input (AIN0) as positive input

#define FOSC 16000000 // Frequency Oscillator 16Mhz for Uno R3
#define BAUD 9600 // 9600 Bits per second
#define MYUBRR FOSC / 16 / BAUD - 1 // My USART Baud Rate Register

volatile float T1 = 0.0;
volatile float T2 = 0.0;
volatile float T_high = 0.0;
volatile float T_low = 0.0;

ISR(TIMER1_CAPT_vect) {
  float capturedValue = ICR1;
  
  if (!(TCCR1B & (1 << ICES1))) {
    // Falling edge
    T1 = capturedValue;
    bitInverse(TCCR1B, ICES1); // Toggle the edge detection
    T_high = T1 - T2;
  } else {
    // Rising edge
    T2 = capturedValue;
    TCCR1B ^= (1 << ICES1); // Toggle the edge detection
    T_low = T2 - T1;
  }
}

void setup() {
    bitSet(DDRD, pwmPIN); // Set PWM pin as output
    bitClear(DDRD, COMP_PIN); // Set Comparator pin as input
    bitClear(DDRD, PWM_INPUT_PIN); // Set PWM input pin as input

    bitSet(ACSR, ACIC); // Analog Comparator Input Capture enabled

    // PWM Configuration
    TCCR0A = (1 << COM0B1) | (1 << WGM01) | (1 << WGM00);
    TCCR0B = (1 << CS01) | (1 << CS00);  
    //OCR0A = 255; // Set the TOP value for 100% duty cycle

    // Timer 1 Configuration
    TCCR1A = 0x00;
    TCCR1B = (1 << ICES1) | (1 << CS12) | (1 << ICNC1); // Noise canceler, edge select, prescaler

    bitSet(TIMSK1, ICIE1); // Enable input capture interrupt

    usart_init(MYUBRR);
    sei(); // Enable global interrupts
}




int main() {
    setup();

    while (1) {
        float realDutyCycle = 0.1;
        while (realDutyCycle <= 1.0) {
            OCR0B = (int)(255 * realDutyCycle);
            _delay_ms(1000); // Adjusting for visibility of changes

            if (T_high + T_low > 0) { // Check to prevent division by zero
                float freq = 1.0 / (T_high + T_low);
                float measuredDutyCycle = T_high / (T_high + T_low);

                usart_tx_string(">duty_real: ");
                usart_tx_float(realDutyCycle, 6, 3);
                usart_transmit('\n');

                usart_tx_string(">duty_estimate: ");
                usart_tx_float(measuredDutyCycle, 6, 3);
                usart_transmit('\n');
            }
            realDutyCycle += 0.1;
        }
    }

    return 0;
}