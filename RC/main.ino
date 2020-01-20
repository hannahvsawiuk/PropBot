#include "pinout.h"

void setup()
{
    // set PWM pins in output mode
    pinMode(R_F_controlPin, OUTPUT);
    pinMode(L_F_controlPin, OUTPUT);
    pinMode(L_B_controlPin, OUTPUT);
    pinMode(R_B_controlPin, OUTPUT);

    /*
    * Fast PWM configuration:
    *   - Clear OC0A on Compare Match, set OC0A at BOTTOM,(non-inverting mode)
    *   - TOP = 0xFF (255)
    *   - Output frequency =  clk_io/64 --> 16MHz/64 = ~976 Hz
    */ 

    // Timer1 Fast PWM setup
    TCCR1A = (1 << COM1A1)  |  (1 << WGM11)     |   (1 << WGM10);
    TCCR1B = (1 << CS11)    |  (1 << CS10); 
    // Set duty cycle initially to 0%
    OCR1A = 0;  // pin 9
    OCR1B = 0;  // pin 10

    // Timer2 Fast PWM setup
    TCCR2A = (1 << COM2A1)  |   (1 << WGM21)    |   (1 << WGM20);
    TCCR2B = (1 << CS21)    |   (1 << CS20);         
    // Set duty cycle initially to 0%
    OCR2A = 0;  // pin 11
    OCR2B = 0;  // pin 3

}

void loop()
{

}