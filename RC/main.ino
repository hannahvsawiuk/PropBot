#include "wheel.h"

void setup()
{
    // set PWM pins in output mode
    pinMode(R_F_controlPin, OUTPUT);
    pinMode(L_F_controlPin, OUTPUT);
    pinMode(L_B_controlPin, OUTPUT);
    pinMode(R_B_controlPin, OUTPUT);


    if (SERIAL == true) {
        // code for setting up serial comms
    } else {
        /*
        * Fast PWM configuration:
        *   - Clear OC0A on Compare Match, set OC0A at BOTTOM,(non-inverting mode)
        *   - TOP = 0xFF (255)
        *   - Output frequency =  clk_io/64 --> 16MHz/64 = ~976 Hz
        */ 
        // Timer3 Fast PWM setup
        TCCR3A = (1 << COM3A1)  |  (1 << WGM31)     |   (1 << WGM30);
        TCCR3B = (1 << CS31)    |  (1 << CS30); 
        // Set duty cycle initially to 0%
        OCR3A = 0;  
        OCR3B = 0;

        pinMode(, OUTPUT);
        pinMode(, OUTPUT);  

        // Timer4 Fast PWM setup
        TCCR4A = (1 << COM4A1)  |   (1 << WGM41)    |   (1 << WGM40);
        TCCR4B = (1 << CS41)    |   (1 << CS40);         
        // Set duty cycle initially to 0%
        OCR4A = 0;  
        OCR4B = 0;  
        pinMode(, OUTPUT);
        pinMode(, OUTPUT);
    }

}

void loop()
{

}