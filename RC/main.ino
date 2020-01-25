#include "wheel.h"
#include "pinout.h"

volatile bool autonomy = false;

void setup()
{
    // Setup RC pins in input mode (input capture)
    pinMode(RC_RIGHT_CHANNEL_PIN,   INPUT);
    pinMode(RC_LEFT_CHANNEL_PIN,    INPUT);
    pinMode(RC_SWA_CHANNEL_PIN,     INPUT);
    pinMode(RC_SWB_CHANNEL_PIN,     INPUT);

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

        // Timer4 Fast PWM setup
        TCCR4A = (1 << COM4A1)  |   (1 << WGM41)    |   (1 << WGM40);
        TCCR4B = (1 << CS41)    |   (1 << CS40);         
        // Set duty cycle initially to 0%
        OCR4A = 0;  
        OCR4B = 0; 

        // Pin setup for all wheel motors
        for (int i = 0; i < NUM_WHEELS; i++)
        {
            pinMode(wheel_to_pins[i].control,       OUTPUT);
            pinMode(wheel_to_pins[i].brake_release, OUTPUT);
            pinMode(wheel_to_pins[i].dir,           OUTPUT);
        }
    }
}

void loop()
{
    if (mode == autonomy) {
        // code for comms with the autonomy computer
    } else {
        // update wheel commands
        wheel_motor_command_t * wheel_motor_commands = fetch_rc_commands();
        for (int i = 0; i < NUM_WHEELS; i++ ) {
            wheel_set(i, (*wheel_motor_commands)[i]);
        }
    }
}