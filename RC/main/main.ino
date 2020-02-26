#include "UCMotor.h"
#include "pinout.h"
#include "rc_commands.h"
#include <string.h>
#define DEBUG_MODE
#include "debug.h"

#include <stdio.h>

// Define PROPBOT to change config
// #define PROPBOT

static bool autonomy = false;
static Array<wheel_motor_command_t, NUM_WHEELS> commands = all_brake_command;

Wheel * wheel_motors[NUM_WHEELS];

UC_DCMotor mL(1);
UC_DCMotor mR(2);
UC_DCMotor * minis[2] = {
    &mL,
    &mR
}; 

void setup()
{

    // For debugging
    Serial.begin(9600);
    Serial.print("Init");

    // Setup RC pins in input mode (input capture)
    pinMode(RC_RIGHT_CHANNEL_PIN,   INPUT);
    pinMode(RC_LEFT_CHANNEL_PIN,    INPUT);
    pinMode(RC_SWA_CHANNEL_PIN,     INPUT);
    pinMode(RC_SWB_CHANNEL_PIN,     INPUT);

    #ifdef PROPBOT
        /*
        * Fast PWM configuration:
        *   - Clear OCxA/B on Compare Match, set OCxA at BOTTOM,(non-inverting mode)
        *   - TOP = 0x03FF (1023) for MEGA and 0x0FF (255) for UNO
        *   - Prescaler = 8
        *   - Output frequency = clk_io(RTC)/8 --> 8kHz (MEGA) and 4kHz (UNO)
        */ 
        #if defined(__AVR_ATmega328P__)
            // Timer2 Fast PWM setup
            TCCR2A = (1 << COM2A1) | (1 << COM2B1) | (1 << WGM22) | (1 << WGM21) | (1 << WGM20);
            TCCR2B = (1 << CS21); // 8kHz
            // Set duty cycle initially to 0%
            OCR2A = 0;  
            OCR2B = 0;

            // Timer0 Fast PWM setup
            TCCR0A = (1 << COM0A1) | (1 << COM0B1) | (1 < WGM02) | (1 << WGM01) | (1 << WGM00);
            TCCR0B = (1 << CS01); // 8KHz    
            // Set duty cycle initially to 0%
            OCR0A = 0;  
            OCR0B = 0;
        #elif (defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
            // Timer3 Fast PWM setup
            TCCR3A = (1 << COM3A1) | (1 << COM3B1) | (1 << WGM32) | (1 << WGM31) | (1 << WGM30);
            TCCR3B = (1 << CS31);
            // Set duty cycle initially to 0%
            OCR3A = 0;  
            OCR3B = 0;

            // Timer4 Fast PWM setup
            TCCR4A = (1 << COM4A1) | (1 << COM4B1) | (1 < WGM42) | (1 << WGM41) | (1 << WGM40);
            TCCR4B = (1 << CS41);         
            // Set duty cycle initially to 0%
            OCR4A = 0;  
            OCR4B = 0;
        #else 
            #error "No timers configured for PWM"
        #endif

        // Pin setup for all wheel motors
        for (int i = wheel_indices::Start + 1; i < wheel_indices::End; i++ ) {
            pinMode(wheel_to_pins[i].control,       OUTPUT);
            pinMode(wheel_to_pins[i].brake_release, OUTPUT);
            pinMode(wheel_to_pins[i].dir,           OUTPUT);
            wheel_motors[i] = &Wheel(i);
        }
    #else 
        Serial.println("Setting up DC motors");
        // Mini RC
        for (int i = 0; i < 2; i++ ) {
            Serial.println(i + 1);
            // wheel_motors[i] = &Wheel(i + 1);
        }
    #endif // PROPBOT
    Serial.println("Setup complete");
}


void sendCommandMini(UC_DCMotor * uc_motor, wheel_motor_command_t command) {
    // BRAKE condition
    if (command.brake_release == ENGAGE_BRAKE) {   
        uc_motor->run(STOP);
        uc_motor->setSpeed(0);
        return;
    }

    // Step 1: set rotation direction
    if (command.dir == DIR_FW) {
        uc_motor->run(FORWARD);
    } else {
        uc_motor->run(BACKWARD);
    }
    uc_motor->setSpeed(command.duty_cycle * TOP);
}

void loop()
{
    commands = fetch_rc_commands(); // update wheel commands
    for (int i = wheel_indices::Start + 1; i < wheel_indices::End; i++ ) {
        sendCommandMini(minis[i], commands[i]);
        // wheel_motors[i]->sendCommand(commands[i]);
    }
}