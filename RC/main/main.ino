#include "pinout.h"
#include "rc_commands.h"
#include "UCMotor.h"
#include <string.h>
#include <stdio.h>

volatile Array<wheel_motor_command_t, NUM_WHEELS> commands = all_brake_command;

// Initialize the UC motor objects
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

    // Indicate setup done
    Serial.println("Setup complete");
}

/**
 * @brief Sends commands to a UC_DC motor 
 * 
 * @param uc_motor reference to the UC_DC motor object
 * @param command the command to send
 * 
 * @return 
 */
void sendCommandMini(UC_DCMotor * uc_motor, wheel_motor_command_t command) {
    // BRAKE condition
    if (command.brake_release == ENGAGE_BRAKE) {  
        Serial.println("Braking...") ;
        uc_motor->run(STOP);
        uc_motor->setSpeed(0);
        return;
    }

    // Step 1: set rotation direction
    String debug_str = String();
    if (command.dir == DIR_FW) {
        debug_str += String("Direction: BW");
        uc_motor->run(FORWARD);
    } else {
        debug_str += String("Direction: BW");
        uc_motor->run(BACKWARD);
    }

    // Step 2: set speed
    debug_str += String("Speed: ") + command.duty_cycle;
    Serial.println(debug_str);
    uc_motor->setSpeed(command.duty_cycle * TOP);
}

void loop()
{
    // Update wheel commands
    commands = fetch_rc_commands(); 

    // Send commands to wheels
    for (int i = wheel_indices::Start + 1; i < wheel_indices::End; i++ ) {
        Serial.println(String("Index: ") + i);
        sendCommandMini(minis[i], commands[i]);
    }
}