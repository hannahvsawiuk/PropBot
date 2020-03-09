#include "rc_commands.h"
#include "wheel.h"
#include "types.h"
#include <stdio.h>

// Define PROPBOT to change config
// #define PROPBOT

wheel_motor_command_t* commands[NUM_WHEELS] = { { NULL } };
Wheel* wheel_motors[NUM_WHEELS] = { { NULL } };

void setup()
{

    // For debugging
    Serial.begin(9600);
    Serial.print("Initializating in process...");

    // Setup RC pins in input mode (input capture)
    initialize_rc();

    // initialize motor commands
    commands = initializeWheelCommands();

    // // Setup and initialize wheel objects
    wheel_motors = initializeWheels(); 

    Serial.println("Init'd motors");
    for (int i = wheel_indices::Start; i < wheel_indices::End; i++ ) {        
        Serial.println(
            String("It index: ") + i + String("\tWheel index: ") +  wheel_motors[i]->wheel_index()
        );
    }

    // Indicate that setup is complete
    Serial.println("Setup complete");
}

void loop()
{
    fetch_rc_commands(commands); // update wheel commands
    for (int i = wheel_indices::Start; i < wheel_indices::End; i++ ) {
        wheel_motors[i]->sendCommand(commands[i]);
    }
}