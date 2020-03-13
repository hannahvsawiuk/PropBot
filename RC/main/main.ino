#include "rc_commands.h"
#include "wheel.h"
#include "types.h"
#include <stdio.h>

// Define PROPBOT to change config
// #define PROPBOT

void setup()
{

    // For debugging
    Serial.begin(9600);
    Serial.print("Initializating in process...");

    // Setup RC pins in input mode (input capture)
    initialize_rc();

    // Initialize motor commands
    initializeWheelCommands();

    // Setup and initialize wheel objects
    initializeWheels();

    // Indicate that the wheels have been initialized
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
    // Update wheel commands
    fetch_rc_commands(commands); 
    // Send commands to the wheels
    for (int i = wheel_indices::Start; i < wheel_indices::End; i++ ) {
        wheel_motors[i]->sendCommand(commands[i]);
    }
}