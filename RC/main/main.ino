#include "rc_commands.h"
#include "wheel.h"
#include "types.h"
#include <stdio.h>

// Define PROPBOT to change config
// #define PROPBOT

static bool autonomy = false;
static Array<wheel_motor_command_t, NUM_WHEELS> commands = all_brake_command;
Wheel ** wheel_motors;

void setup()
{

    // For debugging
    Serial.begin(9600);
    Serial.print("Initializating in process...");

    // Setup RC pins in input mode (input capture)
    initialize_rc();

    // Setup and initialize wheel objects
    wheel_motors = initializeWheels();
    Serial.println("Init'd motors");
    for (int i = wheel_indices::Start + 1; i < wheel_indices::End; i++ ) {
        Serial.println(
            String("It index: ") + i + String("\tWheel index: ") +  wheel_motors[i]->wheel_index()
        );
    }

    // Indicate that setup is complete
    Serial.println("Setup complete");
}

void loop()
{
    commands = fetch_rc_commands(); // update wheel commands
    for (int i = wheel_indices::Start + 1; i < wheel_indices::End; i++ ) {
        wheel_motors[i]->sendCommand(commands[i]);
        // Serial.println(
        //     String("\nIndex ") + i +  String("and fetched index ") + wheel_motors[i]->getSpeed()
        // );
    }
}