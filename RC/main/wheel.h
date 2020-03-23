#ifndef Wheel_h
#define Wheel_h

#include <Arduino.h>
#include "pinout.h"
#include "util.h"
#include "UCMotor.h"

/* Define Important Values */
#if defined(__AVR_ATmega328P__)
        #define TOP 0XFF
    #elif (defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
        #define TOP 0x03FF     // Top value of the registers (configuration done in MAIN) 
#endif


/**
 * @brief motor command structure that includes duty, brake, and direction
 * 
 */
typedef struct wheel_motor_command_t
{
    float duty_cycle; // float from 0 to 1
    bool brake_release;
    bool dir;
};

// Direction values
#define DIR_FW          true
#define DIR_BW          false
#define ENGAGE_BRAKE    false
#define RELEASE_BRAKE   true

 
/* Else, define values for mini rc */
#define NUM_WHEELS 2 // driving with only two drive signals

typedef enum wheel_index_t
{
    Start   = -1,
    L       = 0U,
    R       = 1U,
    End     = 2U,
} wheel_indices;

const Array<wheel_motor_command_t, NUM_WHEELS> all_brake_command = {{
    {0.0, ENGAGE_BRAKE, FORWARD},
    {0.0, ENGAGE_BRAKE, FORWARD}
}};

#endif // !Wheel_h
