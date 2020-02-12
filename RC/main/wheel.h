#ifndef Wheel_h
#define Wheel_h

#include <Arduino.h>
#include "pinout.h"
#include "util.h"

/* Define Important Values */
#define NUM_WHEELS      4
#define TOP             0x03FF     // Top value of the registers (configuration done in MAIN)
// Direction values
#define FORWARD         true
#define BACKWARD        false
#define BRAKE           false
#define RELEASE_BRAKE   true

/**
 * @brief Wheel index mapping
 * 
 *   Front
 * 
 *  3-----0
 * 
 *  Middle
 * 
 *  2-----1
 *   
 *   Back
 */
typedef enum wheel_index_t
{
    Start = -1,
    RF  = 0U,
    RB  = 1U,
    LF  = 2U,
    LB  = 3U,
    End = 4U
} wheel_indices;

/* Pseudo map bc Arduino does not support STL */
const Array<String, 4> wheel_index_name_map = {"RF", "RB", "LF", "LB"};

/**
 * @brief wheel motor pin struct
 * 
 */
typedef struct wheel_pins_t {
    uint8_t control;
    uint8_t brake_release;
    uint8_t dir;
};

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
/* default motor commands */
// Brake: duty = 0, brake on
const Array<wheel_motor_command_t, 4> all_brake_command = {{
    {0.0, BRAKE, FORWARD},
    {0.0, BRAKE, FORWARD},
    {0.0, BRAKE, FORWARD},
    {0.0, BRAKE, FORWARD}
}};
// Coast: duty = 0, brake released
const Array<wheel_motor_command_t, 4> all_coast_command = {{
    {0.0, RELEASE_BRAKE, FORWARD},
    {0.0, RELEASE_BRAKE, FORWARD},
    {0.0, RELEASE_BRAKE, FORWARD},
    {0.0, RELEASE_BRAKE, FORWARD}
}};

/**
 * @brief maps wheel index to a struct that defines the wheel pins (digital pin index)
 * 
 */
const Array<wheel_pins_t, 4> wheel_to_pins = {{
    {RF_controlPin, RF_brakeReleasePin, RF_dirPin},
    {RB_controlPin, RB_brakeReleasePin, RB_dirPin},
    {LF_controlPin, LF_brakeReleasePin, LF_dirPin},
    {LB_controlPin, LB_brakeReleasePin, LB_dirPin}   
}};

/**
 * @brief mapping of wheel index to the output compare associated with PWM generation
 *        update the register value with the duty wrt to the TOP value to alter the duty
 * 
 */
uint16_t* wheel_to_register[4] = {};
//     &OCR3B,     // RF: digital pin 2, PE4
//     &OCR3A,     // RB: digital pin 5, PE3
//     &OCR4A,     // LF: digital pin 6, PH3
//     &OCR4B      // LB: digital pin 7, PH4 
// };

/*  Function prototypes for wheel motion functions */
int wheel_set(wheel_index_t index, wheel_motor_command_t command);
int wheel_brake(wheel_index_t index);
int wheel_coast(wheel_index_t index);

#endif // !Wheel_h
