#ifndef Wheel_h
#define Wheel_h

#include <Arduino.h>
#include "pinout.h"
#include "util.h"

/* Define Important Values */
#define NUM_WHEELS 4
#define TOP         255     // Top value of the registers (configuration done in MAIN)
// Direction values
#define FORWARD     true
#define BACKWARD    false
#define BRAKE       false
#define RELEASE_BRAKE true
/**
 * @brief Wheel index mapping
 * 
 *   Front
 * 
 *  0-----1
 * 
 *  Middle
 * 
 *  3-----2
 *   
 *   Back
 */
typedef enum 
{
    RF  = 0U,
    LF  = 1U,
    LB  = 2U,
    RB  = 3U
} wheel_index_t;

// Pseudo map bc Arduino does not support STL
const Array<String, 4> wheel_index_name_map = {"RF", "LF", "LB", "RB"};

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
const Array<wheel_motor_command_t, 4> all_brake = {{
    {0.0, BRAKE, FORWARD},
    {0.0, BRAKE, FORWARD},
    {0.0, BRAKE, FORWARD},
    {0.0, BRAKE, FORWARD}
}};
// Coast: duty = 0, brake released
const Array<wheel_motor_command_t, 4> all_coast = {{
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
    {R_F_controlPin, R_F_brakeReleasePin, R_F_dirPin},
    {L_F_controlPin, L_F_brakeReleasePin, L_F_dirPin},
    {L_B_controlPin, L_B_brakeReleasePin, L_B_dirPin},   
    {R_B_controlPin, R_B_brakeReleasePin, R_B_dirPin}
}};

/**
 * @brief mapping of wheel index to the output compare associated with PWM generation
 *        update the register value with the duty wrt to the TOP value to alter the duty
 * 
 */
uint16_t* wheel_to_register[4] = {
    &OCR3A,     // LF: pin 5, digital 5
    &OCR3B,     // RF: pin 6, digital 2
    &OCR4A,     // LB: pin 15, digital 6
    &OCR4B      // RB: pin 16, digital 7 
};


/*  Function prototypes for wheel motion functions */
int wheel_set(wheel_index_t index, wheel_motor_command_t command);
int wheel_brake(wheel_index_t index);
int wheel_coast(wheel_index_t index);

#endif // !Wheel_h
