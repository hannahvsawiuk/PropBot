#ifndef Wheel_h
#define Wheel_h

#include "pinout.h"

// Top value of the registers (configuration done in MAIN)
#define TOP         255
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

/**
 * @brief wheel motor pin struct
 * 
 */
typedef struct wheel_pins_t {
    uint8_t control,
    uint8_t brake_release,
    uint8_t dir
};

/**
 * @brief motor command structure that includes duty, brake, and direction
 * 
 */
typedef struct wheel_motor_command_t
{
    uint16_t duty_cycle; // float from 0 to 1
    bool brake_release;
    bool dir;
};
/* default motor commands */
// Brake: duty = 0, brake on
wheel_motor_command_t * all_brake[] = {
    {.duty_cycle = 0, .brake_release = BRAKE, .dir = FORWARD},
    {.duty_cycle = 0, .brake_release = BRAKE, .dir = FORWARD},
    {.duty_cycle = 0, .brake_release = BRAKE, .dir = FORWARD},
    {.duty_cycle = 0, .brake_release = BRAKE, .dir = FORWARD}
}
// Coast: duty = 0, brake released
wheel_motor_command_t * all_coast[] = {
    {.duty_cycle = 0, .brake_release = RELEASE_BRAKE, .dir = FORWARD},
    {.duty_cycle = 0, .brake_release = RELEASE_BRAKE, .dir = FORWARD},
    {.duty_cycle = 0, .brake_release = RELEASE_BRAKE, .dir = FORWARD},
    {.duty_cycle = 0, .brake_release = RELEASE_BRAKE, .dir = FORWARD}
}

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

/**
 * @brief maps wheel index to a struct that defines the wheel pins (digital pin index)
 * 
 */
static wheel_pins_t wheel_to_pins[4] = {
    {R_F_controlPin, R_F_brakeReleasePin, R_F_dirPin},
    {L_F_controlPin, L_F_brakeReleasePin, L_F_dirPin},
    {L_B_controlPin, L_B_brakeReleasePin, L_B_dirPin},   
    {R_B_controlPin, R_B_brakeReleasePin, R_B_dirPin}
};

/*  Function prototypes for wheel motion functions */
wheel_set(wheel_index_t index, wheel_motor_command_t command);
wheel_brake(wheel_index_t index);
wheel_coast(wheel_index_t index);

#endif // !Wheel_h
