#include "rc_commands.h"

// Initialize the RC channel inputs as volatile 16-bit ints (values > 255)
volatile uint16_t sw_a       = 0;
volatile uint16_t sw_b       = 0;
volatile uint16_t rc_right   = 0;
volatile uint16_t rc_left    = 0;

/**
 * @brief fetches RC commands and returns motor commands
 * 
 * @return returns pointer to a list of motor commands
 */
Array<wheel_motor_command_t, 4> fetch_rc_commands() 
{
    // Get commands
    sw_a       = pulseIn(RC_SWA_CHANNEL_PIN, HIGH);
    sw_b       = pulseIn(RC_SWB_CHANNEL_PIN, HIGH);
    rc_right   = pulseIn(RC_RIGHT_CHANNEL_PIN, HIGH);
    rc_left    = pulseIn(RC_LEFT_CHANNEL_PIN, HIGH);
    
    // define temp variables
    uint16_t right_duty = 0;
    bool right_dir      = FORWARD;  // default direction
    uint16_t left_duty  = 0;
    bool left_dir       = FORWARD;  // default direction

    /* Check if stop asserted */
    if (sw_a < RC_SWX_HIGH_MAX && sw_a > RC_SWX_HIGH_MIN) {
        return all_brake;
    }
    // /* Check if mode changed */
    // if (sw_b < RC_SWX_HIGH_MAX && sw_b > RC_SWX_HIGH_MIN) {
    //     return None;
    // }
    /* Right side longitudinal wheel set */
    // Forward
    if (rc_right < RC_RIGHT_SET_FW_MAX && rc_right > RC_RIGHT_SET_FW_MIN) {
        // map the duty from 0 to 1 given the min and max threshold values
        right_duty = map(rc_right, RC_RIGHT_SET_FW_MIN, RC_RIGHT_SET_FW_MAX, 0, 1);
    }
    // Backward 
    else if (rc_right < RC_RIGHT_SET_BW_MAX && rc_right > RC_RIGHT_SET_BW_MIN)
    {
        right_duty = 1 - map(rc_right, RC_RIGHT_SET_BW_MIN, RC_RIGHT_SET_BW_MIN, 0, 1);
        right_dir = BACKWARD;
    } 
    /* Left side longitudinal wheel set */
    // Forward
    if (rc_left < RC_LEFT_SET_FW_MAX && rc_left > RC_LEFT_SET_FW_MIN) {
        left_duty = map(rc_left, RC_LEFT_SET_FW_MIN, RC_LEFT_SET_FW_MIN, 0, 1);
    }
    // Backward 
    else if (rc_left < RC_LEFT_SET_BW_MAX && rc_left > RC_LEFT_SET_BW_MIN)
    {
        left_duty = 1 - map(rc_left, RC_LEFT_SET_BW_MIN, RC_LEFT_SET_BW_MAX, 0, 1);
        left_dir = BACKWARD;
    }

    /* Define and fill wheel motor commands */
    Array<wheel_motor_command_t, 4> wheel_motor_commands {};
    // right set
    wheel_motor_commands[RF] = {.duty_cycle = right_duty, .brake_release = RELEASE_BRAKE, .dir = right_dir};
    wheel_motor_commands[RB] = {.duty_cycle = right_duty, .brake_release = RELEASE_BRAKE, .dir = right_dir};
    // left set
    wheel_motor_commands[LF] = {.duty_cycle = left_duty, .brake_release = RELEASE_BRAKE, .dir = left_dir};
    wheel_motor_commands[LB] = {.duty_cycle = left_duty, .brake_release = RELEASE_BRAKE, .dir = left_dir};

    /*  Return reference to the command set */
    return wheel_motor_commands;
}