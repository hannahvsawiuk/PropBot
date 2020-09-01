#include "rc_commands.h"

/**
 * @brief initializes RC comms
 * 
 */
void initialize_rc()
{
    pinMode(RC_RIGHT_CHANNEL_PIN,   INPUT);
    pinMode(RC_LEFT_CHANNEL_PIN,    INPUT);
    pinMode(RC_SWA_CHANNEL_PIN,     INPUT);
    pinMode(RC_SWB_CHANNEL_PIN,     INPUT);
}

/**
 * @brief fetches RC commands and modifies motor commands
 * 
 */
void fetch_rc_commands(wheel_motor_command_t* commands[NUM_WHEELS])
{
    // Get commands
    uint16_t sw_a       = pulseIn(RC_SWA_CHANNEL_PIN,   HIGH);
    uint16_t sw_b       = pulseIn(RC_SWB_CHANNEL_PIN,   HIGH);
    uint16_t rc_right   = pulseIn(RC_RIGHT_CHANNEL_PIN, HIGH);
    uint16_t rc_left    = pulseIn(RC_LEFT_CHANNEL_PIN,  HIGH);
    
    // define temp variables
    uint8_t right_duty    = 0;
    bool right_dir      = DIR_FW;  // default direction
    uint8_t left_duty     = 0;
    bool left_dir       = DIR_FW;  // default direction
    bool autonomy_mode  = false;

    /* Switches */
    // Check if stop asserted
    if (sw_a < RC_SWX_HIGH_MAX && sw_a > RC_SWX_HIGH_MIN) {
        brake(commands);
    }
    // Check if mode changed
    if (sw_b < RC_SWX_HIGH_MAX && sw_b > RC_SWX_HIGH_MIN) {
        autonomy_mode = true;
    }

    /* Right side longitudinal wheel set */
    // Forward
    if (rc_right < RC_RIGHT_SET_FW_MAX && rc_right > RC_RIGHT_SET_FW_MIN) {
        // map the duty from 0 to 1 given the min and max threshold values
        right_duty = speedMapToInt(rc_right, RC_RIGHT_SET_FW_MIN, RC_RIGHT_SET_FW_MAX, 0, 1, true);
    }
    // Backward 
    else if (rc_right < RC_RIGHT_SET_BW_MAX && rc_right > RC_RIGHT_SET_BW_MIN)
    {
        right_duty = speedMapToInt(rc_right, RC_RIGHT_SET_BW_MIN, RC_RIGHT_SET_BW_MAX, 0, 1, false);
        right_dir = DIR_BW;
    } 
    /* Left side longitudinal wheel set */
    // Forward
    if (rc_left < RC_LEFT_SET_FW_MAX && rc_left > RC_LEFT_SET_FW_MIN) {
        left_duty = speedMapToInt(rc_left, RC_LEFT_SET_FW_MIN, RC_LEFT_SET_FW_MAX, 0, 1, true);
    }
    // Backward 
    else if (rc_left < RC_LEFT_SET_BW_MAX && rc_left > RC_LEFT_SET_BW_MIN)
    {
        left_duty = speedMapToInt(rc_left, RC_LEFT_SET_BW_MIN, RC_LEFT_SET_BW_MAX, 0, 1, false);
        left_dir = DIR_BW;
    }

    /* Define and fill wheel motor commands */

    #ifdef PROPBOT
        for (int i = wheel_indices::Start; i < wheel_indices::End; i++ ) {
            if (i < (NUM_WHEELS / 2) - 1) {
                *commands[i] = wheel_motor_command_t{right_duty, RELEASE_BRAKE, right_dir}; 
            } else {
                *commands[i] = wheel_motor_command_t{left_duty, RELEASE_BRAKE, left_dir}; 
            }
        }
    #else
        *commands[0] = wheel_motor_command_t{left_duty, RELEASE_BRAKE, left_dir};
        *commands[1] = wheel_motor_command_t{right_duty, RELEASE_BRAKE, right_dir};
    #endif 
}