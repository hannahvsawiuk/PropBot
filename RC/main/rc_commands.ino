#include "rc_commands.h"
#include "util.h"
#define DEBUG_MODE
#include "debug.h"


/**
 * @brief fetches RC commands and returns motor commands
 * 
 * @return returns array of motor commands
 */
Array<wheel_motor_command_t, NUM_WHEELS> fetch_rc_commands() 
{
    // Get commands
    uint16_t sw_a       = pulseIn(RC_SWA_CHANNEL_PIN,   HIGH);
    uint16_t sw_b       = pulseIn(RC_SWB_CHANNEL_PIN,   HIGH);
    uint16_t rc_right   = pulseIn(RC_RIGHT_CHANNEL_PIN, HIGH);
    uint16_t rc_left    = pulseIn(RC_LEFT_CHANNEL_PIN,  HIGH);
    
    // define temp variables
    float right_duty    = 0.0;
    bool right_dir      = DIR_FW;  // default direction
    float left_duty     = 0.0;
    bool left_dir       = DIR_FW;  // default direction
    bool autonomy_mode  = false;

    /* Switches */
    // Check if stop asserted
    if (sw_a < RC_SWX_HIGH_MAX && sw_a > RC_SWX_HIGH_MIN) {
        DEBUG_PRINT("All brake");
        return all_brake_command; // fix to also return mode
    }
    // Check if mode changed
    if (sw_b < RC_SWX_HIGH_MAX && sw_b > RC_SWX_HIGH_MIN) {
        DEBUG_PRINT("Manual Mode")
        autonomy_mode = true;
    }

    /* Right side longitudinal wheel set */
    // Forward
    if (rc_right < RC_RIGHT_SET_FW_MAX && rc_right > RC_RIGHT_SET_FW_MIN) {
        // map the duty from 0 to 1 given the min and max threshold values
        // right_duty = logMapToFloat(rc_right, RC_RIGHT_SET_FW_MIN, RC_RIGHT_SET_FW_MAX, 0, 1);
        right_duty = linMapToFloat(rc_right, RC_RIGHT_SET_FW_MIN, RC_RIGHT_SET_FW_MAX, 0, 1);
    }
    // Backward 
    else if (rc_right < RC_RIGHT_SET_BW_MAX && rc_right > RC_RIGHT_SET_BW_MIN)
    {
        // right_duty = logMapToFloat(rc_right, RC_RIGHT_SET_BW_MIN, RC_RIGHT_SET_BW_MAX, 0, 1, true);
        right_duty = 1 - linMapToFloat(rc_right, RC_RIGHT_SET_BW_MIN, RC_RIGHT_SET_BW_MAX, 0, 1);
        right_dir = DIR_BW;
    } 
    /* Left side longitudinal wheel set */
    // Forward
    if (rc_left < RC_LEFT_SET_FW_MAX && rc_left > RC_LEFT_SET_FW_MIN) {
        // left_duty = logMapToFloat(rc_left, RC_LEFT_SET_FW_MIN, RC_LEFT_SET_FW_MAX, 0, 1);
        left_duty = linMapToFloat(rc_left, RC_LEFT_SET_FW_MIN, RC_LEFT_SET_FW_MAX, 0, 1);
    }
    // Backward 
    else if (rc_left < RC_LEFT_SET_BW_MAX && rc_left > RC_LEFT_SET_BW_MIN)
    {
        // left_duty = logMapToFloat(rc_left, RC_LEFT_SET_BW_MIN, RC_LEFT_SET_BW_MAX, 0, 1, true);
        left_duty = 1 - linMapToFloat(rc_left, RC_LEFT_SET_BW_MIN, RC_LEFT_SET_BW_MAX, 0, 1);
        left_dir = DIR_BW;
    }

    /* Define and fill wheel motor commands */
    #ifdef PROPBOT
        Array<wheel_motor_command_t, NUM_WHEELS> wheel_motor_commands = {{
            {right_duty, RELEASE_BRAKE, right_dir},
            {right_duty, RELEASE_BRAKE, right_dir},
            {left_duty, RELEASE_BRAKE, left_dir},
            {left_duty, RELEASE_BRAKE, left_dir}
        }};
    #else
        Array<wheel_motor_command_t, NUM_WHEELS> wheel_motor_commands = {{
            {left_duty, RELEASE_BRAKE, left_dir},
            {right_duty, RELEASE_BRAKE, right_dir},
        }};
    #endif 
    
    /*  Return reference to the command set */
    return wheel_motor_commands;
}