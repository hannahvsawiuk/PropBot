#include <mini_rc.h>

int mini_wheel_set(UC_DCMotor* motor, wheel_motor_command_t command)
{
    /* Custom code for either CAN commands or PWM setting */
    
    // BRAKE condition
    if (command.brake_release != RELEASE_BRAKE) {   
        motor->run(STOP)
        return;
    }

    // Step 1: set rotation direction
    if (command.dir == FORWARD) {
        motor->run(MINI_FW);
    } else {
        motor->run(MINI_FW);
    }

    // Step 2: set speed
    motor->setSpeed(command.duty_cycle * MINI_TOP);

    return 0;
}