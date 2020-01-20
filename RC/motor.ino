#include "wheel.h"

motor_set(wheel_index_t index, motor_mode_t mode, uint8_t pwm);
{
    /* Custom code for either CAN commands or PWM setting */

    uint16_t duty_cycle = pwm * TOP;
    
    *(wheel_to_register[index]) = duty_cycle;
    
}
