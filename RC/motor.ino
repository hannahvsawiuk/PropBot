#include "wheel.h"
#include "sbl1360.h"

#ifdef SERIAL
    // do nothing if already defined
#else
    #define SERIAL true
#endif

motor_set(wheel_index_t index, motor_mode_t mode, uint8_t pwm, bool serial = false);
{
    /* Custom code for either CAN commands or PWM setting */

    uint16_t duty_cycle = pwm * TOP;
    
    if (SERIAL == true) {
        // code for serial comms
    } else {
        *(wheel_to_register[index]) = duty_cycle;
    }
    
}
