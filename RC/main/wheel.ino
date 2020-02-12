#include "wheel.h"
#include "sbl1360.h"

#ifdef SERIAL
    // do nothing if SERIAL already defined (if sbl1360 drivers being used)
#else
    #define SERIAL false
#endif

/**
 * @brief Construct a new motor set object
 * 
 * @param index: wheel index
 * @param command: wheel motion command 
 */
int wheel_set(wheel_index_t index, wheel_motor_command_t command)
{
    /* Custom code for either CAN commands or PWM setting */

    uint16_t duty_cycle = command.duty_cycle * TOP;
    
    if (SERIAL == true) {
        // code for serial comms
    } else {
        // disable interrupts
        _disableInterrupts();
        // update the brake state
        _digitalWrite(wheel_to_pins[index].brake_release, (command.brake_release == RELEASE_BRAKE) ? HIGH : LOW);
        // update the duty
        *(wheel_to_register[index]) = command.duty_cycle * TOP;
        // update the direction
        _digitalWrite(wheel_to_pins[index].dir, (command.dir == FORWARD) ? HIGH : LOW);
        // re-enable interrupts
        _enableInterrupts();
    }
    return 0;
}
/**
 * @brief Send brake command to a wheel
 * 
 * @param index: wheel index
 */
int wheel_brake(wheel_index_t index) 
{
    if (SERIAL == true) {
        // code for serial comms
    } else {
        // disable interrupts
        _disableInterrupts();
        // apply braking
        _digitalWrite(wheel_to_pins[index].brake_release, LOW);
        // set duty to 0
        *(wheel_to_register[index]) = 0;
        // re-enable interrupts
        _enableInterrupts();
    }
    return 0;
}

/**
 * @brief Send motor coast command to a wheel
 * 
 * @param index: wheel index
 */
int wheel_coast(wheel_index_t index) 
{
    if (SERIAL == true) {
        // code for serial comms
    } else {
        // disable interrupts
        _disableInterrupts();
        // release the brake
        _digitalWrite(wheel_to_pins[index].brake_release, HIGH);
        // set duty to 0
        *(wheel_to_register[index]) = 0;
        // re-enable interrupts
        _enableInterrupts();
    }
    return 0;
}


// class Wheel
// {
// private:
//     wheel_pins_t wheel_pins;
//     uint16_t* control_register;
// public:
//     Wheel(wheel_pins_t pins, uint16_t* control_register);
//     ~Wheel();
//     setSpeed();
//     getSpeed();
    
// };

// Wheel::Wheel(wheel_pins_t wheel_pins, uint16_t* control_register)
//     : wheel_pins{wheel_pins}
//     , control_register{control_register}
//     {}

// Wheel::~Wheel() 
// {
//     delete control_register
// }

// Wheel::setSpeed(wheel_motor_command_t command)
// {
//     /* Custom code for either CAN commands or PWM setting */

//     uint16_t duty_cycle = command.duty_cycle * TOP;
    
//     if (SERIAL == true) {
//         // code for serial comms
//     } else {
//         // disable interrupts
//         _disableInterrupts();
//         // update the brake state
//         _digitalWrite(wheel_pins.brake_release, (command.brake_release == RELEASE_BRAKE) ? HIGH : LOW);
//         // update the duty
//         *control_register = command.duty_cycle * TOP;
//         // update the direction
//         _digitalWrite(wheel_pins.dir, (command.dir == FORWARD) ? HIGH : LOW);
//         // re-enable interrupts
//         _enableInterrupts();
//     }
//     return 0;
// }
// }


/**
 * @brief Construct a new Wheel::get_speed object
 * 
 */
/*
uint16_t Wheel::get_speed()
{
    // return wheel speed from encoders
}
*/