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
        cli();
        // update the brake state
        digitalWrite(wheel_to_pins[index].brake_release, (command.brake_release == RELEASE_BRAKE) ? HIGH : LOW);
        // update the duty
        *(wheel_to_register[index]) = command.duty_cycle * TOP;
        // update the direction
        digitalWrite(wheel_to_pins[index].dir, (command.dir == FORWARD) ? HIGH : LOW);
        // re-enable interrupts
        sei();
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
        cli();
        // apply braking
        digitalWrite(wheel_to_pins[index].brake_release, LOW);
        // set duty to 0
        *(wheel_to_register[index]) = 0;
        // re-enable interrupts
        sei();
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
        cli();
        // release the brake
        digitalWrite(wheel_to_pins[index].brake_release, HIGH);
        // set duty to 0
        *(wheel_to_register[index]) = 0;
        // re-enable interrupts
        sei();
    }
    return 0;
}

/*
class Wheel
{
private:
    
public:
    Wheel(wheel_index_t index);
    ~Wheel();
    set_speed();
    get_speed();
    
};

Wheel::Wheel(wheel_index_t index)
{

}

Wheel::~Wheel()
{
}

Wheel::set_speed(uint8_t pwm)
{
    
}
*/

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