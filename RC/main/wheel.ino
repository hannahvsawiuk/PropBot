#include "wheel.h"

#ifdef PROPBOT

    Wheel::Wheel(uint8_t wheel_index)
        : 
        , wheel_index(wheel_index)
        , wheel_pins{wheel_to_pins[wheel_index]}
        , control_register{wheel_to_register[wheel_index]}
    {
    }

    Wheel::~Wheel() 
    {
        delete control_register
    }

    Wheel::sendCommand(wheel_motor_command_t command)
    {
        /* Custom code for either CAN commands or PWM setting */

        uint16_t duty_cycle = command.duty_cycle * TOP;
        
        // disable interrupts
        _disableInterrupts();
        // update the brake state
        _digitalWrite(wheel_pins.brake_release, (command.brake_release == RELEASE_BRAKE) ? HIGH : LOW);
        // update the duty
        *control_register = command.duty_cycle * TOP;
        // update the direction
        _digitalWrite(wheel_pins.dir, (command.dir == DIR_FW) ? HIGH : LOW);
        // re-enable interrupts
        _enableInterrupts();
        }
        return 0;
    }

    /**
     * @brief Construct a new Wheel::get_speed object
     * 
     */

    uint16_t Wheel::get_speed()
    {
        // return wheel speed from encoders
        return 0;
    }
#else
    /* Mini RC wheel class */
    Wheel::Wheel(uint8_t wheel_index)
    : wheel_index(wheel_index)
    , uc_motor(UC_DCMotor(wheel_index))
    {
        // uc_motor = UC_DCMotor(wheel_index);
        Serial.println(String("Init motor: ") + wheel_index);
        uc_motor.run(STOP);
        uc_motor.setSpeed(0);
    }

    Wheel::~Wheel() 
    {
        // delete uc_motor;
    }

    Wheel::sendCommand(wheel_motor_command_t command)
    {
        // BRAKE condition
        if (command.brake_release == ENGAGE_BRAKE) {   
            // Serial.println("BRAKE");
            uc_motor.run(STOP);
            uc_motor.setSpeed(0);
            return;
        }

        // Step 1: set rotation direction
        if (command.dir == DIR_FW) {
            // Serial.println("FW");
            uc_motor.run(FORWARD);
        } else {
            // Serial.println("BW");
            uc_motor.run(BACKWARD);
        }
        uc_motor.setSpeed(command.duty_cycle * TOP);
    }

#endif